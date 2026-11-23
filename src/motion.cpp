// ---------------------------------------------------------------------------
// Rally CAN Gauge - motion implementation
// ---------------------------------------------------------------------------
// Layout of this file:
//   1. vectors + the orientation (sensor axes -> vehicle axes)
//   2. the low-g path: g-meter, high-g trim, jumps
//   3. the gyro path: stillness, gyro zero, roll/pitch
//   4. the high-g path: peak hold, the impact trigger, the capture ring
//   5. the 20 ms tick: publishing + the LEVEL SET gesture
//   6. what the black box and the capture store read
//
// THE THREE STREAMS AND WHAT EACH IS TRUSTED FOR
//
//   low-g accel, 120 Hz   accurate (+-12 mg offset) but stops at 16 g.
//                         The g-meter, "which way is up", and the
//                         reference the high-g channel is trimmed against.
//   gyro, 120 Hz          how the car is ROTATING. Integrated, it carries
//                         roll and pitch through everything the
//                         accelerometer cannot see past (cornering, braking,
//                         a landing, a roll).
//   high-g accel, 960 Hz  reads to 320 g but its zero is only good to
//                         +-1.5 g. Trimmed, it is the impact channel: peak
//                         hold for the log, the capture trigger, the
//                         waveform that goes to the FRAM.
//
// ATTITUDE, IN ONE PARAGRAPH. The filter tracks one thing: the direction of
// "up" as seen from inside the car (a unit vector in vehicle axes; level =
// (0, 0, 1)). Each gyro sample turns that vector by the rotation the car
// just made. Each accelerometer sample that looks like plain gravity (total
// within IMU_ATT_GATE_G of 1 g) nudges it toward where gravity says up is,
// with time constant IMU_ATT_TAU_S. Roll and pitch are read off the vector.
// It never needs the heading, so there is nothing to drift in yaw.
//
// THE HIGH-G TRIM. Both accelerometers feel the same acceleration, so the
// difference between them, averaged, IS the high-g channel's offset. Every
// low-g sample is compared with the mean of the high-g samples that arrived
// since the previous one (eight of them), while the low-g channel is well
// inside its range and no capture is running.
//
// THE CAPTURE RING. High-g samples are decimated 4:1 into frames (per axis,
// the sample of largest size - a peak survives the decimation), frames are
// packed 12 to a block with the attitude at the block's end, and the last
// 128 blocks (6.4 s) are always in RAM. A trigger therefore finds its own
// lead-in already recorded: the capture is defined as "the 20 blocks before
// the trigger block and the 80 from it on", and fram_events.cpp copies
// blocks out of the ring to the FRAM as they complete.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <math.h>

#include "motion.h"
#include "imu.h"       // counts -> g / deg/s
#include "imu_codec.h" // g -> stored byte
#include "config.h"
#include "vehicle_data.h"
#include "fram_log.h"  // the level calibration lives in the FRAM

// --- 1. vectors + orientation -------------------------------------------------------

struct Vec3
    {
    float x, y, z;
    };

static inline Vec3  add(Vec3 a, Vec3 b)    { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline Vec3  sub(Vec3 a, Vec3 b)    { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline Vec3  scale(Vec3 a, float k) { return { a.x * k, a.y * k, a.z * k }; }
static inline float dot(Vec3 a, Vec3 b)    { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float length(Vec3 a)         { return sqrtf(dot(a, a)); }
static inline Vec3  cross(Vec3 a, Vec3 b)
    {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }
static inline Vec3 normalized(Vec3 a)
    {
    const float n = length(a);
    return (n > 1e-9f) ? scale(a, 1.0f / n) : Vec3{ 0.0f, 0.0f, 1.0f };
    }

// Move 'state' a fraction 'alpha' of the way to 'target': one step of an
// exponential moving average.
static inline void follow(Vec3& state, Vec3 target, float alpha)
    {
    state = add(state, scale(sub(target, state), alpha));
    }

// alpha for a time constant tau at a sample interval dt.
static constexpr float emaAlpha(float dt, float tau) { return dt / (tau + dt); }

static const float DEG2RAD = 0.01745329252f;
static const float RAD2DEG = 57.2957795131f;
static const float G0_MPS2 = 9.80665f;

static const float LG_DT = 1.0f / IMU_LG_RATE_HZ; // seconds per low-g / gyro sample
static const float HG_DT = 1.0f / IMU_HG_RATE_HZ; // seconds per high-g sample

// Step 1 of the orientation: the signed permutation from config.h. For each
// vehicle axis (forward, left, up): which sensor axis, and with what sign.
static uint8_t s_axisIndex[3];
static float   s_axisSign[3];

// Step 2: the LEVEL SET rotation, "nominal" vehicle axes -> true vehicle
// axes. Identity until a level set has been done.
static float s_level[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

static Vec3 toNominal(const float s[3])
    {
    return { s_axisSign[0] * s[s_axisIndex[0]],
             s_axisSign[1] * s[s_axisIndex[1]],
             s_axisSign[2] * s[s_axisIndex[2]] };
    }

static Vec3 toVehicle(Vec3 n)
    {
    return { s_level[0][0] * n.x + s_level[0][1] * n.y + s_level[0][2] * n.z,
             s_level[1][0] * n.x + s_level[1][1] * n.y + s_level[1][2] * n.z,
             s_level[2][0] * n.x + s_level[2][1] * n.y + s_level[2][2] * n.z };
    }

static void decodeAxis(ImuAxis a, uint8_t& index, float& sign)
    {
    index = (uint8_t)a / 2;                // POS_X, NEG_X -> 0 ... POS_Z, NEG_Z -> 2
    sign  = ((uint8_t)a & 1) ? -1.0f : 1.0f;
    }

// The rotation that carries the measured "up" direction onto +Z by the
// shortest route (Rodrigues' formula about the axis up x Z). Shortest route
// matters: the gauge is clamped to a bar that runs across the car, so the
// only large unknown is how far it is tipped ABOUT that bar - and the
// shortest rotation from a vector tipped about Y back to Z is a rotation
// about Y. Left and right stay left and right.
static void buildLevelRotation(Vec3 up)
    {
    up = normalized(up);
    const float c = up.z;                              // cos(angle between up and +Z)
    const float s = sqrtf(up.x * up.x + up.y * up.y);  // sin(same angle)

    float m[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    if (s > 1e-6f)
        {
        const float kx = up.y / s, ky = -up.x / s;     // unit axis = up x Z (its z part is 0)
        const float v  = 1.0f - c;
        m[0][0] = c + kx * kx * v;  m[0][1] = kx * ky * v;      m[0][2] =  ky * s;
        m[1][0] = kx * ky * v;      m[1][1] = c + ky * ky * v;  m[1][2] = -kx * s;
        m[2][0] = -ky * s;          m[2][1] = kx * s;           m[2][2] = c;
        }
    else if (c < 0.0f)
        {
        // Measured exactly upside down: any horizontal axis would do. Turn
        // about the forward axis. (A gauge mounted inverted should have its
        // axis map changed in config.h instead.)
        m[1][1] = -1.0f;
        m[2][2] = -1.0f;
        }
    memcpy(s_level, m, sizeof(s_level));
    }

// --- module state ---------------------------------------------------------------------

// low-g path
static Vec3     s_lgVeh;          // last low-g sample, vehicle axes, g
static float    s_lgMag;          // its size
static bool     s_haveLg;         // one has arrived since the last gap
static Vec3     s_gFilt;          // smoothed for the g-meter (IMU_G_FILTER_TAU_S)
static Vec3     s_airFilt;        // 25 ms average: what the free-fall test looks at
static bool     s_filtSeeded;

// high-g trim (sensor axes)
static float    s_hgAcc[3];       // sum of high-g samples since the last low-g one
static uint16_t s_hgAccN;
static float    s_hgOffset[3];
static bool     s_trimSeeded;
static uint16_t s_trimCount;      // trim updates so far (saturates)

// gyro path
static Vec3     s_gyroBias;       // learned zero, vehicle axes, deg/s
static Vec3     s_gyroLp;         // 0.5 s average of the raw rate
static Vec3     s_rate;           // last rate, zero removed, deg/s
static float    s_yawFilt;
static uint16_t s_stillSamples;   // consecutive samples that looked still
static Vec3     s_up = { 0, 0, 1 }; // the attitude: "up" in vehicle axes
static bool     s_attValid;

// peak hold
static float    s_binMag[IMU_PEAK_BINS];
static Vec3     s_binAxis[IMU_PEAK_BINS];
static uint8_t  s_binIndex;
static uint8_t  s_binCount;
static float    s_maxPeak;

// black box interval
static float    s_logLonSum, s_logLatSum, s_logPeak;
static uint32_t s_logN;

// jumps
static uint16_t s_airSamples;     // consecutive low-g samples in free fall
static uint16_t s_jumpCount;
static uint16_t s_lastAirMs;
static float    s_lastLandG;
static uint16_t s_landLeft;       // high-g samples left in the landing window

// the capture ring
static const uint8_t RING_BLOCKS = 128;
static_assert(RING_BLOCKS >= IMU_EVENT_BLOCKS + 16, "the ring must hold a whole capture plus the store's lag");
static_assert(IMU_EVENT_PRE_BLOCKS < IMU_EVENT_BLOCKS, "a capture must extend past its trigger");
static_assert((uint32_t)IMU_EVENT_BLOCKS * IMU_EVENT_BLOCK_FRAMES < 65536, "peakFrame is 16 bits");

static uint8_t  s_ring[RING_BLOCKS][MOTION_BLOCK_BYTES];
static uint32_t s_blockAbs;       // number of the block being filled (counts up forever)
static uint32_t s_blocksValid;    // finished blocks since the last gap (lead-in available)
static uint8_t  s_frameInBlock;
static uint8_t  s_sampleInFrame;
static Vec3     s_frame;          // per axis: largest-size sample of the frame so far

// the trigger + the capture in progress
static uint8_t  s_overRun;        // consecutive samples at or above the trigger level
static float    s_runPeak;        // largest of that run (a capture starts with it)
static Vec3     s_runAxis;
static bool     s_gateOpen;       // above the delta-v gate right now
static Vec3     s_gateDv;         // delta-v of the pulse so far, before any capture claims it
static bool     s_evHas, s_evActive, s_evComplete, s_storeBusy;
static uint32_t s_evId, s_nextEventId = 1;
static uint32_t s_evStartAbs;
static uint8_t  s_evPre, s_evTrigFrame, s_evFlags;
static uint16_t s_evPeakFrame;
static float    s_evPeak;
static Vec3     s_evAxis, s_evDv;
static uint32_t s_evOverSamples;
static uint16_t s_evAirMs;

// LEVEL SET gesture
static bool     s_lsOnDetent, s_lsCollect;
static uint32_t s_lsStartMs;
static Vec3     s_lsSum;          // nominal axes
static Vec3     s_lgNominal;
static uint32_t s_lsN;

static const uint16_t STILL_SAMPLES     = (uint16_t)(IMU_STILL_MS * IMU_LG_RATE_HZ / 1000);
static const uint16_t STILL_NOW_SAMPLES = IMU_LG_RATE_HZ / 10; // 0.1 s: "not moving right now"

static inline uint16_t airMs(uint16_t samples)
    {
    return (uint16_t)min((uint32_t)samples * 1000u / IMU_LG_RATE_HZ, 65535u);
    }

// Keep whichever of 'held' and 'value' is larger in size, sign included.
static inline void holdLargest(float& held, float value)
    {
    if (fabsf(value) > fabsf(held)) held = value;
    }

// --- 2. the low-g path ----------------------------------------------------------------

void motionFeedLowG(const int16_t counts[3])
    {
    const float sg[3] = { counts[0] * IMU_LG_G_PER_COUNT,
                          counts[1] * IMU_LG_G_PER_COUNT,
                          counts[2] * IMU_LG_G_PER_COUNT };

    // High-g trim: the mean of the high-g samples since the previous low-g
    // sample, minus this sample, is the high-g offset (plus noise).
    if (s_hgAccN >= 4)
        {
        const bool inRange = fabsf(sg[0]) < IMU_HG_TRIM_MAX_G && fabsf(sg[1]) < IMU_HG_TRIM_MAX_G
                          && fabsf(sg[2]) < IMU_HG_TRIM_MAX_G;
        if (inRange && !s_evActive)
            {
            static const float alpha = emaAlpha(LG_DT, IMU_HG_TRIM_TAU_S);
            for (uint8_t i = 0; i < 3; i++)
                {
                const float diff = s_hgAcc[i] / s_hgAccN - sg[i];
                if (!s_trimSeeded) s_hgOffset[i] = diff; // first look: take it whole
                else               s_hgOffset[i] += alpha * (diff - s_hgOffset[i]);
                }
            s_trimSeeded = true;
            if (s_trimCount < 65535) s_trimCount++;
            }
        }
    s_hgAcc[0] = s_hgAcc[1] = s_hgAcc[2] = 0.0f;
    s_hgAccN = 0;

    s_lgNominal = toNominal(sg);
    s_lgVeh     = toVehicle(s_lgNominal);
    s_lgMag     = length(s_lgVeh);
    s_haveLg    = true;

    if (!s_filtSeeded)
        {
        s_gFilt = s_airFilt = s_lgVeh;
        s_filtSeeded = true;
        }
    else
        {
        static const float aG   = emaAlpha(LG_DT, IMU_G_FILTER_TAU_S);
        static const float aAir = emaAlpha(LG_DT, 0.025f);
        follow(s_gFilt,   s_lgVeh, aG);
        follow(s_airFilt, s_lgVeh, aAir);
        }

    s_logLonSum += s_lgVeh.x;
    s_logLatSum += s_lgVeh.y;
    s_logN++;

    if (s_lsCollect)
        {
        s_lsSum = add(s_lsSum, s_lgNominal);
        s_lsN++;
        }

    // Jumps: free fall is the one time an accelerometer reads nothing.
    if (length(s_airFilt) < IMU_AIR_G)
        {
        if (s_airSamples < 65535) s_airSamples++;
        }
    else
        {
        if (airMs(s_airSamples) >= IMU_AIR_MIN_MS)
            {
            // Wheels just came back: that was a jump. Open the landing window.
            if (s_jumpCount < 65535) s_jumpCount++;
            s_lastAirMs = airMs(s_airSamples);
            s_lastLandG = 0.0f;
            s_landLeft  = (uint16_t)((uint32_t)IMU_LAND_WINDOW_MS * IMU_HG_RATE_HZ / 1000);
            }
        s_airSamples = 0;
        }
    }

// --- 3. the gyro path -----------------------------------------------------------------

void motionFeedGyro(const int16_t counts[3])
    {
    const float sd[3] = { counts[0] * IMU_GYRO_DPS_PER_COUNT,
                          counts[1] * IMU_GYRO_DPS_PER_COUNT,
                          counts[2] * IMU_GYRO_DPS_PER_COUNT };
    // A rotation rate turns between frames exactly like an acceleration
    // does, as long as the axis map is right-handed (checked in motionInit).
    const Vec3 raw = toVehicle(toNominal(sd));

    // Stillness. Judged on how steady the gyro is against its own short
    // average, not against zero: until the zero has been learned once, a
    // perfectly still gyro reads its offset (up to a degree or two a second).
    static const float aLp = emaAlpha(LG_DT, 0.5f);
    follow(s_gyroLp, raw, aLp);
    const bool gyroSteady = length(sub(raw, s_gyroLp)) < IMU_STILL_GYRO_DPS
                         && length(s_gyroLp) < IMU_STILL_MAX_DPS;
    const bool accelAtRest = s_filtSeeded && fabsf(length(s_gFilt) - 1.0f) < IMU_STILL_ACC_G;
    const bool engineQuiet = !g_data.ecuOnline() || g_data.ecu.rpm < IMU_STILL_MAX_RPM;

    if (gyroSteady && accelAtRest && engineQuiet)
        {
        if (s_stillSamples < 65535) s_stillSamples++;
        }
    else
        s_stillSamples = 0;

    if (s_stillSamples >= STILL_SAMPLES)
        {
        static const float aBias = emaAlpha(LG_DT, IMU_GYRO_BIAS_TAU_S);
        follow(s_gyroBias, s_gyroLp, aBias);
        }

    s_rate = sub(raw, s_gyroBias);
    static const float aG = emaAlpha(LG_DT, IMU_G_FILTER_TAU_S);
    s_yawFilt += aG * (s_rate.z - s_yawFilt);

    // Attitude.
    if (!s_attValid)
        {
        // (Re)start from the accelerometer, once it shows something that
        // could be gravity alone.
        if (s_haveLg && s_lgMag > 0.7f && s_lgMag < 1.3f)
            {
            s_up = normalized(s_lgVeh);
            s_attValid = true;
            }
        return;
        }

    // The car turned by (rate x dt). A direction fixed in the world, seen
    // from the turning car, moves the other way: d(up)/dt = up x omega.
    s_up = normalized(add(s_up, scale(cross(s_up, scale(s_rate, DEG2RAD)), LG_DT)));

    // The accelerometer's vote, weighted by how much it looks like gravity
    // and nothing else: full weight at exactly 1 g, none at the gate.
    const float off = fabsf(s_lgMag - 1.0f);
    if (off < IMU_ATT_GATE_G)
        {
        static const float aAtt = emaAlpha(LG_DT, IMU_ATT_TAU_S);
        const float weight = 1.0f - off / IMU_ATT_GATE_G;
        follow(s_up, scale(s_lgVeh, 1.0f / s_lgMag), aAtt * weight);
        s_up = normalized(s_up);
        }
    }

// --- 4. the high-g path ---------------------------------------------------------------

static void startEvent()
    {
    // The lead-in is whatever the ring holds, up to IMU_EVENT_PRE_BLOCKS -
    // less right after boot or after a hole in the data.
    const uint8_t pre = (s_blocksValid < IMU_EVENT_PRE_BLOCKS) ? (uint8_t)s_blocksValid
                                                               : IMU_EVENT_PRE_BLOCKS;
    s_evStartAbs    = s_blockAbs - pre;
    s_evPre         = pre;
    s_evTrigFrame   = s_frameInBlock;
    s_evId          = s_nextEventId++;
    s_evHas         = true;
    s_evActive      = true;
    s_evComplete    = false;
    s_evFlags       = (uint8_t)((g_data.imu.levelCal ? MOTION_EVF_LEVEL_CAL : 0)
                              | (s_trimCount >= IMU_LG_RATE_HZ ? MOTION_EVF_HG_TRIM : 0));
    s_evPeak        = 0.0f;                 // accumulateEvent() fills these in from
    s_evAxis        = { 0.0f, 0.0f, 0.0f }; // the trigger run on the same sample
    s_evPeakFrame   = (uint16_t)(pre * IMU_EVENT_BLOCK_FRAMES + s_frameInBlock);
    s_evDv          = s_gateDv;                  // the pulse's delta-v up to and including this sample
    s_evOverSamples = (uint32_t)(s_overRun - 1); // the run's earlier samples; this one counts itself
    s_evAirMs       = (s_landLeft > 0) ? s_lastAirMs : 0; // a landing?
    }

// dv = this sample's contribution to delta-v (zero below the gate).
static void accumulateEvent(Vec3 v, float mag, bool clipped, Vec3 dv)
    {
    if (mag > s_evPeak)
        {
        s_evPeak      = mag;
        s_evPeakFrame = (uint16_t)((s_blockAbs - s_evStartAbs) * IMU_EVENT_BLOCK_FRAMES + s_frameInBlock);
        }
    holdLargest(s_evAxis.x, v.x);
    holdLargest(s_evAxis.y, v.y);
    holdLargest(s_evAxis.z, v.z);
    if (mag >= IMU_EVENT_TRIGGER_G) s_evOverSamples++;
    s_evDv = add(s_evDv, dv);
    if (clipped) s_evFlags |= MOTION_EVF_CLIPPED;
    }

static void finishBlock()
    {
    // The attitude rides along at 20 Hz: roll in 2-degree steps (a byte
    // then covers a full turn), pitch in degrees, yaw rate in 4 deg/s steps.
    const float roll  = atan2f(s_up.y, s_up.z) * RAD2DEG;
    const float pitch = asinf(constrain(s_up.x, -1.0f, 1.0f)) * RAD2DEG;
    uint8_t* b = s_ring[s_blockAbs % RING_BLOCKS];
    b[IMU_EVENT_BLOCK_FRAMES * 3 + 0] = (uint8_t)(int8_t)lroundf(constrain(roll / 2.0f, -127.0f, 127.0f));
    b[IMU_EVENT_BLOCK_FRAMES * 3 + 1] = (uint8_t)(int8_t)lroundf(constrain(pitch, -127.0f, 127.0f));
    b[IMU_EVENT_BLOCK_FRAMES * 3 + 2] = (uint8_t)(int8_t)lroundf(constrain(s_rate.z / 4.0f, -127.0f, 127.0f));

    s_blockAbs++;
    s_frameInBlock = 0;
    if (s_blocksValid < 0xFFFFFFFFu) s_blocksValid++;

    if (s_evActive && s_blockAbs - s_evStartAbs >= IMU_EVENT_BLOCKS)
        {
        s_evActive   = false;
        s_evComplete = true;
        }
    }

void motionFeedHighG(const int16_t counts[3])
    {
    const float sg[3] = { counts[0] * IMU_HG_G_PER_COUNT,
                          counts[1] * IMU_HG_G_PER_COUNT,
                          counts[2] * IMU_HG_G_PER_COUNT };
    s_hgAcc[0] += sg[0];
    s_hgAcc[1] += sg[1];
    s_hgAcc[2] += sg[2];
    s_hgAccN++;

    // Until the first trim (8 ms after the stream starts) a reading can be
    // 1.5 g out on every axis: not worth a peak, let alone a trigger.
    if (!s_trimSeeded)
        return;

    const bool clipped = abs(counts[0]) >= 32700 || abs(counts[1]) >= 32700 || abs(counts[2]) >= 32700;
    const float trimmed[3] = { sg[0] - s_hgOffset[0], sg[1] - s_hgOffset[1], sg[2] - s_hgOffset[2] };
    const Vec3  v   = toVehicle(toNominal(trimmed));
    const float mag = length(v);

    // Peak hold: a ring of short bins. The value sent on CAN is the largest
    // across all of them, so a hit stays visible for the whole hold time.
    if (mag > s_binMag[s_binIndex]) s_binMag[s_binIndex] = mag;
    holdLargest(s_binAxis[s_binIndex].x, v.x);
    holdLargest(s_binAxis[s_binIndex].y, v.y);
    holdLargest(s_binAxis[s_binIndex].z, v.z);
    if (++s_binCount >= IMU_PEAK_BIN_SAMPLES)
        {
        s_binCount = 0;
        s_binIndex = (uint8_t)((s_binIndex + 1) % IMU_PEAK_BINS);
        s_binMag[s_binIndex]  = 0.0f;
        s_binAxis[s_binIndex] = { 0.0f, 0.0f, 0.0f };
        }

    if (mag > s_logPeak) s_logPeak = mag;
    if (mag > s_maxPeak) s_maxPeak = mag;

    if (s_landLeft > 0)
        {
        s_landLeft--;
        if (mag > s_lastLandG) s_lastLandG = mag;
        }

    // Delta-v. What an accelerometer feels is the car's true acceleration
    // PLUS 1 g pointing up; take that 1 g out (the attitude filter knows
    // where up is, and the gyro keeps it honest through the hit) and what is
    // left, summed over time, is the change in the car's velocity. Only
    // counted above the gate, so sensor offset cannot accumulate between
    // hits. A pulse is tallied from the moment it crosses the gate - before
    // anyone knows whether it will reach the trigger - and a capture that
    // starts inherits the tally.
    Vec3 dv = { 0.0f, 0.0f, 0.0f };
    if (mag >= IMU_EVENT_DV_GATE_G)
        {
        const Vec3 up = s_attValid ? s_up : Vec3{ 0.0f, 0.0f, 1.0f };
        dv = scale(sub(v, up), G0_MPS2 * HG_DT);
        if (!s_gateOpen) { s_gateOpen = true; s_gateDv = { 0.0f, 0.0f, 0.0f }; }
        s_gateDv = add(s_gateDv, dv);
        }
    else
        s_gateOpen = false;

    // The trigger: IMU_EVENT_MIN_SAMPLES in a row at or above the level.
    // The run's own peak is remembered, because the hardest sample of a
    // sharp hit is usually the FIRST one, before the count is satisfied.
    if (mag >= IMU_EVENT_TRIGGER_G)
        {
        if (s_overRun == 0) { s_runPeak = 0.0f; s_runAxis = { 0.0f, 0.0f, 0.0f }; }
        if (s_overRun < 255) s_overRun++;
        if (mag > s_runPeak) s_runPeak = mag;
        holdLargest(s_runAxis.x, v.x);
        holdLargest(s_runAxis.y, v.y);
        holdLargest(s_runAxis.z, v.z);
        }
    else
        s_overRun = 0;

    if (!s_evActive && !s_storeBusy && s_overRun >= IMU_EVENT_MIN_SAMPLES)
        {
        startEvent(); // takes s_gateDv, which already includes this sample
        s_evPeak = s_runPeak;
        s_evAxis = s_runAxis;
        dv = { 0.0f, 0.0f, 0.0f };
        }
    if (s_evActive)
        accumulateEvent(v, mag, clipped, dv);

    // The capture ring: 4 samples -> 1 frame -> 3 bytes.
    holdLargest(s_frame.x, v.x);
    holdLargest(s_frame.y, v.y);
    holdLargest(s_frame.z, v.z);
    if (++s_sampleInFrame >= IMU_EVENT_DECIM)
        {
        uint8_t* f = &s_ring[s_blockAbs % RING_BLOCKS][s_frameInBlock * 3];
        f[0] = (uint8_t)imuCompand(s_frame.x, IMU_HG_FULL_SCALE_G);
        f[1] = (uint8_t)imuCompand(s_frame.y, IMU_HG_FULL_SCALE_G);
        f[2] = (uint8_t)imuCompand(s_frame.z, IMU_HG_FULL_SCALE_G);
        s_frame = { 0.0f, 0.0f, 0.0f };
        s_sampleInFrame = 0;
        if (++s_frameInBlock >= IMU_EVENT_BLOCK_FRAMES)
            finishBlock();
        }
    }

void motionOnDataGap()
    {
    // Time has jumped by an unknown amount. Averages restart, the attitude
    // re-seeds from the accelerometer, and a capture that starts now gets
    // no lead-in from before the hole. A capture already running keeps
    // going, flagged.
    s_haveLg      = false;
    s_filtSeeded  = false;
    s_attValid    = false;
    s_stillSamples = 0;
    s_hgAcc[0] = s_hgAcc[1] = s_hgAcc[2] = 0.0f;
    s_hgAccN      = 0;
    s_overRun     = 0;
    s_gateOpen    = false;
    s_airSamples  = 0;
    s_landLeft    = 0;
    s_blocksValid = 0;
    if (s_evActive) s_evFlags |= MOTION_EVF_DATA_GAP;
    }

// --- 5. the 20 ms tick ----------------------------------------------------------------

static void resetMaxima()
    {
    ImuData& d = g_data.imu;
    d.maxAccelG = d.maxBrakeG = d.maxLeftG = d.maxRightG = 0.0f;
    s_maxPeak = 0.0f;
    }

// The LEVEL SET gesture, built like FUEL FILL: the dial sitting on a detent
// is the whole interface. The car has to be still for the full hold; any
// movement starts the count again.
static void levelSetGesture(bool online)
    {
    ImuData& d = g_data.imu;

    if (g_data.mode.mode != MODE_LEVEL_SET || g_data.sw.rotaryOpen)
        {
        d.levelSetActive = d.levelSetWaiting = d.levelSetDone = false;
        d.levelSetPct = 0;
        s_lsOnDetent  = false;
        s_lsCollect   = false;
        return;
        }

    const uint32_t now = millis();
    if (!s_lsOnDetent)
        {
        // Arriving on the detent: clear the g-meter maxima (this detent is
        // the "reset" for them) and start a fresh hold.
        s_lsOnDetent = true;
        s_lsStartMs  = now;
        s_lsSum      = { 0.0f, 0.0f, 0.0f };
        s_lsN        = 0;
        resetMaxima();
        }
    d.levelSetActive = true;
    if (d.levelSetDone)
        return; // one capture per visit; leave and return to do it again

    if (!online || s_stillSamples < STILL_NOW_SAMPLES)
        {
        d.levelSetWaiting = true;
        d.levelSetPct     = 0;
        s_lsCollect       = false;
        s_lsStartMs       = now;
        s_lsSum           = { 0.0f, 0.0f, 0.0f };
        s_lsN             = 0;
        return;
        }

    d.levelSetWaiting = false;
    s_lsCollect       = true;
    const uint32_t held = now - s_lsStartMs;
    d.levelSetPct = (uint8_t)min(held * 100u / IMU_LEVEL_SET_MS, 100u);

    if (held >= IMU_LEVEL_SET_MS && s_lsN >= IMU_LG_RATE_HZ)
        {
        const Vec3 up = normalized(s_lsSum);
        buildLevelRotation(up);
        const float cal[3] = { up.x, up.y, up.z };
        framImuCalSave(cal);

        d.levelCal     = true;
        d.levelSetDone = true;
        d.levelSetPct  = 100;
        s_lsCollect    = false;

        // Everything filtered so far was in the old axes: start over. That
        // includes the g-meter's maxima collected on the way here.
        resetMaxima();
        s_gFilt      = { 0.0f, 0.0f, 1.0f }; // by definition: level and at rest, right now
        s_airFilt    = s_gFilt;
        s_filtSeeded = false;
        s_attValid   = false;
        s_gyroBias   = { 0.0f, 0.0f, 0.0f };
        s_gyroLp     = { 0.0f, 0.0f, 0.0f };

        Serial.printf("IMU LEVEL SET: the gauge sat %.1f deg nose-up and %.1f deg right-side-down of level\n",
                      asinf(constrain(up.x, -1.0f, 1.0f)) * RAD2DEG,
                      atan2f(up.y, up.z) * RAD2DEG);
        }
    }

void motionUpdate()
    {
    ImuData&   d      = g_data.imu;
    const bool online = g_data.imuOnline();

    levelSetGesture(online);

    d.hgTrimmed = s_trimCount >= IMU_LG_RATE_HZ; // a second's worth of comparisons
    d.hgOffsetG[0] = s_hgOffset[0];
    d.hgOffsetG[1] = s_hgOffset[1];
    d.hgOffsetG[2] = s_hgOffset[2];

    // The capture summary is published even when the IMU has just dropped
    // out: the store may still be writing it.
    if (!online && s_evActive)
        {
        // No more samples are coming: close the capture with what it has.
        s_evActive   = false;
        s_evComplete = true;
        s_evFlags   |= MOTION_EVF_DATA_GAP;
        }
    ImuEventData& e = d.event;
    e.id        = s_evHas ? s_evId : 0;
    e.active    = s_evActive;
    e.peakG     = s_evPeak;
    e.peakLonG  = s_evAxis.x;
    e.peakLatG  = s_evAxis.y;
    e.peakVertG = s_evAxis.z;
    e.dvLonMps  = s_evDv.x;
    e.dvLatMps  = s_evDv.y;
    e.dvVertMps = s_evDv.z;
    e.overMs    = (uint16_t)min(s_evOverSamples * 1000u / IMU_HG_RATE_HZ, 65535u);
    e.airMs     = s_evAirMs;

    d.jumpCount = s_jumpCount;
    d.lastAirMs = s_lastAirMs;
    d.lastLandG = s_lastLandG;
    d.maxPeakG  = s_maxPeak;

    if (!online)
        {
        // Stale motion must not look like motion: everything live goes to
        // zero. (A total of 0 g is itself the tell - a working sensor at
        // rest reads 1.)
        d.lonG = d.latG = d.vertG = 0.0f;
        d.rollDeg = d.pitchDeg = d.yawRateDps = 0.0f;
        d.peakG = d.peakLonG = d.peakLatG = d.peakVertG = 0.0f;
        d.still = d.airborne = false;
        return;
        }

    d.lonG       = s_gFilt.x;
    d.latG       = s_gFilt.y;
    d.vertG      = s_gFilt.z;
    d.yawRateDps = s_yawFilt;
    d.rollDeg    = s_attValid ? atan2f(s_up.y, s_up.z) * RAD2DEG : 0.0f;
    d.pitchDeg   = s_attValid ? asinf(constrain(s_up.x, -1.0f, 1.0f)) * RAD2DEG : 0.0f;
    d.still      = s_stillSamples >= STILL_SAMPLES;
    d.airborne   = airMs(s_airSamples) >= IMU_AIR_MIN_MS;

    if (d.lonG  > d.maxAccelG) d.maxAccelG = d.lonG;
    if (-d.lonG > d.maxBrakeG) d.maxBrakeG = -d.lonG;
    if (d.latG  > d.maxLeftG)  d.maxLeftG  = d.latG;
    if (-d.latG > d.maxRightG) d.maxRightG = -d.latG;

    float peak = 0.0f;
    Vec3  axis = { 0.0f, 0.0f, 0.0f };
    for (uint8_t i = 0; i < IMU_PEAK_BINS; i++)
        {
        if (s_binMag[i] > peak) peak = s_binMag[i];
        holdLargest(axis.x, s_binAxis[i].x);
        holdLargest(axis.y, s_binAxis[i].y);
        holdLargest(axis.z, s_binAxis[i].z);
        }
    d.peakG     = peak;
    d.peakLonG  = axis.x;
    d.peakLatG  = axis.y;
    d.peakVertG = axis.z;
    }

void motionInit()
    {
    decodeAxis(IMU_AXIS_FORWARD, s_axisIndex[0], s_axisSign[0]);
    decodeAxis(IMU_AXIS_LEFT,    s_axisIndex[1], s_axisSign[1]);
    decodeAxis(IMU_AXIS_UP,      s_axisIndex[2], s_axisSign[2]);

    // The map must be a ROTATION: each sensor axis used once, and forward x
    // left = up. A mirror image would make the gyro and the accelerometer
    // disagree about which way the car is turning.
    Vec3 row[3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
    for (uint8_t i = 0; i < 3; i++)
        {
        float* r = &row[i].x;
        r[s_axisIndex[i]] = s_axisSign[i];
        }
    const Vec3 fxl = cross(row[0], row[1]);
    if (length(sub(fxl, row[2])) > 0.01f)
        Serial.println("IMU: WARNING - IMU_AXIS_FORWARD/LEFT/UP in config.h are not a right-handed set; "
                       "roll and pitch will be wrong");

    if (IMU_EVENT_DV_GATE_G > IMU_EVENT_TRIGGER_G)
        Serial.println("IMU: WARNING - IMU_EVENT_DV_GATE_G is above IMU_EVENT_TRIGGER_G in config.h; "
                       "delta-v will miss the start of every hit");

    float up[3];
    if (framImuCalLoad(up))
        {
        buildLevelRotation({ up[0], up[1], up[2] });
        g_data.imu.levelCal = true;
        Serial.printf("IMU: level calibration restored (mounting %.1f deg nose-up, %.1f deg right-side-down)\n",
                      asinf(constrain(up[0], -1.0f, 1.0f)) * RAD2DEG, atan2f(up[1], up[2]) * RAD2DEG);
        }
    else
        Serial.println("IMU: no level calibration - roll/pitch include the mounting angle "
                       "until a LEVEL SET is done");
    }

// --- 6. readers -----------------------------------------------------------------------

MotionLogSample motionLogTake()
    {
    MotionLogSample s;
    s.valid    = s_logN > 0;
    s.lonMeanG = s.valid ? s_logLonSum / s_logN : 0.0f;
    s.latMeanG = s.valid ? s_logLatSum / s_logN : 0.0f;
    s.peakG    = s_logPeak;
    s_logLonSum = s_logLatSum = s_logPeak = 0.0f;
    s_logN = 0;
    return s;
    }

bool motionEventView(MotionEventView& view)
    {
    if (!s_evHas)
        return false;
    const uint32_t done = s_blockAbs - s_evStartAbs; // finished blocks since its start
    view.id           = s_evId;
    view.complete     = s_evComplete;
    view.blocksReady  = (uint8_t)min(done, (uint32_t)IMU_EVENT_BLOCKS);
    view.preBlocks    = s_evPre;
    view.triggerFrame = s_evTrigFrame;
    view.peakFrame    = s_evPeakFrame;
    view.flags        = s_evFlags;
    return true;
    }

const uint8_t* motionEventBlock(uint8_t index)
    {
    if (!s_evHas || index >= IMU_EVENT_BLOCKS)
        return nullptr;
    const uint32_t abs = s_evStartAbs + index;
    if (abs >= s_blockAbs)                  return nullptr; // not finished yet
    if (s_blockAbs - abs >= RING_BLOCKS)    return nullptr; // overwritten
    return s_ring[abs % RING_BLOCKS];
    }

void motionEventSetNextId(uint32_t id)
    {
    if (id > s_nextEventId) s_nextEventId = id;
    }

void motionEventStoreBusy(bool busy)
    {
    s_storeBusy = busy;
    }
