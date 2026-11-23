// PC-side tests of the gauge's IMU logic: the real motion.cpp, fram_events.cpp
// and fram_log.cpp built against a RAM model of the FRAM and synthetic
// sensor samples.
#include <Arduino.h>
#include <vector>
#include <string>
#include "config.h"
#include "vehicle_data.h"
#include "imu.h"
#include "imu_codec.h"
#include "motion.h"
#include "fram.h"
#include "fram_log.h"
#include "fram_events.h"
#include "alarms.h"
#include "pdm.h"
#include "fuel_calc.h"
#include "fram_fake.h"

uint32_t g_fakeMillis = 1;
HardwareSerial Serial, Serial7;

// --- stubs for the parts of the firmware that are not under test -------------------
const char* alarmName(uint8_t)  { return "TESTALRM"; }
AlarmColor  alarmColor(uint8_t) { return ALARM_YELLOW; }
bool        pdmDeviceFresh(uint8_t) { return false; }
const char* pdmVerdictName(uint8_t) { return "TRP"; }
void fuelCalcGetState(double& r, double& b, bool& v) { r = 8.0; b = 100.0; v = true; }

// --- tiny test framework -----------------------------------------------------------
static int s_checks, s_fails;
#define CHECK(cond) do { s_checks++; if (!(cond)) { s_fails++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) do { s_checks++; const double _a = (a), _b = (b); if (!(fabs(_a - _b) <= (tol))) { s_fails++; \
    printf("  FAIL %s:%d  %s = %.4f, expected %.4f +- %.4f\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); } } while (0)
static void section(const char* name) { printf("--- %s\n", name); }

// --- the simulated sensor ----------------------------------------------------------
// Vehicle-frame truth in, sensor-frame counts out, honouring the axis map in
// config.h (forward = -X, left = -Y, up = +Z), a mounting tilt of the gauge,
// and a high-g zero offset the firmware has to trim out.
struct Sim
    {
    float hgOffset[3] = { 1.20f, -0.80f, 0.55f }; // sensor axes, g - inside the +-1.5 g typical
    float mountPitchDeg = 0.0f;                   // gauge nose-up relative to the car
    float mountRollDeg  = 0.0f;                   // gauge right-side-down relative to the car
    uint64_t sample = 0;                          // high-g samples so far
    uint32_t nextTickMs = 20;
    bool   runStore = true;                       // call framEventsTick + pump the FRAM
    uint32_t noise = 12345;

    float rnd() { noise = noise * 1664525u + 1013904223u; return ((noise >> 8) & 0xFFFF) / 32768.0f - 1.0f; }

    // vehicle axes -> the gauge's nominal axes (the gauge is tilted in the car)
    void toNominal(const float v[3], float n[3]) const
        {
        const float b = mountPitchDeg * 0.01745329252f, r = mountRollDeg * 0.01745329252f;
        // pitch about Y first: gauge forward = (cos b, 0, sin b), up = (-sin b, 0, cos b)
        const float f1 = v[0] * cosf(b) + v[2] * sinf(b);
        const float l1 = v[1];
        const float u1 = -v[0] * sinf(b) + v[2] * cosf(b);
        // then roll about the gauge's forward axis: left = (0, cos r, sin r), up = (0, -sin r, cos r)
        n[0] = f1;
        n[1] = l1 * cosf(r) + u1 * sinf(r);
        n[2] = -l1 * sinf(r) + u1 * cosf(r);
        }
    void toSensor(const float n[3], float s[3]) const { s[0] = -n[0]; s[1] = -n[1]; s[2] = n[2]; }

    static int16_t counts(float v, float perCount)
        {
        const float c = roundf(v / perCount);
        return (int16_t)(c > 32767 ? 32767 : c < -32768 ? -32768 : c);
        }

    // one high-g sample period; a low-g + gyro pair every 8th
    void step(const float accelVeh[3], const float gyroVeh[3], bool withNoise = false)
        {
        float n[3], s[3];
        toNominal(accelVeh, n);
        toSensor(n, s);
        int16_t hg[3];
        for (int i = 0; i < 3; i++)
            hg[i] = counts(s[i] + hgOffset[i] + (withNoise ? 0.03f * rnd() : 0.0f), IMU_HG_G_PER_COUNT);
        motionFeedHighG(hg);
        sample++;

        if (sample % (IMU_HG_RATE_HZ / IMU_LG_RATE_HZ) == 0)
            {
            int16_t lg[3], gy[3];
            float gn[3], gs[3];
            for (int i = 0; i < 3; i++)
                {
                const float clipped = s[i] > 16.0f ? 16.0f : s[i] < -16.0f ? -16.0f : s[i];
                lg[i] = counts(clipped + (withNoise ? 0.002f * rnd() : 0.0f), IMU_LG_G_PER_COUNT);
                }
            toNominal(gyroVeh, gn);
            toSensor(gn, gs);
            for (int i = 0; i < 3; i++) gy[i] = counts(gs[i], IMU_GYRO_DPS_PER_COUNT);
            motionFeedLowG(lg);
            motionFeedGyro(gy);
            }

        g_fakeMillis = (uint32_t)(1 + sample * 1000 / IMU_HG_RATE_HZ);
        g_data.imu.present    = true;
        g_data.imu.lastDataMs = g_fakeMillis;
        while (g_fakeMillis >= nextTickMs)
            {
            nextTickMs += 20;
            motionUpdate();
            if (runStore)
                {
                framEventsTick();
                for (int i = 0; i < 4; i++) framPump(); // the superloop pumps many times a tick
                }
            }
        }

    void run(float seconds, const float accelVeh[3], const float gyroVeh[3], bool withNoise = false)
        {
        const uint64_t n = (uint64_t)(seconds * IMU_HG_RATE_HZ + 0.5f);
        for (uint64_t i = 0; i < n; i++) step(accelVeh, gyroVeh, withNoise);
        }
    };

static const float REST[3] = { 0, 0, 1 };
static const float ZERO[3] = { 0, 0, 0 };

// the level calibration as fram_log.cpp stores it
static void newChip()
    {
    memset(g_framMem, 0xA5, FRAM_SIZE_BYTES); // a chip that has never been formatted
    g_data.fram = {};
    framInit();
    Serial.quiet = true;
    framLogInit();
    Serial.quiet = false;
    }

// copy of the on-chip capture header, for inspecting the fake chip
struct __attribute__((packed)) Hdr
    {
    uint16_t magic; uint8_t state, rev; uint32_t id, logSeq; uint8_t session, hh, mm, ss;
    uint8_t blocksWritten, blocksTotal, preBlocks, triggerFrame; uint16_t hgRateHz; uint8_t decim, blockFrames;
    uint8_t flags, tpsPct; uint16_t rpm, fullScaleG, trigG100, peakG100; int16_t peakLon100, peakLat100, peakVert100;
    uint16_t peakFrame, overMs; int16_t dvLon100, dvLat100, dvVert100; uint16_t airMs; uint8_t pad[10]; uint16_t crc;
    };
static bool slotHeader(const uint8_t* mem, int slot, Hdr& out)
    {
    bool found = false;
    for (int copy = 0; copy < 2; copy++)
        {
        Hdr h;
        memcpy(&h, mem + FRAM_ADDR_EVENTS + slot * FRAM_EVENT_SLOT_BYTES + copy * 64, 64);
        if (h.magic != 0xE30E || h.id == 0 || framCrc16(&h, 62) != h.crc) continue;
        if (!found || h.id > out.id || (h.id == out.id && h.rev > out.rev)) { out = h; found = true; }
        }
    return found;
    }
static const uint8_t* slotBlock(const uint8_t* mem, int slot, int block)
    {
    return mem + FRAM_ADDR_EVENTS + slot * FRAM_EVENT_SLOT_BYTES + 128 + block * MOTION_BLOCK_BYTES;
    }

int main()
    {
    static_assert(sizeof(Hdr) == 64, "test copy of the header");

    // ------------------------------------------------------------------------------
    section("companding");
        {
        float worst = 0;
        int8_t last = -128;
        for (float g = -330; g <= 330; g += 0.01f)
            {
            const int8_t c = imuCompand(g, 320);
            CHECK(c >= last); last = c;                     // monotonic
            const float back = imuExpand(c, 320);
            if (fabsf(g) >= 6 && fabsf(g) <= 320) worst = fmaxf(worst, fabsf(back - g) / fabsf(g));
            }
        printf("  worst relative error 6..320 g: %.1f %%\n", worst * 100);
        CHECK(worst < 0.06f);
        CHECK(imuCompand(1000, 320) == 127 && imuCompand(-1000, 320) == -127);
        CHECK(imuCompand(0, 320) == 0);
        NEAR(imuExpandU(imuCompandU(1.0f, 320), 320), 1.0, 0.08);
        NEAR(imuExpandU(imuCompandU(10.0f, 320), 320), 10.0, 0.25);
        NEAR(imuExpandU(imuCompandU(100.0f, 320), 320), 100.0, 0.8);
        }

    // ------------------------------------------------------------------------------
    section("boot on a blank chip; at rest, level");
    newChip();
    CHECK(g_data.fram.mounted);
    CHECK(g_data.fram.logCapacity == 2210);
    motionInit();
    CHECK(!g_data.imu.levelCal);
        {
        Sim sim;
        sim.run(3.0f, REST, ZERO, true);
        const ImuData& u = g_data.imu;
        NEAR(u.lonG, 0, 0.01); NEAR(u.latG, 0, 0.01); NEAR(u.vertG, 1, 0.01);
        NEAR(u.rollDeg, 0, 0.3); NEAR(u.pitchDeg, 0, 0.3);
        NEAR(u.peakG, 1.0, 0.12);                 // high-g channel reads 1 g once trimmed
        NEAR(u.hgOffsetG[0], 1.20, 0.03); NEAR(u.hgOffsetG[1], -0.80, 0.03); NEAR(u.hgOffsetG[2], 0.55, 0.03);
        CHECK(u.hgTrimmed);
        CHECK(u.still);
        CHECK(u.event.id == 0);                   // noise alone must not trigger
        CHECK(u.jumpCount == 0);
        }

    // ------------------------------------------------------------------------------
    section("static attitude: signs");
        {
        Sim sim;
        const float d = 0.01745329252f;
        const float noseUp10[3]   = { sinf(10 * d), 0, cosf(10 * d) };   // gravity seen with the nose up
        sim.run(15.0f, noseUp10, ZERO);
        NEAR(g_data.imu.pitchDeg, 10, 0.3); NEAR(g_data.imu.rollDeg, 0, 0.3);
        NEAR(g_data.imu.lonG, sinf(10 * d), 0.01);                         // a slope reads as "accelerating"
        const float rightDown15[3] = { 0, sinf(15 * d), cosf(15 * d) };
        sim.run(15.0f, rightDown15, ZERO);
        NEAR(g_data.imu.rollDeg, 15, 0.3); NEAR(g_data.imu.pitchDeg, 0, 0.3);
        sim.run(15.0f, REST, ZERO);
        }

    // ------------------------------------------------------------------------------
    section("gyro carries the attitude; accel and gyro agree on direction");
        {
        // Truth from rotation matrices, not from the filter's own equation:
        // roll phi about +X (forward) puts world-up at (0, sin phi, cos phi)
        // in the body; pitching the nose UP by theta is a NEGATIVE turn about
        // +Y (left) and puts it at (sin theta, 0, cos theta).
        Sim sim;
        const float d = 0.01745329252f;
        sim.run(2.0f, REST, ZERO);
        float maxErr = 0;
        for (int i = 0; i < 960; i++)              // 1 s rolling right at 30 deg/s
            {
            const float phi = 30.0f * (i + 1) / 960.0f;
            const float a[3] = { 0, sinf(phi * d), cosf(phi * d) };
            const float w[3] = { 30.0f, 0, 0 };
            sim.step(a, w);
            if (i % 96 == 95) maxErr = fmaxf(maxErr, fabsf(g_data.imu.rollDeg - phi));
            }
        printf("  roll tracking error during a 30 deg/s roll: %.2f deg\n", maxErr);
        CHECK(maxErr < 1.2f);                       // one tick of lag at 30 deg/s is 0.6 deg
        NEAR(g_data.imu.rollDeg, 30, 0.8);

        // Now the accelerometer LIES for 2 s (1 g of cornering): the gate must
        // keep it out and the gyro must hold the 30 degrees.
        const float cornering[3] = { 0, sinf(30 * d) + 1.0f, cosf(30 * d) };
        sim.run(2.0f, cornering, ZERO);
        NEAR(g_data.imu.rollDeg, 30, 0.8);

        // pitch the nose up at 20 deg/s for 1 s from level
        const float back[3] = { 0, 0, 1 };
        sim.run(20.0f, back, ZERO);
        NEAR(g_data.imu.rollDeg, 0, 0.3);
        maxErr = 0;
        for (int i = 0; i < 960; i++)
            {
            const float th = 20.0f * (i + 1) / 960.0f;
            const float a[3] = { sinf(th * d), 0, cosf(th * d) };
            const float w[3] = { 0, -20.0f, 0 };   // nose up = negative about +Y
            sim.step(a, w);
            if (i % 96 == 95) maxErr = fmaxf(maxErr, fabsf(g_data.imu.pitchDeg - th));
            }
        printf("  pitch tracking error during a 20 deg/s pitch-up: %.2f deg\n", maxErr);
        CHECK(maxErr < 1.0f);
        NEAR(g_data.imu.pitchDeg, 20, 0.6);
        NEAR(g_data.imu.rollDeg, 0, 0.5);

        // yaw rate sign: turning left is positive about +Z
        const float tilt20[3] = { sinf(20 * d), 0, cosf(20 * d) };
        const float yawLeft[3] = { 0, 0, 25.0f };
        (void)tilt20;
        sim.run(20.0f, REST, ZERO);
        sim.run(1.0f, REST, yawLeft);
        NEAR(g_data.imu.yawRateDps, 25, 1.0);
        NEAR(g_data.imu.rollDeg, 0, 0.5); NEAR(g_data.imu.pitchDeg, 0, 0.5);
        sim.run(3.0f, REST, ZERO);
        }

    // ------------------------------------------------------------------------------
    section("LEVEL SET takes out the mounting angle and survives a reboot");
        {
        Sim sim;
        sim.mountPitchDeg = 25.0f;   // the gauge clamped tipped back on the bar
        sim.mountRollDeg  = 3.0f;
        sim.run(20.0f, REST, ZERO);
        printf("  before: roll %.1f pitch %.1f (the mounting angle shows)\n", g_data.imu.rollDeg, g_data.imu.pitchDeg);
        CHECK(fabsf(g_data.imu.pitchDeg) > 20);

        g_data.imu.maxAccelG = 0.7f;
        g_data.mode.mode = MODE_LEVEL_SET;
        // moving: the hold must not progress
        const float wobble[3] = { 0, 0, 12.0f };
        for (int i = 0; i < 960 * 2; i++)
            {
            const float w[3] = { 9.0f * sinf(i * 0.05f), 0, 0 };
            sim.step(REST, w);
            }
        (void)wobble;
        CHECK(g_data.imu.levelSetActive && g_data.imu.levelSetWaiting && !g_data.imu.levelSetDone);
        CHECK(g_data.imu.maxAccelG < 0.5f);         // arriving on the detent cleared the 0.7 (the mount's own 0.42 is back)
        sim.run(2.0f, REST, ZERO);
        CHECK(!g_data.imu.levelSetWaiting && g_data.imu.levelSetPct > 10 && g_data.imu.levelSetPct < 60);
        CHECK(!g_data.imu.levelSetDone);
        sim.run(5.0f, REST, ZERO);
        CHECK(g_data.imu.levelSetDone && g_data.imu.levelCal);
        CHECK(g_data.imu.maxAccelG < 0.02f);        // ...and the level set starts the maxima from clean
        g_data.mode.mode = MODE_BRIGHT;
        sim.run(4.0f, REST, ZERO);
        NEAR(g_data.imu.rollDeg, 0, 0.3); NEAR(g_data.imu.pitchDeg, 0, 0.3);
        NEAR(g_data.imu.lonG, 0, 0.01); NEAR(g_data.imu.vertG, 1, 0.01);
        CHECK(!g_data.imu.levelSetActive);

        // braking at 0.8 g must now read as pure lon, not leak into vert
        const float braking[3] = { -0.8f, 0, 1 };
        sim.run(1.5f, braking, ZERO);
        NEAR(g_data.imu.lonG, -0.8, 0.02); NEAR(g_data.imu.latG, 0, 0.05); NEAR(g_data.imu.vertG, 1, 0.02);
        CHECK(g_data.imu.maxBrakeG > 0.75f);
        // and a left-hander at 0.6 g as +lat
        const float leftTurn[3] = { 0, 0.6f, 1 };
        sim.run(1.5f, leftTurn, ZERO);
        NEAR(g_data.imu.latG, 0.6, 0.05);
        sim.run(1.0f, REST, ZERO);

        // "reboot": the calibration comes back from the FRAM
        for (int i = 0; i < 16; i++) framPump();
        float up[3];
        CHECK(framImuCalLoad(up));
        NEAR(up[0], sinf(25 * 0.01745329252f), 0.01);
        g_data.imu.levelCal = false;
        Serial.quiet = true; motionInit(); Serial.quiet = false;
        CHECK(g_data.imu.levelCal);
        }

    // From here on: a level-mounted gauge again (a fresh chip, no calibration).
    newChip();
    Serial.quiet = true; motionInit(); Serial.quiet = false;
    // motionInit() found no calibration but the module still holds the old
    // rotation in RAM (a real boot starts from zeroed statics): redo a level
    // set with a level gauge to put the identity back.
        {
        Sim sim;
        g_data.mode.mode = MODE_LEVEL_SET;
        sim.run(7.0f, REST, ZERO);
        CHECK(g_data.imu.levelSetDone);
        g_data.mode.mode = MODE_BRIGHT;
        sim.run(2.0f, REST, ZERO);
        NEAR(g_data.imu.pitchDeg, 0, 0.3);
        }

    // ------------------------------------------------------------------------------
    section("peak hold and the trigger");
    uint32_t firstId = 0;
        {
        Sim sim;
        sim.run(3.0f, REST, ZERO);
        CHECK(g_data.imu.event.id == 0);

        // ONE sample at 30 g: visible in the peak hold, but not an impact
        const float spike[3] = { 0, 0, 30.0f };
        sim.step(spike, ZERO);
        sim.run(0.05f, REST, ZERO);
        NEAR(g_data.imu.peakG, 30, 0.2);
        NEAR(g_data.imu.peakVertG, 30, 0.2);
        CHECK(g_data.imu.event.id == 0);
        sim.run(0.10f, REST, ZERO);
        NEAR(g_data.imu.peakG, 30, 0.2);            // still held 150 ms later
        sim.run(0.15f, REST, ZERO);
        NEAR(g_data.imu.peakG, 1.0, 0.1);           // and released after the hold
        NEAR(g_data.imu.maxPeakG, 30, 0.2);

        // a real hit: hardest sample FIRST (41 g), then 3 more above the trigger
        const float hit1[3] = { -22.0f, 6.0f, 34.0f };   // |.| = 40.94
        const float hit2[3] = { -10.0f, 2.0f, 15.0f };
        sim.step(hit1, ZERO);
        CHECK(g_data.imu.event.id == 0);            // one sample is not enough yet
        sim.step(hit2, ZERO); sim.step(hit2, ZERO); sim.step(hit2, ZERO);
        sim.run(0.1f, REST, ZERO);
        const ImuEventData& e = g_data.imu.event;
        CHECK(e.id == 1 && e.active);
        firstId = e.id;
        NEAR(e.peakG, 40.94, 0.15);                 // the first sample's peak was not lost
        NEAR(e.peakLonG, -22, 0.1); NEAR(e.peakLatG, 6, 0.1); NEAR(e.peakVertG, 34, 0.1);
        CHECK(e.overMs == 4);                       // 4 samples at 960 Hz = 4.17 ms
        MotionEventView v;
        CHECK(motionEventView(v));
        CHECK(v.preBlocks == IMU_EVENT_PRE_BLOCKS && !v.complete);
        CHECK(v.flags & MOTION_EVF_LEVEL_CAL); CHECK(v.flags & MOTION_EVF_HG_TRIM);
        CHECK(!(v.flags & MOTION_EVF_DATA_GAP));

        // 1 s in: the capture is still open but most of what exists is on the chip
        sim.run(0.9f, REST, ZERO);
        Hdr h;
        CHECK(slotHeader(g_framMem, 0, h));
        CHECK(h.id == 1 && h.state == 1);
        printf("  1 s after the trigger: header rev %u says %u blocks on the chip\n", h.rev, h.blocksWritten);
        CHECK(h.blocksWritten >= 30);               // 20 lead-in + ~20 live, less up to 10 since the last header
        NEAR(h.peakG100 / 100.0, 40.94, 0.15);

        // a second hit inside the same window belongs to the same capture
        const float hitB[3] = { 0, -50.0f, 5.0f };
        sim.step(hitB, ZERO); sim.step(hitB, ZERO); sim.step(hitB, ZERO);
        sim.run(1.0f, REST, ZERO);
        CHECK(g_data.imu.event.id == 1);
        NEAR(g_data.imu.event.peakLatG, -50, 0.1);

        sim.run(3.0f, REST, ZERO);                  // past the end of the 5 s window
        CHECK(!g_data.imu.event.active);
        CHECK(motionEventView(v) && v.complete && v.blocksReady == IMU_EVENT_BLOCKS);
        CHECK(slotHeader(g_framMem, 0, h));
        CHECK(h.state == 2 && h.blocksWritten == IMU_EVENT_BLOCKS && h.blocksTotal == IMU_EVENT_BLOCKS);
        CHECK(h.preBlocks == IMU_EVENT_PRE_BLOCKS && h.hgRateHz == 960 && h.decim == 4 && h.blockFrames == 12);
        CHECK(h.fullScaleG == 320 && h.trigG100 == 600);
        NEAR(h.peakG100 / 100.0, sqrt(50.0 * 50 + 25), 0.2);

        // the waveform on the chip: the trigger frame holds the first hit
        const uint8_t* tb = slotBlock(g_framMem, 0, h.preBlocks);
        int f = h.triggerFrame;
        // the 41 g sample was the run's first; the trigger fired one sample
        // later, in the same frame or the next. Search both.
        float bestVert = 0, bestLon = 0;
        for (int k = (f > 0 ? f - 1 : 0); k <= f && k < 12; k++)
            if (fabsf(imuExpand((int8_t)tb[k * 3 + 2], 320)) > fabsf(bestVert))
                { bestVert = imuExpand((int8_t)tb[k * 3 + 2], 320); bestLon = imuExpand((int8_t)tb[k * 3 + 0], 320); }
        printf("  trigger frame on the chip: lon %.1f vert %.1f (true -22 / 34)\n", bestLon, bestVert);
        NEAR(bestVert, 34, 1.0); NEAR(bestLon, -22, 0.8);
        // a quiet frame before it reads 1 g up
        const uint8_t* pb = slotBlock(g_framMem, 0, 3);
        NEAR(imuExpand((int8_t)pb[2], 320), 1.0, 0.25);
        NEAR(imuExpand((int8_t)pb[0], 320), 0.0, 0.1);
        // attitude bytes: level
        CHECK((int8_t)pb[36] == 0 && (int8_t)pb[37] == 0);

        CHECK(g_framMinFree >= 4);                  // the store never crowded the fuel/log records out
        printf("  FRAM writes for one capture: %lu, lowest queue headroom %d of 8\n",
               (unsigned long)g_framWrites, g_framMinFree);
        }

    // ------------------------------------------------------------------------------
    section("a jump: air time, landing, delta-v");
        {
        Sim sim;
        sim.run(2.0f, REST, ZERO);
        const uint16_t jumpsBefore = g_data.imu.jumpCount;
        const float flight[3] = { 0.02f, 0, 0.06f };
        sim.run(0.40f, flight, ZERO);
        CHECK(g_data.imu.airborne);
        // landing: half-sine, 60 ms, 9 g peak, vertical
        for (int i = 0; i < 58; i++)
            {
            const float a[3] = { 0, 0, 1.0f + 8.0f * sinf(3.14159265f * i / 58.0f) };
            sim.step(a, ZERO);
            }
        sim.run(1.0f, REST, ZERO);
        const ImuData& u = g_data.imu;
        CHECK(u.jumpCount == jumpsBefore + 1);
        NEAR(u.lastAirMs, 400, 40);
        NEAR(u.lastLandG, 9.0, 0.15);
        CHECK(!u.airborne);
        CHECK(u.event.id == firstId + 1);
        NEAR(u.event.airMs, 400, 40);               // the capture knows it was a landing
        NEAR(u.event.peakG, 9.0, 0.15);
        // delta-v: the integral of (a - 1 g) while above 2 g. The whole pulse
        // is (2/pi) * 8 g * 60.4 ms = 3.02 m/s; the 2 g gate trims its feet
        // to 2.99.
        printf("  landing delta-v %.2f m/s vertical (whole pulse 3.02, gated 2.99)\n", u.event.dvVertMps);
        NEAR(u.event.dvVertMps, 2.99, 0.06);
        NEAR(u.event.dvLonMps, 0, 0.05);
        sim.run(5.0f, REST, ZERO);
        }

    // ------------------------------------------------------------------------------
    section("a frontal crash pulse: delta-v, clipping of the low-g channel");
        {
        Sim sim;
        sim.run(2.0f, REST, ZERO);
        // half-sine, 100 ms, 30 g peak deceleration
        for (int i = 0; i < 96; i++)
            {
            const float a[3] = { -30.0f * sinf(3.14159265f * i / 96.0f), 0, 1 };
            sim.step(a, ZERO);
            }
        sim.run(6.0f, REST, ZERO);
        const ImuEventData& e = g_data.imu.event;
        const float expectDv = -(2.0f / 3.14159265f) * 30.0f * 9.80665f * 0.100f; // -18.73 m/s
        printf("  crash delta-v %.2f m/s lon (whole pulse %.2f), peak %.1f g, %u ms over 6 g\n",
               e.dvLonMps, expectDv, e.peakG, e.overMs);
        NEAR(e.dvLonMps, expectDv, 0.10);
        NEAR(e.dvVertMps, 0, 0.05);                 // gravity was taken out, not integrated
        NEAR(e.peakLonG, -30, 0.2);
        NEAR(e.overMs, 87, 4);                      // a 30 g half-sine is above 6 g for 87 % of its 100 ms
        // the high-g trim must not have been dragged by the clipped low-g channel
        NEAR(g_data.imu.hgOffsetG[0], 1.20, 0.05);
        }

    // ------------------------------------------------------------------------------
    section("power lost one second into a capture");
        {
        Sim sim;
        sim.run(2.0f, REST, ZERO);
        const uint32_t idBefore = g_data.imu.event.id;
        const float hit[3] = { 12.0f, 0, 20.0f };
        sim.step(hit, ZERO); sim.step(hit, ZERO); sim.step(hit, ZERO);
        sim.run(1.0f, REST, ZERO);
        CHECK(g_data.imu.event.id == idBefore + 1 && g_data.imu.event.active);

        // The kill switch: whatever is still in the write queue never lands...
        fakeFramDropQueue();
        static uint8_t snapshot[FRAM_SIZE_BYTES];
        memcpy(snapshot, g_framMem, FRAM_SIZE_BYTES);
        // ...and the header copy that was written last is torn in half.
        int slot = -1; Hdr h;
        for (int s = 0; s < FRAM_EVENT_SLOTS; s++)
            if (slotHeader(snapshot, s, h) && h.id == idBefore + 1) { slot = s; break; }
        CHECK(slot >= 0);
        const uint8_t revBefore = h.rev, blocksBefore = h.blocksWritten;
        memset(snapshot + FRAM_ADDR_EVENTS + slot * FRAM_EVENT_SLOT_BYTES + (h.rev & 1) * 64 + 20, 0xFF, 44);
        CHECK(slotHeader(snapshot, slot, h));        // the OTHER copy still stands
        printf("  newest header rev %u (%u blocks) torn -> rev %u (%u blocks) recovered, state %s\n",
               revBefore, blocksBefore, h.rev, h.blocksWritten, h.state == 1 ? "RECORDING" : "?");
        CHECK(h.id == idBefore + 1 && h.state == 1);
        CHECK(h.rev == revBefore - 1);
        CHECK(h.blocksWritten >= 20);                // at least the whole lead-in + the hit
        NEAR(h.peakG100 / 100.0, sqrt(144.0 + 400), 0.15);
        // and the hit itself is in the stored blocks
        const uint8_t* tb = slotBlock(snapshot, slot, h.preBlocks);
        float mx = 0;
        for (int k = 0; k < 12; k++) mx = fmaxf(mx, imuExpand((int8_t)tb[k * 3 + 2], 320));
        NEAR(mx, 20, 0.8);

        // finish this capture normally for the tests below
        sim.run(5.0f, REST, ZERO);
        }

    // ------------------------------------------------------------------------------
    section("slot policy: the biggest hits are kept, the oldest of the rest is recycled");
        {
        Sim sim;
        sim.run(1.0f, REST, ZERO);
        // captures so far: #1 (50 g), #2 (9 g landing), #3 (30 g crash), #4 (23 g)
        // add hits of 8..14 g until every slot has been recycled at least once
        for (int n = 0; n < 12; n++)
            {
            const float g = 8.0f + (n % 7);
            const float hit[3] = { 0, 0, g };
            sim.step(hit, ZERO); sim.step(hit, ZERO); sim.step(hit, ZERO);
            sim.run(5.6f, REST, ZERO);
            }
        CHECK(g_data.imu.event.id == 16);
        bool have50 = false, have30 = false;
        uint32_t ids[FRAM_EVENT_SLOTS]; int n = 0;
        for (int s = 0; s < FRAM_EVENT_SLOTS; s++)
            {
            Hdr h;
            CHECK(slotHeader(g_framMem, s, h));
            CHECK(h.state == 2);
            ids[n++] = h.id;
            if (h.id == 1) have50 = true;
            if (h.id == 3) have30 = true;
            }
        printf("  ids on the chip after 16 captures:");
        for (int i = 0; i < n; i++) printf(" %lu", (unsigned long)ids[i]);
        printf("\n");
        CHECK(have50 && have30);                     // the two hardest ever (50 g and 30 g) survive
        int recent = 0;
        for (int i = 0; i < n; i++) if (ids[i] >= 11) recent++;
        CHECK(recent == 6);                          // the other six slots hold the six newest (11..16)
        }

    // ------------------------------------------------------------------------------
    section("reboot: ids continue, the list and the dump read back");
        {
        Serial.quiet = true;
        framLogInit();                               // mounts, including the capture slots
        Serial.quiet = false;
        Sim sim;
        sim.run(2.0f, REST, ZERO);
        const float hit[3] = { 0, 7.5f, 1.0f };
        sim.step(hit, ZERO); sim.step(hit, ZERO); sim.step(hit, ZERO);
        sim.run(5.6f, REST, ZERO);
        CHECK(g_data.imu.event.id == 17);

        printf("  ---- console 'e' ----\n");
        framEventsConsoleKey('e');
        printf("  ---- console 'E' (first lines) ----\n");
        framEventsConsoleKey('E');
        CHECK(framEventsDumping());
        int passes = 0;
        // let it print the note lines + the first block, then go quiet
        framEventsDumpPump();
        Serial.quiet = true;
        while (framEventsDumping() && passes < 5000) { framEventsDumpPump(); passes++; }
        Serial.quiet = false;
        CHECK(!framEventsDumping());
        CHECK(passes >= 800 && passes <= 820);       // 8 captures x 100 blocks, one block per pass
        }

    // ------------------------------------------------------------------------------
    section("a hole in the data");
        {
        Sim sim;
        sim.run(2.0f, REST, ZERO);
        motionOnDataGap();
        sim.run(0.2f, REST, ZERO);                   // only 4 blocks of lead-in exist now
        const float hit[3] = { 0, 0, 12.0f };
        sim.step(hit, ZERO); sim.step(hit, ZERO); sim.step(hit, ZERO);
        sim.run(0.5f, REST, ZERO);
        MotionEventView v;
        CHECK(motionEventView(v));
        printf("  lead-in after a data gap: %u blocks (20 normally)\n", v.preBlocks);
        CHECK(v.preBlocks >= 3 && v.preBlocks <= 5);
        motionOnDataGap();                           // and another hole DURING the capture
        sim.run(6.0f, REST, ZERO);
        CHECK(motionEventView(v) && (v.flags & MOTION_EVF_DATA_GAP));
        NEAR(g_data.imu.rollDeg, 0, 0.5);            // the attitude re-seeded itself
        }

    // ------------------------------------------------------------------------------
    section("IMU drops out mid-capture; no FRAM at all");
        {
        Sim sim;
        sim.run(2.0f, REST, ZERO);
        const float hit[3] = { 0, 0, 12.0f };
        sim.step(hit, ZERO); sim.step(hit, ZERO); sim.step(hit, ZERO);
        sim.run(0.5f, REST, ZERO);
        CHECK(g_data.imu.event.active);
        const uint32_t id = g_data.imu.event.id;
        // the chip goes silent: only the tick keeps running
        g_data.imu.present = false;
        for (int t = 0; t < 30; t++) { g_fakeMillis += 20; motionUpdate(); framEventsTick(); for (int i = 0; i < 4; i++) framPump(); }
        CHECK(!g_data.imu.event.active);
        CHECK(g_data.imu.peakG == 0.0f && g_data.imu.vertG == 0.0f);   // stale motion reads as nothing
        Hdr h; bool found = false;
        for (int s = 0; s < FRAM_EVENT_SLOTS; s++) if (slotHeader(g_framMem, s, h) && h.id == id) { found = true; break; }
        CHECK(found && h.state == 2 && h.blocksWritten < IMU_EVENT_BLOCKS && h.blocksWritten >= 25);
        CHECK(h.flags & MOTION_EVF_DATA_GAP);
        sim.nextTickMs = g_fakeMillis + 20;
        sim.sample = (uint64_t)g_fakeMillis * IMU_HG_RATE_HZ / 1000;
        motionOnDataGap();
        sim.run(3.0f, REST, ZERO);

        // FRAM dies: captures still happen (CAN, screens), nothing blocks
        g_data.fram.present = false;
        sim.step(hit, ZERO); sim.step(hit, ZERO); sim.step(hit, ZERO);
        sim.run(6.0f, REST, ZERO);
        CHECK(g_data.imu.event.id == id + 1 && !g_data.imu.event.active);
        sim.step(hit, ZERO); sim.step(hit, ZERO); sim.step(hit, ZERO);
        sim.run(6.0f, REST, ZERO);
        CHECK(g_data.imu.event.id == id + 2);        // the store being absent does not hold up the next one
        g_data.fram.present = true;
        }

    // ------------------------------------------------------------------------------
    section("black box: the four IMU bytes, and the CSV dump");
        {
        newChip();
        Sim sim;
        sim.runStore = false;
        g_data.ecu.rpm = 3000;
        g_data.ecu.frameLastRxMs[ECU_F_SPEEDS] = g_fakeMillis;
        g_data.board.clockValid = true; g_data.board.hh = 14; g_data.board.mm = 3; g_data.board.ss = 9;
        // 2 s of a 0.8 g left-hander with a 12.5 g single-sample kerb strike in it
        const float corner[3] = { 0.1f, 0.8f, 1.0f };
        motionLogTake();
        sim.run(1.0f, corner, ZERO);
        const float kerb[3] = { 0, 0, 12.5f };
        sim.step(kerb, ZERO);
        sim.run(1.0f, corner, ZERO);
        g_data.ecu.frameLastRxMs[ECU_F_SPEEDS] = g_fakeMillis;
        framLogTick1s();                             // first worthy tick logs at once
        for (int i = 0; i < 8; i++) framPump();
        CHECK(g_data.fram.logRecords == 1);
        // decode the record straight off the fake chip
        const uint8_t* r = g_framMem + FRAM_ADDR_LOG;
        CHECK(framCrc16(r, 42) == (uint16_t)(r[42] | (r[43] << 8)));
        const int8_t gLon = (int8_t)r[38], gLat = (int8_t)r[39];
        printf("  record: gLon %.2f gLat %.2f gPeak %.1f impact %u\n", gLon / 50.0, gLat / 50.0,
               imuExpandU(r[40], 320), r[41]);
        NEAR(gLon / 50.0, 0.1, 0.02); NEAR(gLat / 50.0, 0.8, 0.02);
        NEAR(imuExpandU(r[40], 320), 12.5, 0.3);
        // IMU gone: the record says so rather than logging zeros as data
        g_data.imu.present = false;
        g_fakeMillis += 2000;
        g_data.ecu.frameLastRxMs[ECU_F_SPEEDS] = g_fakeMillis;
        framLogTick1s(); framLogTick1s();
        for (int i = 0; i < 8; i++) framPump();
        CHECK(g_data.fram.logRecords == 2);
        const uint8_t* r2 = g_framMem + FRAM_ADDR_LOG + 44;
        CHECK((int8_t)r2[38] == -128 && (int8_t)r2[39] == -128 && r2[40] == 0);
        // the ring ends where the capture slots begin
        CHECK(FRAM_ADDR_LOG + 2210u * 44u <= FRAM_ADDR_EVENTS);
        CHECK(FRAM_ADDR_LOG + 2211u * 44u > FRAM_ADDR_EVENTS);
        }

    printf("\n%d checks, %d failed\n", s_checks, s_fails);
    return s_fails ? 1 : 0;
    }
