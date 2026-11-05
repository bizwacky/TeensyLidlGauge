// ---------------------------------------------------------------------------
// Rally CAN Gauge - mixture window implementation
// ---------------------------------------------------------------------------
// WHY A GATE: a wideband only tells the truth while the engine burns what
// the ECU meters. On overrun it doesn't - a fuel cut (DFCO, the rev limiter)
// shows the sensor free air, and closed-throttle misfires push unburnt
// oxygen past it. The old average (an exponential one, fed every tick the
// engine ran) swallowed every lift-off: a 2.5 s overrun latched a yellow
// AFR LEAN, and when the throttle reopened - MAP > 90 kPa, mixture dead on
// target - the average still held the overrun and the alarm went RED for
// about six seconds. A red that cries wolf at every hairpin teaches the
// driver to ignore red.
//
// THE RULES. A tick's sample counts only when ALL of these hold:
//   1. the engine is running, and the lambda, target, throttle, status and
//      fuel frames are all fresh;
//   2. the sensor reports a real reading and a target exists (rusEFI sends
//      lambda 0 while the wideband warms up or has failed);
//   3. the driver is asking for power: pedal (TPS on a cable throttle) at
//      or above LAMBDA_MIN_THROTTLE_PCT. This is what excludes DFCO: rusEFI
//      (dfco.cpp) only cuts fuel while its "driver throttle intent" - the
//      pedal, else TPS, the same pick EcuData::throttlePct() makes - is
//      below the tune's coastingFuelCutTps;
//   4. rusEFI is not cutting fuel or spark (limiter / limp: the rev-limit
//      status bit), and its last injection pulse is nonzero (rusEFI
//      computes 0 ms for zero fuel mass, i.e. a cut - a belt-and-braces
//      check behind rule 3 and the status bit);
//   5. at least LAMBDA_HOLDOFF_MS has passed since the last tick that
//      failed any of the above - exhaust transport delay, sensor lag, and
//      the intake fuel film refilling after a cut.
//
// THE WINDOW. Any failing tick EMPTIES the window. Valid ticks append one
// sample each, and the published average is the plain mean of the last
// LAMBDA_AVG_WINDOW_MS of them - a boxcar, the plan's "two-second average"
// taken literally. The alarms may judge it only once the window is full
// (lambdaJudgeable), so no verdict ever rests on a handful of samples or on
// history from before the last gap. The AFR LEAN red needs every sample in
// the window taken under load (lambdaUnderLoad: MAP above
// LAMBDA_CRIT_MIN_MAP, or throttle above LAMBDA_CRIT_MIN_THROTTLE_PCT), so
// the lean-under-load rule can't borrow part-throttle or overrun history.
//
// THE PRICE: after the throttle opens, a real lean condition reaches the
// alarm table no sooner than holdoff + window (3 s with the defaults), plus
// the alarm's own set time. Mixture at closed throttle - idle included - is
// never judged at all.
//
// The ECU repeats a value until its next broadcast, so at a 50 ms CAN data
// period each value lands in ~2.5 consecutive 20 ms samples. Harmless for a
// mean: every value is simply weighted by how long it was current.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "mixture.h"
#include "config.h"
#include "vehicle_data.h"

static_assert(LAMBDA_AVG_WINDOW_MS % TICK_FAST_MS == 0,
              "the mixture window must hold a whole number of ticks");
static const uint16_t WINDOW_N = LAMBDA_AVG_WINDOW_MS / TICK_FAST_MS;

static float    s_err[WINDOW_N];  // lambda - target, one per valid tick
static bool     s_load[WINDOW_N]; // was that tick under load?
static uint16_t s_count;          // valid samples held (0..WINDOW_N); after a
                                  // clear they fill slots 0, 1, 2, ...
static uint16_t s_next;           // slot the next sample goes into
static uint32_t s_lastBadMs;      // millis() of the last excluded tick

// Rules 1-4 (rule 5, the holdoff, needs the clock - see mixtureUpdate()).
static bool sampleValid(const EcuData& e)
    {
    return g_data.ecuOnline()
        && g_data.ecuFrameFresh(ECU_F_LAMBDA)
        && g_data.ecuFrameFresh(ECU_F_AFR_TARGET)
        && g_data.ecuFrameFresh(ECU_F_THROTTLE)
        && g_data.ecuFrameFresh(ECU_F_STATUS)
        && g_data.ecuFrameFresh(ECU_F_FUELING)
        && e.rpm > ENGINE_RUNNING_RPM
        && e.lambda >= LAMBDA_SENSOR_MIN
        && e.lambdaTarget >= LAMBDA_SENSOR_MIN
        && e.throttlePct() >= LAMBDA_MIN_THROTTLE_PCT
        && !e.revLimited()
        && e.injPwMs > 0.0f;
    }

// "Under load" for the AFR LEAN red: the plan's MAP rule, or a wide-open-ish
// throttle (MAP alone can't reach 90 kPa at altitude - config.h).
static bool underLoad(const EcuData& e)
    {
    const bool mapHigh = g_data.ecuFrameFresh(ECU_F_SENSORS1)
                      && e.mapKpa > LAMBDA_CRIT_MIN_MAP;
    return mapHigh || e.throttlePct() >= LAMBDA_CRIT_MIN_THROTTLE_PCT;
    }

static void publishEmpty(EcuData& e)
    {
    e.lambdaErr2s     = 0.0f;
    e.lambdaJudgeable = false;
    e.lambdaUnderLoad = false;
    }

void mixtureUpdate()
    {
    EcuData& e = g_data.ecu;
    const uint32_t now = millis();

    if (!sampleValid(e))
        {
        // Not a sample worth having: forget the window and restart the
        // holdoff clock, so nothing measured before this gap can reach a
        // verdict after it.
        s_count     = 0;
        s_next      = 0;
        s_lastBadMs = now;
        publishEmpty(e);
        return;
        }

    if (now - s_lastBadMs < LAMBDA_HOLDOFF_MS)
        {
        publishEmpty(e); // window is already empty; the sensor is catching up
        return;
        }

    s_err[s_next]  = e.lambda - e.lambdaTarget;
    s_load[s_next] = underLoad(e);
    s_next = (uint16_t)((s_next + 1) % WINDOW_N);
    if (s_count < WINDOW_N) s_count++;

    // Recompute from scratch rather than keep a running sum: 100 floats a
    // tick is nothing at 600 MHz, and nothing can drift.
    float    sum    = 0.0f;
    uint16_t loaded = 0;
    for (uint16_t i = 0; i < s_count; i++)
        {
        sum += s_err[i];
        if (s_load[i]) loaded++;
        }

    e.lambdaErr2s     = sum / s_count; // partial mean while filling (debug only)
    e.lambdaJudgeable = (s_count == WINDOW_N);
    e.lambdaUnderLoad = e.lambdaJudgeable && (loaded == WINDOW_N);
    }
