// ---------------------------------------------------------------------------
// Rally CAN Gauge - calculated fuel implementation
// ---------------------------------------------------------------------------
// The physics, once, in full (constants and their sources: config.h):
//
//   * While an injector is open, fuel flows at a fixed rate set by the
//     nozzle and the rail pressure: 254 cm3/min for our injectors at 3.5 bar.
//   * rusEFI broadcasts the ELECTRICAL pulse width (BASE5 "InjPW"). The
//     injector is an electromagnet, so for the first ~0.4 ms of every pulse
//     (more at low battery voltage) it is still physically closed - rusEFI
//     adds that "dead time" onto the pulse it commands, and we subtract it
//     back out:
//         open time = injPw - deadTime(battVolts)
//     (For a fuel cut rusEFI computes a 0 ms pulse - getInjectionDuration()
//     returns 0 for zero fuel mass - which this math already turns into
//     zero flow.)
//   * With sequential injection each injector fires once per engine cycle =
//     once per TWO crank revolutions, so each injector fires rpm/120 times
//     per second, and there are six of them.
//
//   fuel rate (cm3/s) =
//       cylinders x (flow/60) x [openMs x (rpm/120) / 1000] x trim
//           |           |             |
//           6        cm3 per       fraction of each second the injector
//                    second open   is actually flowing ("duty")
//
// Each 20 ms tick contributes rate x dt to the total. Sanity anchor: 10 ms
// open at 6000 rpm is 50% duty = 12.7 cm3/s = ~12 gal/h - full-load-ish
// numbers for a 2.5 L engine, which is how you smell-test the math on the
// bench (debug print shows gal/h live).
//
// WHY THE ACCUMULATORS ARE double: a float carries ~7 significant digits.
// One idle tick burns ~0.000001 gal; added to a running total of 10 gal
// that increment is at the very edge of what a float can even represent -
// hours of idle would quietly evaporate from the total. A double (15-16
// digits) makes the problem unrepresentable, and the Teensy 4's FPU does
// double-precision in hardware, so the cost is nil. The g_data copies are
// float because displays don't care about the 10th digit.
//
// SEEDING: integration tells us what LEFT the tank, so it needs a starting
// point. The primary seed is the FRAM fuel record (framFuelLoad at init):
// the gauge wakes up knowing exactly what it knew when the kill switch hit.
// The fallback - blank/corrupt/absent FRAM - is the slosh-filtered sender
// once it primes (~5 s after boot, car presumably sitting still). A 60 s
// full-tank gesture also seeds (it is an absolute statement).
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "fuel_calc.h"
#include "config.h"
#include "vehicle_data.h"
#include "inputs.h"   // fuelSloshReset() on fill events
#include "fram_log.h" // framFuelLoad() - the boot-time seed

// The authoritative state - doubles, per the header comment above.
static double   s_remainingGal;     // meaningful only once calcValid
static double   s_burnedTotalGal;   // lifetime burn, survives power cycles via
                                    // FRAM (the trim-calibration odometer)
static uint32_t s_lastIntegrateMs;  // 0 = integration chain broken (no dt yet)

// --- injector dead time -----------------------------------------------------------
// Linear interpolation through the spec sheet's voltage table, clamped at
// the ends. "Linear between known points" is the same trick TunerStudio
// tables use everywhere - the curve is smooth enough that it's accurate.

static float injDeadTimeMs(float volts)
    {
    if (volts <= INJ_DEAD_TIME[0].volts)
        return INJ_DEAD_TIME[0].ms;
    if (volts >= INJ_DEAD_TIME[INJ_DEAD_TIME_N - 1].volts)
        return INJ_DEAD_TIME[INJ_DEAD_TIME_N - 1].ms;

    for (uint8_t i = 1; i < INJ_DEAD_TIME_N; i++)
        {
        if (volts < INJ_DEAD_TIME[i].volts)
            {
            const InjDeadTime& lo = INJ_DEAD_TIME[i - 1];
            const InjDeadTime& hi = INJ_DEAD_TIME[i];
            const float frac = (volts - lo.volts) / (hi.volts - lo.volts);
            return lo.ms + frac * (hi.ms - lo.ms);
            }
        }
    return INJ_DEAD_TIME[INJ_DEAD_TIME_N - 1].ms; // unreachable; keeps -Wall happy
    }

// --- the integrator ---------------------------------------------------------------

static void integrateBurn()
    {
    FuelData& f = g_data.fuel;
    const uint32_t now = millis();

    // Only integrate on live data with the engine turning. When either goes
    // away, break the dt chain (s_lastIntegrateMs = 0): otherwise the first
    // tick after a 10 s CAN dropout would integrate a stale pulse width
    // across the whole gap. The honest answer during a gap is "we don't
    // know what burned" - the NO CAN alarm tells the driver the estimate
    // is degraded, and the trim calibration absorbs what little is lost.
    if (!g_data.ecuOnline() || !g_data.ecuFrameFresh(ECU_F_FUELING)
        || g_data.ecu.rpm == 0)
        {
        s_lastIntegrateMs = 0;
        f.burnGph = 0.0f;
        return;
        }

    if (s_lastIntegrateMs == 0)
        {
        s_lastIntegrateMs = now; // first live tick: establish t0, burn nothing
        return;
        }

    const float dtS = (now - s_lastIntegrateMs) / 1000.0f;
    s_lastIntegrateMs = now;
    if (dtS <= 0.0f || dtS > 0.5f)
        return; // scheduler hiccup - skip one tick rather than mis-integrate

    // Dead-time lookup wants the system voltage; BASE4 carries it. If that
    // one frame happens to be stale, assume normal alternator voltage rather
    // than halting the whole estimate over a 0.1 ms detail.
    const float volts = g_data.ecuFrameFresh(ECU_F_SENSORS2)
                            ? g_data.ecu.battVolts : 14.0f;
    const float openMs = g_data.ecu.injPwMs - injDeadTimeMs(volts);

    float ccPerS = 0.0f;
    if (openMs > 0.0f)
        {
        const float eventsPerS = (float)g_data.ecu.rpm
                                 * INJ_EVENTS_PER_ENGINE_CYCLE / 120.0f;
        ccPerS = ENGINE_CYLINDERS * (INJ_FLOW_CC_PER_MIN / 60.0f)
               * (openMs * eventsPerS / 1000.0f) * FUEL_CALC_TRIM;
        }

    const double gal = (double)ccPerS * dtS / CC_PER_GALLON;
    s_burnedTotalGal += gal;
    if (f.calcValid)
        {
        s_remainingGal -= gal;
        if (s_remainingGal < 0.0) s_remainingGal = 0.0;
        }

    // Displayed burn rate: short EMA so the number is readable, not a blur.
    const float gph = ccPerS * 3600.0f / CC_PER_GALLON;
    f.burnGph += (dtS / FUEL_BURN_GPH_TAU_S) * (gph - f.burnGph);
    }

// --- seeding ----------------------------------------------------------------------

static void trySeed()
    {
    FuelData& f = g_data.fuel;
    if (f.calcValid)
        return;

    // This sender bootstrap is the FALLBACK seed - it only runs when the
    // FRAM record was missing or corrupt at boot (fuelCalcInit), because a
    // restored record sets calcValid and we never get here.
    if (f.sloshPrimed && !f.faultOpen && !f.faultShort)
        {
        s_remainingGal = (double)(f.levelPct / 100.0f) * FUEL_TANK_GAL;
        f.calcValid    = true;
        Serial.printf("Fuel calc seeded from sender: %.1f gal (%.0f%%)\n",
                      (float)s_remainingGal, f.levelPct);
        }
    // Sender faulted or not primed yet: stay invalid and keep trying. Burn
    // still accumulates in s_burnedTotalGal, and a 60 s full-tank gesture
    // can always seed us manually (the recovery path for a dead sender).
    }

// --- the FUEL FILL gesture --------------------------------------------------------
// All state the gesture needs already exists: modesUpdate() resolved the
// detent into MODE_FUEL_RESET, and sw.rotaryHeldMs() says how long the dial
// has sat there. The published fill* fields double as our one-shot latches:
// each event fires once per visit, and leaving the detent re-arms them.

static void fillGesture()
    {
    FuelData& f = g_data.fuel;

    // A dial that reads open-circuit is not "held on the detent", whatever
    // position it last showed: abandon the gesture. inputs.cpp restarts the
    // held timer when the wiper comes back, so a reconnect can't instantly
    // credit a can off the old hold time.
    if (g_data.mode.mode != MODE_FUEL_RESET || g_data.sw.rotaryOpen)
        {
        f.fillActive    = false;
        f.fillCansAdded = 0;     // leaving the detent re-arms both events
        f.fillFullDone  = false;
        return;
        }

    f.fillActive = true;
    const uint32_t held = g_data.sw.rotaryHeldMs();

    if (!f.fillFullDone && held >= FUEL_FILL_FULL_MS)
        {
        // A full tank is an absolute statement - it doesn't need a valid
        // starting point, it IS one. (Holding to 60 s also crossed 30 s and
        // credited a can on the way; harmless, this overwrites it.)
        s_remainingGal = FUEL_TANK_GAL;
        f.calcValid    = true;
        f.fillFullDone = true;
        fuelSloshReset();
        Serial.printf("FUEL FILL: full tank (%.1f gal)\n", FUEL_TANK_GAL);
        }
    else if (f.fillCansAdded == 0 && held >= FUEL_FILL_CAN_MS)
        {
        f.fillCansAdded = 1; // latch even if skipped below, so no print spam
        if (f.calcValid)
            {
            s_remainingGal += FUEL_FILL_CAN_GAL;
            if (s_remainingGal > FUEL_TANK_GAL) s_remainingGal = FUEL_TANK_GAL;
            fuelSloshReset();
            Serial.printf("FUEL FILL: +%.0f gal (1 can) -> %.1f gal\n",
                          FUEL_FILL_CAN_GAL, (float)s_remainingGal);
            }
        else
            {
            // +5 on top of "no idea" is still "no idea". Keep holding to
            // 60 s for the full reset instead.
            Serial.println("FUEL FILL: can ignored (no valid base - hold for full reset)");
            }
        }
    }

// --- public API -------------------------------------------------------------------

void fuelCalcInit()
    {
    // Primary seed: the FRAM fuel record (CRC + sanity checked inside
    // framFuelLoad). Restoring the doubles verbatim makes a power cycle
    // invisible to the estimate - not even a rounding step.
    double rem, burned;
    if (framFuelLoad(rem, burned))
        {
        s_remainingGal        = rem;
        s_burnedTotalGal      = burned;
        g_data.fuel.calcValid = true;
        Serial.printf("Fuel calc restored from FRAM: %.2f gal (%.0f%%), lifetime burned %.2f gal\n",
                      (float)rem, (float)(rem / FUEL_TANK_GAL * 100.0),
                      (float)burned);
        }
    Serial.printf("Fuel calc: %u inj x %.0f cc/min, tank %.1f gal, trim %.3f%s\n",
                  ENGINE_CYLINDERS, INJ_FLOW_CC_PER_MIN, FUEL_TANK_GAL, FUEL_CALC_TRIM,
                  g_data.fuel.calcValid ? "" : " - awaiting seed");
    }

void fuelCalcGetState(double& remainingGal, double& burnedTotalGal, bool& valid)
    {
    remainingGal   = s_remainingGal;
    burnedTotalGal = s_burnedTotalGal;
    valid          = g_data.fuel.calcValid;
    }

void fuelCalcUpdate()
    {
    trySeed();
    integrateBurn();
    fillGesture();

    // Publish float copies for consumers (the doubles stay private so no
    // consumer can accidentally write our accumulators).
    FuelData& f = g_data.fuel;
    f.burnedTotalGal = (float)s_burnedTotalGal;
    f.calcGallons    = f.calcValid ? (float)s_remainingGal : 0.0f;
    f.calcPct        = f.calcValid
        ? constrain((float)(s_remainingGal / FUEL_TANK_GAL) * 100.0f, 0.0f, 100.0f)
        : 0.0f;
    }
