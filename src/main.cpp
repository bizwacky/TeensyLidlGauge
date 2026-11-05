// ---------------------------------------------------------------------------
// Rally CAN Gauge - main firmware
// ---------------------------------------------------------------------------
// CURRENT PHASE: full gauge - inputs + fuel calculation + alarms + audio +
// LED tach + the three displays. The superloop matches the "Programmational
// Thoughts" section of the project plan.
//
// The architecture in one picture:
//
//    CAN interrupt -> ring buffer -> canBusPoll() ---+
//    ADC pins  ------> inputsPoll()  (20 ms tick) ---+--> g_data (raw+derived)
//    RTC + die temp -/                                      |
//                                                     alarmsUpdate()
//                                                           v
//                                                    g_data.alarms --> audio,
//                                                    (verdicts)     LED tach,
//                                                                   screens
//
// Two layers on purpose: input modules establish WHAT IS (temperatures,
// voltages, freshness), the alarm layer judges WHAT MATTERS (active
// warnings, priority). Consumers only ever read verdicts - no display or
// audio code ever touches a threshold. The screens follow the same split
// one more time: gauge_render.cpp decides WHAT a frame shows (reading only
// g_data), screens.cpp decides HOW it reaches the glass (buffers, diffs,
// DMA - never blocking).
//
// The superloop is a "cooperative scheduler": loop() spins as fast as it can,
// and each pass asks two timers whether their work is due. Nothing ever
// blocks or sleeps, so the CAN queue is drained with low latency and the
// display DMA transfers overlap everything else.
//
//   every pass : drain the CAN queue, pump DFPlayer notifications, pump the
//                FRAM write queue + serial console (all cheap when idle)
//   20 ms tick : poll inputs, derived values, alarm state machine, LED tach
//                frame, screens 1-3 (draw + diff + DMA), backlight PWM,
//                feed the watchdog
//   1 s tick   : PCS heartbeat, audio arbitration, FRAM fuel record +
//                black-box log record, debug snapshot
//
// Verifying this phase on the bench: open the serial monitor (115200) and
// watch the once-per-second snapshot of everything the gauge knows. Wiggle an
// input, watch the number move - and watch the [SCRN] line: draw time must
// stay a small fraction of the 20 ms tick, skips should stay near zero.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "config.h"
#include "vehicle_data.h"
#include "can_bus.h"
#include "inputs.h"
#include "modes.h"
#include "fuel_calc.h"
#include "alarms.h"
#include "audio.h"
#include "led_tach.h"
#include "screens.h"
#include "backlight.h"
#include "watchdog.h"
#include "fram.h"
#include "fram_log.h"

// --- derived values -----------------------------------------------------------
// Numbers computed FROM inputs live here, downstream of acquisition: the
// decoders store exactly what was measured, and everything calculated gets
// derived in one visible place each tick.

static void computeDerived()
    {
    // Injector duty cycle. With sequential injection each injector fires
    // once per engine cycle (2 crank revs), so the time available per
    // injection at a given rpm is (2 * 60000 / rpm) ms. duty = pw / available:
    //   duty% = pw(ms) * rpm / 1200   (at 1 event per cycle)
    // e.g. 10 ms of pulsewidth at 6000 rpm -> 20 ms window -> 50%.
    if (g_data.ecu.rpm > 0)
        {
        g_data.ecu.injDutyPct = g_data.ecu.pw1Ms * (float)g_data.ecu.rpm
                                * INJ_EVENTS_PER_ENGINE_CYCLE / 1200.0f;
        }
    else
        {
        g_data.ecu.injDutyPct = 0.0f;
        }

    // Two-second lambda-error average - the AFR alarms are defined on this
    // (plan: "Lamda (two-second average)"), never on a single noisy sample.
    // lambda error = (AFR - AFR target) / stoich; positive = lean.
    // Only meaningful when the engine is running and both source groups are
    // fresh; otherwise it pins to 0, which conveniently deactivates every
    // AFR alarm condition without them needing their own validity checks.
    static bool lambdaPrimed = false;
    const bool lambdaValid = g_data.ecuGroupFresh(1, ECU_FRESH_SLOW_MS)  // afrTarget
                          && g_data.ecuGroupFresh(3, ECU_FRESH_FAST_MS)  // afr
                          && g_data.ecu.afrTarget > 5.0f  // 0 until first group-1 frame
                          && g_data.ecu.rpm > ENGINE_RUNNING_RPM;
    if (lambdaValid)
        {
        const float err   = (g_data.ecu.afr - g_data.ecu.afrTarget) / AFR_STOICH;
        const float alpha = (TICK_FAST_MS / 1000.0f) / LAMBDA_AVG_TAU_S;
        if (!lambdaPrimed) { g_data.ecu.lambdaErr2s = err; lambdaPrimed = true; }
        else               { g_data.ecu.lambdaErr2s += alpha * (err - g_data.ecu.lambdaErr2s); }
        }
    else
        {
        g_data.ecu.lambdaErr2s = 0.0f;
        lambdaPrimed = false;
        }
    }

// --- 1 Hz debug snapshot ---------------------------------------------------------
// The whole g_data struct, one section per line. This is the bring-up and
// debugging tool for the input phase (and a permanent friend: it costs
// nothing when no USB host is listening).

static void debugPrint()
    {
    const EcuData&    e = g_data.ecu;
    const FuelData&   f = g_data.fuel;
    const SwitchData& s = g_data.sw;
    const BoardData&  b = g_data.board;

    Serial.printf("[ECU ] %s rpm=%u map=%.1fkPa clt=%.1fC mat=%.1fC batt=%.1fV afr=%.1f(tgt %.1f) ego=%+.1f%%\n",
                  g_data.ecuOnline() ? "ONLINE " : "OFFLINE",
                  e.rpm, e.mapKpa, e.cltC, e.matC, e.battVolts, e.afr, e.afrTarget, e.egoCorPct);
    Serial.printf("       oilP=%.1fpsi oilT=%.1fC pw1=%.2fms duty=%.1f%% adv=%.1fdeg eng=0x%02X frames=%lu unk=%lu\n",
                  e.oilPressPsi, e.oilTempC, e.pw1Ms, e.injDutyPct, e.sparkAdvDeg,
                  e.engineBits, (unsigned long)e.frameCount,
                  (unsigned long)g_data.unknownFrameCount);

    const BmsData& m = g_data.bms;
    static const char* bmsModes[] = { "IDLE", "CHG", "DIS" };
    Serial.printf("[BMS ] %s %.2fV %+.1fA soc=%u%% cells %.3f-%.3fV (d=%.3f) temp %.1f..%.1fC avg=%.1fC\n",
                  g_data.bmsOnline() ? "ONLINE " : "OFFLINE",
                  m.packVolts, m.packAmps, m.socPct,
                  m.minCellVolts, m.maxCellVolts, m.cellDeltaVolts(),
                  m.minCellTempC, m.maxCellTempC, m.avgCellTempC);
    Serial.printf("       mode=%s fail=%u cyc=%u bal=0x%04X mosC=%d mosD=%d prot=%02X.%02X.%02X.%02X alrm=%02X.%02X err=%02X frames=%lu\n",
                  (m.opMode < 3) ? bmsModes[m.opMode] : "?",
                  m.failureLevel, m.cycleCount, m.balanceBits,
                  m.chargeMosOn(), m.dischargeMosOn(),
                  m.protection[0], m.protection[1], m.protection[2], m.protection[3],
                  m.alarm[0], m.alarm[1], m.sysError, (unsigned long)m.frameCount);
    // Both fuel estimates side by side. "fast" is the 2 s debug filter -
    // comparing it against the slosh value shows how much motion the slosh
    // filter is absorbing at that moment.
    Serial.printf("[FUEL] sender %.1fohm (fast %.1f) = %.0f%%%s%s%s raw=%u | ",
                  f.ohmsSlosh, f.ohmsFiltered, f.levelPct,
                  f.sloshPrimed ? "" : " PRIMING",
                  f.faultOpen ? " OPEN-CIRCUIT" : "", f.faultShort ? " SHORTED" : "",
                  f.rawCounts);
    if (f.calcValid)
        Serial.printf("calc %.2fgal = %.0f%% burn %.2fgph used %.2fgal",
                      f.calcGallons, f.calcPct, f.burnGph, f.burnedTotalGal);
    else
        Serial.printf("calc awaiting seed (used %.2fgal)", f.burnedTotalGal);
    if (f.fillActive)
        Serial.printf(" | FILL held %lus cans=%u full=%d",
                      (unsigned long)(s.rotaryHeldMs() / 1000),
                      f.fillCansAdded, f.fillFullDone);
    Serial.println();
    Serial.printf("[SW  ] rotary=%u=%s (held %lus, %.0fohm raw=%u%s) head=%d brights=%d blkL=%d blkR=%d\n",
                  s.rotaryPos, modeName(g_data.mode.mode),
                  (unsigned long)(s.rotaryHeldMs() / 1000), s.rotaryOhms, s.rotaryRawCounts,
                  s.rotaryOpen ? " OPEN" : "",
                  s.headlights, s.brights, s.blinkerL, s.blinkerR);
    Serial.printf("[MODE] night=%d silent=%d dispOff=%d ledBright=%u scrBright=%u bl=%.0f/%.0f/%.0f%%%s\n",
                  g_data.mode.night, g_data.mode.silent, g_data.mode.displaysOff,
                  g_data.mode.ledBrightness, g_data.mode.screenBrightness,
                  backlightDutyPct(0), backlightDutyPct(1), backlightDutyPct(2),
                  backlightPhaseLocked() ? "" : " (BL NOT PHASE-LOCKED)");
    Serial.printf("[BRD ] cpu=%.1fC clock=", b.cpuTempC);
    if (b.clockValid) Serial.printf("%02u:%02u:%02u\n", b.hh, b.mm, b.ss);
    else              Serial.printf("--:--:-- (RTC not set)\n");

    // Screen pipeline health: worst draw+diff+kickoff cost over the last
    // second (budget: a small fraction of the 20 ms tick) and cumulative
    // frames dropped because a DMA upload was still running. A skip after a
    // full-screen change (mode/theme flip) is normal; a CLIMBING count
    // during ordinary running means frames are too big or SPI is too slow.
    Serial.printf("[SCRN] draw max %luus, skipped=%lu\n",
                  (unsigned long)screensDrawUsMax(),
                  (unsigned long)screensSkippedFrames());

    const FramData& fr = g_data.fram;
    if (fr.present)
        Serial.printf("[FRAM] %s log %lu/%lu s%u seq=%lu fuelSaves=%lu w=%lu err=%lu drop=%lu\n",
                      fr.logging ? "REC " : "idle",
                      (unsigned long)fr.logRecords, (unsigned long)fr.logCapacity,
                      fr.session, (unsigned long)fr.logSeq,
                      (unsigned long)fr.fuelSaves, (unsigned long)fr.writesDone,
                      (unsigned long)fr.writeErrors, (unsigned long)fr.dropped);
    else
        Serial.printf("[FRAM] MISSING (persistence + black box disabled)\n");

    const AlarmsData& a = g_data.alarms;
    static const char* colorNames[] = { "GRN", "BLU", "YEL", "RED" };
    if (a.top == ALARM_NONE)
        {
        Serial.printf("[ALRM] NOMINAL\n");
        }
    else
        {
        Serial.printf("[ALRM] top=%s(%s) active:", alarmName(a.top), colorNames[alarmColor(a.top)]);
        for (uint8_t i = 0; i < ALARM_COUNT; i++)
            if (a.active[i])
                Serial.printf(" %s/P%u", alarmName(i), alarmPriority(i));
        Serial.printf("  red=%d yel=%d\n", a.anyRed, a.anyYellow);
        }
    }

// --- setup -------------------------------------------------------------------------

void setup()
    {
    Serial.begin(115200);
    delay(400); // give a just-plugged-in serial monitor a moment to attach

    Serial.println("\n=== Rally CAN Gauge - inputs + alarms + FRAM build ===");
    watchdogPrintResetCause(); // first thing: did we get here via a crash?
    inputsInit();
    canBusInit();
    framInit();    // bring up the I2C bus, probe the chip
    framLogInit(); // verify/format the layout, recover the log head
    fuelCalcInit();// restores the FRAM fuel record - must follow framLogInit
    alarmsInit();
    audioInit();
    ledTachInit();
    screensInit(); // the NV3007 init sequence takes a moment (sleep-out delays)
    backlightInit();

    // The watchdog arms LAST, after every init that could legitimately take
    // a while - from here on, only the 20 ms tick's feed keeps us alive.
    watchdogInit();
    }

// --- the superloop ------------------------------------------------------------------

void loop()
    {
    // Drain the CAN queue on EVERY pass, not just on the tick: it keeps
    // per-call work tiny and receive latency low. Same for the DFPlayer's
    // serial port - its "track finished" / "online" frames arrive whenever
    // the module feels like it.
    canBusPoll();
    audioLoop();
    framPump();    // start the next queued FRAM write if the bus is idle
    framLogPoll(); // serial console + (if one is running) the chunked CSV dump

    // 20 ms tick. elapsedMillis is a Teensy type that counts up on its own;
    // subtracting the period (rather than resetting to 0) means no drift:
    // if this pass ran at 21 ms, the next fires at 19. The catch-up guard
    // stops a long stall from causing a rapid-fire burst of ticks.
    static elapsedMillis sinceFast;
    if (sinceFast >= TICK_FAST_MS)
        {
        sinceFast -= TICK_FAST_MS;
        if (sinceFast >= TICK_FAST_MS) sinceFast = 0; // fell behind: skip, don't burst

        inputsPoll();     // fuel, rotary, switches, temp, clock -> g_data
        modesUpdate();    // dial position -> resolved mode flags (g_data.mode)
        computeDerived(); // math on this tick's snapshot
        fuelCalcUpdate(); // burn integration + FUEL FILL gesture -> g_data.fuel
        alarmsUpdate();   // evaluate the whole warning catalog -> g_data.alarms
        ledTachUpdate();  // paint the strip from this tick's verdicts (50 Hz)
        screensUpdate();  // compose + diff + DMA-push all three panels
        backlightUpdate();// fade screen backlights toward the mode's level

        watchdogFeed();   // last: everything above ran to completion again
        }

    // 1 s housekeeping tick.
    static elapsedMillis sinceSlow;
    if (sinceSlow >= TICK_SLOW_MS)
        {
        sinceSlow -= TICK_SLOW_MS;
        if (sinceSlow >= TICK_SLOW_MS) sinceSlow = 0;

        canBusTick1s();  // PCS heartbeat -> BMS
        audioTick1s();   // pick + start at most one voice callout
        framLogTick1s(); // fuel record (if changed) + black-box record (if
                         // this second is worth remembering) -> write queue
        debugPrint();
        }
    }
