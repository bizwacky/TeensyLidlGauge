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
//    ADC + switch pins -> inputsPoll() (20 ms tick) -+--> g_data (raw+derived:
//    RTC + die temp -/                                |    PDM verdicts, mixture
//    IMU FIFO -> imuPump() -> motion.cpp ------------+    window, fuel calc,
//                                                     |    attitude + g)
//                                                     alarmsUpdate()
//                                                           v
//                                                    g_data.alarms --> audio,
//                                                    (verdicts)     LED tach,
//                                                                   screens
//
// One arrow runs the other way since the roof switch panel moved onto the
// gauge: its switches and pots go g_data -> canBusTick20ms() -> the PDMs
// (the car's light switches), and three of the pots -> fansUpdate() -> PWM.
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
//                FRAM write queue + serial console, drain the IMU's FIFO
//                (all cheap when idle)
//   20 ms tick : poll inputs, PDM verdicts, resolve the dial, motion (g,
//                attitude, LEVEL SET), mixture window, fuel calc, fan PWM,
//                alarm state machine, switch/pot + IMU frames onto CAN,
//                impact capture -> FRAM, LED tach frame, screens 1-3 (draw +
//                diff + DMA), backlight PWM, feed the watchdog
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
#include "pdm.h"
#include "modes.h"
#include "mixture.h"
#include "fuel_calc.h"
#include "fans.h"
#include "alarms.h"
#include "audio.h"
#include "led_tach.h"
#include "screens.h"
#include "backlight.h"
#include "watchdog.h"
#include "fram.h"
#include "fram_log.h"
#include "fram_events.h"
#include "imu.h"
#include "motion.h"

// --- 1 Hz debug snapshot ---------------------------------------------------------
// The whole g_data struct, one section per line. This is the bring-up and
// debugging tool (and a permanent friend: it costs nothing when no USB host
// is listening).

static void debugPrint()
    {
    const EcuData&    e = g_data.ecu;
    const FuelData&   f = g_data.fuel;
    const SwitchData& s = g_data.sw;
    const BoardData&  b = g_data.board;

    // ECU link: OK = every decoded frame fresh; PARTIAL names the first
    // stale one (exactly what NO CAN is complaining about); OFFLINE = not
    // even the rpm heartbeat. docs/ECU_CAN_SETUP.md maps names to settings.
    const uint8_t stale = g_data.ecuFirstStaleFrame();
    if (stale == ECU_F_COUNT)    Serial.printf("[ECU ] OK      ");
    else if (g_data.ecuOnline()) Serial.printf("[ECU ] PARTIAL (no %s) ", ecuFrameName(stale));
    else                         Serial.printf("[ECU ] OFFLINE ");
    Serial.printf("rpm=%u map=%.1fkPa clt=%.0fC iat=%.0fC batt=%.2fV lam=%.3f(tgt %.3f) trim=%+.1f%%\n",
                  e.rpm, e.mapKpa, e.cltC, e.matC, e.battVolts,
                  e.lambda, e.lambdaTarget, e.egoCorPct);
    Serial.printf("       oilP=%.1fpsi oilT=%.0fC tps=%.1f%% pedal=%.1f%% pw=%.2fms duty=%.1f%% adv=%.1fdeg st=0x%02X%s%s warn=%u err=%u\n",
                  e.oilPressPsi, e.oilTempC, e.tpsPct, e.pedalPct, e.injPwMs,
                  e.injDutyPct, e.sparkAdvDeg, e.statusBits,
                  e.revLimited() ? " REVLIM" : "", e.checkEngine() ? " CEL" : "",
                  e.warningCount, e.lastError);
    // The mixture window: "judging" once mixture.cpp has a full window of
    // valid samples, "gated" while it is excluding them (closed throttle,
    // fuel cut, holdoff, no reading). unk should stay 0; ign climbs by design.
    Serial.printf("       mix %s err2s=%+.3f%s frames=%lu unk=%lu ign=%lu\n",
                  e.lambdaJudgeable ? "judging" : "gated", e.lambdaErr2s,
                  e.lambdaUnderLoad ? " (under load)" : "",
                  (unsigned long)e.frameCount, (unsigned long)g_data.unknownFrameCount,
                  (unsigned long)g_data.ignoredFrameCount);

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
    // rusEFI's own flow figure next to ours - a free cross-check of the
    // injector model (0 if the ECU build doesn't compute it). g/s -> gal/h
    // at ~0.745 kg/L petrol, so expect a percent or two of slack.
    Serial.printf(" | ecu %.2fg/s (~%.2fgph)", e.fuelFlowGps,
                  e.fuelFlowGps * 3600.0f / 745.0f / 3.78541f);
    if (f.fillActive)
        Serial.printf(" | FILL held %lus cans=%u full=%d",
                      (unsigned long)(s.rotaryHeldMs() / 1000),
                      f.fillCansAdded, f.fillFullDone);
    Serial.println();
    // Each PDM: link state, device state, the three device numbers, then one
    // token per output that is not simply Off - "5:PUMP=On/8A" - with the
    // verdict appended when pdm.cpp has one ("TRP", "OPN", "FLT") and x3 for
    // its trip count. "ext=no" = the frames for outputs 9-14 have never
    // arrived (a stock 8-output board, or a fork that isn't sending them).
    static const char* devStates[] = { "RUN", "SLEEP", "OVERTEMP", "ERROR" };
    static const char* outStates[] = { "Off", "On", "OC", "FAULT" };
    for (uint8_t box = 0; box < PDM_COUNT; box++)
        {
        const PdmData& p = g_data.pdm[box];
        if (p.frameCount == 0)
            {
            Serial.printf("[%s] never heard (base id 0x%03lX)\n", pdmName(box),
                          (unsigned long)PDM_BASE_ID[box]);
            continue;
            }
        bool extSeen = false;
        for (uint8_t row = 0; row < PDM_FRAME_COUNT; row++)
            if (PDM_FRAMES[row].need == PDM_NEED_ONCE_SEEN && g_data.pdmFrameSeen(box, row))
                extSeen = true;
        Serial.printf("[%s] %s %s %.0fA %.1fV %.1fC ext=%s frames=%lu |",
                      pdmName(box), g_data.pdmOnline(box) ? "ONLINE " : "STALE  ",
                      (p.deviceState < 4) ? devStates[p.deviceState] : "?",
                      p.totalAmps, p.battVolts, p.boardTempC,
                      extSeen ? "yes" : "no", (unsigned long)p.frameCount);
        for (uint8_t i = 0; i < PDM_MAX_OUTPUTS; i++)
            {
            const PdmOutput& o = p.out[i];
            if (!o.valid || (o.state == PDM_STATE_OFF && o.verdict == PDM_OUT_OK)) continue;
            Serial.printf(" %u:%s=%s/%.0fA", i + 1, PDM_OUTPUTS[box][i].tag,
                          (o.state < 4) ? outStates[o.state] : "?", o.amps);
            if (o.verdict != PDM_OUT_OK) Serial.printf(" %s", pdmVerdictName(o.verdict));
            if (o.trips)                 Serial.printf(" x%u", o.trips);
            }
        Serial.println();
        }

    const LampData& l = g_data.lamps;
    Serial.printf("[SW  ] rotary=%u=%s (held %lus, %.0fohm raw=%u%s) roof=",
                  s.rotaryPos, modeName(g_data.mode.mode),
                  (unsigned long)(s.rotaryHeldMs() / 1000), s.rotaryOhms, s.rotaryRawCounts,
                  s.rotaryOpen ? " OPEN" : "");
    for (uint8_t i = 0; i < ROOF_SWITCH_COUNT; i++)
        Serial.print(s.roof[i] ? '1' : '0'); // SW1 first
    Serial.printf(" lamps: head=%d brights=%d turnL=%d turnR=%d\n",
                  l.headlights, l.brights, l.turnL, l.turnR);
    // Pots: percent (what the PDMs are sent), ohms and raw counts (what to
    // calibrate POT_CAL from), '*' when active, OPEN on a broken wire.
    Serial.printf("[POT ]");
    for (uint8_t i = 0; i < POT_COUNT; i++)
        Serial.printf(" %u=%.0f%%%s(%.0fohm raw=%u%s)", i + 1, s.pot[i].pct,
                      s.pot[i].active ? "*" : "", s.pot[i].ohms, s.pot[i].rawCounts,
                      s.pot[i].open ? " OPEN" : "");
    Serial.printf(" | fan duty %u/%u/%u%%\n", g_data.fans.dutyPct[0],
                  g_data.fans.dutyPct[1], g_data.fans.dutyPct[2]);
    Serial.printf("[MODE] night=%d silent=%d dispOff=%d ledBright=%u labelBright=%u scrBright=%u bl=%.0f/%.0f/%.0f%%%s\n",
                  g_data.mode.night, g_data.mode.silent, g_data.mode.displaysOff,
                  g_data.mode.ledBrightness, g_data.mode.labelBrightness,
                  g_data.mode.screenBrightness,
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

    // The IMU. First line: the car's motion (smooth values from the low-g
    // channel, then the peak-held high-g channel). Second line: the chip's
    // health - rate should read ~960, fifo well under 256, and bus/ovr/tag
    // should all sit at 0 (docs/IMU_SETUP.md says what each one means).
    ImuData& u = g_data.imu;
    if (!u.present)
        Serial.printf("[IMU ] MISSING (no motion data; retrying)\n");
    else
        {
        Serial.printf("[IMU ] %s lon=%+.2f lat=%+.2f vert=%+.2fg roll=%+.1f pitch=%+.1fdeg yaw=%+.1fdps%s%s | "
                      "peak %.2fg (%+.2f %+.2f %+.2f) max %.1fg\n",
                      g_data.imuOnline() ? "OK   " : "STALE",
                      u.lonG, u.latG, u.vertG, u.rollDeg, u.pitchDeg, u.yawRateDps,
                      u.still ? " still" : "", u.airborne ? " AIRBORNE" : "",
                      u.peakG, u.peakLonG, u.peakLatG, u.peakVertG, u.maxPeakG);
        Serial.printf("       rate=%.0fHz fifo=%u bus=%lu ovr=%lu tag=%lu | level=%s hgTrim=%+.2f/%+.2f/%+.2fg%s | "
                      "jumps=%u last %ums/%.1fg | impact #%lu %.1fg%s\n",
                      u.hgRateHz, u.fifoPeakWords, (unsigned long)u.busErrors,
                      (unsigned long)u.fifoOverruns, (unsigned long)u.tagErrors,
                      u.levelCal ? "set" : "NOT SET",
                      u.hgOffsetG[0], u.hgOffsetG[1], u.hgOffsetG[2], u.hgTrimmed ? "" : " (settling)",
                      u.jumpCount, u.lastAirMs, u.lastLandG,
                      (unsigned long)u.event.id, u.event.peakG, u.event.active ? " RECORDING" : "");
        if (u.levelSetActive)
            Serial.printf("       LEVEL SET: %s\n", u.levelSetDone ? "done"
                          : u.levelSetWaiting ? "waiting for the car to be still" : "hold still...");
        }
    u.fifoPeakWords = 0; // "deepest since the last print"

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

    Serial.println("\n=== Rally CAN Gauge ===");
    watchdogPrintResetCause(); // first thing: did we get here via a crash?
    inputsInit();
    fansInit();    // park the fan PWM pins early: until then they float
    canBusInit();
    pdmInit();
    framInit();    // bring up the I2C bus, probe the chip
    framLogInit(); // verify/format the layout, recover the log head
    fuelCalcInit();// restores the FRAM fuel record - must follow framLogInit
    imuInit();     // bring up the IMU's own I2C bus, configure the chip
    motionInit();  // restores the level calibration - must follow framLogInit
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
    imuPump();     // drain the IMU's FIFO -> motion.cpp (g, attitude, impacts)

    // 20 ms tick. elapsedMillis is a Teensy type that counts up on its own;
    // subtracting the period (rather than resetting to 0) means no drift:
    // if this pass ran at 21 ms, the next fires at 19. The catch-up guard
    // stops a long stall from causing a rapid-fire burst of ticks.
    static elapsedMillis sinceFast;
    if (sinceFast >= TICK_FAST_MS)
        {
        sinceFast -= TICK_FAST_MS;
        if (sinceFast >= TICK_FAST_MS) sinceFast = 0; // fell behind: skip, don't burst

        inputsPoll();     // fuel, rotary, roof switches, pots, temp, clock -> g_data
        pdmUpdate();      // PDM status -> per-output verdicts + lamp states
        modesUpdate();    // dial position (+ lamps) -> resolved mode flags (g_data.mode)
        motionUpdate();   // IMU samples so far -> g_data.imu; the LEVEL SET gesture
        mixtureUpdate();  // gated two-second lambda window -> g_data.ecu
        fuelCalcUpdate(); // burn integration + FUEL FILL gesture -> g_data.fuel
        fansUpdate();     // pots -> cabin fan PWM duty
        alarmsUpdate();   // evaluate the whole warning catalog -> g_data.alarms
        canBusTick20ms(); // roof switches + pots -> the PDMs (on change / 10 Hz);
                          // the IMU frames, each at its own period
        framEventsTick(); // an impact capture in progress -> the FRAM, block by block
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
        imuTick1s();     // measured IMU sample rate, for the line below
        debugPrint();
        }
    }
