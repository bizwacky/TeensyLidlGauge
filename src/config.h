// ---------------------------------------------------------------------------
// Rally CAN Gauge - central configuration
// ---------------------------------------------------------------------------
// Every pin number, electrical constant, and tunable value lives in this one
// file. If the wiring or the ECU configuration changes, this should be the
// only file that needs editing (the .cpp files just consume these names).
//
// Pin assignments follow the "Teensy Pinout" table in
// ../../Rally CAN Gauge Project Plan.md - update both together.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

// --- superloop timing ---------------------------------------------------------

static const uint32_t TICK_FAST_MS = 20;   // 50 Hz: inputs, mixture window, alarms, LEDs, screens, backlights
static const uint32_t TICK_SLOW_MS = 1000; // 1 Hz: BMS heartbeat, audio arbitration, FRAM, debug print

// --- CAN bus ------------------------------------------------------------------
// One physical bus shared by the rusEFI ECU (a Super uaEFI - it has a single
// CAN port, so its on-board wideband controller talks on this bus too), the
// JK BMS and this gauge. Everyone must agree on the bitrate. The ECU side is
// documented in docs/ECU_CAN_SETUP.md - keep that file and these constants
// in sync!

static const uint32_t CAN_BITRATE = 500000; // 500 kbps (rusEFI default)

// rusEFI's native broadcast ("Enable rusEFI CAN broadcast" in TunerStudio):
// twelve fixed 8-byte frames at base + 0..11, LITTLE-endian, all sent
// together once per "rusEFI CAN data period". Their layouts are defined by
// rusEFI itself (firmware/controllers/can/rusEFI_CAN_verbose.dbc, which
// calls them BASE0..BASE11); can_bus.cpp decodes BASE0..BASE7.
//
// rusEFI's default base is 0x200 - the same id as one of the JK BMS's
// frames (docs/BMS_CAN_SETUP.md). Two nodes transmitting one id corrupt
// each other, so the ECU is moved: set "rusEFI CAN data base address" to
// 1664 (= 0x680) in TunerStudio. 0x680..0x68B is clear of everything else
// on this bus.
static const uint32_t RUSEFI_CAN_BASE_ID     = 0x680;
static const uint8_t  RUSEFI_CAN_FRAME_COUNT = 12; // BASE0..BASE11

// The one value the native broadcast lacks is the TARGET lambda, which the
// mixture alarms are judged against. rusEFI's "MS Simplified Broadcast"
// (TunerStudio: CAN dash type) carries it in its third frame as 'afrtgt1':
// one byte, AFR x 10, = target lambda x 14.7. That byte is all we take from
// the five MS frames; the other four are ignored. rusEFI fixes their ids
// (1512..1516) and sends them every 10 ms.
static const uint32_t MS_DASH_BASE_ID       = 0x5E8; // 1512
static const uint8_t  MS_DASH_FRAME_COUNT   = 5;
static const uint32_t MS_DASH_AFR_TARGET_ID = 0x5EA; // 1514: afrtgt1 in byte 0

// rusEFI wideband controllers - including the Super uaEFI's on-board one -
// broadcast their own lambda frames from 0x190 up. Expected traffic, but not
// decoded: the ECU already re-broadcasts the lambda it uses in BASE7.
static const uint32_t RUSEFI_WBO_FIRST_ID = 0x190;
static const uint32_t RUSEFI_WBO_LAST_ID  = 0x19F;

// Freshness: how old an ECU frame may be before its values stop counting
// (display shows "--", alarms gate off, NO CAN raises). Several broadcast
// periods, so a lost frame never trips anything. Sized for a rusEFI CAN
// data period of 50 ms or faster; rusEFI slows its broadcast 5x while
// TunerStudio talks to it OVER CAN, and at 50 ms that (250 ms) still fits.
static const uint32_t ECU_FRESH_MS = 500;

// JK BMS speaking the "Deye" protocol (PCS CAN-Bus-protocol-DY-low-voltage,
// select protocol 001 in the JK app). Standard 11-bit ids, LITTLE-endian
// payloads (opposite of the ECU!), everything broadcast on a 1 s cycle.
// Frame layouts: docs/BMS_CAN_SETUP.md.
static const uint8_t  BMS_ADDR = 1; // single pack; per-module ids are base+(addr-1)

static const uint32_t BMS_ID_SOC       = 0x355;                  // SOC / SOH
static const uint32_t BMS_ID_PACK      = 0x356;                  // pack V / A / avg temp
static const uint32_t BMS_ID_PROTECT   = 0x359;                  // fault + alarm bitfields
static const uint32_t BMS_ID_CELLS     = 0x361;                  // max/min cell V + temp
static const uint32_t BMS_ID_STATE     = 0x400 + (BMS_ADDR - 1); // mode/fail/cycles/balance
static const uint32_t BMS_ID_FAULT_MOS = 0x110 + (BMS_ADDR - 1); // MOSFET states (byte 7)

// The protocol expects the inverter ("PCS") to send a 0x305 heartbeat every
// second; without one the BMS may raise its "PCS communication failure" bit.
// There is no inverter in the car, so the gauge plays that role. Set false
// if it ever causes trouble on the bench.
static const bool     BMS_SEND_PCS_HEARTBEAT = true;
static const uint32_t BMS_ID_PCS_HEARTBEAT   = 0x305;

// 1 s broadcast cycle -> allow a few missed frames before declaring it gone.
static const uint32_t BMS_OFFLINE_MS = 5000;

// --- analog input pins ----------------------------------------------------------

// (Sept 2026 pin shuffle: brights/headlights moved from 22/25 onto 14/15,
// which the DFPlayer vacated for Serial7 on 28/29 - see AUDIO_SERIAL - and
// display 3's PWM moved from 4 to 22 so all three backlights share one
// FlexPWM module - see PIN_BACKLIGHT. Pins 4 and 25 are now unused.)

static const uint8_t PIN_FUEL_SENDER = 23; // A9  - fuel sender divider node
static const uint8_t PIN_ROTARY      = 24; // A10 - 12-position rotary divider
static const uint8_t PIN_HEADLIGHTS  = 15; // A1  - 12V input via 100k/10k divider
static const uint8_t PIN_BRIGHTS     = 14; // A0  - 12V input (high beam + light bars)
static const uint8_t PIN_BLINKER_L   = 16; // A2  - 12V input
static const uint8_t PIN_BLINKER_R   = 17; // A3  - 12V input

// --- ADC configuration ----------------------------------------------------------

static const uint8_t  ADC_RESOLUTION_BITS = 12;   // 0..4095 counts
static const int      ADC_MAX_COUNTS      = 4095;
static const uint8_t  ADC_AVERAGING       = 16;   // hardware averaging: ~16x less noise,
                                                  // still well under 0.1 ms per pin

// --- fuel sender ----------------------------------------------------------------
// Circuit (Schematic Ref.pdf, sheet 1):
//
//   +3V3 --[R14 150R precision]--+-- FUEL_IN --> 2 x E30 sender in series --> GND
//                                |
//                          [R13 1k]--+-- FUEL_READ (pin 23)
//                                    |
//                               [C12 100n]
//                                    |
//                                   GND
//
// R14 and the senders form the divider; R13/C12 are only an RC filter on
// the ADC pin (1.6 kHz corner). The ADC draws no DC current, so R13 drops
// nothing and the pin sees the divider node exactly. Divider and ADC share
// the 3.3V rail, so the reading is RATIOMETRIC - the rail cancels out:
//
//   f = counts / 4095 = Rs / (Rs + 150)    =>    Rs = 150 * f / (1 - f)
//
// Only R14's tolerance matters, which is why it is a precision part. It
// dissipates 67 mW with the tank full (Rs = 6R -> 21 mA): the schematic's
// "up to 70 mW". Expected readings, each sender being 3R full .. 72R empty:
//
//   full   Rs =   6R   f = 0.038   0.13 V    157 counts
//   empty  Rs = 144R   f = 0.490   1.62 V   2006 counts   (~13 counts/ohm)
//   open   Rs = inf    f = 1.000   3.30 V   4095 counts
//   short  Rs =   0R   f = 0.000   0.00 V      0 counts

static const float FUEL_REF_OHMS = 150.0f; // R14, the precision reference

// Dual E30 senders in series: 2 x (3R full .. 72R empty) per the schematic
// note. STILL CALIBRATE ON THE CAR: fill with known amounts and record the
// slosh-filtered ohms the debug print shows - senders are rarely truly
// linear. If the gauge reads backwards, just swap these two numbers.
static const float FUEL_OHMS_EMPTY = 144.0f; // 2 x 72R
static const float FUEL_OHMS_FULL  = 6.0f;   // 2 x 3R

// Fast debug filter: a short exponential moving average that tracks the
// sender closely. NOT used for display or alarms - it exists so the serial
// print shows what the sender is physically doing (slosh and all), which is
// how you sanity-check the slosh filter below against reality.
static const float FUEL_FILTER_TAU_S = 2.0f;

// Fault detection, in ohms (the same math as the reading itself): a broken
// wire lets R14 pull the node to the rail (Rs -> infinite), a wire chafed
// onto the chassis reads ~0R. Both limits sit far outside the 6..144R
// sender range while staying well clear of the fault readings themselves.
static const float FUEL_OHMS_OPEN  = 400.0f; // above this = wiring open
static const float FUEL_OHMS_SHORT = 1.0f;   // below this = shorted to ground

// --- fuel slosh filter ------------------------------------------------------------
// The display/alarm level runs through a TWO-STAGE filter (inputs.cpp):
//
//   stage 1: keep the last FUEL_SLOSH_MEDIAN_N raw readings, sampled every
//            FUEL_SLOSH_SAMPLE_MS, and take their MEDIAN. Why a median and
//            not more averaging? Slosh is not symmetric noise: in a long
//            corner the fuel piles up on one side and the float can PIN at an
//            end stop for seconds. An average gets dragged toward that lie;
//            a median simply ignores it as long as it lasts for less than
//            half the window.
//   stage 2: a slow exponential average (time constant FUEL_SLOSH_TAU_S)
//            over the median, which irons out what remains.
//
// Net effect: honest step response of ~1 minute. That is fine - real fuel
// level only moves at ~1%/minute at race pace. The fill gesture bypasses the
// lag by resetting the chain (fuelSloshReset()) when fuel is actually added.
static const uint32_t FUEL_SLOSH_SAMPLE_MS = 500;   // one median sample per 0.5 s
static const uint8_t  FUEL_SLOSH_MEDIAN_N  = 9;     // 9 samples = 4.5 s window
static const float    FUEL_SLOSH_TAU_S     = 30.0f; // stage-2 smoothing

// --- calculated fuel: injector-pulse integration (fuel_calc.cpp) -------------------
// rusEFI broadcasts the injector pulse width it is commanding (BASE5, once
// per CAN data period). Injectors are precision metering devices: open time
// x flow rate = fuel delivered. Integrating that gives a fuel estimate far
// steadier than any float in a sloshing tank.

static const uint8_t ENGINE_CYLINDERS = 6; // M54: one injector per cylinder

// Siemens A2C59514053 (M54B30 OEM): 254 cm3/min at 3.5 bar fuel pressure
// (from "M54 Injector Spec.png" in the project root). If rail pressure ever
// changes, flow scales with sqrt(new bar / 3.5).
static const float INJ_FLOW_CC_PER_MIN = 254.0f;

// Injector dead time vs battery voltage, from the same spec sheet
// ("ip_ti_add_dly__vb"). An injector is an electromagnet: it takes this long
// to physically open after the pulse starts, during which NO fuel flows.
// rusEFI adds its own dead-time correction onto the pulse it broadcasts
// (InjectorModelBase::getInjectionDuration returns base + dead time), so the
// fuel math must subtract it back out - at a 2 ms idle pulse the ~0.4 ms
// dead time is 20% of the reading, so skipping this correction would
// overcount idle burn by a fifth. Keep this table the same as the injector
// dead-time (battery lag) table in the rusEFI tune - the subtraction is only
// right if both sides agree. Interpolated linearly, clamped at the ends.
struct InjDeadTime
    {
    float volts;
    float ms;
    };

static const InjDeadTime INJ_DEAD_TIME[] =
    {
    {  6.0f, 3.328f },
    {  8.0f, 1.408f },
    { 10.0f, 0.864f },
    { 12.0f, 0.576f },
    { 14.0f, 0.384f }, // normal running voltage sits right about here
    { 16.0f, 0.256f },
    { 25.0f, 0.032f },
    };
static const uint8_t INJ_DEAD_TIME_N = sizeof(INJ_DEAD_TIME) / sizeof(INJ_DEAD_TIME[0]);

// Calibration trim for systematic error (real injectors are not exactly
// nominal, fuel temp varies, etc). CALIBRATE: run a few tanks, compare
// gallons the gauge says were burned against gallons the pump says went
// back in, then set trim = actual / calculated. Expect within a few %.
static const float FUEL_CALC_TRIM = 1.000f;

static const float CC_PER_GALLON = 3785.41f; // US gallon

// MEASURE ON THE CAR: usable tank capacity. Late E30 tank is nominally
// 16.4 gal (62 L) - verify what a full fill actually takes.
static const float FUEL_TANK_GAL = 16.4f;

// Smoothing for the displayed burn rate (gal/h). Display nicety only - the
// integration itself always uses the raw per-tick values.
static const float FUEL_BURN_GPH_TAU_S = 2.0f;

// --- fuel fill gesture --------------------------------------------------------------
// Refueling is told to the gauge with the rotary dial: sit on the FUEL FILL
// detent and wait. Crossing 30 s credits one 5-gal can; staying to 60 s
// declares a full tank (an absolute reset, so it also recovers a lost/silly
// calculated value). Two cans = do the 30 s hold twice (leave the detent and
// come back). Progress is published in g_data.fuel for the screens phase.
static const float    FUEL_FILL_CAN_GAL = 5.0f;
static const uint32_t FUEL_FILL_CAN_MS  = 30000;
static const uint32_t FUEL_FILL_FULL_MS = 60000;

// --- rotary mode switch ----------------------------------------------------------
// Circuit (Schematic Ref.pdf, sheet 1): the CTS 212 12-position switch
// carries a chain of 11 x 220R resistors; its wiper picks a tap and the
// far end of the chain returns to PCB ground. On the PCB a 2.2k pull-up
// feeds the wiper node and a 10k/100n RC filters it into the ADC pin:
//
//   +3V3 --[R31 2.2k]--+-- DIAL_IN --> wiper --> k x 220R --> GND
//                      |
//                [R30 10k]--+-- DIAL_READ (pin 24)
//                           |
//                      [C15 100n]
//                           |
//                          GND
//
// So position k (0..11) presents Rk = k x 220R to ground and the pin sees
// 3.3V x Rk / (Rk + 2200). That is NOT a linear voltage ladder - the steps
// shrink toward the top (position 0->1 is 0.30 V, 10->11 is 0.08 V) - but
// it IS linear in RESISTANCE, so inputs.cpp does what the fuel sender does:
// convert counts to ohms ratiometrically (Rk = 2200 * f / (1 - f)), then
// position = round(Rk / 220). Every decision boundary then sits halfway
// between taps, +-110R of margin each. Expected readings:
//
//   pos  0     0R  0.000 V     0 counts    pos  6  1320R  1.238 V  1536
//   pos  1   220R  0.300 V   372           pos  7  1540R  1.359 V  1686
//   pos  2   440R  0.550 V   683           pos  8  1760R  1.467 V  1820
//   pos  3   660R  0.762 V   945           pos  9  1980R  1.563 V  1940
//   pos  4   880R  0.943 V  1170           pos 10  2200R  1.650 V  2048
//   pos  5  1100R  1.100 V  1365           pos 11  2420R  1.729 V  2145
//   open (wiper lifted / broken wire)     3.300 V  4095
//
// Margin budget at the top of the ladder, where one count is worth ~2.4R:
// 1% resistors put the ratio off by up to ~50R, and the 10k filter resistor
// turns any ADC input leakage into an offset (1 uA = 10 mV = ~30R). Against
// a 110R half-step that is fine in practice but not lavish - if positions
// 10 and 11 ever misread on the bench, shrink R30 first (1k, like the fuel
// input) and tighten R31 to 0.1% second.
//
// FAULT DETECTION (DIALFAIL): an open wiper - broken wire, dead switch, or
// a non-shorting switch caught between detents - reads far above the last
// tap. Anything past ROTARY_OPEN_OHMS is called open. inputs.cpp debounces
// that like any position, holds the last good position while it lasts, and
// after ROTARY_OPEN_MS parks the dial on ROTARY_FAILSAFE_POS: position 0 =
// BRIGHT, i.e. "behave as if the dial weren't there" - screens on, full
// brightness, audio unmuted - so the DIALFAIL banner and callout can
// actually reach the driver instead of being trapped behind DISPLAYS OFF
// or SILENT. The alarm (alarms.cpp) uses the same ROTARY_OPEN_MS, so the
// banner and the fallback appear together. A short to ground is
// indistinguishable from position 0, which is the fail-safe mode anyway.

static const uint8_t  ROTARY_POSITIONS      = 12;
static const float    ROTARY_PULLUP_OHMS    = 2200.0f; // R31
static const float    ROTARY_STEP_OHMS      = 220.0f;  // one ladder resistor
static const float    ROTARY_OPEN_OHMS      = 3000.0f; // last tap is 2420R; this is 2.6
                                                       // steps past it and still less than
                                                       // a quarter of the way to open (inf)
static const uint8_t  ROTARY_DEBOUNCE_TICKS = 3;       // 3 x 20 ms = stable for 60 ms
static const uint16_t ROTARY_OPEN_MS        = 2000;    // open this long = DIALFAIL + fail-safe
static const uint8_t  ROTARY_FAILSAFE_POS   = 0;       // = MODE_BRIGHT in ROTARY_MODE_MAP

// --- gauge modes: what each detent of the dial means ------------------------------
// The dial is the gauge's ONLY control, one mode per detent. Every tick,
// modesUpdate() (src/modes.cpp) reads the debounced position, looks it up
// in this table and publishes the RESOLVED decisions (silent? night colors?
// what brightness? displays off?) to g_data.mode. Consumers read only those
// flags, never the raw position - so remapping the dial is an edit to this
// table and nothing else.

enum GaugeMode : uint8_t
    {
    MODE_BRIGHT = 0,   // full brightness
    MODE_MEDIUM,       // mid brightness
    MODE_DARK,         // minimum brightness + night color scheme
    MODE_DEBUG,        // dense all-parameters pages on the three screens
                       // (gauge_render.cpp, gaugeRenderDebug)
    MODE_SILENT,       // no voice, no flashing - "I know, stop telling me":
                       // for a known-bad sensor that would nag all stage
    MODE_FUEL_RESET,   // the FUEL FILL detent: hold 30 s = +5 gal,
                       // hold 60 s = full-tank reset (fuel_calc.cpp)
    MODE_DISPLAYS_OFF, // every LED dark, screens black, backlights off.
                       // Audio alarms STILL play - only the light goes away.
    MODE_UNASSIGNED,   // spare detents - Darude Sandstorm, fun themes and
                       // clock-set from the plan land on these one day
    };

static const GaugeMode ROTARY_MODE_MAP[ROTARY_POSITIONS] =
    {
    MODE_BRIGHT,       // position 0  (assumed = the 0V end of the ladder)
    MODE_MEDIUM,       // position 1
    MODE_DARK,         // position 2
    MODE_DEBUG,        // position 3
    MODE_SILENT,       // position 4
    MODE_FUEL_RESET,   // position 5
    MODE_DISPLAYS_OFF, // position 6
    MODE_UNASSIGNED,   // position 7
    MODE_UNASSIGNED,   // position 8
    MODE_UNASSIGNED,   // position 9
    MODE_UNASSIGNED,   // position 10
    MODE_UNASSIGNED,   // position 11
    };

// --- 12V switch inputs (headlights, brights, blinkers) -----------------------------
// Each is 12V into a 100k/10k divider -> ~1.1-1.3V at the pin when on, ~0V when
// off. Two thresholds (hysteresis) so a reading hovering near one threshold
// can't flicker the state: must rise above ON to turn on, fall below OFF to
// turn off.
//   counts = volts / 3.3 * 4095

static const int SWITCH_ON_COUNTS  = 745; // ~0.60 V at the pin (~6 V at the input)
static const int SWITCH_OFF_COUNTS = 496; // ~0.40 V at the pin (~4 V at the input)

// --- injection mode ----------------------------------------------------------------
// Sequential injection (rusEFI on the M54): each injector fires once per
// engine cycle = once per two crank revolutions -> 1 event per cycle. The
// fuel calculation (fuel_calc.cpp) needs this to turn a pulse width into a
// flow rate. (Injector DUTY is not computed by the gauge - rusEFI broadcasts
// its own figure in BASE1.)

static const float INJ_EVENTS_PER_ENGINE_CYCLE = 1.0f;

// --- alarm thresholds ---------------------------------------------------------------
// Transcribed from the "Warning Thresholds" section of the project plan -
// keep the two in sync. Values marked TUNE have no number in the plan and
// were picked as sane starting points.
//
// Each threshold has a matching hysteresis constant: an alarm that turned ON
// at a threshold only turns OFF once the value has retreated past the
// threshold by the hysteresis amount. Without this, a value hovering exactly
// at the limit would flick the alarm on and off every tick.

// The engine counts as "running" above this rpm. Gates the alarms that are
// meaningless with the engine off (oil pressure, alternator voltage, AFR) -
// otherwise you would sit in the pits staring at a red OIL LOW.
static const uint16_t ENGINE_RUNNING_RPM = 400;

// Petrol stoichiometric AFR. Only used to turn the MS dash frame's target AFR
// back into lambda, so it must equal the factor rusEFI multiplied by when it
// sent it: rusEFI's STOICH_RATIO, 14.7f (rusefi_types.h) - NOT the "stoich
// ratio" setting in the tune.
static const float AFR_STOICH = 14.7f;

// oil pressure (psi). "Ratio" = rpm / psi: pressure demand scales with rpm,
// so 4000 rpm at 30 psi (ratio 133) is marginal while idle at 15 psi is fine.
static const float OILP_LOW_WARN_PSI  = 10.0f;
static const float OILP_LOW_CRIT_PSI  = 8.0f;
static const float OILP_HIGH_WARN_PSI = 80.0f;
static const float OILP_RATIO_WARN    = 133.0f; // rpm per psi
static const float OILP_RATIO_CRIT    = 250.0f;
static const float OILP_HYST_PSI      = 2.0f;
static const float OILP_RATIO_HYST    = 15.0f;

// oil temperature (deg C, rusEFI's Oil Temperature sensor via BASE4)
static const float OILT_COLD_C = 75.0f;
static const float OILT_WARN_C = 130.0f;
static const float OILT_CRIT_C = 140.0f;

// coolant temperature (deg C)
static const float CLT_COLD_C = 70.0f;
static const float CLT_WARN_C = 100.0f;
static const float CLT_CRIT_C = 115.0f;

static const float TEMP_HYST_C = 3.0f; // shared by all temperature alarms

// lambda vs target (judged on a TWO-SECOND average of lambda - target per the
// plan; + = lean). The average and the rules for which samples may enter it
// live in mixture.cpp - see the "mixture window" block right below.
static const float LAMBDA_RICH_DELTA    = 0.10f; // this much below target -> blue info
static const float LAMBDA_LEAN_WARN     = 0.03f; // above target -> yellow
static const float LAMBDA_LEAN_CRIT     = 0.05f; // above target AND under load for the
                                                 // whole window -> red. "Load" =
static const float LAMBDA_CRIT_MIN_MAP  = 90.0f; //   MAP above this (kPa, the plan's rule)
static const float LAMBDA_CRIT_MIN_THROTTLE_PCT = 70.0f; // OR pedal/TPS at least this: a
                                                 // naturally aspirated engine at altitude
                                                 // never sees 90 kPa (barometric pressure
                                                 // is ~84 kPa at 1500 m), so MAP alone would
                                                 // disarm the red on mountain stages. Set
                                                 // above 100 to restore the MAP-only rule.
static const float LAMBDA_HYST          = 0.01f;

// --- mixture window (mixture.cpp) ---------------------------------------------------
// A wideband only tells the truth while the engine burns what the ECU
// meters. On overrun it doesn't: a fuel cut (DFCO, rev limiter) shows the
// sensor free air, and closed-throttle misfires push unburnt oxygen past it.
// An average that swallows a lift-off reads LEAN for seconds afterward - the
// old averaging turned every overrun into a yellow AFR LEAN, and into a RED
// one the moment the throttle reopened (MAP > 90 kPa with the average still
// full of overrun). So a sample only enters the window while the driver is
// asking for power, the ECU isn't cutting fuel, and the sensor has had time
// to recover from the last time that wasn't true. Full rules: mixture.cpp.
//
// Price of that honesty: once the throttle opens, a real lean condition can
// reach the alarm table no sooner than HOLDOFF + WINDOW (3 s here) plus the
// alarm's own set time.
static const uint32_t LAMBDA_AVG_WINDOW_MS    = 2000; // the plan's two seconds; a multiple of
                                                      // TICK_FAST_MS (one sample per tick)
static const float    LAMBDA_MIN_THROTTLE_PCT = 5.0f; // pedal (TPS without a pedal sensor) at or
                                                      // above this = asking for power. TUNE: keep it
                                                      // above the tune's DFCO TPS threshold
                                                      // (coastingFuelCutTps), so every fuel-cut
                                                      // lift falls below it
static const uint32_t LAMBDA_HOLDOFF_MS       = 1000; // after any excluded sample, ignore this long:
                                                      // exhaust transport + sensor lag + the intake
                                                      // fuel film refilling after a cut. TUNE from a
                                                      // log of a lift-and-reapply
static const float    LAMBDA_SENSOR_MIN       = 0.5f; // below this is not a reading: rusEFI sends
                                                      // lambda 0 while the wideband warms up or has
                                                      // failed (and a target of 0 before it has one)

// 12V system voltage (as the ECU measures it)
static const float VOLT_LOW_WARN_V  = 14.0f; // alternator should hold >= this when running
static const float VOLT_LOW_CRIT_V  = 13.2f;
static const float VOLT_HIGH_WARN_V = 15.5f; // TUNE: plan lists VLT HIGH but no number;
                                             // set vs the LTO pack's max charge voltage
static const float VOLT_HYST_V      = 0.2f;

// battery pack cells (from the BMS)
static const float CELL_BAL_WARN_V = 0.10f; // max-min cell spread
static const float CELL_LOW_CRIT_V = 2.0f;  // any (= the minimum) cell below this
static const float CELL_V_HYST     = 0.03f;
static const float BATT_HOT_WARN_C = 50.0f; // hottest cell
static const float BATT_HOT_CRIT_C = 65.0f;
static const float BATT_COLD_C     = 5.0f;  // TUNE: informational blue only

// fuel level (percent). There are TWO estimates: the slosh-filtered sender
// and the injector-integration calculation. Either one below the warn line
// raises yellow. The red NO GAS is stricter about the calculated value: a
// calc-only critical is VETOED while a healthy sender reads above
// FUEL_SENDER_VETO_PCT - if the float insists a quarter tank is left, the
// most likely story is a missed fill gesture, not an empty tank, and a
// false red teaches the driver to ignore red. (A sender-driven critical is
// never vetoed by the calc - the sender is the physical measurement.)
static const float FUEL_WARN_PCT        = 20.0f;
static const float FUEL_CRIT_PCT        = 10.0f;
static const float FUEL_PCT_HYST        = 3.0f;
static const float FUEL_SENDER_VETO_PCT = 25.0f;

// --- audio: DFPlayer Mini -----------------------------------------------------------
// The module speaks 9600-baud serial on Serial7: pin 29 (TX7 -> module RX)
// and pin 28 (RX7 <- module TX), per the pinout table ("MP3 Serial TX/RX").
// It moved off Serial3 (pins 14/15) in the Sept 2026 pin shuffle so those
// two analog-capable pins could take the brights/headlights inputs.
// Driver in audio.cpp.

static HardwareSerial& AUDIO_SERIAL = Serial7;

// Volume on the DFPlayer's 0..30 scale: baseline (boot greeting + yellow
// warnings) at 70%, red alarms at 85%. TUNE on the car against exhaust
// drone at stage pace.
static const uint8_t AUDIO_VOL_NORMAL = 21; // 70% of 30
static const uint8_t AUDIO_VOL_RED    = 26; // 85% of 30

// The module needs ~1.5-3 s after power-up to mount its SD card. It
// announces readiness itself (an "online" frame the driver listens for);
// this deadline is the fallback after which arbitration runs regardless,
// so a missing module still lets the serial prints show what WOULD play.
static const uint32_t AUDIO_BOOT_GRACE_MS = 4000;

// Clip end is normally the module's "track finished" notification; this
// timeout is the fallback when that frame never arrives (lost byte, or no
// module wired at all). Long enough that no real clip gets its tail talked
// over, short enough that a lost frame doesn't gag the voice for long.
static const uint32_t AUDIO_CLIP_TIMEOUT_MS = 6000;

// Ignore a "finished" frame arriving sooner than this after a play command:
// clone chips report the track a play command CUT OFF as finished, right
// after the new clip starts - that must not count as the new clip ending.
// No real voice clip is shorter than this.
static const uint32_t AUDIO_FINISH_GUARD_MS = 400;

// --- LED tachometer: WS2812B strip ---------------------------------------------------
// One 15-LED WS2812B strip (96/m, "v6" silicon = accepts our 3.3V data line
// directly), reflected onto the windshield. Driven by the WS2812Serial
// library on pin 8 ("LED Serial" in the pinout = Serial2 TX): the library
// encodes the LED bitstream into serial framing and a DMA engine clocks it
// out on its own, so repainting the strip costs the superloop microseconds.
// Repainted on the 20 ms tick = 50 Hz (rpm itself updates at the rusEFI CAN
// data period - see docs/ECU_CAN_SETUP.md).
//
// Strip layout (logical index 0 = LEFT end):
//
//    [ 0  1  2 ]   [ 3  4  5  6  7  8  9 10 | 11 12 13 14 ]
//     status LEDs    tach: 8 green            4 orange
//
// All behavior/priority logic lives in led_tach.cpp; every number it uses
// lives here.

static const uint8_t PIN_LED_STRIP = 8;

static const uint8_t LED_COUNT       = 15;
static const uint8_t LED_STATUS_LEDS = 3;  // left block: vehicle status
static const uint8_t LED_TACH_LEDS   = 12; // right block: the tachometer

// If the strip ends up physically mounted with its input wire on the RIGHT,
// flip this instead of rewiring - it mirrors the whole strip in software.
static const bool LED_STRIP_REVERSED = false;

// Tach scale: tach LED k (k = 0..11) lights once rpm reaches
//     TACH_START_RPM + k * TACH_STEP_RPM
// -> first LED at 3500, last LED at 3500 + 11*375 = 7625 rpm.
static const uint16_t TACH_START_RPM = 3500;
static const uint16_t TACH_STEP_RPM  = 375;

// The first this-many tach LEDs are green, the rest orange.
static const uint8_t TACH_GREEN_LEDS = 8;

// Shift light: at this rpm the whole 12-LED tach flashes - blue by day,
// orange at night (a blue flare off the windshield wrecks night vision).
static const uint16_t SHIFT_FLASH_RPM = 7800;

// Staged warmup rev limits, keyed on OIL temperature - viscosity is what
// we are protecting, and oil warms far slower than coolant. Tach LEDs whose
// rpm sits above the current stage's limit show solid blue: the "cold band"
// of a factory BMW tachometer, shrinking as the oil warms. Display-only -
// the ECU enforces nothing from here.
//
// The table must stay ordered coldest -> warmest. A stage is entered the
// moment oil temp reaches minOilTempC, but only left downward again once
// the temp has fallen WARMUP_STAGE_HYST_C below it, so a reading dithering
// on a boundary can't flick an LED between blue and live.
struct WarmupStage
    {
    float    minOilTempC; // stage applies from this oil temperature upward
    uint16_t rpmLimit;    // LEDs above this rpm show the blue block
    };

static const WarmupStage WARMUP_STAGES[] =
    {
    { -999.0f, 4000 }, // cold (below 50 C)
    {   50.0f, 5000 }, // warmup
    {   70.0f, 6500 }, // nearly there
    {   80.0f, 8000 }, // full send - above the top LED, so no blue at all
    };
static const uint8_t WARMUP_STAGE_COUNT  = sizeof(WARMUP_STAGES) / sizeof(WARMUP_STAGES[0]);
static const float   WARMUP_STAGE_HYST_C = 3.0f;

// Colors as 0xRRGGBB. Note the two oranges are red-heavy mixes: an even
// red+green "orange" reads as yellow once it bounces off glass.
static const uint32_t LED_COLOR_TACH_GREEN  = 0x00FF00;
static const uint32_t LED_COLOR_TACH_ORANGE = 0xFF5000;
static const uint32_t LED_COLOR_WARMUP_BLUE = 0x0000FF;
static const uint32_t LED_COLOR_SHIFT_DAY   = 0x0000FF; // blue
static const uint32_t LED_COLOR_SHIFT_NIGHT = 0xFF5000; // orange
static const uint32_t LED_COLOR_STATUS_OK   = 0x00FF00; // the single green
static const uint32_t LED_COLOR_STATUS_WARN = 0xFFD000; // yellow, x3
static const uint32_t LED_COLOR_ALERT_RED   = 0xFF0000;

// Global brightness (0..255), applied to every color above at paint time.
// One level per brightness mode on the dial; every OTHER mode (Silent,
// Fuel Reset, Debug, unassigned) defaults to MEDIUM. TUNE all three on the
// car - and note perception is nonlinear: 140/255 looks much brighter than
// "half", which is why DARK sits so low.
static const uint8_t LED_BRIGHTNESS_BRIGHT = 255;
static const uint8_t LED_BRIGHTNESS_MEDIUM = 140;
static const uint8_t LED_BRIGHTNESS_DARK   = 60;

// Flash half-periods (light is ON this long, then OFF this long). Keep them
// multiples of TICK_FAST_MS: the strip repaints on the 20 ms tick, so a
// half-period that isn't a tick multiple flashes visibly unevenly.
static const uint32_t LED_RED_FLASH_HALF_MS   = 240; // ~2 Hz whole-gauge red
static const uint32_t LED_SHIFT_FLASH_HALF_MS = 60;  // ~8 Hz shift light

// --- display backlights: low-side N-FET switches, phase-staggered PWM -----------------
// Circuit, per screen (Schematic Ref.pdf, sheet 2; the schematic's AO3400A
// and the BOM's AOSS32334C are interchangeable SOT-23 logic-level parts):
//
//   +5V --> backlight LED (3.0 V, 5 chips in parallel) --> [R8 21R] --> drain
//   DisplayN.PWM (Teensy pin) --[R3 470R]--+-- gate           source --> GND
//                                          |
//                                    [R2 100k]
//                                          |
//                                         GND
//
// A HIGH pin turns the FET on and the LED lights: (5 - 3.0 - ~0) / 21R =
// ~95 mA, the panel's rated current, set by the ballast resistor. A fixed
// current while on means light output tracks duty cycle linearly - the
// same physics as the WS2812 strip, so the three SCREEN_BRIGHTNESS_*
// levels can mirror the LED ones. The 100k keeps the gate low (backlights
// OFF) while the Teensy is resetting, until backlightInit() takes over.
//
// 3.3 V gate drive is plenty: the AOSS32334C's Vgs(th) is 1.3-2.3 V and
// its transfer curve carries amps at Vgs = 3 V (datasheet fig. 2); we ask
// for 0.1 A, so the FET sits deep in its ohmic region and drops millivolts.
// Ciss ~600 pF / Qg ~6 nC through 470R gives edges of a microsecond or two.
//
// PIN TOPOLOGY - why pin 4 became pin 22. The i.MX RT's FlexPWM modules
// are split into four submodules, each with its own counter and an A/B
// output pair. Pins 2 and 3 are FlexPWM4 submodule 2, channels A and B;
// the old pin 4 was FlexPWM2 - a different module, with no way to lock its
// counter to FlexPWM4's. Pin 22 is FlexPWM4 submodule 0, channel A: same
// module, and submodule 0 is the one that can broadcast its sync to the
// others. backlight.cpp slaves submodule 2 to submodule 0's sync, so all
// three counters run in lockstep and each window can be placed at a chosen
// phase. (The QuadTimer pins - 10-15, 18, 19 - can't do any of this.)

static const uint8_t PIN_BACKLIGHT[3] = { 2, 3, 22 }; // Display 1, 2, 3

// PHASE STAGGER. With analogWrite() every channel's ON window starts at
// counter zero, so all three FETs switch on in the same nanosecond and the
// 5V rail sees a 300 mA step per cycle. Offsetting the windows by a third
// of a period each turns that into three separate 100 mA steps: below 33%
// duty the loads never overlap at all, above it the overlap grows, but the
// STEP the rail sees is always one backlight's worth. That is what "lower
// bus capacitance" is about - the bulk cap has to ride out I x t_on / dV,
// and both the I (100 vs 300 mA) and the t_on (see the frequency note)
// just got smaller. Fractions of the PWM period; set all three to 0 to get
// the old aligned behavior back for an A/B comparison on the scope.
static const float BL_PHASE[3] = { 0.0f, 1.0f / 3.0f, 2.0f / 3.0f };

// PWM FREQUENCY. The BCR405 driver this design replaced wanted 1 kHz (a
// 10 us turn-off delay ate into every cycle); the FET switches in ~1 us and
// has no such constraint, and everything else argues for going well up:
//   * Rail ripple: the 5V bulk cap sees each backlight as I x t_on. At
//     1 kHz that is 100 mA for up to 1 ms = 4.5 V of droop across 22 uF,
//     i.e. the regulator has to supply every pulse itself. At 20 kHz it is
//     50 us = 0.23 V on the same cap, so the 3 x 22 uF already on the
//     board absorb the switching and the TSR 2-2450 sees a DC load. (Its
//     600 uF load-capacitance limit means bulk was never the fix at 1 kHz.)
//   * Peripheral vision: 1 kHz PWM is invisible to a fixed stare but shows
//     as a dotted trail ("phantom array") whenever the eye sweeps across
//     the panel - exactly how this gauge is read, from a vibrating car.
//     Above a few kHz the effect is gone.
//   * Audible noise: X7R caps sing at the PWM rate under a 100 mA chopped
//     load; 1 kHz is the ear's most sensitive band, 20 kHz is outside it.
//   * Cost: ~10 mW of switching loss per FET, 7500 duty steps per period
//     at the 150 MHz bus clock, and a 2 us edge is 4% of a 50 us period
//     (rise and fall cancel to first order). DARK mode's 23% is 12 us of
//     on-time - nowhere near the edges.
// Anything from ~5 to ~30 kHz would work; 20 kHz is the audibility line.
static const float BL_PWM_HZ = 20000.0f;

// PWM polarity. true = a HIGH pin turns the backlight ON - the low-side
// N-FET above. If a board revision ever moves to a high-side P-FET (pin LOW
// = on), flip this instead of respinning: it inverts in software.
static const bool BL_ACTIVE_HIGH = true;

// Brightness per rotary mode, 0..255 = fraction of full output - the SAME
// perceptual scale as the LED tach above, on purpose: both the WS2812s and
// a PWMed constant-current backlight are linear duty-to-light, so the same
// numbers produce the same perceived bright/medium/dark steps on both.
// Displays-off mode forces 0 (handled in backlight.cpp, not a level here).
static const uint8_t SCREEN_BRIGHTNESS_BRIGHT = 255;
static const uint8_t SCREEN_BRIGHTNESS_MEDIUM = 140;
static const uint8_t SCREEN_BRIGHTNESS_DARK   = 60;

// Per-screen trim, 0.00..1.00, multiplied onto the mode level. This is the
// binning knob: LED batches differ in efficiency, so three "identical"
// panels at identical duty can visibly mismatch. TUNE ON THE BENCH: set
// all three to 1.00, put the gauge in Bright, and dim the brighter screens
// down until all three match by eye (the 1 Hz debug print shows the final
// duty each screen is actually getting). Trim only ever REDUCES a screen -
// the dimmest panel stays at 1.00 and defines full brightness.
static const float BL_TRIM[3] = { 1.00f, 1.00f, 1.00f };

// Mode changes fade over this long rather than snapping (a 0->full step at
// night is a flashbang to dark-adapted eyes). Full-scale traverse time;
// smaller steps take proportionally less. Also gives a gentle fade-in on
// boot. 0 = snap instantly.
static const uint32_t BL_FADE_MS = 250;

// --- screens: 3x ER-TFT2.79-1 (NV3007) over SPI ---------------------------------------
// Each display sits alone on its own SPI bus, so all three DMA uploads can
// run at the same time. Pin sets follow the pinout table ("SPI0/1/2" blocks).
// Note every DC pin is a HARDWARE CS pin of its bus - that is deliberate:
// the ILI9341_T4 driver toggles DC mid-stream via the SPI peripheral itself
// (no CPU interrupt per command) only when DC is on a hardware CS pin, and
// falls back to a much slower interrupt-per-toggle mode otherwise. The CS
// pins themselves carry no such requirement - any GPIO works.
//
// MISO is 255 everywhere: the NV3007 has no MISO line at all (write-only
// display), and the port forces this in begin() - see PORTING_NV3007.md.

struct ScreenPins
    {
    uint8_t cs, dc, sck, mosi, rst;
    };

static const ScreenPins SCREEN_PINS[3] =
    {
    {  9, 10, 13, 11,  6 }, // display 1 on SPI  (LPSPI4)
    { 21,  0, 27, 26, 20 }, // display 2 on SPI1 (LPSPI3)
    { 39, 36, 37, 35, 38 }, // display 3 on SPI2 (LPSPI1, the bottom SD pads)
    };

// SPI clock. The NV3007 datasheet rates writes to 100 MHz; the practical
// ceiling is the Teensy + wiring, typically 40-60 MHz. 40 MHz is the proven
// safe start (full frame = ~24 ms; typical differential gauge frames are a
// few percent of that). Try 60 MHz once the real harness exists - if the
// image corrupts, come back down.
static const uint32_t SCREEN_SPI_HZ = 40000000;

// Native landscape geometry (rotation 0), as mounted. See PORTING_NV3007.md
// for why 428x142 is the native orientation and not a rotation.
static const int SCREEN_LX = 428;
static const int SCREEN_LY = 142;

// Diff buffers: the library compares each new frame against what is already
// on the glass and uploads only changed pixels. 8 KB per buffer is generous
// for gauge content (mostly-static frames); two per display lets the next
// diff compute while the previous upload still streams. If printStats()
// ever shows diff overflows, raise this.
static const int SCREEN_DIFF_BUF_BYTES = 8192;

// Diff gap: pixel runs shorter than this between two changed regions get
// merged into one upload. Small = tighter uploads but more command
// overhead; the library docs and the bring-up test both land on ~6-10 for
// this class of content.
static const int SCREEN_DIFF_GAP = 6;

// What each physical display shows. Swap entries if the cluster ends up
// mounted in a different order - nothing else in the firmware cares.
enum ScreenContent : uint8_t
    {
    SCREEN_OIL_VOLTS = 0, // oil temp + volts + the warning banner
    SCREEN_CLT_FUEL,      // coolant temp + fuel % / bar / clock
    SCREEN_PSI_LAMBDA,    // oil pressure + lambda + turn/light icons
    };

static const ScreenContent SCREEN_CONTENT[3] =
    {
    SCREEN_OIL_VOLTS,  // display 1
    SCREEN_CLT_FUEL,   // display 2
    SCREEN_PSI_LAMBDA, // display 3
    };

// The fuel filler arrow (the "Moylan arrow") beside the FUEL label: the
// little arrow every OEM fuel gauge tucks next to the pump symbol so you
// never pull up on the wrong side of the pump. true = filler on the
// passenger/right side (the E30's flap is on the right rear quarter) - the
// arrow sits to the right of the label and points right; false = left.
static const bool FUEL_FILLER_ARROW_RIGHT = true;

// The warning banner flashes on any CRITICAL (red) alarm: 500 ms on,
// 500 ms off = the plan's 1 Hz eye-catcher. Must stay a multiple of
// TICK_FAST_MS (frames repaint on the tick, so a non-multiple half-period
// renders unevenly). Silent mode disables flashing (banner goes solid).
static const uint32_t SCREEN_RED_FLASH_HALF_MS = 500;

// --- screen colors, 0xRRGGBB (same convention as the LED colors above) ----------------
// These get packed to RGB565 (5 bits red / 6 green / 5 blue) at theme-build
// time, once per tick. Values match the bench-tested PC prototype in
// ../Graphics/GaugeClusterPrototype.h; anything marked TUNE was added for
// the alarm-coloring feature and has not met real eyeballs yet.
//
// "Info blue" is SEMANTIC: it is whatever color means "cold / informational"
// right now - dodger blue by day, amber at night (night vision: a bright
// blue element is the single worst color for dark-adapted eyes, same
// reasoning as the LED shift light's night color). Nothing below ever asks
// for "blue" - it asks for COL_INFO and the theme answers.

static const uint32_t COL_SCREEN_BG      = 0x42314A; // = raw565(8,12,9), the prototype bg
static const uint32_t COL_SCREEN_GREEN   = 0x2CFF7A; // nominal readouts + labels
static const uint32_t COL_SCREEN_YELLOW  = 0xFFBE00; // TUNE: warning-state values
static const uint32_t COL_SCREEN_RED     = 0xFF4650; // TUNE: critical-state values
static const uint32_t COL_SCREEN_WHITE   = 0xFFFFFF; // text on blue/red banner fills
static const uint32_t COL_SCREEN_DARKTXT = 0x0A0F0A; // text on yellow/green banner fills
static const uint32_t COL_SCREEN_DIVIDER = 0x14321E; // separator lines + off segments/icons

static const uint32_t COL_INFO_DAY    = 0x1E90FF; // dodger blue
static const uint32_t COL_INFO_NIGHT  = 0xFF7800; // TUNE: amber replacement at night
// The brights icon keeps a dim ghost of its color when off (it sits alone
// in a row of green icons, so a green "off" state would read as ON).
static const uint32_t COL_BRIGHTS_OFF_DAY   = 0x0C2337; // dim blue
static const uint32_t COL_BRIGHTS_OFF_NIGHT = 0x3C1E00; // TUNE: dim amber

// Banner fills per alarm color (text color pairs with each: white on
// blue/red, dark on yellow/green - dark-on-yellow beats white-on-yellow
// for contrast, and red keeps white text because red + dark text muddies).
static const uint32_t COL_BANNER_YELLOW = 0xFFA500; // amber fill, dark text
static const uint32_t COL_BANNER_RED    = 0xEB232D; // red fill, white text
// (blue banner fill = COL_INFO_*, green banner fill = COL_SCREEN_GREEN)

// --- FRAM: FM24V10 1-Mbit I2C F-RAM ---------------------------------------------------
// Nonvolatile memory for two jobs: the persistent fuel record (survives the
// kill switch) and the black-box flight recorder (backup to the ECU's own
// log). On the Wire pins per the pinout table: 18 = SDA, 19 = SCL.
// A1/A2/WP all have internal pulldowns and are left grounded -> device
// address 0x50 (+1 for the upper 64 KB page). SDA/SCL each need an external
// pull-up to 3.3V: 2.2k is right for 1 MHz operation.
//
// The chip is 128 KB, writes at full bus speed with NO write delays (F-RAM,
// not EEPROM), and endures 10^14 writes - writing every second for 30 years
// uses 0.000001% of its life. Wear is a non-problem; the design worries
// about exactly one thing: POWER DYING MID-WRITE (kill switch). Every record
// therefore carries a sequence number + CRC, and nothing is ever the only
// copy of itself. See docs/FRAM_BLACKBOX.md for the full memory map.

static const uint32_t FRAM_I2C_HZ      = 1000000; // FM24V10 F/S-mode tops out at 1 MHz
static const uint8_t  FRAM_I2C_ADDR    = 0x50;    // 1010 A2 A1 (A2=A1=0); bit0 selects the page
static const uint32_t FRAM_SIZE_BYTES  = 131072;  // 128 KB

// Async write-queue geometry (fram.cpp). Worst case per 1 s tick is three
// writes (log record + head pointer + fuel record), so depth 8 is generous.
static const uint8_t  FRAM_QUEUE_DEPTH     = 8;
static const uint8_t  FRAM_MAX_WRITE       = 32;  // largest single record
static const uint32_t FRAM_XFER_TIMEOUT_MS = 5;   // one 34-byte transfer takes ~0.3 ms;
                                                  // 5 ms of silence = the bus is wedged
static const uint8_t  FRAM_MAX_CONSEC_ERRS = 8;   // give up (mark chip absent) after this
                                                  // many back-to-back failures

// Memory map. Everything below FRAM_ADDR_LOG is bookkeeping (~1 KB); the
// rest of the chip - about 127 KB - is the black-box ring.
static const uint32_t FRAM_ADDR_HEADER = 0x0000; // 32 B  format marker + geometry
static const uint32_t FRAM_ADDR_FUEL   = 0x0020; // 16 x 32 B fuel record ring
static const uint8_t  FRAM_FUEL_SLOTS  = 16;
static const uint32_t FRAM_ADDR_LOGPTR = 0x0220; // 2 x 16 B log head pointer (ping-pong)
static const uint32_t FRAM_ADDR_LOG    = 0x0400; // black-box ring, to end of chip

// Black-box cadence. Records are written on the 1 s tick while the engine
// runs (or a real alarm is active - see fram_log.cpp for the gating), every
// LOG_NOMINAL_EVERY_S seconds normally and every LOG_ALARM_EVERY_S once any
// yellow/red alarm is up. At 32 B/record the ~127 KB ring holds 4064
// records: ~2.2 h of nominal running, ~68 min if an alarm is active the
// whole time. After the engine stops (or the alarm clears) logging continues
// for LOG_GRACE_MS - the aftermath of a stall is exactly the data you want -
// then halts so a car left sitting with the ignition on doesn't grind away
// the interesting part of the ring.
static const uint32_t LOG_NOMINAL_EVERY_S = 2;
static const uint32_t LOG_ALARM_EVERY_S   = 1;
static const uint32_t LOG_GRACE_MS        = 30000;

// --- watchdog -------------------------------------------------------------------------
// Hardware watchdog timeout. The 20 ms tick feeds it; if the firmware ever
// hangs for this long the chip hard-resets and the gauge is back in ~1 s.
// Must be a multiple of 500 ms (hardware granularity).

static const uint32_t WATCHDOG_TIMEOUT_MS = 1000;
