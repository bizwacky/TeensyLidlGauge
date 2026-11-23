// ---------------------------------------------------------------------------
// Rally CAN Gauge - the central data model
// ---------------------------------------------------------------------------
// One global struct, g_data, is the single source of truth for everything the
// gauge knows about the car. The dataflow is strictly one-way:
//
//     input modules (can_bus, inputs,  --write-->  g_data  --read-->  consumers
//                    imu + motion)
//                                                            (alarms, audio,
//                                                             screens, LED tach,
//                                                             FRAM log, fan PWM,
//                                                             the CAN frames the
//                                                             gauge sends the PDMs)
//
// Why one big struct instead of values scattered across modules?
//   * Every consumer sees a consistent snapshot - there is exactly one place
//     a value can live, so no stale copies drift apart.
//   * Debugging is trivial: print g_data and you know the whole system state.
//   * The alarm logic, screens and LEDs depend only on this header, not on
//     HOW a value was acquired. Swap the ECU protocol and nothing above the
//     decoder changes (exactly what happened when the gauge moved from the
//     Megasquirt broadcast to rusEFI's own).
//
// Thread-safety note: everything here is written and read from loop() context
// only (the CAN interrupt fills a queue inside FlexCAN_T4; frames are decoded
// when canBusPoll() drains that queue in the superloop). Single writer +
// single thread = no locking, no volatile needed.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include "config.h"

// --- values decoded from the ECU CAN broadcast --------------------------------
// rusEFI's native broadcast (frames BASE0..BASE7, named as in rusEFI's DBC)
// plus one byte of its MS Simplified dash broadcast (the target AFR).
// Layouts: docs/ECU_CAN_SETUP.md. Units are converted at decode time to the
// ones the project thinks in: C, kPa, psi for oil pressure (the plan's
// thresholds are psi), lambda for mixture.
//
// How rusEFI encodes "no valid reading" (it never sends a separate flag):
// coolant/oil/intake temperature 0 C, pedal/TPS -1 %, lambda 0. A dead oil
// pressure sensor decodes as 0 psi (can_bus.cpp explains why).

// Index into EcuData::frameLastRxMs - one slot per ECU frame we decode, so
// staleness can be judged per frame.
enum EcuFrame : uint8_t
    {
    ECU_F_STATUS = 0, // BASE0 - warning counter, last error, status bits
    ECU_F_SPEEDS,     // BASE1 - rpm, timing, injector duty (the "ECU online" heartbeat)
    ECU_F_THROTTLE,   // BASE2 - pedal + throttle position
    ECU_F_SENSORS1,   // BASE3 - MAP, coolant, intake air temperature
    ECU_F_SENSORS2,   // BASE4 - oil pressure, oil temperature, battery voltage
    ECU_F_FUELING,    // BASE5 - injector pulse width
    ECU_F_FUELING2,   // BASE6 - fuel trim, rusEFI's own fuel flow
    ECU_F_LAMBDA,     // BASE7 - measured lambda
    ECU_F_AFR_TARGET, // MS Simplified frame 0x5EA - target AFR (-> lambdaTarget)
    ECU_F_COUNT
    };

// Bits of EcuData::statusBits = byte 4 of BASE0, verbatim (DBC bits 32..39).
static const uint8_t ECU_STATUS_REV_LIMIT      = 0x01; // fuel or spark cut by rusEFI's limiter / limp logic
static const uint8_t ECU_STATUS_MAIN_RELAY     = 0x02;
static const uint8_t ECU_STATUS_FUEL_PUMP      = 0x04;
static const uint8_t ECU_STATUS_CHECK_ENGINE   = 0x08;
static const uint8_t ECU_STATUS_O2_HEATER      = 0x10;
static const uint8_t ECU_STATUS_LAMBDA_PROTECT = 0x20; // rusEFI's lean protection is cutting
static const uint8_t ECU_STATUS_FAN            = 0x40;
static const uint8_t ECU_STATUS_FAN2           = 0x80;

struct EcuData
    {
    // BASE0 - status
    uint16_t warningCount;  // rusEFI's warning counter (counts every warning since boot)
    uint16_t lastError;     // code of the most recent warning/error rusEFI raised
    uint8_t  statusBits;    // ECU_STATUS_* flags

    // BASE1
    uint16_t rpm;           // RPM
    float    sparkAdvDeg;   // deg - ignition timing
    float    injDutyPct;    // %   - injector duty, as rusEFI computes it

    // BASE2 - rusEFI sends -1 % for a sensor it doesn't have or can't trust
    float pedalPct;         // %   - accelerator pedal (< 0 = none, e.g. a cable throttle)
    float tpsPct;           // %   - throttle position

    // BASE3
    float mapKpa;           // kPa - manifold pressure (part of the AFR-critical load rule)
    float cltC;             // C   - coolant temperature
    float matC;             // C   - intake air temperature

    // BASE4
    float oilPressPsi;      // psi - converted from rusEFI's kPa
    float oilTempC;         // C
    float battVolts;        // V   - battery voltage as the ECU sees it

    // BASE5
    float injPwMs;          // ms  - injector pulse width INCLUDING rusEFI's dead-time
                            //       correction (rusEFI computes 0 for a fuel cut)

    // BASE6
    float egoCorPct;        // %   - short-term fuel trim, bank 1
    float fuelFlowGps;      // g/s - rusEFI's own fuel-flow figure (0 if its build
                            //       doesn't compute one) - a cross-check for fuel_calc

    // BASE7 + the MS dash frame
    float lambda;           // measured lambda (0 = no valid reading)
    float lambdaTarget;     // target lambda = target AFR / AFR_STOICH

    // derived by mixture.cpp (the plan's AFR alarms are defined on these,
    // never on instantaneous readings)
    float lambdaErr2s;      // mean of (lambda - target) over the last
                            // LAMBDA_AVG_WINDOW_MS of VALID samples. + = lean.
                            // 0 while no valid samples exist.
    bool  lambdaJudgeable;  // the window is full: the AFR alarms may judge it
    bool  lambdaUnderLoad;  // ...and every sample in it was taken under load
                            // (the AFR LEAN red rule)

    // bookkeeping
    uint32_t frameLastRxMs[ECU_F_COUNT]; // millis() each frame last arrived
                                         // (0 = never; drives staleness/NO CAN)
    uint32_t frameCount;    // decoded ECU frames since boot

    bool revLimited() const  { return statusBits & ECU_STATUS_REV_LIMIT; }
    bool checkEngine() const { return statusBits & ECU_STATUS_CHECK_ENGINE; }

    // The driver's throttle demand: the pedal when the car has one (drive by
    // wire), else the throttle plate. Negative = no trustworthy reading.
    float throttlePct() const { return (pedalPct >= 0.0f) ? pedalPct : tpsPct; }
    };

// --- values decoded from the JK BMS ---------------------------------------------
// Protocol: "PCS CAN-Bus-protocol-DY-low-voltage" (the JK app calls it "Deye",
// protocol 001). Little-endian, 1 s broadcast cycle - the mirror image of the
// ECU protocol in byte order and 50x slower. Frame layouts:
// docs/BMS_CAN_SETUP.md and the PDF in the project root.

// Index into BmsData::frameLastRxMs - one slot per frame type we decode, so
// staleness can be judged per frame (e.g. cell data missing while pack data
// still flows).
enum BmsFrame : uint8_t
    {
    BMS_F_SOC = 0,  // 0x355 - state of charge / health
    BMS_F_PACK,     // 0x356 - pack voltage / current / avg temp (the heartbeat)
    BMS_F_CELLS,    // 0x361 - max/min cell voltage + temperature
    BMS_F_FAULT,    // 0x359 - protection / alarm bitfields
    BMS_F_STATE,    // 0x400 - mode, failure level, cycles, balancing
    BMS_F_MOS,      // 0x110 - charge/discharge MOSFET states
    BMS_F_COUNT
    };

struct BmsData
    {
    // 0x355
    uint16_t socPct;        // %   - state of charge
    uint16_t sohPct;        // %   - state of health

    // 0x356
    float packVolts;        // V   - total pack voltage
    float packAmps;         // A   - discharge positive, charge negative (per spec v2.5)
    float avgCellTempC;     // C

    // 0x361 - the cell-balance alarms live off these two voltages. The Deye
    // protocol reports only max/min, not every cell; that is enough for both
    // planned checks (delta > 0.1 V warning, any cell < 2 V critical).
    float maxCellVolts;     // V
    float minCellVolts;     // V
    float maxCellTempC;     // C  - battery temp alarms use this one (hottest cell)
    float minCellTempC;     // C

    // 0x359 - raw fault bitfields, kept verbatim. No alarm reads them yet -
    // they reach the debug print and nothing else. Bit meanings: Tables 1-7
    // in the protocol PDF.
    uint8_t protection[4];  // bytes 0-3: protection faults (tables 1-4)
    uint8_t alarm[2];       // bytes 4-5: early-warning alarms (tables 5-6)
    uint8_t sysError;       // byte 6:    system errors (table 7)

    // 0x400
    uint8_t  opMode;        // 0 standstill, 1 charge, 2 discharge
    uint8_t  failureLevel;  // 0 none, 1 minor, 2 major - the BMS's own summary
    uint16_t cycleCount;    // charge/discharge cycles
    uint16_t balanceBits;   // bit n = cell n+1 currently being balanced

    // 0x110 byte 7 - MOSFET switch states. Per the (translated) spec bits are
    // "0:open 1:close", i.e. 1 = closed = conducting. VERIFY on the bench -
    // translation of open/closed switch states is a classic trap.
    uint8_t mosStateBits;

    // bookkeeping
    uint32_t frameLastRxMs[BMS_F_COUNT]; // 0 = never received (same idea as the ECU frames)
    uint32_t frameCount;                 // decoded BMS frames since boot

    // convenience accessors (derived, always current with the raw fields)
    float cellDeltaVolts() const  { return maxCellVolts - minCellVolts; }
    bool  chargeMosOn() const     { return mosStateBits & 0x10; }
    bool  dischargeMosOn() const  { return mosStateBits & 0x20; }
    bool  balancing() const       { return balanceBits != 0; }
    bool  anyProtection() const   { return protection[0] || protection[1]
                                        || protection[2] || protection[3] || sysError; }
    bool  anyAlarm() const        { return alarm[0] || alarm[1]; }
    };

// --- fuel: sender measurement + injector-integration estimate --------------------
// TWO independent estimates of the same quantity, kept side by side on
// purpose. The sender is a physical measurement - it can't drift, but it
// sloshes and only resolves ~5% steps. The calculated value integrates the
// ECU's injector pulses - smooth and precise, but it only knows what LEFT
// the tank; fills must be told to it (the FUEL FILL gesture), and any error
// accumulates until the next full-tank reset. Display split (per the plan):
// bar gauge = sender, percentage readout = calculated. The fuel alarms
// cross-check one against the other (see alarms.cpp).

struct FuelData
    {
    // -- sender chain (inputs.cpp) --
    uint16_t rawCounts;     // raw ADC reading (debug/calibration aid)
    float    ohms;          // this tick's sender resistance
    float    ohmsFiltered;  // fast-filtered (2 s) - DEBUG ONLY: shows slosh
    float    ohmsSlosh;     // slosh-filtered (median + 30 s EMA) - the real one
    bool     sloshPrimed;   // slosh chain has a full window; trust levelPct
    float    levelPct;      // 0..100% from ohmsSlosh via the linear map
    bool     faultOpen;     // wiring broken (reads as infinite ohms)
    bool     faultShort;    // wiring shorted to ground

    // -- calculated fuel (fuel_calc.cpp) --
    bool  calcValid;        // false until seeded (FRAM record at boot, else the sender)
    float calcGallons;      // best-estimate fuel on board
    float calcPct;          // calcGallons as % of FUEL_TANK_GAL
    float burnGph;          // smoothed burn rate, gal/hour (0 with engine off)
    float burnedTotalGal;   // lifetime burn (persisted in the FRAM fuel record -
                            // the odometer for calibrating FUEL_CALC_TRIM)

    // -- FUEL FILL gesture progress (fuel_calc.cpp writes; only the serial
    //    debug print shows it so far - no screen element draws it yet) --
    bool    fillActive;     // dial is sitting on the FUEL FILL detent
    uint8_t fillCansAdded;  // 5-gal cans credited during this visit
    bool    fillFullDone;   // full-tank reset applied during this visit
    };

// --- values decoded from the two PDMs ---------------------------------------------
// dingoFW's cyclic status frames (docs/PDM_CAN_SETUP.md; frame table:
// PDM_FRAMES in config.h). can_bus.cpp writes the raw fields; pdm.cpp adds
// the per-output verdicts the alarm rows are built on.

// dingoFW's output states, verbatim from the status frame's nibbles.
static const uint8_t PDM_STATE_OFF         = 0;
static const uint8_t PDM_STATE_ON          = 1;
static const uint8_t PDM_STATE_OVERCURRENT = 2; // off, waiting out its reset time, will retry
static const uint8_t PDM_STATE_FAULT       = 3; // off until the PDM is power cycled

// dingoFW's device states (status frame 0, low nibble of byte 1).
static const uint8_t PDM_DEV_RUN      = 0;
static const uint8_t PDM_DEV_SLEEP    = 1;
static const uint8_t PDM_DEV_OVERTEMP = 2; // board above 55 C; outputs stay on
static const uint8_t PDM_DEV_ERROR    = 3; // fatal: every output off until power cycle

// What pdm.cpp concludes about one output, worst first. This is the word the
// banner prints after the load tag.
enum PdmVerdict : uint8_t
    {
    PDM_OUT_OK = 0,
    PDM_OUT_TRP, // tripped within the last PDM_TRIP_HOLD_MS (and recovered, or retrying)
    PDM_OUT_OPN, // on, but drawing less than its catalogue says it must
    PDM_OUT_FLT, // dingoFW Fault, or stuck in Overcurrent
    };

struct PdmOutput
    {
    // -- raw, from the status frames (can_bus.cpp) --
    uint8_t state;      // PDM_STATE_*
    float   amps;       // output current (0 below the driver's ~0.5 A floor)
    uint8_t ocCount;    // overcurrent trips since the request went on (wraps at 255)
    uint8_t dutyPct;    // PWM duty, when the output is PWMed

    // -- derived each tick (pdm.cpp) --
    bool       valid;   // the frames carrying this output's state + current are fresh
    PdmVerdict verdict; // PDM_OUT_OK while !valid: stale data raises nothing
    bool       stuck;   // spent PDM_OC_STUCK_MS of the last PDM_OC_WINDOW_MS in Overcurrent
    uint16_t   trips;   // trips counted since boot (for the debug print)
    };

struct PdmData
    {
    // status frame 0
    uint8_t digInBits;      // the box's own digital inputs 1-2 (bit 0, bit 1)
    uint8_t deviceState;    // PDM_DEV_*
    uint8_t boardType;      // 0 dingoPDM, 1 dingoPDM-Max (the fork may add its own)
    float   totalAmps;
    float   battVolts;
    float   boardTempC;

    PdmOutput out[PDM_MAX_OUTPUTS];

    // bookkeeping, one slot per row of PDM_FRAMES
    uint32_t frameLastRxMs[PDM_FRAME_COUNT]; // 0 = never received
    uint32_t frameCount;                     // decoded status frames since boot
    };

// --- lamp states, resolved from the PDMs ------------------------------------------
// What the indicator icons draw and what flips the gauge to night colors.
// pdm.cpp ORs the PDM outputs listed in config.h (LAMP_*) each tick. When a
// source box is stale its lamps read OFF - an icon must not freeze lit on
// dead data - but nightHint keeps its last known value: losing a CAN link
// in the dark must not flip every screen back to daytime blue.

struct LampData
    {
    bool headlights; // a dipped lamp set is on
    bool brights;    // a main-beam / auxiliary set is on
    bool turnL;      // carries the flasher's on/off wave, so drawing the raw
    bool turnR;      //  state IS the blink (gauge_render.cpp)
    bool nightHint;  // headlights, held through a stale link (modes.cpp)
    };

// --- roof switch panel + pots ------------------------------------------------------

struct PotData
    {
    uint16_t rawCounts; // raw ADC reading (calibration aid)
    float    ohms;      // rheostat resistance this tick
    float    pct;       // 0..100 % of travel: filtered, end-deadbanded, and only
                        //  republished when it has moved POT_REPORT_STEP. While
                        //  the pot reads open this is its failsafe value.
    bool     active;    // above the on/off threshold (with hysteresis): the
                        //  load this pot controls is wanted
    bool     open;      // wire open for POT_OPEN_MS: pct is the failsafe
    };

// --- driver switchgear -----------------------------------------------------------

struct SwitchData
    {
    bool    roof[ROOF_SWITCH_COUNT]; // debounced; true = that switch is ON
    uint8_t roofBits;                // the same, packed: bit 0 = SW1 ... bit 5 = SW6

    PotData pot[POT_COUNT];

    uint8_t  rotaryPos;       // 0..11, debounced. While the dial reads OPEN
                              //  it holds the last good position, and after
                              //  ROTARY_OPEN_MS falls back to
                              //  ROTARY_FAILSAFE_POS (inputs.cpp)
    uint16_t rotaryRawCounts; // raw ADC (debug/diagnosis of a dirty ladder)
    float    rotaryOhms;      // wiper-to-ground resistance the ladder math
                              //  derived this tick (compare ROTARY_OHMS)
    bool     rotaryOpen;      // debounced: the wiper reads open-circuit
                              //  (broken wire / dead switch). The DIALFAIL
                              //  alarm adds persistence on top of this.
    uint32_t rotaryChangedMs; // millis() of the last accepted position change
                              //  (also restarted when the dial recovers from
                              //  an open - see inputs.cpp)

    // How long the switch has sat in the current position - the fuel-fill
    // reset gesture ("hold position X for 30/60 s") builds on this.
    uint32_t rotaryHeldMs() const { return millis() - rotaryChangedMs; }
    };

// --- resolved gauge mode -----------------------------------------------------------
// The rotary dial position, translated into DECISIONS by modesUpdate()
// (src/modes.cpp) each tick. Consumers (LEDs, audio, screens, backlights)
// read these flags and never the raw position: what "position 4" means
// lives in exactly one table (ROTARY_MODE_MAP in config.h), and a future
// remap of the dial can't leave a consumer behind.

struct ModeData
    {
    GaugeMode mode;             // this tick's entry from ROTARY_MODE_MAP
    bool      silent;           // no voice clips, no flashing alarms
    bool      displaysOff;      // all LEDs dark, screens black, backlights off
    bool      night;            // use the night color scheme (dim colors, no blue
                                //  glare): headlights on (as the PDMs report
                                //  them), or Dark mode selected
    uint8_t   ledBrightness;    // 0..255 global LED scale for this tick
    uint8_t   labelBrightness;  // 0..255 level for the switch-label backlights
                                //  on the end of the LED chain (led_tach.cpp)
    uint8_t   screenBrightness; // 0..255 backlight level for this tick (before
                                //  per-screen trim - that lives in backlight.cpp)
    };

// --- gauge-board housekeeping -----------------------------------------------------

struct BoardData
    {
    float   cpuTempC;       // Teensy die temperature (built-in monitor)
    bool    clockValid;     // false until the RTC has a plausible date set
    uint8_t hh, mm, ss;     // wall-clock time from the battery-backed RTC
    };

// --- FRAM status ------------------------------------------------------------------
// Written by fram.cpp / fram_log.cpp; read by the debug print (and one day a
// screen corner). Like everything else in g_data these are OBSERVATIONS -
// all decisions about what/when to write live in fram_log.cpp.

struct FramData
    {
    bool     present;     // chip answered on the I2C bus at boot
    bool     mounted;     // header checked out; fuel + log state recovered
    bool     logging;     // black box is currently recording (gating verdict)
    uint8_t  session;     // this boot's run number (stamps every log record)
    uint32_t logSeq;      // sequence number the NEXT log record will carry
    uint32_t logRecords;  // records currently readable in the ring
    uint32_t logCapacity; // ring size in records (set at mount)
    uint32_t fuelSaves;   // fuel records written since boot
    uint32_t writesDone;  // completed async writes since boot
    uint32_t writeErrors; // failed transfers (NACK/timeout) - should stay 0
    uint32_t dropped;     // writes refused because the queue was full - a
                          // nonzero value means the pump isn't keeping up
    };

// --- cabin fan PWM (fans.cpp) -------------------------------------------------------

struct FanData
    {
    uint8_t dutyPct[FAN_COUNT]; // speed being commanded on each PWM pin, 0..100 %
    };

// --- the last PDM output event, for the black box ----------------------------------
// pdm.cpp records every change for the worse on any output (a trip, an open
// load, a fault); fram_log.cpp writes the most recent one into its next
// record and clears 'pending'. If several land in the same second the worst
// verdict wins - the log is a backup, not a sequence-of-events recorder.

struct PdmEventData
    {
    bool     pending;  // not yet written to the log
    uint8_t  pdm;      // PDM_E / PDM_L
    uint8_t  output;   // 0-based
    uint8_t  verdict;  // PdmVerdict
    float    amps;     // a trip: the highest current seen while the output was
                       //  on; otherwise the current when the verdict changed
    uint16_t count;    // events since boot
    };

// --- IMU: motion, attitude, impacts ------------------------------------------------
// From the LSM6DSV320X (imu.cpp drains its FIFO; motion.cpp turns samples
// into everything below). The vehicle frame is X forward, Y left, Z up, and
// every acceleration is what an accelerometer feels ("specific force"), in
// g: parked on level ground the car reads lon 0, lat 0, vert +1.
//
//   lon   + = speeding up        - = braking
//   lat   + = turning left       - = turning right
//   vert  + = pressed into the seat (1 at rest, 0 in free fall)
//   roll  + = right side down    pitch + = nose up    yaw rate + = to the left
//
// Two families of acceleration values: the SMOOTH ones (lonG...) come from
// the low-g channel and are for the g-meter; the PEAK ones (peakG...) come
// from the high-g channel and hold the largest reading of the last 0.2 s,
// which is what makes a millisecond-long hit loggable (config.h, PEAK HOLD).

// Summary of an impact capture: the one being recorded, else the last one.
struct ImuEventData
    {
    uint32_t id;         // 0 = none yet. Counts on from the newest capture in
                         //  the FRAM, so ids are unique across power cycles
    bool     active;     // its window is still running: the numbers below
                         //  can still grow
    float    peakG;      // largest total acceleration in the capture
    float    peakLonG;   // per axis: the reading of largest size, with its sign
    float    peakLatG;
    float    peakVertG;
    float    dvLonMps;   // delta-v, m/s: what the hit changed the car's speed by
    float    dvLatMps;
    float    dvVertMps;
    uint16_t overMs;     // time spent at or above the trigger level
    uint16_t airMs;      // air time that ended in this hit (0 = not a landing)
    };

struct ImuData
    {
    // -- the chip (imu.cpp) --
    bool     present;       // answered WHO_AM_I and took its configuration
    uint32_t lastDataMs;    // millis() of the last sample batch (0 = never)
    uint32_t hgSamples;     // high-g samples received since boot
    float    hgRateHz;      // measured over the last second: should read ~960
    uint32_t busErrors;     // failed I2C transfers - should sit at 0
    uint32_t fifoOverruns;  // times the chip's FIFO filled before we drained it
                            //  (samples lost) - should sit at 0
    uint32_t tagErrors;     // FIFO words with a tag nothing here expects
    uint16_t fifoPeakWords; // deepest the FIFO has been found since the last
                            //  debug print (it holds about 256)

    // -- calibration (motion.cpp) --
    bool  levelCal;         // a LEVEL SET has been done (else roll and pitch
                            //  include the gauge's mounting angle)
    bool  hgTrimmed;        // the high-g offset trim has settled
    float hgOffsetG[3];     // that trim, sensor axes X/Y/Z, g

    // -- smooth motion, low-g channel + gyro --
    float lonG, latG, vertG;
    float rollDeg, pitchDeg;
    float yawRateDps;
    bool  still;            // nothing is moving (gyro zero is being learned)

    // -- g-meter maxima since the LEVEL SET detent was last visited --
    float maxAccelG, maxBrakeG, maxLeftG, maxRightG;

    // -- peak-held impact channel, high-g --
    float peakG;            // largest total acceleration of the last 0.2 s
    float peakLonG;         // per axis: the reading of largest size in the
    float peakLatG;         //  same 0.2 s, with its sign
    float peakVertG;
    float maxPeakG;         // largest total acceleration since the LEVEL SET
                            //  detent was last visited

    // -- jumps --
    bool     airborne;      // in free fall right now
    uint16_t jumpCount;     // jumps since boot
    uint16_t lastAirMs;     // air time of the last one
    float    lastLandG;     // its landing: largest total acceleration

    // -- impact captures --
    ImuEventData event;

    // -- the LEVEL SET gesture (published for the screens) --
    bool    levelSetActive;   // dial is on the LEVEL SET detent
    bool    levelSetWaiting;  // ...but the car is moving (or the IMU is down)
    uint8_t levelSetPct;      // hold progress, 0..100
    bool    levelSetDone;     // captured during this visit
    };

// --- the warning/alarm system --------------------------------------------------------
// The alarm catalogue: the project plan's "Text warnings" list, re-ordered
// and extended with the PDM rows by the Nov 2026 alarm structure
// (gauge-alarm-structure-pdm.md). The enum is ordered BY PRIORITY (most
// important first), so "scan the array from 0 and take the first active
// entry" IS the priority arbitration - no sorting needed anywhere. Names,
// colors and behavior live in the definition table in alarms.cpp, in this
// exact order. The numbers in the comments are the catalogue's row numbers.
//
// The four rules the order encodes:
//   red     the crew must act NOW and acting changes the outcome
//   yellow  "live": change how you drive or what is switched on
//   yellow  "deferred": nothing to do until the car stops
//   blue    information
// (Hiding the deferred and blue rows at stage pace - the "pace gate" - is
// NOT implemented: that was deferred on 21 Nov 2026. The table carries each
// row's tier so the gate is a small change when it comes.)

enum AlarmColor : uint8_t
    {
    ALARM_GREEN = 0, // status (cranking)
    ALARM_BLUE,      // informational (cold / rich)
    ALARM_YELLOW,    // warning - needs attention
    ALARM_RED        // critical - needs action NOW
    };

enum AlarmTier : uint8_t
    {
    TIER_LIVE = 0,   // worth showing at any speed
    TIER_DEFERRED,   // could wait for the car to stop (see the note above)
    };

enum AlarmId : uint8_t
    {
    ALARM_OIL_LOW_CRIT = 0, //  1 OIL LOW  red
    ALARM_CLT_HOT_CRIT,     //  2 CLT HOT  red
    ALARM_AFR_LEAN_CRIT,    //  3 AFR LEAN red
    ALARM_OIL_HOT_CRIT,     //  4 OIL HOT  red
    ALARM_ENGINE_PWR_CRIT,  //  5 PUMP/COIL/MAIN + FLT/OPN red: an engine-class
                            //    PDM output is faulted, stuck tripped, or open
    ALARM_PDME_ERR,         //  6 PDME ERR red: PDM-E reports device Error
    ALARM_PDME_HOT_CRIT,    //  7 PDME HOT red: PDM-E within 5 K of its thermal latch
    ALARM_VOLT_LOW_CRIT,    //  8 VOLT LOW red
    ALARM_CELL_LOW_CRIT,    //  9 CELL LOW red
    ALARM_BATT_HOT_CRIT,    // 10 BATT HOT red
    ALARM_NO_GAS,           // 11 NO GAS   red
    ALARM_OIL_LOW_WARN,     // 12 OIL LOW  yellow
    ALARM_ENGINE_PWR_TRIP,  // 13 PUMP/COIL/MAIN + TRP yellow: an engine-class
                            //    output tripped (says the stumble was electrical)
    ALARM_VOLT_LOW_WARN,    // 14 VOLT LOW yellow
    ALARM_CLT_HOT_WARN,     // 15 CLT HOT  yellow
    ALARM_OIL_HOT_WARN,     // 16 OIL HOT  yellow
    ALARM_FAN_FAULT,        // 17 FAN1/FAN2 + TRP/FLT/OPN yellow
    ALARM_PDM_HOT_WARN,     // 18 PDME HOT / PDML HOT yellow: within 15 K of the latch
    ALARM_VOLT_HIGH_WARN,   // 19 VLT HIGH yellow
    ALARM_VISION_FAULT,     // 20 lamp / wiper / defrost tag + TRP/FLT yellow
    ALARM_PDML_ERR,         // 21 PDML ERR yellow: PDM-L reports device Error
    ALARM_NO_CAN,           // 22 NO CAN   yellow (ECU link lost)
    ALARM_PDM_CAN,          // 23 PDME CAN / PDML CAN yellow (PDM status stale)
    ALARM_AFR_LEAN_WARN,    // 24 AFR LEAN yellow, deferred
    ALARM_CELL_BAL,         // 25 CELL BAL yellow, deferred
    ALARM_BATT_HOT_WARN,    // 26 BATT HOT yellow, deferred
    ALARM_OIL_HIGH_WARN,    // 27 OIL HIGH yellow, deferred
    ALARM_FUEL_LOW,         // 28 FUEL LOW yellow, deferred
    ALARM_BODY_FAULT,       // 29 body-class tag + TRP/FLT yellow, deferred
    ALARM_BATT_CAN,         // 30 BATT CAN yellow, deferred (BMS link lost)
    ALARM_DIAL_FAIL,        // 31 DIALFAIL yellow, deferred (rotary dial reads open)
    ALARM_BATT_COLD,        // 32 BATTCOLD blue
    ALARM_CRANKING,         // 33 CRANKING green
    ALARM_AFR_RICH,         // 34 AFR RICH blue
    ALARM_CLT_COLD,         // 35 CLT COLD blue
    ALARM_OIL_COLD,         // 36 OIL COLD blue
    ALARM_COUNT,
    ALARM_NONE = 255        // "nothing active" = NOMINAL
    };

// The alarm system's OUTPUT, written by alarmsUpdate() each tick. Consumers
// (screens, LED tach, audio) read only this - they never re-evaluate
// thresholds themselves.
struct AlarmsData
    {
    bool    active[ALARM_COUNT]; // fully-latched states (persistence applied)
    uint8_t top;                 // highest-priority active AlarmId, or ALARM_NONE
    bool    anyRed;              // fast summaries for flash/silent-mode logic
    bool    anyYellow;
    };

// --- the whole car ----------------------------------------------------------------

struct VehicleData
    {
    EcuData    ecu;
    BmsData    bms;
    PdmData    pdm[PDM_COUNT]; // indexed by PdmId (config.h): PDM_E, PDM_L
    FuelData   fuel;
    SwitchData sw;
    LampData   lamps;
    FanData    fans;
    ModeData   mode;
    BoardData  board;
    AlarmsData alarms;
    PdmEventData pdmEvent;
    FramData   fram;
    ImuData    imu;

    uint32_t unknownFrameCount; // standard-id CAN frames nothing here expects -
                                // should sit at 0 on a healthy bus; climbing
                                // means a config mismatch or a stranger
    uint32_t ignoredFrameCount; // traffic we recognise but don't use (the other
                                // rusEFI/MS/BMS frames, the wideband, 29-bit
                                // ids) - climbs steadily, that's normal

    // Staleness helpers. NO CAN = !ecuDataComplete(), BATT CAN = !bmsOnline().
    bool ecuFrameFresh(uint8_t frame, uint32_t maxAgeMs = ECU_FRESH_MS) const;
    bool ecuOnline() const;           // heartbeat only: the rpm frame is arriving
    bool ecuDataComplete() const;     // EVERY ECU frame the gauge decodes is fresh
    uint8_t ecuFirstStaleFrame() const; // an EcuFrame, or ECU_F_COUNT if none
    bool bmsFrameFresh(uint8_t frame, uint32_t maxAgeMs) const;
    bool bmsOnline() const;

    // PDM staleness, per row of PDM_FRAMES. PDME CAN / PDML CAN = !pdmOnline().
    bool pdmFrameFresh(uint8_t pdm, uint8_t row) const; // that status frame is arriving
    bool pdmFrameSeen(uint8_t pdm, uint8_t row) const;  // ...has arrived at least once
    bool pdmOnline(uint8_t pdm) const; // every frame the box owes us is fresh

    // The IMU is configured and samples arrived within IMU_FRESH_MS. Every
    // consumer of g_data.imu asks this first: stale motion data draws as "--".
    bool imuOnline() const;
    };

// Short name of an ECU frame for debug prints ("BASE4", "MS-AFRTGT").
const char* ecuFrameName(uint8_t frame);

// "PDME" / "PDML" - the 4-character box names the banner and prints use.
const char* pdmName(uint8_t pdm);

// The one and only instance, defined in vehicle_data.cpp. As a global it is
// zero-initialized before setup() runs - every value starts at a known 0.
extern VehicleData g_data;
