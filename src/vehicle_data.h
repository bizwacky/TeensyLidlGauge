// ---------------------------------------------------------------------------
// Rally CAN Gauge - the central data model
// ---------------------------------------------------------------------------
// One global struct, g_data, is the single source of truth for everything the
// gauge knows about the car. The dataflow is strictly one-way:
//
//     input modules (can_bus, inputs)  --write-->  g_data  --read-->  consumers
//                                                            (alarms, audio,
//                                                             screens, LED tach,
//                                                             FRAM log)
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

// --- driver switchgear -----------------------------------------------------------

struct SwitchData
    {
    bool headlights;        // raw electrical state of each 12V input. The
    bool brights;           // blinker inputs carry the flasher's on/off wave
    bool blinkerL;          // itself, so drawing the raw state on screen IS
    bool blinkerR;          // the blink (gauge_render.cpp)

    uint8_t  rotaryPos;       // 0..11, debounced. While the dial reads OPEN
                              //  it holds the last good position, and after
                              //  ROTARY_OPEN_MS falls back to
                              //  ROTARY_FAILSAFE_POS (inputs.cpp)
    uint16_t rotaryRawCounts; // raw ADC (debug/diagnosis of a dirty ladder)
    float    rotaryOhms;      // wiper-to-ground resistance the ladder math
                              //  derived this tick (position k ~ k x 220 R)
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
                                //  glare): headlights on, or Dark mode selected
    uint8_t   ledBrightness;    // 0..255 global LED scale for this tick
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

// --- the warning/alarm system --------------------------------------------------------
// The alarm catalog from the project plan's "Text warnings" list. The enum is
// ordered BY PRIORITY (most important first), so "scan the array from 0 and
// take the first active entry" IS the priority arbitration - no sorting
// needed anywhere. Names, colors and behavior live in the definition table
// in alarms.cpp, in this exact order.

enum AlarmColor : uint8_t
    {
    ALARM_GREEN = 0, // status (cranking)
    ALARM_BLUE,      // informational (cold / rich)
    ALARM_YELLOW,    // warning - needs attention
    ALARM_RED        // critical - needs action NOW
    };

enum AlarmId : uint8_t
    {
    ALARM_OIL_LOW_CRIT = 0, // P1  OIL LOW  red
    ALARM_CLT_HOT_CRIT,     // P2  CLT HOT  red
    ALARM_AFR_LEAN_CRIT,    // P3  AFR LEAN red
    ALARM_OIL_HOT_CRIT,     // P4  OIL HOT  red
    ALARM_VOLT_LOW_CRIT,    // P5  VOLT LOW red
    ALARM_CELL_LOW_CRIT,    // P6  CELL LOW red
    ALARM_BATT_HOT_CRIT,    // P7  BATT HOT red
    ALARM_NO_GAS,           // P8  NO GAS   red
    ALARM_OIL_LOW_WARN,     // P9  OIL LOW  yellow
    ALARM_VOLT_LOW_WARN,    // P10 VOLT LOW yellow
    ALARM_CLT_HOT_WARN,     // P11 CLT HOT  yellow
    ALARM_OIL_HOT_WARN,     // P12 OIL HOT  yellow
    ALARM_AFR_LEAN_WARN,    // P13 AFR LEAN yellow
    ALARM_CELL_BAL,         // P13 CELL BAL yellow (plan has two P13s; this order breaks the tie)
    ALARM_BATT_HOT_WARN,    // P14 BATT HOT yellow
    ALARM_VOLT_HIGH_WARN,   // P15 VLT HIGH yellow
    ALARM_OIL_HIGH_WARN,    // P16 OIL HIGH yellow
    ALARM_FUEL_LOW,         // P17 FUEL LOW yellow
    ALARM_BATT_CAN,         // P18 BATT CAN yellow (BMS link lost)
    ALARM_NO_CAN,           // P19 NO CAN   yellow (ECU link lost)
    ALARM_DIAL_FAIL,        // P20 DIALFAIL yellow (rotary mode switch reads
                            //     open - added after the plan's list; sits
                            //     with the other "gauge hardware" yellows)
    ALARM_BATT_COLD,        // P20 BATTCOLD blue
    ALARM_CRANKING,         // P21 CRANKING green
    ALARM_AFR_RICH,         // P22 AFR RICH blue
    ALARM_CLT_COLD,         // P23 CLT COLD blue
    ALARM_OIL_COLD,         // P24 OIL COLD blue
    ALARM_COUNT,
    ALARM_NONE = 255        // "nothing active" = the plan's NOMINAL, P25
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
    FuelData   fuel;
    SwitchData sw;
    ModeData   mode;
    BoardData  board;
    AlarmsData alarms;
    FramData   fram;

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
    };

// Short name of an ECU frame for debug prints ("BASE4", "MS-AFRTGT").
const char* ecuFrameName(uint8_t frame);

// The one and only instance, defined in vehicle_data.cpp. As a global it is
// zero-initialized before setup() runs - every value starts at a known 0.
extern VehicleData g_data;
