// ---------------------------------------------------------------------------
// Rally CAN Gauge - alarm state machine implementation
// ---------------------------------------------------------------------------
// Layout of this file:
//   1. small helpers (hysteresis comparisons, data-validity gates)
//   2. one condition function per alarm - pure "is this bad right now?"
//   3. THE TABLE - every alarm's name/color/priority/timing/audio/condition
//   4. the state machine that runs the table
//
// Design: table-driven. Adding an alarm = one enum entry (vehicle_data.h,
// in priority position), one condition function, one table row. The state
// machine, display and audio code never change.
//
// The catalogue's shape (36 rows) comes from the Nov 2026 alarm structure:
// red = act now and it changes the outcome; yellow "live" = change how you
// drive or what is switched on; yellow "deferred" = nothing to do until the
// car stops; blue = information. The PDM rows class a fault by what it
// costs, not by which of 28 channels it is on: four load classes, and the
// banner names the load ("PUMP FLT", "FAN1 OPN", "LOBM TRP").
// ---------------------------------------------------------------------------

#include "alarms.h"
#include "config.h"
#include "pdm.h"

// --- 1. helpers -----------------------------------------------------------------

// Threshold comparisons with hysteresis. 'on' is whether this alarm is
// currently active: an active alarm uses a threshold pulled back by 'hyst',
// so the value must clearly retreat before the alarm releases.
//
//   above(v, 100, 3, off) -> trips at v > 100
//   above(v, 100, 3, on)  -> holds until v <= 97
static bool above(float v, float th, float hyst, bool on)
    {
    return v > (on ? th - hyst : th);
    }

static bool below(float v, float th, float hyst, bool on)
    {
    return v < (on ? th + hyst : th);
    }

// Data-validity gates. Every condition first proves its inputs are real:
// a stale value must produce NO alarm (the link alarms NO CAN / BATT CAN
// report the staleness itself). Without these gates, unplugging the ECU
// would light every red warning at once.
static bool engineRunning()
    {
    return g_data.ecuOnline() && g_data.ecu.rpm > ENGINE_RUNNING_RPM;
    }

static bool oilDataOk()  { return g_data.ecuFrameFresh(ECU_F_SENSORS2); } // oil P/T + volts
static bool cltDataOk()  { return g_data.ecuFrameFresh(ECU_F_SENSORS1); } // MAP/CLT/IAT
static bool voltDataOk() { return g_data.ecuFrameFresh(ECU_F_SENSORS2); }
static bool cellDataOk() { return g_data.bmsFrameFresh(BMS_F_CELLS, BMS_OFFLINE_MS); }

// The sender estimate counts only with healthy wiring AND a primed slosh
// filter (right after boot or a fill reset the filtered level isn't real yet).
static bool fuelSenderOk()
    {
    return !g_data.fuel.faultOpen && !g_data.fuel.faultShort
        && g_data.fuel.sloshPrimed;
    }

// rpm per psi - the "pressure must scale with rpm" measure from the plan.
// The floor on psi keeps a dead sensor from dividing by zero; a near-zero
// pressure at speed then produces a huge ratio, which is exactly the alarm
// we want anyway.
static float oilRatio()
    {
    const float psi = (g_data.ecu.oilPressPsi < 0.5f) ? 0.5f : g_data.ecu.oilPressPsi;
    return g_data.ecu.rpm / psi;
    }

// --- 2. conditions ----------------------------------------------------------------
// All thresholds live in config.h; the plan's "Warning Thresholds" section
// is the source of truth for the numbers.

static bool condOilLowCrit(bool on)
    {
    if (!oilDataOk() || !engineRunning()) return false;
    return below(g_data.ecu.oilPressPsi, OILP_LOW_CRIT_PSI, OILP_HYST_PSI, on)
        || above(oilRatio(), OILP_RATIO_CRIT, OILP_RATIO_HYST, on);
    }

static bool condOilLowWarn(bool on)
    {
    if (!oilDataOk() || !engineRunning()) return false;
    return below(g_data.ecu.oilPressPsi, OILP_LOW_WARN_PSI, OILP_HYST_PSI, on)
        || above(oilRatio(), OILP_RATIO_WARN, OILP_RATIO_HYST, on);
    }

static bool condOilHighWarn(bool on)
    {
    if (!oilDataOk() || !engineRunning()) return false;
    return above(g_data.ecu.oilPressPsi, OILP_HIGH_WARN_PSI, OILP_HYST_PSI, on);
    }

static bool condOilHotCrit(bool on)
    {
    return oilDataOk() && above(g_data.ecu.oilTempC, OILT_CRIT_C, TEMP_HYST_C, on);
    }

static bool condOilHotWarn(bool on)
    {
    return oilDataOk() && above(g_data.ecu.oilTempC, OILT_WARN_C, TEMP_HYST_C, on);
    }

static bool condOilCold(bool on)
    {
    return oilDataOk() && below(g_data.ecu.oilTempC, OILT_COLD_C, TEMP_HYST_C, on);
    }

static bool condCltHotCrit(bool on)
    {
    return cltDataOk() && above(g_data.ecu.cltC, CLT_CRIT_C, TEMP_HYST_C, on);
    }

static bool condCltHotWarn(bool on)
    {
    return cltDataOk() && above(g_data.ecu.cltC, CLT_WARN_C, TEMP_HYST_C, on);
    }

static bool condCltCold(bool on)
    {
    return cltDataOk() && below(g_data.ecu.cltC, CLT_COLD_C, TEMP_HYST_C, on);
    }

// AFR alarms judge the gated two-second window from mixture.cpp, and only
// once it is full (lambdaJudgeable): engine running, data fresh, throttle
// open, no fuel cut, sensor recovered - all already proven there. The red
// also needs the WHOLE window under load, so it can never fire on overrun
// history the way the old MAP-at-this-instant test did.
static bool condAfrLeanCrit(bool on)
    {
    const EcuData& e = g_data.ecu;
    return e.lambdaJudgeable && e.lambdaUnderLoad
        && above(e.lambdaErr2s, LAMBDA_LEAN_CRIT, LAMBDA_HYST, on);
    }

static bool condAfrLeanWarn(bool on)
    {
    const EcuData& e = g_data.ecu;
    return e.lambdaJudgeable
        && above(e.lambdaErr2s, LAMBDA_LEAN_WARN, LAMBDA_HYST, on);
    }

static bool condAfrRich(bool on)
    {
    const EcuData& e = g_data.ecu;
    return e.lambdaJudgeable
        && below(e.lambdaErr2s, -LAMBDA_RICH_DELTA, LAMBDA_HYST, on);
    }

// Voltage alarms watch the alternator, so they only mean something with the
// engine turning (a resting pack sits below 14.0 V by design).
//
// They have a second trigger since Nov 2026: the pack DISCHARGING with the
// engine above idle. The LTO pack is stiff - with the alternator dead it
// holds the bus above 13.2 V for a good while - so the voltage rule alone
// speaks up late. Current leaving the pack at speed says "you are on the
// battery" within seconds. The persistence lives here (not in the table's
// setMs, which both VOLT LOW rows also need for their voltage rule), and
// is evaluated once per tick in alarmsUpdate() because two rows read it.
static bool s_packDischarging;

static void updatePackDischarging(uint32_t now)
    {
    static uint32_t s_sinceMs = 0; // 0 = not currently discharging

    const bool raw = g_data.bmsFrameFresh(BMS_F_PACK, BMS_OFFLINE_MS)
                  && g_data.ecuOnline()
                  && g_data.ecu.rpm >= VOLT_DISCHARGE_MIN_RPM
                  && g_data.bms.packAmps > VOLT_DISCHARGE_AMPS;
    if (!raw)
        {
        s_sinceMs = 0;
        s_packDischarging = false;
        return;
        }
    if (s_sinceMs == 0) s_sinceMs = now ? now : 1;
    s_packDischarging = (now - s_sinceMs) >= VOLT_DISCHARGE_MS;
    }

static bool condVoltLowCrit(bool on)
    {
    return s_packDischarging
        || (voltDataOk() && engineRunning()
            && below(g_data.ecu.battVolts, VOLT_LOW_CRIT_V, VOLT_HYST_V, on));
    }

static bool condVoltLowWarn(bool on)
    {
    return s_packDischarging
        || (voltDataOk() && engineRunning()
            && below(g_data.ecu.battVolts, VOLT_LOW_WARN_V, VOLT_HYST_V, on));
    }

static bool condVoltHighWarn(bool on)
    {
    // no running gate: overvoltage is dangerous whenever it happens
    return voltDataOk() && above(g_data.ecu.battVolts, VOLT_HIGH_WARN_V, VOLT_HYST_V, on);
    }

static bool condCellLowCrit(bool on)
    {
    return cellDataOk() && below(g_data.bms.minCellVolts, CELL_LOW_CRIT_V, CELL_V_HYST, on);
    }

static bool condCellBal(bool on)
    {
    return cellDataOk() && above(g_data.bms.cellDeltaVolts(), CELL_BAL_WARN_V, CELL_V_HYST, on);
    }

static bool condBattHotCrit(bool on)
    {
    return cellDataOk() && above(g_data.bms.maxCellTempC, BATT_HOT_CRIT_C, TEMP_HYST_C, on);
    }

static bool condBattHotWarn(bool on)
    {
    return cellDataOk() && above(g_data.bms.maxCellTempC, BATT_HOT_WARN_C, TEMP_HYST_C, on);
    }

static bool condBattCold(bool on)
    {
    return cellDataOk() && below(g_data.bms.minCellTempC, BATT_COLD_C, TEMP_HYST_C, on);
    }

// Fuel alarms judge BOTH estimates: the slosh-filtered sender (a physical
// measurement) and the injector-integration calc (precise, but only as good
// as its last fill reset). Rationale for the veto: config.h, next to
// FUEL_SENDER_VETO_PCT.
static bool condNoGas(bool on)
    {
    const FuelData& f = g_data.fuel;

    // The sender saying critical is always believed - it measures the tank.
    const bool senderCrit = fuelSenderOk()
        && below(f.levelPct, FUEL_CRIT_PCT, FUEL_PCT_HYST, on);

    // The calc saying critical is believed UNLESS a healthy sender clearly
    // disagrees (reads above the veto line) - then the likely story is a
    // missed fill gesture, and FUEL LOW below still keeps it yellow. With
    // the sender faulted there is no veto: the calc flies solo.
    const bool senderVeto = fuelSenderOk() && f.levelPct > FUEL_SENDER_VETO_PCT;
    const bool calcCrit   = f.calcValid && !senderVeto
        && below(f.calcPct, FUEL_CRIT_PCT, FUEL_PCT_HYST, on);

    return senderCrit || calcCrit;
    }

static bool condFuelLow(bool on)
    {
    // Yellow is cheap and worth being conservative about: EITHER estimate
    // below the warn line raises it, no veto.
    const FuelData& f = g_data.fuel;
    return (fuelSenderOk() && below(f.levelPct, FUEL_WARN_PCT, FUEL_PCT_HYST, on))
        || (f.calcValid    && below(f.calcPct,  FUEL_WARN_PCT, FUEL_PCT_HYST, on));
    }

static bool condBattCan(bool)
    {
    return !g_data.bmsOnline();
    }

// ANY ECU frame the gauge decodes going stale - not just the rpm heartbeat.
// Losing one frame while the rest flow (a broadcast switched off in
// TunerStudio, the MS dash type set back to None) would otherwise quietly
// disarm every alarm fed by it.
static bool condNoCan(bool)
    {
    return !g_data.ecuDataComplete();
    }

// The rotary mode switch reads open-circuit (inputs.cpp already debounced
// the raw reading over a few ticks; this alarm's setMs adds the seconds of
// persistence that separate a broken wire from a slow turn between
// detents). While it is up, inputs.cpp has parked the dial on its
// fail-safe position, so the driver also sees full-bright, audible mode.
static bool condDialFail(bool)
    {
    return g_data.sw.rotaryOpen;
    }

static bool condCranking(bool)
    {
    // rusEFI's broadcast has no cranking flag; turning but below running
    // speed is the starter. The 300 ms set time in the table keeps a stall
    // spinning down through this band from flashing CRANKING.
    return g_data.ecuOnline()
        && g_data.ecu.rpm > 0 && g_data.ecu.rpm <= ENGINE_RUNNING_RPM;
    }

// --- the PDM rows --------------------------------------------------------------------
// These rows have no fixed text: the banner says WHICH load and WHAT is
// wrong with it. Each condition below, when it finds something, writes the
// 8 characters into its row's slot of s_dynName; alarmName() hands that
// out instead of the table's placeholder. A condition that finds nothing
// leaves the text alone, so the banner keeps naming the culprit for the
// row's clear time after the fault is gone.
//
// All the judging - what counts as a trip, how long "stuck" is, when a
// load is open - already happened in pdm.cpp, which also drops every
// verdict to OK while a box's data is stale. So these stay one-liners and
// the validity gate is built in.

static char s_dynName[ALARM_COUNT][9];
static bool s_engineTripIsPump; // picks ENGINE_PWR_TRIP's voice clip

// "PUMP FLT": 4-character load tag, a space, the 3-character verdict.
static void nameLoad(uint8_t id, uint8_t box, uint8_t output, PdmVerdict v)
    {
    snprintf(s_dynName[id], sizeof(s_dynName[id]), "%-4.4s %-3.3s",
             PDM_OUTPUTS[box][output].tag, pdmVerdictName(v));
    }

// "PDML HOT": the box's 4-character name, a space, a 3-character word.
static void nameBox(uint8_t id, uint8_t box, const char* word)
    {
    snprintf(s_dynName[id], sizeof(s_dynName[id]), "%-4.4s %-3.3s", pdmName(box), word);
    }

// The four load-class rows share one shape: "is any output of this class in
// this range of verdicts - and if so, name the worst".
static bool condLoadClass(uint8_t id, LoadClass cls, PdmVerdict lo, PdmVerdict hi)
    {
    uint8_t    box = 0, output = 0;
    PdmVerdict v = PDM_OUT_OK;
    if (!pdmFindVerdict(cls, lo, hi, box, output, v))
        return false;
    nameLoad(id, box, output, v);
    if (id == ALARM_ENGINE_PWR_TRIP)
        s_engineTripIsPump = (strncmp(PDM_OUTPUTS[box][output].tag, "PUMP", 4) == 0);
    return true;
    }

// Red: an engine-class output is gone (FLT) or feeding nothing (OPN). A
// trip that recovers is NOT red - losing the output for two seconds is, and
// pdm.cpp's "stuck" rule turns that into FLT.
static bool condEnginePwrCrit(bool)
    {
    return condLoadClass(ALARM_ENGINE_PWR_CRIT, LOAD_ENGINE, PDM_OUT_OPN, PDM_OUT_FLT);
    }

// Yellow: an engine-class output tripped and came back. Nothing to do about
// it at speed - but it tells the crew the stumble they just felt was
// electrical, which is worth knowing before the next one.
static bool condEnginePwrTrip(bool)
    {
    return condLoadClass(ALARM_ENGINE_PWR_TRIP, LOAD_ENGINE, PDM_OUT_TRP, PDM_OUT_TRP);
    }

static bool condFanFault(bool)
    {
    return condLoadClass(ALARM_FAN_FAULT, LOAD_COOLING, PDM_OUT_TRP, PDM_OUT_FLT);
    }

static bool condVisionFault(bool)
    {
    return condLoadClass(ALARM_VISION_FAULT, LOAD_VISION, PDM_OUT_TRP, PDM_OUT_FLT);
    }

static bool condBodyFault(bool)
    {
    return condLoadClass(ALARM_BODY_FAULT, LOAD_BODY, PDM_OUT_TRP, PDM_OUT_FLT);
    }

// Device-level rows. PDM-E can stop the car, so its Error and its last 5 K
// before the thermal latch are red; PDM-L cannot (forward lighting is split
// across both boxes), so the same conditions on it are yellow.
static bool condPdmeErr(bool)
    {
    return pdmDeviceFresh(PDM_E) && g_data.pdm[PDM_E].deviceState == PDM_DEV_ERROR;
    }

static bool condPdmlErr(bool)
    {
    return pdmDeviceFresh(PDM_L) && g_data.pdm[PDM_L].deviceState == PDM_DEV_ERROR;
    }

static bool pdmHotter(uint8_t box, float marginC, bool on)
    {
    return pdmDeviceFresh(box)
        && above(g_data.pdm[box].boardTempC, PDM_LATCH_TEMP_C - marginC, PDM_TEMP_HYST_C, on);
    }

static bool condPdmeHotCrit(bool on)
    {
    return pdmHotter(PDM_E, PDM_HOT_CRIT_MARGIN_C, on);
    }

// Either box within 15 K of its latch: shed load. (PDM-L's own 5 K point
// stays yellow, so it needs no second row - it is inside this one.)
static bool condPdmHotWarn(bool on)
    {
    for (uint8_t box = 0; box < PDM_COUNT; box++) // PDM-E first: it names the banner
        if (pdmHotter(box, PDM_HOT_WARN_MARGIN_C, on))
            {
            nameBox(ALARM_PDM_HOT_WARN, box, "HOT");
            return true;
            }
    return false;
    }

// A box has gone quiet. Like NO CAN, the banner is the only thing a stale
// box produces: every verdict from it is already OK.
static bool condPdmCan(bool)
    {
    for (uint8_t box = 0; box < PDM_COUNT; box++)
        if (!g_data.pdmOnline(box))
            {
            nameBox(ALARM_PDM_CAN, box, "CAN");
            return true;
            }
    return false;
    }

// --- 3. THE TABLE -------------------------------------------------------------------
// Order MUST match enum AlarmId in vehicle_data.h (= the catalogue's
// priority order; a row's catalogue number is its index + 1).
// setMs/clearMs choices in words: oil pressure trips fast (200 ms)
// because at 7000 rpm every second without oil matters; temperatures move
// slowly so 2 s filters sensor noise for free; voltage waits 3 s so a big
// load dip at idle doesn't cry alternator; fuel waits 5 s on top of the
// already-slosh-filtered level; the CAN link alarms wait a few seconds so
// a normal ECU boot-up doesn't flash NO CAN at every ignition-on.
// The PDM load rows carry almost no persistence of their own: pdm.cpp
// already built it into the verdicts (a TRP is held 10 s, FLT-by-stuck
// took 2 s to earn, OPN took 1-2 s). 200 ms = two status frames, enough
// to shrug off one corrupt nibble. TRP rows set at once - the verdict is
// already an event that happened.
//
// "tier" is the catalogue's live/deferred column. NOTHING READS IT YET: it
// is carried so the pace gate (hide deferred rows at stage pace), deferred
// on 21 Nov 2026, is a small change when it is wanted.
//
// Names in parentheses are placeholders for rows whose text is written at
// run time (the PDM rows above).

struct AlarmDef
    {
    const char* name;       // 8 chars, exactly as on the banner
    AlarmColor  color;
    AlarmTier   tier;       // see the note above - not consumed yet
    uint16_t    setMs;      // condition true this long -> active
    uint16_t    clearMs;    // condition false this long -> inactive
    AudioClip   clip;       // CLIP_NONE = silent
    uint32_t    repeatMs;   // 0 = say once per activation; else GPWS-repeat
    bool      (*cond)(bool currentlyActive);
    bool        dynamic;    // text comes from s_dynName, not 'name'
    };

static const AlarmDef k_defs[ALARM_COUNT] =
    {
    //  name        color         tier           set    clear  clip                repeat  condition         dynamic
    { "OIL LOW ", ALARM_RED,    TIER_LIVE,      200,  1000, CLIP_OIL_PRESSURE, 10000, condOilLowCrit,    false }, //  1
    { "CLT HOT ", ALARM_RED,    TIER_LIVE,     2000,  3000, CLIP_WATER_TEMP,   10000, condCltHotCrit,    false }, //  2
    { "AFR LEAN", ALARM_RED,    TIER_LIVE,      500,  2000, CLIP_AFR_LEAN,     10000, condAfrLeanCrit,   false }, //  3
    { "OIL HOT ", ALARM_RED,    TIER_LIVE,     2000,  3000, CLIP_OIL_TEMP,     10000, condOilHotCrit,    false }, //  4
    { "(ENGINE)", ALARM_RED,    TIER_LIVE,      200,  2000, CLIP_ENGINE_POWER, 10000, condEnginePwrCrit, true  }, //  5
    { "PDME ERR", ALARM_RED,    TIER_LIVE,      500,  2000, CLIP_PDM_FAULT,    15000, condPdmeErr,       false }, //  6
    { "PDME HOT", ALARM_RED,    TIER_LIVE,     2000,  3000, CLIP_PDM_TEMP,     15000, condPdmeHotCrit,   false }, //  7
    { "VOLT LOW", ALARM_RED,    TIER_LIVE,     3000,  3000, CLIP_ALTERNATOR,   15000, condVoltLowCrit,   false }, //  8
    { "CELL LOW", ALARM_RED,    TIER_LIVE,     2000,  3000, CLIP_CELL_LOW,     15000, condCellLowCrit,   false }, //  9
    { "BATT HOT", ALARM_RED,    TIER_LIVE,     2000,  3000, CLIP_BATT_TEMP,    15000, condBattHotCrit,   false }, // 10
    { "NO GAS  ", ALARM_RED,    TIER_LIVE,     5000,  5000, CLIP_NO_GAS,       30000, condNoGas,         false }, // 11
    { "OIL LOW ", ALARM_YELLOW, TIER_LIVE,      500,  1000, CLIP_NONE,             0, condOilLowWarn,    false }, // 12
    { "(ENGINE)", ALARM_YELLOW, TIER_LIVE,        0,   500, CLIP_PUMP_TRIP,        0, condEnginePwrTrip, true  }, // 13
    { "VOLT LOW", ALARM_YELLOW, TIER_LIVE,     3000,  3000, CLIP_ALTERNATOR,       0, condVoltLowWarn,   false }, // 14
    { "CLT HOT ", ALARM_YELLOW, TIER_LIVE,     2000,  3000, CLIP_NONE,             0, condCltHotWarn,    false }, // 15
    { "OIL HOT ", ALARM_YELLOW, TIER_LIVE,     2000,  3000, CLIP_NONE,             0, condOilHotWarn,    false }, // 16
    { "(FAN)   ", ALARM_YELLOW, TIER_LIVE,      200,  1000, CLIP_FAN_FAULT,        0, condFanFault,      true  }, // 17
    { "(PDM)HOT", ALARM_YELLOW, TIER_LIVE,     2000,  3000, CLIP_PDM_TEMP,         0, condPdmHotWarn,    true  }, // 18
    { "VLT HIGH", ALARM_YELLOW, TIER_LIVE,     1000,  2000, CLIP_OVERVOLT,         0, condVoltHighWarn,  false }, // 19
    { "(VISION)", ALARM_YELLOW, TIER_LIVE,      200,  1000, CLIP_LIGHT_FAULT,      0, condVisionFault,   true  }, // 20
    { "PDML ERR", ALARM_YELLOW, TIER_LIVE,      500,  2000, CLIP_PDM_FAULT,        0, condPdmlErr,       false }, // 21
    { "NO CAN  ", ALARM_YELLOW, TIER_LIVE,     3000,  2000, CLIP_NO_CAN,           0, condNoCan,         false }, // 22
    { "(PDM)CAN", ALARM_YELLOW, TIER_LIVE,     1000,  2000, CLIP_PDM_LINK,         0, condPdmCan,        true  }, // 23
    { "AFR LEAN", ALARM_YELLOW, TIER_DEFERRED,  500,  2000, CLIP_NONE,             0, condAfrLeanWarn,   false }, // 24
    { "CELL BAL", ALARM_YELLOW, TIER_DEFERRED, 3000,  3000, CLIP_CELL_BAL,         0, condCellBal,       false }, // 25
    { "BATT HOT", ALARM_YELLOW, TIER_DEFERRED, 2000,  3000, CLIP_BATT_TEMP,        0, condBattHotWarn,   false }, // 26
    { "OIL HIGH", ALARM_YELLOW, TIER_DEFERRED, 1000,  2000, CLIP_NONE,             0, condOilHighWarn,   false }, // 27
    { "FUEL LOW", ALARM_YELLOW, TIER_DEFERRED, 5000,  5000, CLIP_FUEL_LOW,         0, condFuelLow,       false }, // 28
    { "(BODY)  ", ALARM_YELLOW, TIER_DEFERRED,  200,  1000, CLIP_CHECK_MSG,        0, condBodyFault,     true  }, // 29
    { "BATT CAN", ALARM_YELLOW, TIER_DEFERRED, 2000,  2000, CLIP_BATT_CAN,         0, condBattCan,       false }, // 30
    { "DIALFAIL", ALARM_YELLOW, TIER_DEFERRED, ROTARY_OPEN_MS, 2000, CLIP_DIAL_FAIL, 0, condDialFail,    false }, // 31
    { "BATTCOLD", ALARM_BLUE,   TIER_DEFERRED, 5000,  5000, CLIP_NONE,             0, condBattCold,      false }, // 32
    { "CRANKING", ALARM_GREEN,  TIER_DEFERRED,  300,   300, CLIP_NONE,             0, condCranking,      false }, // 33
    { "AFR RICH", ALARM_BLUE,   TIER_DEFERRED, 1000,  2000, CLIP_NONE,             0, condAfrRich,       false }, // 34
    { "CLT COLD", ALARM_BLUE,   TIER_DEFERRED, 2000,  3000, CLIP_NONE,             0, condCltCold,       false }, // 35
    { "OIL COLD", ALARM_BLUE,   TIER_DEFERRED, 2000,  3000, CLIP_NONE,             0, condOilCold,       false }, // 36
    };

// --- 4. the state machine --------------------------------------------------------

// Persistence timers. 0 = "the raw condition is not currently in this
// state"; otherwise the millis() when it entered it.
static uint32_t s_trueSinceMs[ALARM_COUNT];
static uint32_t s_falseSinceMs[ALARM_COUNT];

void alarmsInit()
    {
    // Until a dynamic row's condition has named something, its slot shows
    // the table's placeholder rather than an empty string.
    for (uint8_t i = 0; i < ALARM_COUNT; i++)
        strlcpy(s_dynName[i], k_defs[i].name, sizeof(s_dynName[i]));

    Serial.printf("Alarms: %u definitions loaded\n", (unsigned)ALARM_COUNT);
    }

void alarmsUpdate()
    {
    const uint32_t now = millis();
    AlarmsData& a = g_data.alarms;

    updatePackDischarging(now); // shared by both VOLT LOW rows - once per tick

    a.top = ALARM_NONE;
    a.anyRed = false;
    a.anyYellow = false;

    for (uint8_t i = 0; i < ALARM_COUNT; i++)
        {
        const AlarmDef& d = k_defs[i];

        // the raw condition, with our current latched state fed back in so
        // the condition can apply hysteresis
        if (d.cond(a.active[i]))
            {
            s_falseSinceMs[i] = 0;
            if (s_trueSinceMs[i] == 0) s_trueSinceMs[i] = now;
            if (!a.active[i] && now - s_trueSinceMs[i] >= d.setMs)
                a.active[i] = true;
            }
        else
            {
            s_trueSinceMs[i] = 0;
            if (s_falseSinceMs[i] == 0) s_falseSinceMs[i] = now;
            if (a.active[i] && now - s_falseSinceMs[i] >= d.clearMs)
                a.active[i] = false;
            }

        // summaries - first active hit in table order is the top alarm
        if (a.active[i])
            {
            if (a.top == ALARM_NONE) a.top = i;
            if (d.color == ALARM_RED)    a.anyRed = true;
            if (d.color == ALARM_YELLOW) a.anyYellow = true;
            }
        }
    }

// --- table accessors -----------------------------------------------------------------

const char* alarmName(uint8_t id)
    {
    if (id >= ALARM_COUNT) return "NOMINAL ";
    return k_defs[id].dynamic ? s_dynName[id] : k_defs[id].name;
    }

AlarmColor alarmColor(uint8_t id)
    {
    return (id < ALARM_COUNT) ? k_defs[id].color : ALARM_GREEN;
    }

AlarmTier alarmTier(uint8_t id)
    {
    return (id < ALARM_COUNT) ? k_defs[id].tier : TIER_DEFERRED;
    }

uint8_t alarmPriority(uint8_t id)
    {
    // The enum IS the priority order, so a row's catalogue number is its
    // index + 1. Nominal sorts after everything.
    return (id < ALARM_COUNT) ? (uint8_t)(id + 1) : (uint8_t)(ALARM_COUNT + 1);
    }

AudioClip alarmClip(uint8_t id)
    {
    if (id >= ALARM_COUNT) return CLIP_NONE;
    // One row, two voices: a tripping PUMP gets its own callout, a tripping
    // coil or main feed the general one.
    if (id == ALARM_ENGINE_PWR_TRIP && !s_engineTripIsPump)
        return CLIP_ENGINE_POWER;
    return k_defs[id].clip;
    }

uint32_t alarmRepeatMs(uint8_t id)
    {
    return (id < ALARM_COUNT) ? k_defs[id].repeatMs : 0;
    }
