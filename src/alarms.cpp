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
// ---------------------------------------------------------------------------

#include "alarms.h"
#include "config.h"

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
static bool condVoltLowCrit(bool on)
    {
    return voltDataOk() && engineRunning()
        && below(g_data.ecu.battVolts, VOLT_LOW_CRIT_V, VOLT_HYST_V, on);
    }

static bool condVoltLowWarn(bool on)
    {
    return voltDataOk() && engineRunning()
        && below(g_data.ecu.battVolts, VOLT_LOW_WARN_V, VOLT_HYST_V, on);
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

// --- 3. THE TABLE -------------------------------------------------------------------
// Order MUST match enum AlarmId in vehicle_data.h (= the plan's priority
// order). setMs/clearMs choices in words: oil pressure trips fast (200 ms)
// because at 7000 rpm every second without oil matters; temperatures move
// slowly so 2 s filters sensor noise for free; voltage waits 3 s so a big
// load dip at idle doesn't cry alternator; fuel waits 5 s on top of the
// already-slosh-filtered level; the CAN link alarms wait a few seconds so
// a normal ECU boot-up doesn't flash NO CAN at every ignition-on.

struct AlarmDef
    {
    const char* name;       // 8 chars, exactly as in the plan
    AlarmColor  color;
    uint8_t     priority;   // the plan's number (info only - order arbitrates)
    uint16_t    setMs;      // condition true this long -> active
    uint16_t    clearMs;    // condition false this long -> inactive
    AudioClip   clip;       // CLIP_NONE = silent
    uint32_t    repeatMs;   // 0 = say once per activation; else GPWS-repeat
    bool      (*cond)(bool currentlyActive);
    };

static const AlarmDef k_defs[ALARM_COUNT] =
    {
    //  name        color         pri  set    clear  clip                repeat  condition
    { "OIL LOW ", ALARM_RED,     1,   200,  1000, CLIP_OIL_PRESSURE, 10000, condOilLowCrit  },
    { "CLT HOT ", ALARM_RED,     2,  2000,  3000, CLIP_WATER_TEMP,   10000, condCltHotCrit  },
    { "AFR LEAN", ALARM_RED,     3,   500,  2000, CLIP_AFR_LEAN,     10000, condAfrLeanCrit },
    { "OIL HOT ", ALARM_RED,     4,  2000,  3000, CLIP_OIL_TEMP,     10000, condOilHotCrit  },
    { "VOLT LOW", ALARM_RED,     5,  3000,  3000, CLIP_ALTERNATOR,   15000, condVoltLowCrit },
    { "CELL LOW", ALARM_RED,     6,  2000,  3000, CLIP_CELL_LOW,     15000, condCellLowCrit },
    { "BATT HOT", ALARM_RED,     7,  2000,  3000, CLIP_BATT_TEMP,    15000, condBattHotCrit },
    { "NO GAS  ", ALARM_RED,     8,  5000,  5000, CLIP_NO_GAS,       30000, condNoGas       },
    { "OIL LOW ", ALARM_YELLOW,  9,   500,  1000, CLIP_NONE,             0, condOilLowWarn  },
    { "VOLT LOW", ALARM_YELLOW, 10,  3000,  3000, CLIP_ALTERNATOR,       0, condVoltLowWarn },
    { "CLT HOT ", ALARM_YELLOW, 11,  2000,  3000, CLIP_NONE,             0, condCltHotWarn  },
    { "OIL HOT ", ALARM_YELLOW, 12,  2000,  3000, CLIP_NONE,             0, condOilHotWarn  },
    { "AFR LEAN", ALARM_YELLOW, 13,   500,  2000, CLIP_NONE,             0, condAfrLeanWarn },
    { "CELL BAL", ALARM_YELLOW, 13,  3000,  3000, CLIP_CELL_BAL,         0, condCellBal     },
    { "BATT HOT", ALARM_YELLOW, 14,  2000,  3000, CLIP_BATT_TEMP,        0, condBattHotWarn },
    { "VLT HIGH", ALARM_YELLOW, 15,  1000,  2000, CLIP_OVERVOLT,         0, condVoltHighWarn},
    { "OIL HIGH", ALARM_YELLOW, 16,  1000,  2000, CLIP_NONE,             0, condOilHighWarn },
    { "FUEL LOW", ALARM_YELLOW, 17,  5000,  5000, CLIP_FUEL_LOW,         0, condFuelLow     },
    { "BATT CAN", ALARM_YELLOW, 18,  2000,  2000, CLIP_BATT_CAN,         0, condBattCan     },
    { "NO CAN  ", ALARM_YELLOW, 19,  3000,  2000, CLIP_NO_CAN,           0, condNoCan       },
    { "DIALFAIL", ALARM_YELLOW, 20, ROTARY_OPEN_MS, 2000, CLIP_DIAL_FAIL,  0, condDialFail    },
    { "BATTCOLD", ALARM_BLUE,   20,  5000,  5000, CLIP_NONE,             0, condBattCold    },
    { "CRANKING", ALARM_GREEN,  21,   300,   300, CLIP_NONE,             0, condCranking    },
    { "AFR RICH", ALARM_BLUE,   22,  1000,  2000, CLIP_NONE,             0, condAfrRich     },
    { "CLT COLD", ALARM_BLUE,   23,  2000,  3000, CLIP_NONE,             0, condCltCold     },
    { "OIL COLD", ALARM_BLUE,   24,  2000,  3000, CLIP_NONE,             0, condOilCold     },
    };

// --- 4. the state machine --------------------------------------------------------

// Persistence timers. 0 = "the raw condition is not currently in this
// state"; otherwise the millis() when it entered it.
static uint32_t s_trueSinceMs[ALARM_COUNT];
static uint32_t s_falseSinceMs[ALARM_COUNT];

void alarmsInit()
    {
    Serial.printf("Alarms: %u definitions loaded\n", (unsigned)ALARM_COUNT);
    }

void alarmsUpdate()
    {
    const uint32_t now = millis();
    AlarmsData& a = g_data.alarms;

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
    return (id < ALARM_COUNT) ? k_defs[id].name : "NOMINAL ";
    }

AlarmColor alarmColor(uint8_t id)
    {
    return (id < ALARM_COUNT) ? k_defs[id].color : ALARM_GREEN;
    }

uint8_t alarmPriority(uint8_t id)
    {
    return (id < ALARM_COUNT) ? k_defs[id].priority : 25; // 25 = the plan's Nominal
    }

AudioClip alarmClip(uint8_t id)
    {
    return (id < ALARM_COUNT) ? k_defs[id].clip : CLIP_NONE;
    }

uint32_t alarmRepeatMs(uint8_t id)
    {
    return (id < ALARM_COUNT) ? k_defs[id].repeatMs : 0;
    }
