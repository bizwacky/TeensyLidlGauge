// ---------------------------------------------------------------------------
// Rally CAN Gauge - local inputs implementation
// ---------------------------------------------------------------------------

#include "inputs.h"
#include "config.h"
#include "vehicle_data.h"

#include <TimeLib.h> // Teensy Time library; RTC survives power-off via coin cell

// The Teensy 4 die-temperature monitor, provided by the core.
extern "C" float tempmonGetTemp(void);

// --- fuel sender ---------------------------------------------------------------

// Slosh-filter state (design rationale: config.h, "fuel slosh filter").
// File-scope statics rather than locals so fuelSloshReset() can reach them.
static float    s_sloshRing[FUEL_SLOSH_MEDIAN_N]; // last N raw ohms samples
static uint8_t  s_sloshCount;                     // how many slots are real
static uint8_t  s_sloshIdx;                       // next slot to overwrite
static uint32_t s_sloshLastSampleMs;

// Median of the current ring contents. N is 9: copying + insertion-sorting
// nine floats is a couple hundred cycles at 600 MHz, twice a second - free.
static float sloshMedian()
    {
    float sorted[FUEL_SLOSH_MEDIAN_N];
    for (uint8_t i = 0; i < s_sloshCount; i++)
        {
        // insertion sort: walk back from the end, shifting bigger values
        // right until v's spot appears
        const float v = s_sloshRing[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > v)
            {
            sorted[j + 1] = sorted[j];
            j--;
            }
        sorted[j + 1] = v;
        }
    return sorted[s_sloshCount / 2];
    }

// Called by fuel_calc.cpp when the FUEL FILL gesture fires: fuel was just
// physically added, so the pre-fill samples in the window and the slow EMA
// are now history that would take a minute to age out. Flushing the chain
// lets it re-converge on the new level in ~FUEL_SLOSH_MEDIAN_N samples.
// ohmsSlosh/levelPct keep publishing their last value while it re-primes -
// a gauge that blanks mid-refuel helps nobody.
void fuelSloshReset()
    {
    s_sloshCount = 0;
    s_sloshIdx   = 0;
    g_data.fuel.sloshPrimed = false;
    }

// Ratiometric resistance-to-ground from a divider fed by a pull-up to the
// ADC's own rail - the fuel sender and the rotary dial are both wired this
// way (config.h draws each circuit):
//   f = counts/4095 = R/(R + pullup)   =>   R = pullup * f / (1 - f)
// The rail voltage cancels out, so only the pull-up's tolerance matters.
// f -> 1 (an open circuit) would divide by zero; clamp it to a huge but
// finite value instead so the callers' "above X ohms = open" tests work.
static float dividerOhms(int raw, float pullupOhms)
    {
    const float f = raw / (float)ADC_MAX_COUNTS;
    if (f >= 0.999f) return 1.0e6f;
    return pullupOhms * f / (1.0f - f);
    }

static void readFuelSender()
    {
    const int raw = analogRead(PIN_FUEL_SENDER);
    g_data.fuel.rawCounts = (uint16_t)raw;

    const float ohms = dividerOhms(raw, FUEL_REF_OHMS);
    g_data.fuel.ohms = ohms;

    // Wiring faults first: an open circuit lets the 150R reference pull the
    // node to the rail (ohms -> huge); a short to ground reads ~0R. Both
    // limits sit far outside the senders' 6..144R range.
    g_data.fuel.faultOpen  = (ohms > FUEL_OHMS_OPEN);
    g_data.fuel.faultShort = (ohms < FUEL_OHMS_SHORT);
    if (g_data.fuel.faultOpen || g_data.fuel.faultShort)
        return; // keep the last good filtered value on the gauge

    // Fast debug filter: an exponential moving average. Every tick, move the
    // filtered value a fixed FRACTION of the way toward the new reading;
    // alpha = dt/tau gives a time constant of FUEL_FILTER_TAU_S seconds.
    // This one tracks slosh almost live - that's its job: the serial print
    // shows it next to the slosh-filtered value, so you can SEE how much
    // motion the real filter is absorbing.
    static bool  primed = false;
    const float alpha = (TICK_FAST_MS / 1000.0f) / FUEL_FILTER_TAU_S;
    if (!primed)
        {
        g_data.fuel.ohmsFiltered = ohms; // first sample: jump straight there
        primed = true;                   // instead of ramping up from 0
        }
    else
        {
        g_data.fuel.ohmsFiltered += alpha * (ohms - g_data.fuel.ohmsFiltered);
        }

    // The slosh filter (see config.h for the full design story):
    // every FUEL_SLOSH_SAMPLE_MS, drop the RAW reading into the ring - raw,
    // not the EMA above, because pre-smoothed samples are correlated with
    // each other and a median only rejects outliers if samples are
    // independent - then push the window's median through a slow EMA.
    const uint32_t nowMs = millis();
    if (nowMs - s_sloshLastSampleMs >= FUEL_SLOSH_SAMPLE_MS)
        {
        s_sloshLastSampleMs = nowMs;

        s_sloshRing[s_sloshIdx] = ohms;
        s_sloshIdx = (uint8_t)((s_sloshIdx + 1) % FUEL_SLOSH_MEDIAN_N);
        if (s_sloshCount < FUEL_SLOSH_MEDIAN_N) s_sloshCount++;

        const float med = sloshMedian();
        if (!g_data.fuel.sloshPrimed)
            {
            // warming up (boot, or a fuelSloshReset() after a fill): follow
            // the median directly instead of slow-walking the EMA there
            g_data.fuel.ohmsSlosh   = med;
            g_data.fuel.sloshPrimed = (s_sloshCount == FUEL_SLOSH_MEDIAN_N);
            }
        else
            {
            // steady state: the slow EMA. alpha uses the SAMPLE period, not
            // the tick period - this stage only advances twice a second.
            const float sloshAlpha = (FUEL_SLOSH_SAMPLE_MS / 1000.0f) / FUEL_SLOSH_TAU_S;
            g_data.fuel.ohmsSlosh += sloshAlpha * (med - g_data.fuel.ohmsSlosh);
            }

        // Linear ohms -> percent map between the two calibration points, a
        // placeholder until we calibrate on the car (senders are rarely truly
        // linear; a lookup table can replace this without touching callers).
        const float pct = (FUEL_OHMS_EMPTY - g_data.fuel.ohmsSlosh)
                          / (FUEL_OHMS_EMPTY - FUEL_OHMS_FULL) * 100.0f;
        g_data.fuel.levelPct = constrain(pct, 0.0f, 100.0f);
        }
    }

// --- rotary mode switch -----------------------------------------------------------

static void readRotary()
    {
    SwitchData& sw = g_data.sw;
    const int raw = analogRead(PIN_ROTARY);
    sw.rotaryRawCounts = (uint16_t)raw;

    // counts -> ohms (ratiometric, like the fuel sender), then the nearest
    // tap of the 220R ladder. The voltage steps are uneven (the pull-up
    // makes it a curve) but the RESISTANCE steps are equal, so rounding in
    // ohms puts every decision boundary halfway between taps.
    const float ohms = dividerOhms(raw, ROTARY_PULLUP_OHMS);
    sw.rotaryOhms = ohms;

    // "Open" is a 13th candidate value that goes through the same debounce
    // as a position: a wiper skating between detents for a few ms neither
    // moves the position nor trips the fault.
    static const uint8_t CAND_OPEN = 255;
    uint8_t candidate;
    if (ohms > ROTARY_OPEN_OHMS)
        {
        candidate = CAND_OPEN;
        }
    else
        {
        const int pos = (int)(ohms / ROTARY_STEP_OHMS + 0.5f); // round to nearest tap
        candidate = (uint8_t)constrain(pos, 0, ROTARY_POSITIONS - 1);
        }

    // Debounce: only accept a candidate after it has read the same for
    // ROTARY_DEBOUNCE_TICKS in a row. (254 is an impossible value that
    // forces a mismatch on the first pass.)
    static uint8_t  lastCandidate = 254;
    static uint8_t  stableTicks   = 0;
    static uint32_t openSinceMs   = 0; // millis() the debounced open began

    if (candidate == lastCandidate)
        {
        if (stableTicks < 255) stableTicks++;
        }
    else
        {
        lastCandidate = candidate;
        stableTicks   = 1;
        }
    if (stableTicks < ROTARY_DEBOUNCE_TICKS)
        return; // still bouncing: everything stays as it was

    const uint32_t now = millis();

    if (candidate == CAND_OPEN)
        {
        if (!sw.rotaryOpen)
            {
            sw.rotaryOpen = true;
            openSinceMs   = now;
            }
        // Hold the last good position for the first ROTARY_OPEN_MS (a slow
        // turn across a non-shorting switch's gap looks exactly like this),
        // then park on the fail-safe position: a dial that is really gone
        // must not leave the gauge stuck in DISPLAYS OFF or SILENT.
        else if (now - openSinceMs >= ROTARY_OPEN_MS && sw.rotaryPos != ROTARY_FAILSAFE_POS)
            {
            sw.rotaryPos       = ROTARY_FAILSAFE_POS;
            sw.rotaryChangedMs = now;
            }
        return;
        }

    // A real tap again. Recovering from an open counts as ARRIVING at the
    // position, even if it is the one that was held throughout: the held
    // timer restarts, so the FUEL FILL gesture can't be credited off time
    // that accrued while the dial was disconnected.
    const bool recovered = sw.rotaryOpen;
    sw.rotaryOpen = false;
    if (candidate != sw.rotaryPos || recovered)
        {
        sw.rotaryPos       = candidate;
        sw.rotaryChangedMs = now; // restarts the rotaryHeldMs() timer
        }
    }

// --- 12V switch inputs -------------------------------------------------------------

// Threshold with hysteresis: above ON -> true, below OFF -> false, in the
// dead band between them -> keep the previous state. This is what stops a
// slowly-rising or noisy input from rattling the state on and off.
static bool readSwitch12V(uint8_t pin, bool current)
    {
    const int raw = analogRead(pin);
    if (raw >= SWITCH_ON_COUNTS)  return true;
    if (raw <= SWITCH_OFF_COUNTS) return false;
    return current;
    }

static void readSwitches()
    {
    g_data.sw.headlights = readSwitch12V(PIN_HEADLIGHTS, g_data.sw.headlights);
    g_data.sw.brights    = readSwitch12V(PIN_BRIGHTS,    g_data.sw.brights);
    g_data.sw.blinkerL   = readSwitch12V(PIN_BLINKER_L,  g_data.sw.blinkerL);
    g_data.sw.blinkerR   = readSwitch12V(PIN_BLINKER_R,  g_data.sw.blinkerR);
    }

// --- board housekeeping --------------------------------------------------------------

static void readBoard()
    {
    g_data.board.cpuTempC = tempmonGetTemp();

    // TimeLib keeps its own soft clock, resynced from the hardware RTC every
    // few minutes (setSyncProvider below). Until someone actually sets the
    // clock the RTC reports 1970/2016-ish dates - flag that so the screens
    // show "--:--:--" instead of a wrong time.
    g_data.board.clockValid = (year() > 2020);
    g_data.board.hh = (uint8_t)hour();
    g_data.board.mm = (uint8_t)minute();
    g_data.board.ss = (uint8_t)second();
    }

// --- public API ------------------------------------------------------------------------

static time_t readRtc()
    {
    return Teensy3Clock.get(); // battery-backed RTC on the Teensy 4.0
    }

void inputsInit()
    {
    analogReadResolution(ADC_RESOLUTION_BITS);
    analogReadAveraging(ADC_AVERAGING);

    // INPUT_DISABLE turns off the digital input buffer + keeper resistor on
    // pins used purely as analog inputs - the keeper would otherwise distort
    // high-impedance readings (a known Teensy 4 gotcha).
    const uint8_t analogPins[] = { PIN_FUEL_SENDER, PIN_ROTARY, PIN_HEADLIGHTS,
                                   PIN_BRIGHTS, PIN_BLINKER_L, PIN_BLINKER_R };
    for (uint8_t pin : analogPins) pinMode(pin, INPUT_DISABLE);

    setSyncProvider(readRtc); // TimeLib: pull the time from the RTC now and
                              // resync periodically from here on

    Serial.printf("Inputs up: ADC %u-bit, x%u averaging. RTC %s.\n",
                  ADC_RESOLUTION_BITS, ADC_AVERAGING,
                  (year() > 2020) ? "has a valid time" : "NOT SET (clock will read --:--)");
    }

void inputsPoll()
    {
    readFuelSender();
    readRotary();
    readSwitches();
    readBoard();
    }
