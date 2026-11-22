// ---------------------------------------------------------------------------
// Rally CAN Gauge - PDM telemetry implementation
// ---------------------------------------------------------------------------
// HOW dingoFW BEHAVES (read from its source, functions/profet.cpp - the
// rules below are built on it):
//
//   * An output over its current limit goes to state Overcurrent: it is OFF,
//     its trip counter goes up by one, and after the configured Reset Time
//     it switches back on by itself. The counter clears when the REQUEST
//     goes off.
//   * State Fault is terminal: off until the PDM is power cycled. An output
//     gets there by running out of retries (reset mode Count), or at once
//     (reset mode None).
//   * Currents under the driver's floor (about 0.5 A on a BTS7002) are
//     reported as zero, and there is no open-load detection.
//   * Status goes out every 100 ms. A trip whose Reset Time is shorter than
//     that can come and go between two frames - which is why the trip
//     COUNTER is watched as well as the state.
//
// THE VERDICTS, worst first:
//
//   FLT  state Fault - or "stuck": the output spent PDM_OC_STUCK_MS of the
//        last PDM_OC_WINDOW_MS in Overcurrent. An output set to retry
//        forever never reaches Fault, but one that trips again every time
//        it retries is just as dead; the crew needs to hear that as one
//        fact, not as a trip banner that never leaves.
//   OPN  commanded on, drawing under its catalogue floor for its catalogue
//        time (pump and fans only). The PDM cannot see an unplugged pump;
//        the missing current is the only evidence.
//   TRP  a trip happened within the last PDM_TRIP_HOLD_MS. Held, because the
//        event itself may be over in 200 ms.
//   OK   none of the above - and ALSO whenever the output's data is stale.
//        Stale data must raise nothing: PDME CAN / PDML CAN report the
//        silence, exactly like NO CAN does for the ECU.
//
// SAMPLING. The stuck test wants "how long in the last five seconds", a
// sliding window. One bit per 100 ms (the status period - finer would only
// re-sample the same frame) in a 64-bit shift register per output: shift
// in "was it in Overcurrent", count the ones in the low fifty bits.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "pdm.h"

// --- per-output memory -------------------------------------------------------------

static const uint32_t k_slotMs      = 100; // one history bit per status period
static const uint8_t  k_windowSlots = (uint8_t)(PDM_OC_WINDOW_MS / k_slotMs);
static const uint8_t  k_stuckSlots  = (uint8_t)(PDM_OC_STUCK_MS / k_slotMs);
static_assert(PDM_OC_WINDOW_MS / k_slotMs <= 64, "the overcurrent history is one 64-bit word per output");

struct OutputTrack
    {
    bool     primed;       // lastState/lastOcCount hold a real earlier sample
    uint8_t  lastState;
    uint8_t  lastOcCount;
    uint64_t ocHistory;    // bit 0 = newest slot; 1 = was in Overcurrent
    uint32_t tripHoldEnd;  // millis() the TRP hold expires; 0 = no hold
    uint32_t openSinceMs;  // millis() the "on but dry" condition began; 0 = not
    float    peakAmps;     // highest current seen since the output last came on
    uint8_t  lastVerdict;  // for "did it just get worse" (the black box event)
    };

static OutputTrack s_track[PDM_COUNT][PDM_MAX_OUTPUTS];

// Which rows of PDM_FRAMES carry each output's state and current (-1 = no
// row does). Worked out once: validity then costs two timestamp compares.
static int8_t s_stateRow[PDM_MAX_OUTPUTS];
static int8_t s_currentRow[PDM_MAX_OUTPUTS];

static uint32_t s_lastSlotMs;

// --- helpers ---------------------------------------------------------------------

static uint8_t popcount64(uint64_t v)
    {
    return (uint8_t)__builtin_popcountll(v);
    }

static int8_t findRow(PdmFrameKind kind, uint8_t output)
    {
    for (uint8_t row = 0; row < PDM_FRAME_COUNT; row++)
        {
        const PdmFrameDef& d = PDM_FRAMES[row];
        if (d.kind == kind && output >= d.firstOutput && output < d.firstOutput + d.outputs)
            return (int8_t)row;
        }
    return -1;
    }

// Queue an event for the black box. A worse verdict replaces a pending
// milder one; a milder one never overwrites a worse one still waiting.
static void recordEvent(uint8_t box, uint8_t output, PdmVerdict verdict, float amps)
    {
    PdmEventData& e = g_data.pdmEvent;
    e.count++;
    if (e.pending && e.verdict > verdict)
        return;
    e.pending = true;
    e.pdm     = box;
    e.output  = output;
    e.verdict = verdict;
    e.amps    = amps;
    }

// --- one output, one tick ----------------------------------------------------------

static void updateOutput(uint8_t box, uint8_t i, uint32_t now, bool newSlot)
    {
    PdmOutput&          o   = g_data.pdm[box].out[i];
    OutputTrack&        t   = s_track[box][i];
    const PdmOutputDef& def = PDM_OUTPUTS[box][i];

    o.valid = s_stateRow[i] >= 0 && s_currentRow[i] >= 0
              && g_data.pdmFrameFresh(box, (uint8_t)s_stateRow[i])
              && g_data.pdmFrameFresh(box, (uint8_t)s_currentRow[i]);

    if (!o.valid || def.cls == LOAD_NONE)
        {
        // Forget everything: when the link returns, the first sample is a new
        // baseline, not a "change" - otherwise a gauge reboot next to an
        // output that tripped an hour ago would announce a trip.
        t = OutputTrack();
        o.verdict = PDM_OUT_OK;
        o.stuck   = false;
        return;
        }

    // -- trip detection: the counter going up, or the state arriving in
    //    Overcurrent. Either alone can be missed (the counter wraps and
    //    clears; a short trip never shows in the state), together they can't.
    const bool trippedNow = t.primed
        && (o.ocCount > t.lastOcCount
            || (o.state == PDM_STATE_OVERCURRENT && t.lastState != PDM_STATE_OVERCURRENT));

    // -- the highest current while on: what the output was pulling when it
    //    tripped (by the time the state says Overcurrent, the current is 0)
    if (o.state == PDM_STATE_ON)
        {
        if (!t.primed || t.lastState != PDM_STATE_ON) t.peakAmps = o.amps; // just came on
        else if (o.amps > t.peakAmps)                 t.peakAmps = o.amps;
        }

    if (trippedNow)
        {
        o.trips++;
        t.tripHoldEnd = now + PDM_TRIP_HOLD_MS;
        if (t.tripHoldEnd == 0) t.tripHoldEnd = 1; // 0 is reserved for "no hold"
        recordEvent(box, i, PDM_OUT_TRP, t.peakAmps);
        t.peakAmps = 0.0f;
        }
    if (t.tripHoldEnd != 0 && (int32_t)(now - t.tripHoldEnd) >= 0)
        t.tripHoldEnd = 0;

    // -- stuck: time in Overcurrent over the sliding window
    if (newSlot)
        t.ocHistory = (t.ocHistory << 1) | (o.state == PDM_STATE_OVERCURRENT ? 1u : 0u);
    const uint64_t windowMask = (k_windowSlots >= 64) ? ~0ULL : ((1ULL << k_windowSlots) - 1);
    o.stuck = popcount64(t.ocHistory & windowMask) >= k_stuckSlots;

    // -- open load: on, and dry for long enough
    bool open = false;
    if (def.openAmps > 0.0f && o.state == PDM_STATE_ON && o.amps < def.openAmps)
        {
        if (t.openSinceMs == 0) t.openSinceMs = now ? now : 1;
        open = (now - t.openSinceMs) >= def.openMs;
        }
    else
        {
        t.openSinceMs = 0;
        }

    // -- the verdict, worst first
    PdmVerdict v = PDM_OUT_OK;
    if (o.state == PDM_STATE_FAULT || o.stuck)                        v = PDM_OUT_FLT;
    else if (open)                                                    v = PDM_OUT_OPN;
    else if (t.tripHoldEnd != 0 || o.state == PDM_STATE_OVERCURRENT)  v = PDM_OUT_TRP;
    o.verdict = v;

    // A change for the worse that isn't a fresh trip (those logged above):
    // the output just faulted, got stuck, or ran dry.
    if (v > t.lastVerdict && v != PDM_OUT_TRP)
        recordEvent(box, i, v, (v == PDM_OUT_FLT) ? t.peakAmps : o.amps);

    t.lastVerdict = v;
    t.lastState   = o.state;
    t.lastOcCount = o.ocCount;
    t.primed      = true;
    }

// --- lamp states -------------------------------------------------------------------

// OR a list of PDM outputs. 'allKnown' comes back false if any of them had
// no fresh data - the caller decides what an unknown lamp means.
static bool anyLampOn(const PdmOutputRef* refs, uint8_t n, bool& allKnown)
    {
    bool on  = false;
    allKnown = true;
    for (uint8_t k = 0; k < n; k++)
        {
        if (refs[k].pdm >= PDM_COUNT || refs[k].output >= PDM_MAX_OUTPUTS) continue;
        const PdmOutput& o = g_data.pdm[refs[k].pdm].out[refs[k].output];
        if (!o.valid)                       allKnown = false;
        else if (o.state == PDM_STATE_ON)   on = true;
        }
    return on;
    }

#define LAMP_ON(list, known) anyLampOn(list, sizeof(list) / sizeof((list)[0]), known)

static void updateLamps()
    {
    LampData& l = g_data.lamps;
    bool known;

    l.brights = LAMP_ON(LAMP_BRIGHTS, known);
    l.turnL   = LAMP_ON(LAMP_TURN_L, known);
    l.turnR   = LAMP_ON(LAMP_TURN_R, known);

    l.headlights = LAMP_ON(LAMP_HEADLIGHTS, known);

    // Night mode follows the headlights, but only on evidence: "on" from any
    // box counts at once, "off" needs every source to be heard from. With a
    // box silent and nothing reported on, keep the last answer - a CAN fault
    // at night must not throw the screens back to daytime blue.
    if (l.headlights)  l.nightHint = true;
    else if (known)    l.nightHint = false;
    }

// --- public API ---------------------------------------------------------------------

void pdmInit()
    {
    for (uint8_t i = 0; i < PDM_MAX_OUTPUTS; i++)
        {
        s_stateRow[i]   = findRow(PDM_KIND_STATE, i);
        s_currentRow[i] = findRow(PDM_KIND_CURRENT, i);
        }

    Serial.printf("PDM: %u boxes x %u outputs, %u status frames each, fresh window %lu ms\n",
                  (unsigned)PDM_COUNT, (unsigned)PDM_MAX_OUTPUTS, (unsigned)PDM_FRAME_COUNT,
                  (unsigned long)PDM_FRESH_MS);
    }

void pdmUpdate()
    {
    const uint32_t now = millis();

    // One history slot per 100 ms. No catch-up after a stall: a missed slot
    // only shortens the window slightly, a burst of fake slots would lie.
    const bool newSlot = (now - s_lastSlotMs) >= k_slotMs;
    if (newSlot) s_lastSlotMs = now;

    for (uint8_t box = 0; box < PDM_COUNT; box++)
        for (uint8_t i = 0; i < PDM_MAX_OUTPUTS; i++)
            updateOutput(box, i, now, newSlot);

    updateLamps();
    }

bool pdmFindVerdict(LoadClass cls, PdmVerdict lo, PdmVerdict hi,
                    uint8_t& box, uint8_t& output, PdmVerdict& verdict)
    {
    bool found = false;
    for (uint8_t b = 0; b < PDM_COUNT; b++)
        {
        for (uint8_t i = 0; i < PDM_MAX_OUTPUTS; i++)
            {
            if (PDM_OUTPUTS[b][i].cls != cls) continue;
            const PdmVerdict v = g_data.pdm[b].out[i].verdict;
            if (v < lo || v > hi) continue;
            if (!found || v > verdict) // strictly worse: the first of equals stays
                {
                found   = true;
                box     = b;
                output  = i;
                verdict = v;
                }
            }
        }
    return found;
    }

bool pdmDeviceFresh(uint8_t box)
    {
    for (uint8_t row = 0; row < PDM_FRAME_COUNT; row++)
        if (PDM_FRAMES[row].kind == PDM_KIND_DEVICE && g_data.pdmFrameFresh(box, row))
            return true;
    return false;
    }

const char* pdmVerdictName(uint8_t verdict)
    {
    switch (verdict)
        {
        case PDM_OUT_TRP: return "TRP";
        case PDM_OUT_OPN: return "OPN";
        case PDM_OUT_FLT: return "FLT";
        default:          return "OK ";
        }
    }
