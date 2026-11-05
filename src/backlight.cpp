// ---------------------------------------------------------------------------
// Rally CAN Gauge - display backlight PWM (3x low-side N-FET, phase-staggered)
// ---------------------------------------------------------------------------
// The dimming chain, per screen, evaluated every 20 ms tick:
//
//   mode level (0..255)      what the dial says (bright/medium/dark),
//     from modes.cpp         resolved once per tick like everything else
//   x BL_TRIM[screen]        the binning correction for THIS panel
//   -> fade toward target    BL_FADE_MS full-scale ramp, so mode changes
//                            glide instead of snapping
//   -> a PWM window          [phase, phase + duty) of the period, written
//                            straight into the FlexPWM compare registers
//
// Displays-off mode targets 0 directly - and because it goes through the
// same fade, flicking the dial to DISPOFF dims the panels out over 250 ms
// instead of cutting them dead.
//
// WHY NOT analogWrite(): it can only make windows that START at counter
// zero, so all three backlights would switch on in the same instant and
// the 5V rail would see one 300 mA step per cycle. The point of the pin
// shuffle (config.h, PIN_BACKLIGHT) was to get all three channels onto
// FlexPWM4 so the windows can be spread a third of a period apart - and
// that needs the compare registers written directly. Everything else -
// clocking, prescaler, period, pin mux, output enable - is still done by
// the Teensy core's analogWriteFrequency()/analogWrite(), exactly as
// before; only the window placement lives here.
//
// THE FlexPWM IN ONE PARAGRAPH (i.MX RT1060 reference manual, ch. 55):
// each submodule has a 16-bit counter running INIT..VAL1 (= the period).
// Channel A goes HIGH when the counter matches VAL2 and LOW when it
// matches VAL3; channel B does the same with VAL4/VAL5. Those registers
// are double-buffered: writes land in a shadow copy and take effect
// together at the next period boundary once the submodule's LDOK bit is
// set - so a window never tears mid-cycle. OCTRL's POLA/POLB bits invert
// a channel (immediately, not buffered). Submodule 0 broadcasts its
// period boundary as the "master sync"; a submodule whose CTRL2.INIT_SEL
// selects that re-initializes its counter on the pulse and therefore runs
// in lockstep with submodule 0. The core itself relies on "VAL2 == VAL3
// == 0 means never high" for analogWrite(pin, 0), and so do we.
//
// WINDOWS THAT WRAP: a window at phase 2/3 with 50% duty runs past the
// end of the period. The compare pair can't express "high from 2/3 to 1/6
// of the NEXT cycle", so describe the OFF window instead (which doesn't
// wrap) and invert the channel: high everywhere outside it. The polarity
// bit only flips when a fading level crosses the wrap point, and a flip
// is at most one 50 us cycle out of step with the buffered VALs. Invisible.
//
// Why there is no gamma table: the FET either passes the ballast-set
// ~95 mA or nothing, so light output is exactly duty cycle - the same
// physics as the WS2812 strip. The nonlinearity of human vision is baked
// into the three chosen SCREEN_BRIGHTNESS_* numbers, not a runtime curve.
//
// BENCH CHECK: scope pins 2, 3 and 22 together in Bright mode. Expect
// three 20 kHz square waves with their rising edges a third of a period
// (16.7 us) apart; the boot line "[BL  ] ... phase-locked" confirms the
// counter lock was measured. Set BL_PHASE to all zeros and the edges
// should line up - that is the analogWrite() behavior this replaces.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "backlight.h"
#include "config.h"
#include "vehicle_data.h"

// --- the hardware map -----------------------------------------------------------
// Which FlexPWM4 submodule/channel each backlight pin lands on, from the
// Teensy core's pin table (cores/teensy4/pwm.c). The core resolves this
// itself inside analogWrite(); we need it to reach the same registers.
// If PIN_BACKLIGHT ever changes, this table must change with it.

struct BlChannel
    {
    uint8_t submodule; // FlexPWM4 submodule 0..3
    bool    chanB;     // false = channel A (VAL2/VAL3), true = channel B (VAL4/VAL5)
    };

static const BlChannel k_chan[3] =
    {
    { 2, false }, // pin 2  = FlexPWM4_2_A  (display 1)
    { 2, true  }, // pin 3  = FlexPWM4_2_B  (display 2)
    { 0, false }, // pin 22 = FlexPWM4_0_A  (display 3)
    };

#define BL_PWM IMXRT_FLEXPWM4
static const uint8_t k_smMask         = (1 << 0) | (1 << 2); // the submodules we own
static const int32_t k_syncTolerance  = 8;                   // counts (= 53 ns at 150 MHz)

// --- state -----------------------------------------------------------------------

// Current output level per screen as a fraction of full duty (0..1), the
// fade's state. Floats so a 250 ms ramp at 50 Hz (~13 steps) accumulates
// without rounding to death.
static float    s_level[3] = { 0.0f, 0.0f, 0.0f };
static uint16_t s_modulo   = 0;     // VAL1 = period - 1, as the core programmed it
static bool     s_locked   = false; // SM2 confirmed in lockstep with SM0

// --- register access ---------------------------------------------------------------

// Place one screen's window into its submodule's shadow registers:
// [start, start + on) counts of the period, or the inverted OFF window
// when that would run past the end. The ONLY compare-register writer.
static void writeWindow(uint8_t screen)
    {
    const BlChannel& c  = k_chan[screen];
    auto&            sm = BL_PWM.SM[c.submodule];
    const uint32_t period = (uint32_t)s_modulo + 1;

    uint32_t on = (uint32_t)(s_level[screen] * period + 0.5f);
    if (on > period) on = period;
    const uint32_t start = (uint32_t)(BL_PHASE[screen] * period + 0.5f) % period;

    // Base polarity: a plain (non-wrapping) window is "high = on" for an
    // active-high driver; the inverting cases toggle from there.
    bool     invert = !BL_ACTIVE_HIGH;
    uint16_t vSet, vClear;

    if (on == 0)
        {
        vSet = 0; vClear = 0;                 // set + clear on the same count: never high
        }
    else if (on >= period)
        {
        vSet = 0; vClear = 0;                 // "never high", inverted = always on
        invert = !invert;
        }
    else if (start + on < period)
        {
        vSet   = (uint16_t)start;             // the plain window
        vClear = (uint16_t)(start + on);
        }
    else
        {
        vSet   = (uint16_t)(start + on - period); // wraps: describe the OFF
        vClear = (uint16_t)start;                 // window and invert
        invert = !invert;
        }

    if (c.chanB)
        {
        sm.VAL4 = vSet;
        sm.VAL5 = vClear;
        if (invert) sm.OCTRL |=  FLEXPWM_SMOCTRL_POLB;
        else        sm.OCTRL &= ~FLEXPWM_SMOCTRL_POLB;
        }
    else
        {
        sm.VAL2 = vSet;
        sm.VAL3 = vClear;
        if (invert) sm.OCTRL |=  FLEXPWM_SMOCTRL_POLA;
        else        sm.OCTRL &= ~FLEXPWM_SMOCTRL_POLA;
        }
    }

// Write all three windows and release them together. CLDOK first, exactly
// as the core's analogWrite does (a still-pending LDOK blocks shadow
// writes); LDOK for both submodules last, so they go live on the same
// period boundary.
static void commitWindows()
    {
    BL_PWM.MCTRL |= FLEXPWM_MCTRL_CLDOK(k_smMask);
    for (uint8_t i = 0; i < 3; i++) writeWindow(i);
    BL_PWM.MCTRL |= FLEXPWM_MCTRL_LDOK(k_smMask);
    }

// --- public API ---------------------------------------------------------------------

void backlightInit()
    {
    // 1. Let the core set up each pin: prescaler + period for BL_PWM_HZ,
    //    output enable, pin mux, and a 0 duty so nothing lights before the
    //    fade-in. (Pins 2 and 3 share submodule 2 - the second frequency
    //    call just rewrites the same period.)
    for (uint8_t i = 0; i < 3; i++)
        {
        analogWriteFrequency(PIN_BACKLIGHT[i], BL_PWM_HZ);
        analogWrite(PIN_BACKLIGHT[i], 0);
        }

    // 2. Read back the period the core chose (VAL1 = period - 1). Both
    //    submodules were asked for the same frequency, so they must agree.
    s_modulo = BL_PWM.SM[0].VAL1;
    const uint16_t modulo2 = BL_PWM.SM[2].VAL1;

    // 3. Slave submodule 2 to submodule 0: INIT_SEL = 2 (master sync
    //    re-initializes the counter), RELOAD_SEL = 1 (shadow registers load
    //    on submodule 0's reload). CLK_SEL stays 0 (IPBus clock) on both so
    //    the prescaler the core programmed applies identically.
    uint16_t ctrl2 = BL_PWM.SM[2].CTRL2;
    ctrl2 &= ~(FLEXPWM_SMCTRL2_INIT_SEL(3) | FLEXPWM_SMCTRL2_RELOAD_SEL
               | FLEXPWM_SMCTRL2_CLK_SEL(3));
    ctrl2 |= FLEXPWM_SMCTRL2_INIT_SEL(2) | FLEXPWM_SMCTRL2_RELOAD_SEL;
    BL_PWM.SM[2].CTRL2 = ctrl2;

    // Start dark in the correct polarity at the new period; the first
    // ticks then fade up to whatever the dial asks for.
    commitWindows();

    // 4. Verify the lock rather than trust it: sample both counters
    //    back-to-back eight times over ~1 ms (longer than a free-running
    //    16-bit counter's 437 us wrap, so a runaway can't hide). Locked
    //    means SM2 tracks SM0 within a few counts and never runs past VAL1.
    //    Anything else: put SM2 back on local sync - every pin still gets
    //    correct PWM, only the SM0-vs-SM2 phase becomes unverified - and
    //    say so, loudly, on serial.
    delayMicroseconds(200);
    int32_t worst   = 0;
    bool    overrun = false;
    const int32_t period = (int32_t)s_modulo + 1;
    for (uint8_t n = 0; n < 8; n++)
        {
        const int32_t c0 = BL_PWM.SM[0].CNT;
        const int32_t c2 = BL_PWM.SM[2].CNT;
        int32_t d = c2 - c0;
        if (d >  period / 2) d -= period; // shortest distance around the wrap
        if (d < -period / 2) d += period;
        if (d < 0) d = -d;
        if (d > worst) worst = d;
        if (c2 > s_modulo) overrun = true;
        delayMicroseconds(120);
        }
    s_locked = !overrun && worst <= k_syncTolerance && modulo2 == s_modulo;
    if (!s_locked)
        {
        BL_PWM.SM[2].CTRL2 = (uint16_t)(BL_PWM.SM[2].CTRL2
                             & ~(FLEXPWM_SMCTRL2_INIT_SEL(3) | FLEXPWM_SMCTRL2_RELOAD_SEL));
        }

    // Actual frequency = bus clock / 2^prescaler / period (the core picks a
    // prescaler only when the period would overflow 16 bits, i.e. below
    // ~2.3 kHz - not at 20 kHz, but the print should stay honest).
    const uint32_t prescale = (BL_PWM.SM[0].CTRL >> 4) & 0x7;
    Serial.printf("[BL  ] FlexPWM4 %lu Hz, period %u counts, phases %.2f/%.2f/%.2f - %s"
                  " (SM0/SM2 skew %ld counts%s%s)\n",
                  (unsigned long)((F_BUS_ACTUAL >> prescale) / (uint32_t)(s_modulo + 1)),
                  (unsigned)(s_modulo + 1), BL_PHASE[0], BL_PHASE[1], BL_PHASE[2],
                  s_locked ? "phase-locked" : "NOT LOCKED, pins 2/3 free-running",
                  (long)worst, overrun ? ", SM2 overran VAL1" : "",
                  (modulo2 != s_modulo) ? ", period mismatch" : "");
    }

void backlightUpdate()
    {
    const ModeData& m = g_data.mode;

    // Largest level change one tick may apply: full scale in BL_FADE_MS.
    // Guard the 0 case (snap mode) rather than divide by it.
    const float step = (BL_FADE_MS > 0)
                           ? (float)TICK_FAST_MS / (float)BL_FADE_MS
                           : 1.0f;

    for (uint8_t i = 0; i < 3; i++)
        {
        const float target = m.displaysOff
                                 ? 0.0f
                                 : (m.screenBrightness / 255.0f) * BL_TRIM[i];

        const float delta = target - s_level[i];
        if (delta >  step)      s_level[i] += step;
        else if (delta < -step) s_level[i] -= step;
        else                    s_level[i] = target;
        }

    commitWindows();
    }

float backlightDutyPct(uint8_t screen)
    {
    return (screen < 3) ? s_level[screen] * 100.0f : 0.0f;
    }

bool backlightPhaseLocked()
    {
    return s_locked;
    }
