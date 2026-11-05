// ---------------------------------------------------------------------------
// Rally CAN Gauge - the WS2812B LED strip (status block + tach + shift light)
// ---------------------------------------------------------------------------
// Every 20 ms this module composes one complete 15-LED frame from scratch in
// a local buffer and hands it to WS2812Serial. "Compose then show" - never
// mutate individual LEDs across ticks - means there is no stale-pixel state
// to get wrong: whatever g_data says right now IS what the strip shows,
// exactly like the screens will re-render from g_data each frame.
//
// The frame is decided by a strict priority ladder, most urgent first:
//
//   0. Displays-off mode      -> every LED dark, nothing else considered
//   1. red alarm, not silent  -> ENTIRE strip flashes red (nothing else shows)
//   2. status block (left 3)  -> silent+red: solid red / yellow: 3x yellow
//                                / quiet: single green
//      tach block (right 12)  -> shift rpm: all 12 flash (blue day/orange night)
//                                / else: bar of green+orange, with the LEDs
//                                above the warmup rev limit blocked in blue
//
// Why WS2812Serial and not the classic Adafruit_NeoPixel: NeoPixel bit-bangs
// the 800 kHz protocol with interrupts DISABLED for the whole transfer.
// WS2812Serial instead expands each pixel into UART framing (12 bytes per
// LED) and lets a DMA channel stream it out of Serial2's TX pin on its own;
// show() just starts the transfer and returns. The superloop keeps draining
// CAN at full rate while the strip is being clocked - non-blocking, like
// everything else in this firmware.
//
// Timing check: 15 LEDs x 30 us + 300 us reset latch = ~0.75 ms on the wire,
// far shorter than the 20 ms between frames, so a show() never has to wait
// for the previous transfer.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <WS2812Serial.h>

#include "led_tach.h"
#include "config.h"
#include "vehicle_data.h"

// --- the strip -------------------------------------------------------------------
// WS2812Serial wants two buffers: drawingMemory is where setPixel() writes
// plain 3-byte RGB values, displayMemory is where show() expands them into
// the UART bit patterns the DMA engine streams (4 wire-bits encode 1 LED
// bit, hence 12 bytes per LED). DMAMEM places the streamed buffer in OCRAM,
// the region the DMA controller reads without competing with code fetches.

static byte        s_drawingMemory[LED_COUNT * 3];
DMAMEM static byte s_displayMemory[LED_COUNT * 12];

static WS2812Serial s_strip(LED_COUNT, s_displayMemory, s_drawingMemory,
                            PIN_LED_STRIP, WS2812_GRB);

// Brightness for the frame being composed (set once per tick from the
// resolved mode, applied inside setLed so no caller can forget it).
static uint8_t s_brightness = LED_BRIGHTNESS_BRIGHT;

// --- tiny helpers ----------------------------------------------------------------

// Scale a 0xRRGGBB color by 0..255. Each channel is multiplied then >>8,
// i.e. (c * b) / 256 - the standard integer approximation of "c * b / 255"
// (worst case one count dim, invisible; division-free and fast).
static uint32_t scaleColor(uint32_t c, uint8_t b)
    {
    const uint32_t r = (((c >> 16) & 0xFF) * b) >> 8;
    const uint32_t g = (((c >>  8) & 0xFF) * b) >> 8;
    const uint32_t bl = ((c        & 0xFF) * b) >> 8;
    return (r << 16) | (g << 8) | bl;
    }

// The ONLY writer to the strip buffer: applies brightness and the
// mirror-if-reversed mapping in one place. "logical" 0 is always the left
// end of the status block regardless of which way the strip is wired.
static void setLed(uint8_t logical, uint32_t color)
    {
    const uint8_t physical = LED_STRIP_REVERSED ? (LED_COUNT - 1 - logical) : logical;
    s_strip.setPixel(physical, scaleColor(color, s_brightness));
    }

// Square-wave flash phase from the shared millis() clock. Using wall time
// (not a per-caller counter) means every flashing element blinks in sync,
// and the phase survives the priority ladder switching branches.
static bool flashOn(uint32_t halfPeriodMs)
    {
    return (millis() / halfPeriodMs) & 1;
    }

// --- warmup rev-limit stage -------------------------------------------------------
// Walks the WARMUP_STAGES table with hysteresis. The current stage is
// remembered in a static (this module's only piece of cross-tick state):
// moving UP happens the moment the oil reaches the next stage's threshold,
// moving DOWN needs the temp to fall WARMUP_STAGE_HYST_C below the current
// stage's own threshold. A reading hovering at 69.9/70.1 C therefore can't
// flick LED 6 between blue and live every tick.
//
// Fail-safe by construction: with the ECU silent (or the sensor dead) the
// oil temp reads 0 C -> stage "cold" -> the most restrictive blue block.

static uint16_t warmupRpmLimit(float oilTempC)
    {
    static uint8_t s_stage = 0;

    while (s_stage + 1 < WARMUP_STAGE_COUNT
           && oilTempC >= WARMUP_STAGES[s_stage + 1].minOilTempC)
        {
        s_stage++;
        }
    while (s_stage > 0
           && oilTempC < WARMUP_STAGES[s_stage].minOilTempC - WARMUP_STAGE_HYST_C)
        {
        s_stage--;
        }
    return WARMUP_STAGES[s_stage].rpmLimit;
    }

// --- public API -------------------------------------------------------------------

void ledTachInit()
    {
    s_strip.begin();

    // Paint a black frame immediately: WS2812s power up in random states,
    // and a strip of random colors during boot looks like a fault.
    for (uint8_t i = 0; i < LED_COUNT; i++)
        s_strip.setPixel(i, 0);
    s_strip.show();
    }

void ledTachUpdate()
    {
    const AlarmsData& a = g_data.alarms;
    const ModeData&   m = g_data.mode;

    // Displays-off mode: one black frame and done - checked before every
    // other rule, so even a red alarm stays dark. Its voice clip still
    // plays; killing the LIGHT is exactly what this detent is for.
    if (m.displaysOff)
        {
        for (uint8_t i = 0; i < LED_COUNT; i++)
            s_strip.setPixel(i, 0);
        s_strip.show();
        return;
        }

    // The dial's decisions arrive pre-resolved from modes.cpp: this module
    // no longer knows (or cares) which position means what.
    const bool silent = m.silent;
    const bool night  = m.night; // affects COLOR choice; brightness is separate
    s_brightness = m.ledBrightness;

    // Trust rpm only while the 50 Hz ECU stream is actually arriving;
    // otherwise the bar would freeze at the last value the dead bus sent.
    // (The NO CAN yellow alarm reports the outage - the tach just goes dark.)
    const uint16_t rpm = g_data.ecuGroupFresh(0, ECU_FRESH_FAST_MS) ? g_data.ecu.rpm : 0;

    // --- priority 1: red alarm -> the whole gauge flashes red ------------------
    // Silent mode opts out (plan: "disables flashing") and is handled as a
    // solid red status block below, leaving the tach usable.
    if (a.anyRed && !silent)
        {
        const uint32_t c = flashOn(LED_RED_FLASH_HALF_MS) ? LED_COLOR_ALERT_RED : 0;
        for (uint8_t i = 0; i < LED_COUNT; i++)
            setLed(i, c);
        s_strip.show();
        return;
        }

    // --- status block: logical LEDs 0..2 ---------------------------------------
    if (a.anyRed) // only reachable in silent mode: solid, no flash
        {
        for (uint8_t i = 0; i < LED_STATUS_LEDS; i++)
            setLed(i, LED_COLOR_ALERT_RED);
        }
    else if (a.anyYellow)
        {
        for (uint8_t i = 0; i < LED_STATUS_LEDS; i++)
            setLed(i, LED_COLOR_STATUS_WARN);
        }
    else
        {
        setLed(0, LED_COLOR_STATUS_OK); // single green = all systems nominal
        setLed(1, 0);
        setLed(2, 0);
        }

    // --- tach block: logical LEDs 3..14 = tach index 0..11 ----------------------
    if (rpm >= SHIFT_FLASH_RPM)
        {
        // Shift light. Deliberately NOT suppressed by silent mode: silent
        // exists to gag a nagging broken-sensor alarm, and the shift light
        // is a driving aid, not an alarm. (To silence it too, add
        // "&& !silent" here.)
        const uint32_t c = flashOn(LED_SHIFT_FLASH_HALF_MS)
                               ? (night ? LED_COLOR_SHIFT_NIGHT : LED_COLOR_SHIFT_DAY)
                               : 0;
        for (uint8_t k = 0; k < LED_TACH_LEDS; k++)
            setLed(LED_STATUS_LEDS + k, c);
        }
    else
        {
        const uint16_t limit = warmupRpmLimit(g_data.ecu.oilTempC);

        for (uint8_t k = 0; k < LED_TACH_LEDS; k++)
            {
            const uint16_t threshold = TACH_START_RPM + (uint16_t)k * TACH_STEP_RPM;

            uint32_t c;
            if (rpm >= threshold)          // the live bar wins over the blue
                c = (k < TACH_GREEN_LEDS) ? LED_COLOR_TACH_GREEN
                                          : LED_COLOR_TACH_ORANGE;
            else if (threshold > limit)    // above the cold-oil rev limit
                c = LED_COLOR_WARMUP_BLUE;
            else
                c = 0;

            setLed(LED_STATUS_LEDS + k, c);
            }
        }

    s_strip.show();
    }
