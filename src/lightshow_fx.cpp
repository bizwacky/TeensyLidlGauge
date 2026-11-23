// ---------------------------------------------------------------------------
// Rally CAN Gauge - light show choreography
// ---------------------------------------------------------------------------
// The show has no recording of what to do when. It has the track's tempo,
// where its first downbeat falls, and a "map": one letter per four-bar
// phrase naming a LOOK (config.h lists them). Everything else is computed
// from the position in the music:
//
//   song time -> beats since the first downbeat (with a fraction) -> which
//   phrase -> which look -> that look's rule for this instant of this beat
//
// so a look is a few lines of "on the beat do this, between beats do that",
// and a new track costs one table row instead of a choreography file.
//
// WHO DOES WHAT. The three light sources behave very differently:
//
//   LED strip    instant, per-LED colour: carries the fast, detailed part.
//   screens      a full-colour repaint takes ~24 ms of SPI, so a panel only
//                changes COLOUR on a beat or a bar; its BRIGHTNESS is the
//                backlight PWM, which is instant - that is what strobes.
//   car lamps    reached through CAN and a PDM, drawing real current into
//                filaments: each lamp changes at most once per half beat,
//                and the forward lamps of the two PDMs take turns so the
//                battery never sees both boxes' lamps switch on together.
//
// Hues come from a wheel whose three channels always sum to 255, so a strip
// of any one hue draws the same current; only white needs the power limit.
// ---------------------------------------------------------------------------

#include "lightshow_fx.h"

static const uint32_t WHITE = 0xFFFFFF;

// Scale every channel of a colour by v / 255.
static uint32_t dim(uint32_t c, uint32_t v)
    {
    v++;
    return ((((c >> 16) & 0xFF) * v >> 8) << 16)
         | ((((c >>  8) & 0xFF) * v >> 8) <<  8)
         |  (((c        & 0xFF) * v >> 8));
    }

// The colour wheel: 0 red, 85 green, 170 blue, and round again.
static uint32_t wheel(uint8_t h)
    {
    const uint32_t x = (h % 85) * 3;
    if (h < 85)  return ((255 - x) << 16) | (x << 8);
    if (h < 170) return ((255 - x) << 8) | x;
    return (x << 16) | (255 - x);
    }

// The kick drum as a brightness: 255 on the beat, falling away to 0 before
// the next one. Squared, so it snaps rather than fades.
static uint8_t kick(uint8_t frac)
    {
    const uint32_t d = 255 - frac;
    return (uint8_t)(d * d >> 8);
    }

// 0 -> 255 -> 0 across one turn of an 8-bit phase.
static uint8_t tri(uint8_t x)
    {
    return (uint8_t)(x < 128 ? x * 2 : (255 - x) * 2);
    }

static void fill(ShowFrame& f, uint32_t strip, uint32_t screens, uint8_t backlight)
    {
    for (uint32_t& p : f.led) p = strip;
    for (uint8_t i = 0; i < 3; i++)
        {
        f.screen[i]    = screens;
        f.backlight[i] = backlight;
        }
    }

void showRender(const ShowTrack& trk, int32_t songMs, ShowFrame& f)
    {
    f = ShowFrame{}; // dark: the lead-in before beat 1, and a '-' phrase

    const int32_t t = songMs - trk.beat1Ms;
    if (t < 0) return;

    // Where we are in the music: beats since the first downbeat, with 8
    // fractional bits. 'frac' runs 0..255 across each beat.
    const uint32_t pos   = (uint32_t)((float)t * trk.bpm * (256.0f / 60000.0f));
    const uint32_t beat  = pos >> 8;
    const uint8_t  frac  = (uint8_t)pos;
    const bool     odd   = beat & 1;
    const bool     early = frac < 128; // the first half of the beat

    const char* map = trk.map ? trk.map : SHOW_DEFAULT_MAP;
    uint32_t len = 0;
    while (map[len]) len++;
    if (len == 0) return;
    const uint32_t phrase = beat / SHOW_PHRASE_BEATS;
    const auto look = [&](uint32_t p) { return map[p % len]; };

    const uint32_t hue = wheel((uint8_t)(beat / 4 * 37)); // a new colour every bar
    uint16_t lamps = 0;

    switch (look(phrase))
        {
        case '-':
            break;

        case 'c': // CHASE: a comet crosses the tach strip once a beat, there and back
            {
            int head = frac * LED_COUNT >> 8;
            if (odd) head = LED_COUNT - 1 - head;
            fill(f, dim(hue, 40), hue, 40);
            for (int i = 0; i < LED_COUNT; i++)
                {
                const int d = i > head ? i - head : head - i;
                f.led[i] = d == 0 ? WHITE : d < 4 ? dim(hue, 255 >> (d * 2)) : 0;
                }
            f.backlight[head * 3 / LED_COUNT] = 255; // ...and across the three screens
            lamps = SHOW_LAMP_TAIL | (odd ? SHOW_LAMP_TURN_R | SHOW_LAMP_BAR_WIDE
                                          : SHOW_LAMP_TURN_L | SHOW_LAMP_BAR_SPOT);
            break;
            }

        case 'b': // BUILD: the strip fills and the strobe doubles twice on the way up
            {
            // A run of consecutive b's is ONE build: find where it starts and ends.
            uint32_t a = phrase, z = phrase;
            while (a > 0 && phrase - a < 8 && look(a - 1) == 'b') a--;
            while (z - phrase < 8 && look(z + 1) == 'b') z++;
            const uint32_t total = (z - a + 1) * SHOW_PHRASE_BEATS;
            const uint32_t done  = beat - a * SHOW_PHRASE_BEATS;
            if (done == total - 1) break; // the breath before the drop: all dark

            const uint32_t prog = done * 256 / total; // 0..255 through the build
            uint8_t div = prog < 85 ? 1 : prog < 170 ? 2 : 4; // flashes per beat
            if (div > SHOW_MAX_FLASH_DIV) div = SHOW_MAX_FLASH_DIV;
            const bool on  = (uint8_t)(frac * div) < 96;
            const int  lit = 1 + (int)(prog * LED_COUNT / 256);

            fill(f, on ? dim(WHITE, 80) : 0, WHITE, on ? 255 : 0);
            for (int i = 0; i < LED_COUNT; i++)
                f.led[i] = i >= lit ? 0 : on ? WHITE : dim(hue, 60);
            lamps = odd ? SHOW_LAMP_BRAKE : SHOW_LAMP_REVERSE;
            if (prog >= 170 && early) lamps |= SHOW_LAMP_TURN_L | SHOW_LAMP_TURN_R;
            break;
            }

        case 'd': // DROP: everything, on the beat
            {
            const uint32_t c   = wheel((uint8_t)(beat * 53)); // a new colour every beat
            const uint32_t hot = c + dim(WHITE - c, kick(frac)); // white-hot on the kick,
                                                                 // cooling to the colour
            const uint8_t  div = SHOW_MAX_FLASH_DIV < 2 ? 1 : 2;
            fill(f, hot, c, (uint8_t)(frac * div) < 128 ? 255 : 0);
            // shimmer: alternate tach LEDs duck, swapping sides each subdivision
            for (int i = 0; i < LED_COUNT; i++)
                if ((i + (frac * SHOW_MAX_FLASH_DIV >> 8)) & 1) f.led[i] = dim(f.led[i], 50);
            // Forward lamps: PDM-L's set on even beats, PDM-E's on odd ones.
            lamps = SHOW_LAMP_TAIL;
            if (early)
                lamps |= odd ? SHOW_LAMP_YEL_LO | SHOW_LAMP_YEL_HI | SHOW_LAMP_BAR_WIDE
                               | SHOW_LAMP_TURN_R | SHOW_LAMP_BRAKE
                             : SHOW_LAMP_LOW | SHOW_LAMP_HIGH | SHOW_LAMP_BAR_SPOT
                               | SHOW_LAMP_TURN_L | SHOW_LAMP_REVERSE;
            break;
            }

        case 'k': // BREAK: colours drift along the strip, nothing flashes
            {
            const uint8_t drift = (uint8_t)(pos >> 4); // once round the wheel in 16 beats
            for (int i = 0; i < LED_TOTAL_COUNT; i++)
                f.led[i] = dim(wheel((uint8_t)(drift + i * 9)), 120);
            for (uint8_t i = 0; i < 3; i++)
                {
                // The screens follow in steps (one repaint a beat) and breathe
                // through their backlights, a third of a cycle apart.
                f.screen[i]    = wheel((uint8_t)(beat * 16 + i * 40));
                f.backlight[i] = (uint8_t)(80 + (tri((uint8_t)((pos >> 3) + i * 85)) * 175 >> 8));
                }
            lamps = SHOW_LAMP_TAIL;
            break;
            }

        default:  // PULSE ('p', and any letter not listed): the colour flashes on every beat
            {
            const uint8_t k = kick(frac);
            fill(f, dim(hue, k < 24 ? 24 : k), hue, k < 48 ? 48 : k);
            if (early)
                lamps = SHOW_LAMP_TAIL | (odd ? 0 : SHOW_LAMP_TURN_L | SHOW_LAMP_TURN_R);
            break;
            }
        }

    f.lamps = SHOW_DRIVE_LAMPS ? lamps : 0;

    // Fit the strip into its current budget (config.h): add up what this
    // frame would draw and, if that is too much, dim every LED by the same
    // factor - the frame keeps its shape and loses only brightness.
    uint32_t sum = 0;
    for (uint32_t c : f.led) sum += (c >> 16) + ((c >> 8) & 0xFF) + (c & 0xFF);
    const uint32_t mA = sum * SHOW_LED_MA_PER_CH / 255;
    if (mA > SHOW_LED_MAX_MA)
        for (uint32_t& c : f.led) c = dim(c, SHOW_LED_MAX_MA * 254 / mA);
    }
