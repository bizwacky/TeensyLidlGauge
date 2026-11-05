// ---------------------------------------------------------------------------
// Rally CAN Gauge - display plumbing implementation
// ---------------------------------------------------------------------------
// THE MEMORY LAYOUT (the part worth understanding first):
//
// A 428x142 RGB565 frame is 121 KB. The Teensy 4.0 has two RAM banks:
// 512 KB of fast DTCM (where normal variables and the stack live) and
// 512 KB of OCRAM (the "DMAMEM" region - slower for the CPU, but the DMA
// engines read it without competing with code execution). Four full frames
// only fit in OCRAM:
//
//   OCRAM: [internal fb 1][internal fb 2][internal fb 3][shared draw fb]
//           121 KB         121 KB         121 KB         121 KB   = 486 KB
//
// Each display keeps its own INTERNAL framebuffer = the pixels currently on
// its glass. There is only ONE draw buffer, shared by all three: the tick
// draws screen 1 into it, hands it to driver 1, then reuses it for screen 2,
// and so on. That works because update() copies what it needs into the
// display's internal buffer before returning - the draw buffer is free to
// be overwritten the moment update() returns, while the DMA transfer to the
// glass streams on in the background from the internal copy.
//
// THE PER-TICK PIPELINE (x3, sequentially):
//
//   asyncUpdateActive()?  ->  still streaming last frame: skip this display
//                             entirely this tick (don't even draw - the
//                             frame would be dropped anyway, so drawing it
//                             would be pure waste)
//   gaugeRender...()      ->  compose the full frame into the shared buffer
//   update()              ->  diff against the internal fb, copy the changed
//                             pixels in, kick off DMA, return immediately
//
// The skip almost never triggers: a typical gauge frame changes a few
// thousand pixels (~1 ms of SPI at 40 MHz), done long before the next tick.
// It exists for the worst case - a full-frame change like a night-mode flip
// costs ~24 ms of streaming, which overlaps INTO the next tick; that next
// frame is sacrificed rather than ever stalling the superloop. VSync
// spacing -1 tells the driver the same thing ("drop, never wait") as a
// second line of defense.
//
// Displays-off mode pushes one black frame per panel and then goes idle -
// the backlights are already fading to zero (backlight.cpp), but black
// pixels also kill the faint glow a lit panel leaks around a dark image,
// and skipping the per-tick draw returns ~3 ms of CPU to the loop.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <ILI9341_T4.h>

#include "screens.h"
#include "config.h"
#include "vehicle_data.h"
#include "gauge_render.h"

// --- the memory ------------------------------------------------------------------

DMAMEM static uint16_t s_internalFb[3][SCREEN_LX * SCREEN_LY]; // per-display "what's on glass"
DMAMEM static uint16_t s_drawFb[SCREEN_LX * SCREEN_LY];        // the one shared canvas

// Diff buffers hold the computed change-list between two frames (two per
// display so the next diff can build while the previous upload streams).
// These are small and CPU-touched, so they stay in fast DTCM - OCRAM is
// fully booked with framebuffers anyway.
static ILI9341_T4::DiffBuffStatic<SCREEN_DIFF_BUF_BYTES> s_diff[3][2];

// --- the drivers -----------------------------------------------------------------
// One driver object per panel, each on its own SPI bus (the driver works
// out which LPSPI peripheral owns the pins it was given). MISO = 255: the
// NV3007 is write-only - see PORTING_NV3007.md.

static ILI9341_T4::ILI9341Driver s_tft0(SCREEN_PINS[0].cs, SCREEN_PINS[0].dc,
                                        SCREEN_PINS[0].sck, SCREEN_PINS[0].mosi,
                                        255, SCREEN_PINS[0].rst);
static ILI9341_T4::ILI9341Driver s_tft1(SCREEN_PINS[1].cs, SCREEN_PINS[1].dc,
                                        SCREEN_PINS[1].sck, SCREEN_PINS[1].mosi,
                                        255, SCREEN_PINS[1].rst);
static ILI9341_T4::ILI9341Driver s_tft2(SCREEN_PINS[2].cs, SCREEN_PINS[2].dc,
                                        SCREEN_PINS[2].sck, SCREEN_PINS[2].mosi,
                                        255, SCREEN_PINS[2].rst);

static ILI9341_T4::ILI9341Driver* const s_tft[3] = { &s_tft0, &s_tft1, &s_tft2 };

// --- instrumentation -------------------------------------------------------------

static uint32_t s_drawUsMax  = 0; // worst whole-tick cost since last read
static uint32_t s_skipped    = 0; // frames skipped because DMA was still busy
static bool     s_blackDone[3];   // displays-off: black frame already pushed?

uint32_t screensDrawUsMax()
    {
    const uint32_t v = s_drawUsMax;
    s_drawUsMax = 0;
    return v;
    }

uint32_t screensSkippedFrames()
    {
    return s_skipped;
    }

// --- public API ------------------------------------------------------------------

void screensInit()
    {
    for (uint8_t i = 0; i < 3; i++)
        {
        s_tft[i]->output(&Serial); // init/debug chatter -> USB serial

        // begin() sends the vendor init sequence. It cannot detect a dead
        // panel (nothing to read back without MISO), so it always succeeds;
        // a miswired display simply stays dark while the loop runs on.
        s_tft[i]->begin(SCREEN_SPI_HZ);

        s_tft[i]->setRotation(0);                          // native landscape 428x142
        s_tft[i]->setFramebuffer(s_internalFb[i]);         // double buffering on
        s_tft[i]->setDiffBuffers(&s_diff[i][0], &s_diff[i][1]);
        s_tft[i]->setDiffGap(SCREEN_DIFF_GAP);
        s_tft[i]->setVSyncSpacing(-1);                     // never wait: drop instead
        }

    Serial.printf("Screens: 3x NV3007 up at %lu MHz SPI\n",
                  (unsigned long)(SCREEN_SPI_HZ / 1000000));
    }

void screensUpdate()
    {
    const ModeData& m = g_data.mode;

    if (m.displaysOff)
        {
        // One black frame per panel (respecting the busy-skip), then idle.
        for (uint8_t i = 0; i < 3; i++)
            {
            if (s_blackDone[i] || s_tft[i]->asyncUpdateActive()) continue;
            memset(s_drawFb, 0, sizeof(s_drawFb));
            s_tft[i]->update(s_drawFb);
            s_blackDone[i] = true;
            }
        return;
        }
    for (uint8_t i = 0; i < 3; i++) s_blackDone[i] = false;

    const uint32_t t0 = micros();
    const bool debugLayout = (m.mode == MODE_DEBUG);

    for (uint8_t i = 0; i < 3; i++)
        {
        if (s_tft[i]->asyncUpdateActive())
            {
            s_skipped++; // previous frame still streaming: sacrifice this one
            continue;
            }

        if (debugLayout) gaugeRenderDebug(s_drawFb, i);
        else             gaugeRenderScreen(s_drawFb, SCREEN_CONTENT[i]);

        s_tft[i]->update(s_drawFb); // diff + copy + DMA kickoff, returns fast
        }

    const uint32_t us = micros() - t0;
    if (us > s_drawUsMax) s_drawUsMax = us;
    }
