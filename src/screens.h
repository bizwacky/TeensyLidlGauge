// ---------------------------------------------------------------------------
// Rally CAN Gauge - the three NV3007 displays (the "how it reaches glass" layer)
// ---------------------------------------------------------------------------
// Owns the display drivers, framebuffers and diff buffers, and runs the
// per-tick draw -> diff -> DMA pipeline. What appears in a frame is
// gauge_render.cpp's job; this module only moves finished frames to the
// panels, without ever blocking the superloop.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

void screensInit();   // bring up all three panels (before watchdogInit - the
                      // NV3007 init sequence contains sleep-out delays)
void screensUpdate(); // call on the 20 ms tick, after alarmsUpdate()

// Instrumentation for the 1 Hz debug print: worst tick cost (draw + diff +
// DMA kickoff, in us) since the last call (read-and-reset), and how many
// per-display frames were skipped because their DMA was still streaming.
uint32_t screensDrawUsMax();
uint32_t screensSkippedFrames();
