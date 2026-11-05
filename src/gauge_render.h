// ---------------------------------------------------------------------------
// Rally CAN Gauge - gauge frame rendering (the "what to draw" layer)
// ---------------------------------------------------------------------------
// Pure functions from g_data to pixels: each call composes one complete
// 428x142 RGB565 frame into the caller's buffer and returns. No hardware
// knowledge lives here - screens.cpp owns the displays and hands this module
// a framebuffer to fill. The split mirrors the rest of the firmware: this
// module decides what the driver SEES, screens.cpp decides how it reaches
// the glass.
//
// The layouts are the bench-tested PC prototype from ../Graphics/
// (GaugeClusterPrototype.h), extended with the alarm-driven color system -
// see the design story at the top of gauge_render.cpp.
//
// The interface passes a raw uint16_t* on purpose: it keeps the heavyweight
// tgx headers out of every other translation unit (only gauge_render.cpp
// pays for them), and RGB565-in-a-uint16 is already the contract the
// display driver uses on the other side.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include "config.h"

// Compose one full frame of the given layout into fb (SCREEN_LX x SCREEN_LY).
void gaugeRenderScreen(uint16_t* fb, ScreenContent content);

// The MODE_DEBUG dense diagnostics layout, page 0..2 (one per display).
void gaugeRenderDebug(uint16_t* fb, uint8_t page);
