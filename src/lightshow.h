// ---------------------------------------------------------------------------
// Rally CAN Gauge - the light show (MODE_LIGHTSHOW)
// ---------------------------------------------------------------------------
// Music from the DFPlayer with the LED strip, screens, backlights and the
// car's exterior lamps flashing in time. This module owns WHEN: the dial
// gesture, the countdown, the interlocks, the clock that follows the music.
// What the lights do at each moment is lightshow_fx.cpp. Its output is
// g_data.show; while show.active is set the strip, screens, backlights and
// CAN transmitter play show.frame instead of doing their normal work.
// Tunables and the track list: config.h. User guide: docs/LIGHTSHOW.md.
// ---------------------------------------------------------------------------

#pragma once

// Call on the 20 ms tick, after alarmsUpdate() (a red alarm ends the show)
// and before everything that reads g_data.show: canBusTick20ms(),
// ledTachUpdate(), screensUpdate(), backlightUpdate().
void lightshowUpdate();
