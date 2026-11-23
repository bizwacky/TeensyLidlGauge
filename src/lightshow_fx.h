// ---------------------------------------------------------------------------
// Rally CAN Gauge - light show choreography (the "what it looks like" layer)
// ---------------------------------------------------------------------------
// One pure function from a position in a music track to a ShowFrame: no
// clock, no hardware, no g_data. lightshow.cpp decides WHEN a show runs and
// where in the track it is; this file decides what the lights do there.
//
// Being pure is what lets the same file run off the car: it is compiled
// unchanged into the simulator (tools/lightshow/) and into the host tests.
// Keep it free of Arduino calls and libc - integer maths and config.h only.
// ---------------------------------------------------------------------------

#pragma once

#include "vehicle_data.h"

// Compose the frame for 'songMs' milliseconds into the audio file of 'trk'.
void showRender(const ShowTrack& trk, int32_t songMs, ShowFrame& f);
