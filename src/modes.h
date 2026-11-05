// ---------------------------------------------------------------------------
// Rally CAN Gauge - rotary dial -> resolved gauge mode
// ---------------------------------------------------------------------------
// Translates the debounced rotary position (g_data.sw.rotaryPos) through
// ROTARY_MODE_MAP into the decision flags in g_data.mode. The one place in
// the firmware that knows what a dial position MEANS.
// ---------------------------------------------------------------------------

#pragma once

#include "config.h"

void modesUpdate(); // call on the 20 ms tick, right after inputsPoll()

const char* modeName(GaugeMode m); // short fixed name, for debug prints
