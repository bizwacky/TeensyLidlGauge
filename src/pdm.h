// ---------------------------------------------------------------------------
// Rally CAN Gauge - PDM telemetry: per-output verdicts + lamp states
// ---------------------------------------------------------------------------
// can_bus.cpp decodes what the two PDMs SAY (state, amps, trip count per
// output). This module works out what that MEANS, once per tick:
//
//   * a verdict per output - OK / TRP / OPN / FLT - with the memory those
//     need (a trip lasts milliseconds but must stay readable for seconds;
//     "stuck" is a statement about the last five seconds; "open" needs a
//     load to have been on and dry for a while),
//   * the lamp states the indicator icons and night mode read, and
//   * the latest output event, for the black box.
//
// It sits between the decoder and the alarm table the way mixture.cpp sits
// between the lambda frame and the AFR alarms: alarms.cpp stays a table of
// one-line conditions, and nothing downstream ever looks at a raw nibble.
// Thresholds and the output catalogue: config.h. Rules and their reasons:
// the top of pdm.cpp.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include "config.h"
#include "vehicle_data.h"

void pdmInit();
void pdmUpdate(); // call on the 20 ms tick, before alarmsUpdate()

// Search every output of load class 'cls' on both boxes for a verdict
// between 'lo' and 'hi' (inclusive). Returns true and reports the WORST one
// found (ties: PDM-E before PDM-L, lower output first). This is the whole
// interface the alarm rows need.
bool pdmFindVerdict(LoadClass cls, PdmVerdict lo, PdmVerdict hi,
                    uint8_t& box, uint8_t& output, PdmVerdict& verdict);

// The box's device frame (state, board temperature, battery volts) is
// arriving - the gate for the PDME/PDML ERR and HOT rows.
bool pdmDeviceFresh(uint8_t box);

// "OK " / "TRP" / "OPN" / "FLT" - the 3-character word the banner prints.
const char* pdmVerdictName(uint8_t verdict);
