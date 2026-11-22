// ---------------------------------------------------------------------------
// Rally CAN Gauge - the warning/alarm state machine
// ---------------------------------------------------------------------------
// Evaluates every alarm in the catalogue against g_data each tick and
// publishes the results to g_data.alarms. Consumers (screens, LED tach,
// audio) read ONLY those results - all thresholds, gating and debounce live
// here and in config.h, in exactly one place. (The PDM rows get their
// per-output verdicts from pdm.cpp, which runs just before this.)
//
// Each alarm goes through a tiny state machine:
//
//     raw condition true  --- for setMs ---->  ACTIVE
//     raw condition false --- for clearMs -->  inactive
//
// plus threshold hysteresis inside the condition itself. The two mechanisms
// kill different kinds of flicker: hysteresis stops a value HOVERING at the
// threshold from oscillating, persistence stops a brief SPIKE (one noisy
// sample, fuel slosh, a shift's rpm flare) from flashing a warning at the
// driver who is mid-corner.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include "vehicle_data.h"
#include "audio.h"

void alarmsInit();
void alarmsUpdate(); // call on the 20 ms tick, after inputs + derived values
                     // (mixtureUpdate, pdmUpdate)

// Read-only access to the alarm definition table (for display + audio).
// id is an AlarmId; out-of-range ids return safe values.
const char* alarmName(uint8_t id);     // 8 characters, space padded. For the
                                       // PDM rows the text is written at run
                                       // time ("PUMP FLT", "PDML CAN") and can
                                       // change while the alarm is up - read
                                       // it each time, don't cache the pointer's
                                       // contents.
AlarmColor  alarmColor(uint8_t id);
AlarmTier   alarmTier(uint8_t id);     // live / deferred (not acted on yet)
uint8_t     alarmPriority(uint8_t id); // catalogue number: 1 = most important
AudioClip   alarmClip(uint8_t id);     // CLIP_NONE = silent alarm
uint32_t    alarmRepeatMs(uint8_t id); // 0 = announce once per activation
