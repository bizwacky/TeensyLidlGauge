// ---------------------------------------------------------------------------
// Rally CAN Gauge - the warning/alarm state machine
// ---------------------------------------------------------------------------
// Evaluates every alarm in the plan's catalog against g_data each tick and
// publishes the results to g_data.alarms. Consumers (screens, LED tach,
// audio) read ONLY those results - all thresholds, gating and debounce live
// here and in config.h, in exactly one place.
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

// Read-only access to the alarm definition table (for display + audio).
// id is an AlarmId; out-of-range ids return safe values.
const char* alarmName(uint8_t id);     // exactly 8 characters, from the plan
AlarmColor  alarmColor(uint8_t id);
uint8_t     alarmPriority(uint8_t id); // the plan's number: 1 = most important
AudioClip   alarmClip(uint8_t id);     // CLIP_NONE = silent alarm
uint32_t    alarmRepeatMs(uint8_t id); // 0 = announce once per activation
