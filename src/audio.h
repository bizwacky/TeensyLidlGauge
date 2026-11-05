// ---------------------------------------------------------------------------
// Rally CAN Gauge - audio warnings (GPWS-style callouts)
// ---------------------------------------------------------------------------
// The plan: "All Systems Nominal" on boot, and spoken callouts for alarms -
// with MORE detail than the 8-character text (a low-voltage warning says
// "ALTERNATOR", because that's the actual thing to go look at).
//
// Two halves live here: ARBITRATION (read the alarm states from g_data once
// per second, decide which single clip - if any - to start, honoring
// priority, per-alarm repeat intervals, and never talking over itself) and
// the DRIVER (a DFPlayer Mini on AUDIO_SERIAL via the Makuna/DFMiniMp3
// library - see audio.cpp for the design notes). The rotary dial's SILENT
// position mutes everything for when a known-bad sensor would nag all stage.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

// Every voice clip the system can play. Track number = enum value: clip N
// is the SD card file /mp3/000N*.mp3 (4-digit zero-padded prefix, anything
// after it is ignored) - e.g. CLIP_ALTERNATOR = 6 -> /mp3/0006-alternator.mp3.
// The folder must be named "mp3" at the card root. Addressing is by filename
// prefix, so the order files were copied onto the card does not matter.
enum AudioClip : uint8_t
    {
    CLIP_NONE = 0,
    CLIP_NOMINAL,      // "all systems nominal"
    CLIP_OIL_PRESSURE, // "oil pressure"
    CLIP_OIL_TEMP,     // "oil temperature"
    CLIP_WATER_TEMP,   // "coolant temperature"
    CLIP_AFR_LEAN,     // "mixture lean"
    CLIP_ALTERNATOR,   // "alternator" (the CAUSE of low voltage, per the plan)
    CLIP_OVERVOLT,     // "overvoltage"
    CLIP_CELL_LOW,     // "battery cell critical"
    CLIP_CELL_BAL,     // "battery unbalanced"
    CLIP_BATT_TEMP,    // "battery temperature"
    CLIP_FUEL_LOW,     // "fuel low"
    CLIP_NO_GAS,       // "fuel critical"
    CLIP_NO_CAN,       // "E C U link lost"
    CLIP_BATT_CAN,     // "battery link lost"
    CLIP_DIAL_FAIL,    // "mode dial fault" (rotary switch reads open)
    CLIP_COUNT
    };

void audioInit();
void audioLoop();   // every superloop pass: pump DFPlayer notifications
void audioTick1s(); // arbitration runs on the 1 s tick, per the plan
