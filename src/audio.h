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
// Tracks 0100 and up in the same folder are the light show's music
// (config.h, SHOW_TRACKS) - never played by the alarm system.
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
    // --- added Nov 2026 with the PDM alarms: tracks 0016..0023. Appended, so
    //     every earlier clip keeps its track number and its file. ---
    CLIP_ENGINE_POWER, // "engine power fault" (pump / coil / main feed dead or dying)
    CLIP_PUMP_TRIP,    // "fuel pump tripping"
    CLIP_FAN_FAULT,    // "fan fault"
    CLIP_PDM_TEMP,     // "P D M temperature"
    CLIP_PDM_FAULT,    // "P D M fault"
    CLIP_PDM_LINK,     // "P D M link lost"
    CLIP_LIGHT_FAULT,  // "light fault" (also wipers and defrost: the vision class)
    CLIP_CHECK_MSG,    // "check messages" (something that can wait for service)
    CLIP_COUNT
    };

void audioInit();
void audioLoop();   // every superloop pass: pump DFPlayer notifications
void audioTick1s(); // arbitration runs on the 1 s tick, per the plan

// --- music for the light show (lightshow.cpp) ---------------------------------
// The show's tracks live in the same /mp3 folder as the voice clips, as
// /mp3/0100*.mp3 upward (config.h, SHOW_TRACKS). While g_data.show.active is
// set the alarm voice stays quiet - a red alarm ends the show first, then
// speaks; anything yellow that came up during the show speaks after it.
bool audioMusicReady();             // the module has booted (or its grace period is over)
void audioMusicArm();               // countdown started: set the music volume
void audioMusicPlay(uint16_t file); // start /mp3/<file>
void audioMusicStop();
bool audioMusicFinished();          // the module reported the end of the track,
                                    // or an error (e.g. the file is not on the card)
