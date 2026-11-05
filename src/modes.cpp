// ---------------------------------------------------------------------------
// Rally CAN Gauge - rotary dial -> resolved gauge mode
// ---------------------------------------------------------------------------
// Sits between the raw input ("the dial is on position 4") and every
// consumer ("am I muted? how bright? night colors?"). The translation runs
// once per tick and lands in g_data.mode; consumers read those flags and
// nothing else. That indirection is the point of the module:
//
//   * one edit remaps the dial (ROTARY_MODE_MAP in config.h),
//   * a mode can affect several subsystems without them coordinating
//     (Dark mode dims the LEDs AND the backlights AND flips the colors to
//     the night scheme), and
//   * the spare detents (MODE_UNASSIGNED) already exist, so giving them a
//     job later can't reshuffle the dial the driver has learned.
//
// Deliberately stateless (no statics): the resolved mode is a pure function
// of this tick's inputs. If the dial says Dark, the gauge is dark - nothing
// here to get stuck.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "modes.h"
#include "config.h"
#include "vehicle_data.h"

void modesUpdate()
    {
    ModeData& m = g_data.mode;

    // inputsPoll() clamps rotaryPos to 0..ROTARY_POSITIONS-1, so this
    // lookup can't run off the table.
    m.mode = ROTARY_MODE_MAP[g_data.sw.rotaryPos];

    m.silent      = (m.mode == MODE_SILENT);
    m.displaysOff = (m.mode == MODE_DISPLAYS_OFF);

    // "Night" drives COLOR choices (e.g. the shift light flashes orange
    // instead of blue, to spare night vision). Headlights are the natural
    // signal - if you can't see the road you turn them on - and selecting
    // Dark mode is the driver saying the same thing explicitly.
    m.night = g_data.sw.headlights || (m.mode == MODE_DARK);

    // Brightness: the three brightness modes force their level; every other
    // mode (Silent, Fuel Reset, Debug, unassigned) defaults to MEDIUM -
    // predictable over clever: the driver always knows what those detents
    // look like. Note headlights do NOT dim these modes (they only flip the
    // color scheme via m.night above); if medium glares off the windshield
    // on a night stage, this default is the line to revisit.
    // LEDs and screen backlights resolve from the same dial position but
    // keep separate values: the strip and the panels are different light
    // sources and may want different tuning one day.
    switch (m.mode)
        {
        case MODE_BRIGHT: m.ledBrightness    = LED_BRIGHTNESS_BRIGHT;
                          m.screenBrightness = SCREEN_BRIGHTNESS_BRIGHT; break;
        case MODE_MEDIUM: m.ledBrightness    = LED_BRIGHTNESS_MEDIUM;
                          m.screenBrightness = SCREEN_BRIGHTNESS_MEDIUM; break;
        case MODE_DARK:   m.ledBrightness    = LED_BRIGHTNESS_DARK;
                          m.screenBrightness = SCREEN_BRIGHTNESS_DARK;   break;
        default:          m.ledBrightness    = LED_BRIGHTNESS_MEDIUM;
                          m.screenBrightness = SCREEN_BRIGHTNESS_MEDIUM; break;
        }

    // The modes that DO things act on m.mode in their own modules - this
    // one only RESOLVES the dial, it never acts on it:
    //   MODE_FUEL_RESET -> the hold-to-fill gesture (fuel_calc.cpp: it
    //                      mutates fuel state, so it belongs to the fuel
    //                      domain)
    //   MODE_DEBUG      -> the dense diagnostics pages (screens.cpp picks
    //                      gaugeRenderDebug instead of the gauge layouts)
    }

const char* modeName(GaugeMode m)
    {
    switch (m)
        {
        case MODE_BRIGHT:       return "BRIGHT";
        case MODE_MEDIUM:       return "MEDIUM";
        case MODE_DARK:         return "DARK";
        case MODE_DEBUG:        return "DEBUG";
        case MODE_SILENT:       return "SILENT";
        case MODE_FUEL_RESET:   return "FUELRST";
        case MODE_DISPLAYS_OFF: return "DISPOFF";
        default:                return "unassigned";
        }
    }
