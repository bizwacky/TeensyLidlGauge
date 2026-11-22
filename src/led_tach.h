// ---------------------------------------------------------------------------
// Rally CAN Gauge - WS2812B LED strip: status block + tachometer + shift light
// ---------------------------------------------------------------------------
// A pure CONSUMER module: it reads g_data (rpm, oil temp, alarm verdicts,
// resolved mode) and paints the 15-LED strip. It never evaluates a threshold
// the alarm system owns - anyRed/anyYellow arrive pre-judged from alarms.cpp,
// and silent/night/brightness/displays-off arrive pre-resolved from
// modes.cpp. All tunable numbers live in config.h.
//
// It also owns the switch-label backlights: extra WS2812s on the end of the
// same data chain, held at one steady color (they never join the tach's
// flashing). See config.h, LED_BACKLIGHT_COUNT.
// ---------------------------------------------------------------------------

#pragma once

void ledTachInit();   // call once in setup(), before the watchdog arms
void ledTachUpdate(); // call on the 20 ms tick, AFTER alarmsUpdate() - it
                      // paints this tick's verdicts, not last tick's
