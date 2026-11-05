// ---------------------------------------------------------------------------
// Rally CAN Gauge - display backlight PWM (3x low-side N-FET, phase-staggered)
// ---------------------------------------------------------------------------
// A pure CONSUMER module, like the LED tach: it reads the resolved mode
// (g_data.mode.screenBrightness + displaysOff) and drives the three
// backlight PWM pins. Per-screen trim, polarity, phase offsets, fade and
// all other tunables live in config.h, along with the hardware story (the
// FET switch, why 20 kHz, why the three pins all sit on FlexPWM4). The
// FlexPWM register mechanics are written up at the top of backlight.cpp.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

void backlightInit();   // call once in setup(), before the watchdog arms
                        // (it spends ~1 ms verifying the counter lock)
void backlightUpdate(); // call on the 20 ms tick, after modesUpdate()

// The duty a screen is ACTUALLY getting right now (mode level x trim, after
// the fade), 0..100%. For the 1 Hz debug print - this is the number you
// watch while tuning BL_TRIM on the bench.
float backlightDutyPct(uint8_t screen);

// Did backlightInit() confirm submodule 2 (pins 2/3) running in lockstep
// with submodule 0 (pin 22)? false = it fell back to free-running counters:
// every pin still gets correct PWM, but the phase between pin 22 and pins
// 2/3 is whatever it happens to be. Bench diagnostic.
bool backlightPhaseLocked();
