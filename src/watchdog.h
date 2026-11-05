// ---------------------------------------------------------------------------
// Rally CAN Gauge - hardware watchdog
// ---------------------------------------------------------------------------
// The safety net for a device you can't reach mid-stage: a hardware timer
// inside the chip that reboots it unless the firmware keeps proving it is
// alive. Firmware hangs (a wedged loop, a peripheral deadlock, a crash) ->
// the feed stops -> WATCHDOG_TIMEOUT_MS later the chip hard-resets and the
// gauges are back about a second after that. No library needed - it is a
// ~10-line register driver (watchdog.cpp) over the i.MX RT1062's WDOG1.
// ---------------------------------------------------------------------------

#pragma once

// Print why the chip last reset (power-on vs watchdog vs other) - call early
// in setup() so a watchdog reboot never goes unnoticed during development.
void watchdogPrintResetCause();

// Arm the watchdog. Call LAST in setup(): from this moment the feed clock is
// ticking, and once armed the hardware cannot be disarmed except by reset.
void watchdogInit();

// Prove we're alive. Called from the 20 ms tick - deliberately from the tick
// and not from bare loop(), so the watchdog also catches the case where
// loop() still spins but the tick machinery has died.
void watchdogFeed();
