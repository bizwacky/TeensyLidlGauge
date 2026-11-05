// ---------------------------------------------------------------------------
// Rally CAN Gauge - local (non-CAN) inputs
// ---------------------------------------------------------------------------
// Everything wired directly to the gauge board:
//   * fuel sender (ratiometric resistance measurement)
//   * 12-position rotary mode switch (resistor-ladder divider, with
//     open-circuit detection for the DIALFAIL alarm)
//   * 12V switch inputs: headlights, brights, left/right blinkers
//   * Teensy die temperature
//   * battery-backed real-time clock
// Results land in g_data (see vehicle_data.h).
// ---------------------------------------------------------------------------

#pragma once

// Configure the ADC and pins, hook up the RTC. Call once from setup().
void inputsInit();

// Read every local input into g_data. Called on the 20 ms tick; total cost is
// well under 1 ms (six averaged ADC reads plus arithmetic).
void inputsPoll();

// Flush the fuel slosh filter so it re-converges quickly. Called by
// fuel_calc.cpp when the FUEL FILL gesture fires - fuel was just physically
// added, and the filter's memory of the old level is now ~1 min of lag.
void fuelSloshReset();
