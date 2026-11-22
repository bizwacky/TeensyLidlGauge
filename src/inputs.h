// ---------------------------------------------------------------------------
// Rally CAN Gauge - local (non-CAN) inputs
// ---------------------------------------------------------------------------
// Everything wired directly to the gauge board:
//   * fuel sender (ratiometric resistance measurement)
//   * 12-position rotary mode switch (resistor-ladder divider, with
//     open-circuit detection for the DIALFAIL alarm)
//   * the roof switch panel: six ground-switched switches (debounced) and
//     five pots (percent of travel, with open-wire detection)
//   * Teensy die temperature
//   * battery-backed real-time clock
// Results land in g_data (see vehicle_data.h). What the switches and pots
// DO is not decided here: can_bus.cpp broadcasts them to the PDMs and
// fans.cpp turns three of the pots into fan PWM.
// ---------------------------------------------------------------------------

#pragma once

// Configure the ADC and pins, hook up the RTC. Call once from setup().
void inputsInit();

// Read every local input into g_data. Called on the 20 ms tick; total cost is
// well under 1 ms (seven averaged ADC reads plus arithmetic).
void inputsPoll();

// Flush the fuel slosh filter so it re-converges quickly. Called by
// fuel_calc.cpp when the FUEL FILL gesture fires - fuel was just physically
// added, and the filter's memory of the old level is now ~1 min of lag.
void fuelSloshReset();
