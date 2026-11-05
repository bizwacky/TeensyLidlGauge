// ---------------------------------------------------------------------------
// Rally CAN Gauge - calculated fuel: injector integration + FUEL FILL gesture
// ---------------------------------------------------------------------------
// The precision half of the fuel picture. The ECU broadcasts the injector
// pulse width 50x a second; pulse width x flow rate x firing rate = fuel
// leaving the tank, integrated here into a running gallons-remaining
// estimate (g_data.fuel.calc*). The sender (inputs.cpp) stays the physical
// cross-check; the alarm logic in alarms.cpp plays them against each other.
//
// Fills are told to the gauge with the rotary dial's FUEL FILL detent:
// hold 30 s = +1 can (5 gal), hold 60 s = full-tank reset.
// ---------------------------------------------------------------------------

#pragma once

// Restore the accumulators from the FRAM fuel record (framFuelLoad) and
// announce the config on serial. Call AFTER framLogInit(); when no valid
// record exists (blank chip, no chip) the sender bootstrap in
// fuelCalcUpdate() seeds us a few seconds later instead.
void fuelCalcInit();

// Call on the 20 ms tick, after modesUpdate() (the fill gesture reads the
// resolved mode) and before alarmsUpdate() (which judges this tick's calc).
void fuelCalcUpdate();

// The accumulators exactly as integrated (doubles, not the float copies in
// g_data) - what fram_log.cpp persists every second they change. valid
// mirrors g_data.fuel.calcValid: false means the estimate has no seed yet
// and there is nothing worth saving.
void fuelCalcGetState(double& remainingGal, double& burnedTotalGal, bool& valid);
