// ---------------------------------------------------------------------------
// Rally CAN Gauge - FRAM persistence: fuel record + black-box flight recorder
// ---------------------------------------------------------------------------
// What the FRAM stores and when. Two customers share the chip:
//
//   1. THE FUEL RECORD - the calculated-fuel accumulators (gallons remaining
//      + lifetime gallons burned), saved every second they change and read
//      back at boot, so the injector-integration estimate survives the kill
//      switch. fuel_calc.cpp calls framFuelLoad() at init; the sender
//      bootstrap it always had remains the fallback for a blank/corrupt/
//      missing FRAM.
//
//   2. THE BLACK BOX - a circular log of one 44-byte snapshot per second-ish
//      (all gauge fields + active warnings + g), the emergency backup for
//      when the ECU's own log is lost with the car in a hedge. ~74 minutes of
//      nominal running fits; details and gating in fram_log.cpp, sizes and
//      cadence in config.h, on-chip layout in docs/FRAM_BLACKBOX.md.
//
// Two smaller tenants: the IMU's level calibration (two slots here) and the
// impact captures (fram_events.cpp, the top 32 KB of the chip).
//
// Also owns the little serial console (the only serial INPUT in the
// firmware): press 'h' in the monitor for help - dump the log as CSV, show
// the fuel slots, list and dump the impact captures, format the chip.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

// Mount the chip: verify the format header (formatting a virgin chip),
// recover the log head + session counter. Call from setup() after framInit();
// safe to call when the chip is absent (everything downstream no-ops).
void framLogInit();

// Fetch the persisted fuel record. Returns true and fills the accumulators
// only if a slot with a valid CRC and sane values exists. Called by
// fuelCalcInit() - which falls back to its sender bootstrap on false.
bool framFuelLoad(double& remainingGal, double& burnedTotalGal);

// The IMU's LEVEL SET result: the direction of "up" in the gauge's nominal
// axes (motion.cpp). Load returns false when there is none (blank chip, no
// chip, never calibrated). Save queues an asynchronous write and returns.
bool framImuCalLoad(float up[3]);
void framImuCalSave(const float up[3]);

// The 1 s tick: save the fuel record if it changed, and append a black-box
// record if the gating says this second is worth remembering.
void framLogTick1s();

// Every superloop pass: the serial console + the chunked CSV dump pump
// (a dump reads/prints a few records per pass so the tick never stalls).
void framLogPoll();
