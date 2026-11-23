// ---------------------------------------------------------------------------
// Rally CAN Gauge - LSM6DSV320X IMU driver (non-blocking FIFO drain)
// ---------------------------------------------------------------------------
// The low-level layer: getting samples out of the chip. What the samples
// MEAN - which way is forward, roll and pitch, peaks, jumps, impact captures
// - lives one floor up in motion.cpp. This file knows registers and I2C and
// nothing about cars.
//
// WHY A FIFO AND NOT "READ IT EVERY TICK": the reason this chip is on the
// board is its +-320 g channel, and a hit worth recording lasts a few
// milliseconds. The superloop looks every 20 ms. So the chip samples on its
// own clock (high-g at 960 Hz, low-g and gyroscope at 120 Hz) into its
// internal FIFO, and this driver empties the FIFO a few hundred times a
// second. Nothing between two looks is ever missed, and the sample timing
// is the chip's, not the loop's. (The usual alternative - a data-ready
// interrupt - is not available: INT1/INT2 are not routed on this board.)
//
// NON-BLOCKING, like the FRAM driver and for the same reason: an I2C byte
// takes 9 us at 1 MHz and a FIFO burst is up to 252 of them. imuPump() only
// starts transfers and collects finished ones; the I2C interrupt moves the
// bytes while loop() runs on. The bus is the IMU's own (LPI2C2, pins 40/41),
// so the FRAM's writes and the IMU's reads never wait on each other.
//
// FAILURE POLICY: a missing or dead IMU must never take the gauge down.
// Without it g_data.imu simply goes stale (imuOnline() false): the motion
// screens draw "--", the CAN frames say "no IMU", nothing else notices. The
// driver retries a silent chip every few seconds.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include "config.h"

// What one count of each channel is worth, at the full scales imu.cpp sets
// (low-g +-16 g, gyro +-2000 dps, high-g per IMU_HG_FULL_SCALE_G). From the
// datasheet's table 3; change a full-scale code in imu.cpp, change its line
// here.
static const float IMU_LG_G_PER_COUNT     = 0.488e-3f; // g per count at +-16 g
static const float IMU_GYRO_DPS_PER_COUNT = 70.0e-3f;  // deg/s per count at +-2000 dps
static constexpr float IMU_HG_G_PER_COUNT =
      (IMU_HG_FULL_SCALE_G == 32  ? 0.976e-3f
     : IMU_HG_FULL_SCALE_G == 64  ? 1.952e-3f
     : IMU_HG_FULL_SCALE_G == 128 ? 3.904e-3f
     : IMU_HG_FULL_SCALE_G == 256 ? 7.808e-3f
     :                              10.417e-3f);

// Bring up the bus, identify and configure the chip. Call once from setup(),
// before motionInit(). Sampling starts with the first imuPump() call, so the
// FIFO is not left filling while the rest of setup() runs. Returns true if
// the chip answered and accepted its configuration (also in
// g_data.imu.present).
bool imuInit();

// Advance the FIFO drain: collect a finished transfer, hand its samples to
// motion.cpp, start the next. Call EVERY superloop pass - a few hundred
// nanoseconds when there is nothing to do.
void imuPump();

// Once a second (the debug tick): refresh the measured sample rate.
void imuTick1s();
