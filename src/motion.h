// ---------------------------------------------------------------------------
// Rally CAN Gauge - motion: what the IMU's samples mean
// ---------------------------------------------------------------------------
// imu.cpp delivers raw counts in the chip's own axes; this module turns them
// into g_data.imu - the car's motion in the car's axes:
//
//   * ORIENTATION: sensor axes -> vehicle axes (the fixed map in config.h,
//     then the LEVEL SET rotation that takes out the mounting angle)
//   * ATTITUDE: roll and pitch, gyro integrated + accelerometer corrected
//   * THE G-METER: smoothed lon/lat/vert from the low-g channel
//   * THE IMPACT CHANNEL: the high-g samples, offset-trimmed against the
//     low-g channel and peak-held for the CAN frames
//   * JUMPS: air time and landing
//   * IMPACT CAPTURES: the trigger, and a ring of the last few seconds of
//     decimated waveform that fram_events.cpp writes to the FRAM
//
// Everything runs in loop() context: the feed functions are called from
// imuPump() as FIFO bursts arrive, motionUpdate() on the 20 ms tick.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include "config.h"

// Load the level calibration from the FRAM and build the orientation. Call
// from setup() after framLogInit() (and after imuInit()).
void motionInit();

// One sample each, raw counts in SENSOR axes (X, Y, Z). Called by imu.cpp in
// the order the chip sampled them.
void motionFeedHighG(const int16_t counts[3]); // 960 Hz
void motionFeedLowG(const int16_t counts[3]);  // 120 Hz
void motionFeedGyro(const int16_t counts[3]);  // 120 Hz

// The sample stream has a hole in it (FIFO overrun, chip restarted): don't
// integrate across it, and don't build a capture's lead-in from before it.
void motionOnDataGap();

// The 20 ms tick: publish g_data.imu and run the LEVEL SET gesture. Call
// after modesUpdate() (the gesture reads the resolved mode).
void motionUpdate();

// --- for the black box (fram_log.cpp) ------------------------------------------------
// What happened since the previous call, then start over. 'valid' is false
// when no IMU data arrived in the interval.
struct MotionLogSample
    {
    bool  valid;
    float lonMeanG;  // average longitudinal acceleration over the interval
    float latMeanG;  // average lateral acceleration
    float peakG;     // largest total acceleration (high-g channel)
    };
MotionLogSample motionLogTake();

// --- for the impact capture store (fram_events.cpp) ----------------------------------
// One block of a capture, exactly as it is stored: IMU_EVENT_BLOCK_FRAMES
// frames of (lon, lat, vert) as companded bytes (imu_codec.h), then the
// attitude at the block's end.
static const uint8_t MOTION_BLOCK_BYTES = IMU_EVENT_BLOCK_FRAMES * 3 + 3;

struct MotionEventView
    {
    uint32_t id;           // = g_data.imu.event.id
    bool     complete;     // every block of it has been produced
    uint8_t  blocksReady;  // finished blocks so far, 0..IMU_EVENT_BLOCKS
    uint8_t  preBlocks;    // how many of them precede the trigger block
    uint8_t  triggerFrame; // frame within the trigger block where it fired
    uint16_t peakFrame;    // frame (from the capture's start) of the largest hit
    uint8_t  flags;        // MOTION_EVF_*
    };

static const uint8_t MOTION_EVF_LEVEL_CAL = 0x01; // axes were level-calibrated
static const uint8_t MOTION_EVF_HG_TRIM   = 0x02; // high-g offset trim had settled
static const uint8_t MOTION_EVF_DATA_GAP  = 0x04; // samples were lost during the capture
static const uint8_t MOTION_EVF_CLIPPED   = 0x08; // the sensor hit its full scale

// The capture in progress or, after it, the last one. Returns false if there
// has been none since boot.
bool motionEventView(MotionEventView& view);

// Block 'index' (0-based) of that capture, MOTION_BLOCK_BYTES long. Returns
// nullptr if it is not finished yet or has already left the RAM ring.
const uint8_t* motionEventBlock(uint8_t index);

// Ids continue from the newest capture on the chip: the store calls this
// once at mount with the id the next capture should carry.
void motionEventSetNextId(uint32_t id);

// The store is still writing the previous capture: hold the next trigger
// until it is done (a few ticks; the hit stays in the ring meanwhile and
// becomes the new capture's lead-in).
void motionEventStoreBusy(bool busy);
