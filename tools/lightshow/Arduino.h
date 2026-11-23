// ---------------------------------------------------------------------------
// Stand-in for the Teensy core's <Arduino.h>, for building parts of the
// firmware OFF the car: the host tests (test_lightshow.cpp) and the browser
// simulator (sim_api.cpp). Only what src/config.h, src/vehicle_data.h and
// the light show sources touch - add to it as they grow. It is found
// instead of the real header because build.sh puts this folder first on
// the include path; the firmware build never sees it.
// ---------------------------------------------------------------------------

#pragma once

#include <stddef.h>
#include <stdint.h>

struct HardwareSerial
    {
    void printf(const char*, ...) {}
    };
extern HardwareSerial Serial, Serial7;

uint32_t millis(); // the tests drive this clock by hand
