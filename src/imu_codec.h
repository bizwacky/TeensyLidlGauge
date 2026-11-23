// ---------------------------------------------------------------------------
// Rally CAN Gauge - acceleration <-> one byte
// ---------------------------------------------------------------------------
// The FRAM stores accelerations as single bytes: three per frame of an
// impact capture, one per black-box record. A linear byte over +-320 g
// would step in 2.5 g - useless for a 6 g landing - so the bytes are
// SQUARE-ROOT COMPANDED: the code is proportional to the square root of the
// value, which spends the resolution where the car spends its life.
//
//   signed, 127 codes each way (impact captures):   g = fs * (code / 127)^2
//
//       reading       step between codes
//        0.5 g             0.2 g
//        6   g             0.7 g
//       30   g             1.6 g
//      100   g             2.8 g
//      320   g             5.0 g
//
// So a decoded sample is within half a step: 0.35 g at 6 g, 1.4 g at 100 g
// (under 6 % of the reading from 6 g up, under 3 % from 30 g up). That is a
// waveform to look at, not to do metrology on - which is why each capture's
// header carries its exact peaks as 16-bit values.
//
// Header-only and free of Arduino, so the PC-side tests build it as is.
// ---------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <math.h>

// Signed: -fs..+fs -> -127..+127. Saturates outside the full scale.
static inline int8_t imuCompand(float g, float fullScale)
    {
    float a = fabsf(g) / fullScale;
    if (!(a < 1.0f)) a = 1.0f;               // also catches NaN
    const int code = (int)(sqrtf(a) * 127.0f + 0.5f);
    return (int8_t)(g < 0.0f ? -code : code);
    }

static inline float imuExpand(int8_t code, float fullScale)
    {
    const float u = code / 127.0f;
    return (code < 0 ? -u * u : u * u) * fullScale;
    }

// Unsigned: 0..fs -> 0..255, for magnitudes (twice the resolution).
static inline uint8_t imuCompandU(float g, float fullScale)
    {
    float a = g / fullScale;
    if (!(a > 0.0f)) a = 0.0f;
    if (a > 1.0f)    a = 1.0f;
    return (uint8_t)(sqrtf(a) * 255.0f + 0.5f);
    }

static inline float imuExpandU(uint8_t code, float fullScale)
    {
    const float u = code / 255.0f;
    return u * u * fullScale;
    }
