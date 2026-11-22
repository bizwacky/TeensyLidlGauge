// ---------------------------------------------------------------------------
// Rally CAN Gauge - cabin fan speed outputs implementation
// ---------------------------------------------------------------------------
// Deliberately the simplest module in the firmware: a pure function of this
// tick's pot readings, like modes.cpp is of the dial. No ramps, no state
// beyond "what did I last write" - the pot filter in inputs.cpp already
// smooths the input, and a fan's own inertia smooths the output.
//
// The mapping, per fan:
//
//   pot inactive (turned down, see POT_ON_PCT/POT_OFF_PCT)   -> 0 %
//   pot active                -> FAN_MIN_DUTY_PCT at the bottom of its
//                                travel, rising linearly to
//                                FAN_MAX_DUTY_PCT at the top
//   pot wire open             -> inputs.cpp already replaced the reading
//                                with that pot's failsafe percent, so the
//                                same mapping applies
//
// 0 % does not necessarily STOP a PWM-input fan (many idle at minimum
// speed). Stopping is the PDM's job: it drops the fan's 12V when the pot's
// "active" bit in the gauge's CAN frame goes to 0.
//
// analogWrite() is fine here - unlike the backlights (backlight.cpp) nothing
// needs these three edges placed relative to each other, and the load on the
// pin is a logic input, not a rail-shaking 100 mA.
//
// IF THE GAUGE IS DEAD or resetting, the pins float. A 4-wire fan's PWM
// input has its own pull-up, so a powered fan then runs at FULL speed -
// loud, but the fail-on direction for a defrost fan.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "fans.h"
#include "config.h"
#include "vehicle_data.h"

// analogWrite() scale at the core's default 8-bit resolution.
static const int k_pwmFullScale = 255;

static int16_t s_lastWritten[FAN_COUNT] = { -1, -1, -1 }; // -1 = nothing written yet

static void writeDuty(uint8_t fan, uint8_t dutyPct)
    {
    // Polarity lives in exactly one place: if a FET stage ever inverts the
    // signal, the pin gets the complement and everything above stays in
    // "percent of speed".
    const uint8_t pinPct = FAN_PWM_ACTIVE_HIGH ? dutyPct : (uint8_t)(100 - dutyPct);
    const int16_t counts = (int16_t)((pinPct * k_pwmFullScale + 50) / 100);

    if (counts == s_lastWritten[fan])
        return; // unchanged: leave the timer alone
    analogWrite(PIN_FAN_PWM[fan], counts);
    s_lastWritten[fan] = counts;
    }

void fansInit()
    {
    for (uint8_t i = 0; i < FAN_COUNT; i++)
        {
        analogWriteFrequency(PIN_FAN_PWM[i], FAN_PWM_HZ);
        writeDuty(i, 0);
        g_data.fans.dutyPct[i] = 0;
        }

    Serial.printf("Fans: %u PWM outputs on pins %u/%u/%u at %.0f Hz (%s)\n",
                  (unsigned)FAN_COUNT, PIN_FAN_PWM[0], PIN_FAN_PWM[1], PIN_FAN_PWM[2],
                  FAN_PWM_HZ, FAN_PWM_ACTIVE_HIGH ? "high = speed" : "inverted");
    }

void fansUpdate()
    {
    for (uint8_t i = 0; i < FAN_COUNT; i++)
        {
        const PotData& pot = g_data.sw.pot[FAN_POT[i]];

        float duty = 0.0f;
        if (pot.active)
            duty = FAN_MIN_DUTY_PCT
                   + (FAN_MAX_DUTY_PCT - FAN_MIN_DUTY_PCT) * (pot.pct / 100.0f);

        const uint8_t dutyPct = (uint8_t)constrain(duty + 0.5f, 0.0f, 100.0f);
        writeDuty(i, dutyPct);
        g_data.fans.dutyPct[i] = dutyPct;
        }
    }
