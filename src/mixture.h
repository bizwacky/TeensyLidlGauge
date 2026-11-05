// ---------------------------------------------------------------------------
// Rally CAN Gauge - mixture window: the gated two-second lambda average
// ---------------------------------------------------------------------------
// The AFR alarms are defined (per the plan) on a TWO-SECOND AVERAGE of the
// lambda error, never on single samples. This module keeps that average in
// g_data.ecu.lambdaErr2s - and decides which samples may enter it at all, so
// overrun and fuel cut can't masquerade as a lean engine. It only describes
// WHAT IS; the verdicts stay in alarms.cpp. Rules and rationale: mixture.cpp
// and the "mixture window" block in config.h.
// ---------------------------------------------------------------------------

#pragma once

// Call on the 20 ms tick, after modesUpdate() and before alarmsUpdate().
// Publishes lambdaErr2s, lambdaJudgeable and lambdaUnderLoad.
void mixtureUpdate();
