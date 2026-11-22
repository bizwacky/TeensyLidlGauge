// ---------------------------------------------------------------------------
// Rally CAN Gauge - cabin fan speed outputs (3x logic-level PWM)
// ---------------------------------------------------------------------------
// Three PWM signals tell the defrost fan and the two seat fans how fast to
// run; a roof-panel pot sets each one. The fans' 12V comes from PDM-L - the
// gauge supplies only the speed signal (and, over CAN, the "this pot is
// turned up" bit the PDM uses to switch the feed). Hardware, frequency and
// polarity notes: config.h, "cabin fan PWM outputs".
// ---------------------------------------------------------------------------

#pragma once

// Set the PWM frequency and park every output at 0 %. Call once from setup().
void fansInit();

// Pot positions -> duty on the three pins, published in g_data.fans. Called
// on the 20 ms tick, after inputsPoll().
void fansUpdate();
