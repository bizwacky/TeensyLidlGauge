// ---------------------------------------------------------------------------
// Rally CAN Gauge - FRAM impact captures
// ---------------------------------------------------------------------------
// The third thing the FRAM stores, next to the fuel record and the black
// box: a few seconds of high-g waveform around each hard hit - a landing, a
// bottoming-out, a crash. motion.cpp detects the hit and keeps the waveform
// in a RAM ring; this module copies it to the chip WHILE IT HAPPENS, so a
// capture survives the kill switch being pulled (or the battery leaving the
// car) a second into the event.
//
// The captures live in fixed slots at the top of the chip, apart from the
// black-box ring: however many bumps a stage has, they can only overwrite
// each other, never the engine log. Sizes and the keep-the-biggest rule are
// in config.h; the on-chip format is at the top of the .cpp and in
// docs/FRAM_BLACKBOX.md.
//
// All calls are safe with the chip absent or unmounted (they do nothing).
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

// Read every slot's header, so the store knows what the chip holds and
// which capture id comes next. Called by framLogInit() once the chip is
// mounted (after a format, if one was needed).
void framEventsMount();

// Blank every slot. Called by the chip format; blocking, console/boot only.
void framEventsFormat();

// The 20 ms tick: move the capture in progress from motion.cpp's ring to
// the chip, a few writes at a time. Call after motionUpdate().
void framEventsTick();

// The serial console. framLogPoll() owns the port and offers each key here
// first; returns true if the key was one of ours ('e', 'E', 'C').
bool framEventsConsoleKey(char c);
void framEventsConsoleHelp();

// Every superloop pass while a capture dump is printing (it is chunked like
// the log dump). framEventsDumping() says whether one is running; any key
// aborts it via framEventsDumpAbort().
void framEventsDumpPump();
bool framEventsDumping();
void framEventsDumpAbort(const char* why);

// "Y" after "C": the confirmation for clearing the captures. Returns true if
// a clear was armed and has now been carried out (so the key is consumed).
// Arming the chip format ("X") disarms a pending clear, so "Y" only ever
// confirms the command typed last.
bool framEventsConfirmClear();
void framEventsDisarmClear();
