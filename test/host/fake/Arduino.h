// Minimal stand-in for the Teensy core, for PC-side builds of the gauge's
// pure-logic and rendering code.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>

#ifndef PROGMEM
#define PROGMEM
#endif
#define DMAMEM

extern uint32_t g_fakeMillis;
static inline uint32_t millis() { return g_fakeMillis; }
static inline uint32_t micros() { return g_fakeMillis * 1000u; }
static inline void delay(uint32_t) {}

template <class T, class L, class H>
static inline T constrain(T v, L lo, H hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }
template <class A, class B>
static inline auto min(A a, B b) -> typename std::common_type<A, B>::type { return a < b ? a : b; }
template <class A, class B>
static inline auto max(A a, B b) -> typename std::common_type<A, B>::type { return a > b ? a : b; }

class HardwareSerial
    {
public:
    bool quiet = false;
    int printf(const char* fmt, ...)
        {
        if (quiet) return 0;
        va_list ap; va_start(ap, fmt); int n = vprintf(fmt, ap); va_end(ap); return n;
        }
    void println(const char* s = "") { if (!quiet) puts(s); }
    void print(const char* s) { if (!quiet) fputs(s, stdout); }
    void print(char c) { if (!quiet) putchar(c); }
    int available() { return 0; }
    int read() { return -1; }
    void begin(unsigned long) {}
    };
extern HardwareSerial Serial, Serial7;

class elapsedMillis
    {
    uint32_t start;
public:
    elapsedMillis() : start(millis()) {}
    operator uint32_t() const { return millis() - start; }
    elapsedMillis& operator=(uint32_t v) { start = millis() - v; return *this; }
    elapsedMillis& operator-=(uint32_t v) { start += v; return *this; }
    };
