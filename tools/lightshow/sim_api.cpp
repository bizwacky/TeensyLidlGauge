// ---------------------------------------------------------------------------
// Light show simulator - the firmware side
// ---------------------------------------------------------------------------
// The whole "engine" of tools/lightshow/lightshow_sim.html is this file plus
// src/lightshow_fx.cpp, compiled to WebAssembly and then to plain JavaScript
// (build.sh). The page therefore draws exactly the frames the gauge would
// compute - there is no second implementation of the choreography to keep
// in step. The page owns the clock and the audio; it asks for one frame per
// animation tick.
//
// Freestanding: no libc exists in this build, hence the memset/memcpy below
// (the compiler emits calls to them for struct copies).
// ---------------------------------------------------------------------------

#include "lightshow_fx.h"

#define EXPORT(name) __attribute__((export_name(#name)))

HardwareSerial Serial, Serial7;
uint32_t millis() { return 0; }

extern "C" void* memset(void* d, int c, size_t n)
    {
    for (size_t i = 0; i < n; i++) ((uint8_t*)d)[i] = (uint8_t)c;
    return d;
    }
extern "C" void* memcpy(void* d, const void* s, size_t n)
    {
    for (size_t i = 0; i < n; i++) ((uint8_t*)d)[i] = ((const uint8_t*)s)[i];
    return d;
    }

static char      s_map[512]; // the page writes the map being auditioned here
static ShowFrame s_frame;

EXPORT(sim_map) char* sim_map() { return s_map; }

// One frame of a track with the given tempo and first beat and the map in
// sim_map() (empty = the firmware's default map). Returns the ShowFrame.
EXPORT(sim_render) const ShowFrame* sim_render(float bpm, int beat1Ms, int songMs)
    {
    const ShowTrack trk = { 0, "", bpm, (int16_t)beat1Ms, 0, s_map[0] ? s_map : nullptr };
    showRender(trk, songMs, s_frame);
    return &s_frame;
    }
