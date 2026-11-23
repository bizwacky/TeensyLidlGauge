// A RAM model of the FM24V10 behind the real fram.h API: an 8-deep write
// queue that framPump() drains one entry at a time, like the bus does.
#include <Arduino.h>
#include "fram.h"
#include "config.h"
#include "vehicle_data.h"
#include "fram_fake.h"

uint8_t  g_framMem[FRAM_SIZE_BYTES];
int      g_framMinFree = FRAM_QUEUE_DEPTH; // lowest queue headroom seen
uint32_t g_framWrites;

struct Req { uint32_t addr; uint8_t len; uint8_t data[FRAM_MAX_WRITE]; };
static Req s_q[FRAM_QUEUE_DEPTH];
static int s_tail, s_count;

bool framInit() { g_data.fram.present = true; return true; }

void framPump()
    {
    if (!s_count) return;
    Req& r = s_q[s_tail];
    memcpy(&g_framMem[r.addr], r.data, r.len);
    s_tail = (s_tail + 1) % FRAM_QUEUE_DEPTH;
    s_count--;
    g_data.fram.writesDone++;
    }

bool framWriteAsync(uint32_t addr, const void* src, uint8_t len)
    {
    if (!g_data.fram.present || len > FRAM_MAX_WRITE) return false;
    if (addr + len > FRAM_SIZE_BYTES) { printf("FAKE FRAM: write past the end of the chip!\n"); abort(); }
    if (s_count >= FRAM_QUEUE_DEPTH) { g_data.fram.dropped++; return false; }
    Req& r = s_q[(s_tail + s_count) % FRAM_QUEUE_DEPTH];
    r.addr = addr; r.len = len; memcpy(r.data, src, len);
    s_count++;
    g_framWrites++;
    if (FRAM_QUEUE_DEPTH - s_count < g_framMinFree) g_framMinFree = FRAM_QUEUE_DEPTH - s_count;
    return true;
    }

bool framRead(uint32_t addr, void* dst, size_t len)
    {
    if (!g_data.fram.present) return false;
    while (s_count) framPump();
    if (addr + len > FRAM_SIZE_BYTES) { printf("FAKE FRAM: read past the end of the chip!\n"); abort(); }
    memcpy(dst, &g_framMem[addr], len);
    return true;
    }

bool    framIdle()      { return s_count == 0; }
uint8_t framQueueFree() { return g_data.fram.present ? (uint8_t)(FRAM_QUEUE_DEPTH - s_count) : 0; }
bool    framWriteSync(uint32_t addr, const void* src, uint8_t len)
    {
    if (!framWriteAsync(addr, src, len)) return false;
    while (s_count) framPump();
    return true;
    }

// identical to fram.cpp
uint16_t framCrc16(const void* data, size_t len)
    {
    const uint8_t* p = (const uint8_t*)data;
    uint16_t crc = 0xFFFF;
    while (len--)
        {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    return crc;
    }

// "the power died": everything still queued is lost
void fakeFramDropQueue() { s_count = 0; s_tail = 0; }
int  fakeFramQueued()    { return s_count; }
