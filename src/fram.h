// ---------------------------------------------------------------------------
// Rally CAN Gauge - FM24V10 F-RAM driver (non-blocking I2C)
// ---------------------------------------------------------------------------
// The low-level layer: moving bytes to/from the chip. What goes WHERE (record
// formats, the memory map, when to write) lives one floor up in fram_log.cpp
// - this file knows nothing about fuel or logging.
//
// WHY NON-BLOCKING: an I2C bus at 1 MHz moves roughly one byte per 9 us, so
// a 32-byte record plus addressing costs ~0.3 ms of BUS time. Done with the
// stock Wire library that is 0.3 ms of CPU time too - Wire spins in a loop
// until the last bit is clocked out. This driver instead uses the
// teensy4_i2c library, which runs the Teensy 4's I2C peripheral from
// interrupts: we hand it a buffer, it clocks bytes out in hardware while
// loop() keeps running, exactly like the display DMA and the WS2812Serial
// LEDs. The superloop's cost for a write is a memcpy into the queue (~1 us).
//
// THE WRITE QUEUE: only one transfer can own the bus at a time, but one tick
// may want several writes (log record + head pointer + fuel record). Writes
// are therefore queued - framWriteAsync() copies the payload and returns
// immediately - and framPump(), called every superloop pass, starts the next
// transfer whenever the bus is idle. Copying (rather than keeping a pointer)
// costs 32 bytes of memcpy but means callers can hand over stack variables
// and forget them - no "buffer must outlive the transfer" bugs.
//
// READS ARE BLOCKING, deliberately: reads only happen at boot (recovering
// state) and from the serial console (dumping the log), where simplicity
// beats latency. framRead() first drains the write queue so operations never
// reorder, then does an addressed read (~9 us per byte + 30 us setup).
//
// FAILURE POLICY: an absent or dead chip must never take the gauge down.
// Every entry point checks g_data.fram.present and quietly no-ops when the
// chip is gone; after FRAM_MAX_CONSEC_ERRS consecutive failed transfers the
// driver declares the chip absent (once, with a serial message). The gauge
// then runs exactly as it did before FRAM existed - fuel seeds from the
// sender, no black box.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

// Bring up the I2C bus and probe the chip. Call once from setup(), before
// framLogInit(). Returns true if the chip answered; the result also lands in
// g_data.fram.present.
bool framInit();

// Advance the write queue: reap the finished transfer (if any) and start the
// next queued one. Call EVERY superloop pass - costs well under 1 us when
// idle. This is the engine that makes "non-blocking" true.
void framPump();

// Queue an asynchronous write of len bytes (max FRAM_MAX_WRITE) to a chip
// address. Copies the data and returns immediately; the bytes hit the chip
// within a few ms. Returns false (and counts a drop) if the queue is full or
// the chip is absent.
bool framWriteAsync(uint32_t addr, const void* src, uint8_t len);

// Blocking read of len bytes. Drains the write queue first, so a read issued
// after a write is guaranteed to see that write's data. Boot + console use
// only - never call this on the fast tick. Returns false on any bus error.
bool framRead(uint32_t addr, void* dst, size_t len);

// True when no transfer is running and the queue is empty.
bool framIdle();

// CRC-16/CCITT (poly 0x1021, init 0xFFFF) - the integrity check every record
// format in fram_log.cpp appends. Lives here so record writers and readers
// share one implementation. ~1 us per 32-byte record at 600 MHz.
uint16_t framCrc16(const void* data, size_t len);
