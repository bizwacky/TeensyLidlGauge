// ---------------------------------------------------------------------------
// Rally CAN Gauge - FM24V10 F-RAM driver implementation
// ---------------------------------------------------------------------------
// Design rationale in fram.h. Chip specifics this file encodes (from the
// FM24V10 datasheet, "FRAM DataSheet.pdf" in the project root):
//
//   * 128 KB needs a 17-bit address. The LOW 16 bits travel as two address
//     bytes after the slave address; the 17th bit (which 64 KB half, the
//     "page") rides INSIDE the I2C slave address as bit 0 of 0x50. So the
//     chip answers on two bus addresses: 0x50 = lower half, 0x51 = upper.
//   * The internal address counter is a full 17 bits and auto-increments
//     across the page boundary, so a single transfer may straddle it - only
//     the STARTING address needs the right page bit.
//   * Writes commit to the ferroelectric array before the chip even ACKs the
//     byte - there is no "write cycle" to poll for (the EEPROM ritual). If
//     power dies mid-record, the bytes already clocked in are stored and the
//     rest simply never arrive - which is why records end in a CRC.
//
// The bus driver is teensy4_i2c's `Master` object = I2C1 on pins 18 (SDA) /
// 19 (SCL), matching the pinout table. Its interrupt handler moves the
// bytes; we only start transfers and check results.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <i2c_driver.h>
#include <imx_rt1060/imx_rt1060_i2c_driver.h>

#include "fram.h"
#include "config.h"
#include "vehicle_data.h"

// --- the write queue --------------------------------------------------------------
// A classic ring buffer of pending writes. Each entry is a fully-formed I2C
// payload: [address high, address low, data...] - prebuilt at enqueue time
// so the pump's job is only "hand the next buffer to the hardware".
//
// The entry at the TAIL is popped only after its transfer completes, because
// the I2C interrupt handler reads from the buffer while the transfer runs -
// popping early would let a new enqueue overwrite bytes mid-flight.

struct WriteReq
    {
    uint8_t devAddr;                  // 0x50 or 0x51 (page bit resolved at enqueue)
    uint8_t len;                      // bytes in buf that go on the wire
    uint8_t buf[2 + FRAM_MAX_WRITE];  // addr hi, addr lo, payload
    };

static WriteReq s_queue[FRAM_QUEUE_DEPTH];
static uint8_t  s_qTail;         // entry being transferred / next to transfer
static uint8_t  s_qCount;        // entries in the queue (including in-flight)
static bool     s_inFlight;      // hardware currently owns s_queue[s_qTail]
static elapsedMillis s_xferMs;   // time since the in-flight transfer started
static uint8_t  s_consecErrors;  // back-to-back failures; reset on any success

// The I2C slave address for a given memory address: 0x50 plus the 17th
// address bit (the 64 KB page select).
static inline uint8_t devAddrFor(uint32_t addr)
    {
    return FRAM_I2C_ADDR | ((addr >> 16) & 1);
    }

// --- error accounting -------------------------------------------------------------

static void countError(const char* what)
    {
    g_data.fram.writeErrors++;
    if (++s_consecErrors >= FRAM_MAX_CONSEC_ERRS && g_data.fram.present)
        {
        // The bus is dead (chip failed, wiring vibrated loose). Say so once,
        // then stand down - every entry point no-ops from here on and the
        // rest of the gauge never notices.
        g_data.fram.present = false;
        Serial.printf("FRAM: %u consecutive errors (last: %s) - giving up on the chip\n",
                      s_consecErrors, what);
        }
    }

// If a transfer has produced neither completion nor error for longer than
// any transfer could take, the bus has wedged (e.g. a noise glitch left the
// chip driving SDA). Re-initializing the peripheral is the recovery.
static void busReset()
    {
    Master.begin(FRAM_I2C_HZ);
    }

// --- public API -------------------------------------------------------------------

bool framInit()
    {
    Master.begin(FRAM_I2C_HZ);
    g_data.fram.present = true; // provisional - framRead() below checks it

    // Probe: read one byte from address 0. Any ACKing FM24V10 will serve it;
    // a missing/miswired chip NACKs its slave address and framRead fails.
    uint8_t probe;
    if (!framRead(0x0000, &probe, 1))
        {
        g_data.fram.present = false;
        Serial.println("FRAM: no response at 0x50 - persistence and black box disabled");
        return false;
        }

    Serial.printf("FRAM: FM24V10 online, %lu KB, I2C at %lu kHz\n",
                  (unsigned long)(FRAM_SIZE_BYTES / 1024),
                  (unsigned long)(FRAM_I2C_HZ / 1000));
    return true;
    }

void framPump()
    {
    if (!g_data.fram.present)
        return;

    if (s_inFlight)
        {
        if (!Master.finished())
            {
            if (s_xferMs > FRAM_XFER_TIMEOUT_MS)
                {
                busReset();
                countError("timeout");
                s_inFlight = false;
                s_qTail = (s_qTail + 1) % FRAM_QUEUE_DEPTH; // abandon this write
                s_qCount--;
                }
            return; // still clocking bits - come back next pass
            }

        // Transfer done: account for it and retire the queue entry.
        if (Master.has_error()) countError("nack/bus error");
        else                    { g_data.fram.writesDone++; s_consecErrors = 0; }
        s_inFlight = false;
        s_qTail = (s_qTail + 1) % FRAM_QUEUE_DEPTH;
        s_qCount--;
        }

    if (s_qCount == 0)
        return;

    // Bus is idle and work is waiting: light up the next transfer. From here
    // the I2C interrupts do everything; we'll reap it on a later pass.
    WriteReq& req = s_queue[s_qTail];
    Master.write_async(req.devAddr, req.buf, req.len, true /*send STOP*/);
    s_inFlight = true;
    s_xferMs   = 0;
    }

bool framWriteAsync(uint32_t addr, const void* src, uint8_t len)
    {
    if (!g_data.fram.present || len > FRAM_MAX_WRITE)
        return false;
    if (s_qCount >= FRAM_QUEUE_DEPTH)
        {
        g_data.fram.dropped++; // queue full - drop rather than block
        return false;
        }

    WriteReq& req = s_queue[(s_qTail + s_qCount) % FRAM_QUEUE_DEPTH];
    req.devAddr = devAddrFor(addr);
    req.buf[0]  = (addr >> 8) & 0xFF; // 16-bit chip address, big-endian on
    req.buf[1]  = addr & 0xFF;        // the wire per the datasheet
    memcpy(&req.buf[2], src, len);
    req.len = 2 + len;
    s_qCount++;

    framPump(); // start immediately if the bus happens to be idle
    return true;
    }

// Spin until the current transfer finishes. Only the blocking-read path uses
// this; the budget covers the largest console read plus margin.
static bool waitFinished(uint32_t budgetMs)
    {
    elapsedMillis waited;
    while (!Master.finished())
        {
        if (waited > budgetMs)
            {
            busReset();
            countError("read timeout");
            return false;
            }
        }
    if (Master.has_error())
        {
        countError("read nack/bus error");
        return false;
        }
    return true;
    }

bool framRead(uint32_t addr, void* dst, size_t len)
    {
    if (!g_data.fram.present)
        return false;

    // Order guarantee: finish everything queued before reading it back.
    elapsedMillis drain;
    while (!framIdle())
        {
        framPump();
        if (drain > 50) return false; // queue should drain in ~3 ms; this is a wedge
        }

    // An FM24V10 read is two chained transfers: a write of the two address
    // bytes WITHOUT a STOP (loading the chip's internal address latch), then
    // a repeated-START read which streams bytes from that address onward.
    uint8_t addrBytes[2] = { (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF) };
    Master.write_async(devAddrFor(addr), addrBytes, 2, false /*no STOP*/);
    if (!waitFinished(2))
        return false;

    Master.read_async(devAddrFor(addr), (uint8_t*)dst, len, true /*STOP*/);
    if (!waitFinished(2 + len / 50)) // ~9 us/byte at 1 MHz -> ~110 bytes/ms
        return false;

    s_consecErrors = 0;
    return true;
    }

bool framIdle()
    {
    return s_qCount == 0 && !s_inFlight;
    }

uint16_t framCrc16(const void* data, size_t len)
    {
    // CRC-16/CCITT, bit-at-a-time. A lookup table would be 16x faster but
    // this runs a handful of times a second on 32-byte records - clarity
    // wins. The properties that matter: catches ALL single-bit and burst
    // errors up to 16 bits, and a truncated record (power died mid-write)
    // has a 65535/65536 chance of failing the check.
    const uint8_t* p   = (const uint8_t*)data;
    uint16_t       crc = 0xFFFF;
    while (len--)
        {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    return crc;
    }
