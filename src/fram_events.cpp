// ---------------------------------------------------------------------------
// Rally CAN Gauge - FRAM impact captures implementation
// ---------------------------------------------------------------------------
// ONE SLOT = ONE CAPTURE, 4096 bytes (FRAM_EVENT_SLOT_BYTES):
//
//   +0     header, copy A    64 B
//   +64    header, copy B    64 B
//   +128   100 blocks x 39 B   the waveform, oldest first (3900 B)
//   +4028  unused              68 B
//
// One BLOCK is 50 ms: 12 frames of (lon, lat, vert), one companded byte
// each (imu_codec.h - decode with the full scale in the header), then three
// attitude bytes for the block's end: roll / 2 deg, pitch deg, yaw rate / 4
// deg/s. A frame is 4 high-g samples; per axis it keeps the one of largest
// size, so no peak is lost to the decimation.
//
// WRITTEN AS IT HAPPENS. The capture that matters most is the one that ends
// with the power gone, so nothing waits for the end of the 5 s window:
//
//   trigger      both header copies are written at once (state RECORDING,
//                peaks so far), then the lead-in blocks already in RAM,
//                three per tick
//   +50 ms each  the next block, as motion.cpp finishes it
//   every 0.5 s  a header copy is rewritten with the blocks-on-chip count
//                and the peaks so far - copies A and B ALTERNATE, so the
//                one being written when the power dies is never the only one
//   the end      a last header, state COMPLETE
//
// Recovery is the same rule as everywhere on this chip: of the two header
// copies, the newest one whose CRC passes wins. A capture cut short reads
// back as "INCOMPLETE, n blocks" with at most half a second missing from
// its tail.
//
// WHICH SLOT. Empty ones first. Then the oldest capture - except that the
// IMU_EVENT_KEEP_BIGGEST hardest hits are never overwritten. Captures are
// numbered from 1 for the life of the chip (or until they are cleared), and
// the black-box records carry the number of the latest one, which is how a
// capture is found in the engine log and the other way round.
//
// PACING. A block is one queued write of 39 bytes; the FRAM driver's queue
// is 8 deep and drains in about 5 ms. This module only queues while more
// than QUEUE_RESERVE entries are free, so the once-a-second fuel and
// black-box records never find the queue full because of a capture.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <math.h>

#include "fram_events.h"
#include "fram.h"
#include "motion.h"
#include "imu_codec.h"
#include "config.h"
#include "vehicle_data.h"

// --- on-chip format ---------------------------------------------------------------

static const uint16_t EVENT_MAGIC        = 0xE30E;
static const uint8_t  EV_STATE_RECORDING = 1;
static const uint8_t  EV_STATE_COMPLETE  = 2;

struct __attribute__((packed)) EventHeader           // 64 B, two copies per slot
    {
    uint16_t magic;         // EVENT_MAGIC
    uint8_t  state;         // EV_STATE_*
    uint8_t  rev;           // +1 per rewrite within a capture: newest copy wins
    uint32_t id;            // capture number, 1.. (0 is never used)
    uint32_t logSeq;        // black-box sequence number at the trigger
    uint8_t  session;       // black-box session at the trigger
    uint8_t  hh, mm, ss;    // RTC at the trigger, 0xFF = clock not set
    uint8_t  blocksWritten; // blocks on the chip when this copy was written
    uint8_t  blocksTotal;   // IMU_EVENT_BLOCKS
    uint8_t  preBlocks;     // blocks before the trigger block
    uint8_t  triggerFrame;  // frame within the trigger block where it fired
    uint16_t hgRateHz;      // high-g sample rate \  frame period =
    uint8_t  decim;         // samples per frame   /  decim / hgRateHz
    uint8_t  blockFrames;   // frames per block
    uint8_t  flags;         // MOTION_EVF_* (motion.h)
    uint8_t  tpsPct;        // throttle at the trigger
    uint16_t rpm;           // engine speed at the trigger
    uint16_t fullScaleG;    // companding full scale of the waveform bytes
    uint16_t trigG100;      // trigger level in force, 0.01 g
    uint16_t peakG100;      // largest total acceleration, 0.01 g
    int16_t  peakLon100;    // per axis: the reading of largest size, signed,
    int16_t  peakLat100;    //  0.01 g
    int16_t  peakVert100;
    uint16_t peakFrame;     // frame (from the capture's start) of peakG
    uint16_t overMs;        // time at or above the trigger level
    int16_t  dvLon100;      // delta-v, 0.01 m/s
    int16_t  dvLat100;
    int16_t  dvVert100;
    uint16_t airMs;         // air time that ended in this hit (0 = none)
    uint8_t  pad[10];
    uint16_t crc;           // CRC-16/CCITT of everything above
    };
static_assert(sizeof(EventHeader) == 64, "capture header must be exactly 64 bytes");
static_assert(sizeof(EventHeader) <= FRAM_MAX_WRITE, "the FRAM write queue must hold a capture header");
static_assert(MOTION_BLOCK_BYTES <= FRAM_MAX_WRITE, "the FRAM write queue must hold a capture block");

static const uint32_t SLOT_OFFSET_BLOCKS = 2 * sizeof(EventHeader);
static_assert(SLOT_OFFSET_BLOCKS + (uint32_t)IMU_EVENT_BLOCKS * MOTION_BLOCK_BYTES <= FRAM_EVENT_SLOT_BYTES,
              "a capture must fit its slot");
static_assert(FRAM_ADDR_EVENTS + (uint32_t)FRAM_EVENT_SLOTS * FRAM_EVENT_SLOT_BYTES == FRAM_SIZE_BYTES,
              "the capture slots must end exactly at the top of the chip");
static_assert(FRAM_ADDR_EVENTS > FRAM_ADDR_LOG, "the capture slots must sit above the black-box ring");
static_assert(IMU_EVENT_KEEP_BIGGEST < FRAM_EVENT_SLOTS, "at least one slot must be free to recycle");

static uint32_t slotAddr(uint8_t slot)
    {
    return FRAM_ADDR_EVENTS + (uint32_t)slot * FRAM_EVENT_SLOT_BYTES;
    }
static uint32_t headerAddr(uint8_t slot, uint8_t copy)
    {
    return slotAddr(slot) + (uint32_t)copy * sizeof(EventHeader);
    }
static uint32_t blockAddr(uint8_t slot, uint8_t block)
    {
    return slotAddr(slot) + SLOT_OFFSET_BLOCKS + (uint32_t)block * MOTION_BLOCK_BYTES;
    }

// --- module state -----------------------------------------------------------------

// What each slot holds, kept in RAM so choosing a slot costs no I2C.
struct SlotInfo
    {
    bool     valid;
    uint32_t id;
    uint16_t peakG100;
    };

static bool     s_mounted;
static SlotInfo s_slot[FRAM_EVENT_SLOTS];

// The capture being written.
static bool        s_recOpen;
static uint8_t     s_recSlot;
static uint8_t     s_recBlocks;    // blocks handed to the write queue
static uint8_t     s_recHdrBlocks; // ...as of the last header written
static uint8_t     s_recRev;
static bool        s_recLost;      // a block left the RAM ring before it was written
static EventHeader s_recHdr;       // the fields fixed at the trigger
static uint32_t    s_doneId;       // id of the last capture finished

static const uint8_t QUEUE_RESERVE       = 4;  // free queue entries left for fuel + black box
static const uint8_t BLOCKS_PER_TICK     = 3;  // catching up on the lead-in: 20 blocks in 7 ticks
static const uint8_t HEADER_EVERY_BLOCKS = 10; // progress header every 0.5 s

// --- helpers ----------------------------------------------------------------------

static inline int16_t enc100(float v)
    {
    return (int16_t)lroundf(constrain(v * 100.0f, -32767.0f, 32767.0f));
    }

// The newest valid header copy of a slot.
static bool readHeader(uint8_t slot, EventHeader& out)
    {
    bool found = false;
    for (uint8_t copy = 0; copy < 2; copy++)
        {
        EventHeader h;
        if (!framRead(headerAddr(slot, copy), &h, sizeof(h))) continue;
        if (h.magic != EVENT_MAGIC || h.id == 0) continue;
        if (framCrc16(&h, sizeof(h) - 2) != h.crc) continue;
        if (!found || h.id > out.id || (h.id == out.id && h.rev > out.rev))
            {
            out   = h;
            found = true;
            }
        }
    return found;
    }

// Where the next capture goes: an empty slot, else the oldest capture that
// is not one of the IMU_EVENT_KEEP_BIGGEST hardest hits.
static uint8_t chooseSlot()
    {
    for (uint8_t i = 0; i < FRAM_EVENT_SLOTS; i++)
        if (!s_slot[i].valid) return i;

    bool keep[FRAM_EVENT_SLOTS] = {};
    for (uint8_t k = 0; k < IMU_EVENT_KEEP_BIGGEST; k++)
        {
        int8_t best = -1;
        for (uint8_t i = 0; i < FRAM_EVENT_SLOTS; i++)
            if (!keep[i] && (best < 0 || s_slot[i].peakG100 > s_slot[best].peakG100)) best = (int8_t)i;
        if (best >= 0) keep[best] = true;
        }

    int8_t oldest = -1;
    for (uint8_t i = 0; i < FRAM_EVENT_SLOTS; i++)
        if (!keep[i] && (oldest < 0 || s_slot[i].id < s_slot[oldest].id)) oldest = (int8_t)i;
    return (uint8_t)oldest; // never -1: KEEP_BIGGEST < SLOTS (static_assert above)
    }

// Write one header copy for the capture in progress: the fixed fields from
// the trigger plus the numbers as they stand now.
static bool writeHeader(const MotionEventView& v, uint8_t state)
    {
    const ImuEventData& e = g_data.imu.event;

    EventHeader h   = s_recHdr;
    h.state         = state;
    h.rev           = (uint8_t)(s_recRev + 1);
    h.blocksWritten = s_recBlocks;
    h.flags         = (uint8_t)(v.flags | (s_recLost ? MOTION_EVF_DATA_GAP : 0));
    h.peakG100      = (uint16_t)lroundf(constrain(e.peakG * 100.0f, 0.0f, 65535.0f));
    h.peakLon100    = enc100(e.peakLonG);
    h.peakLat100    = enc100(e.peakLatG);
    h.peakVert100   = enc100(e.peakVertG);
    h.peakFrame     = v.peakFrame;
    h.overMs        = e.overMs;
    h.dvLon100      = enc100(e.dvLonMps);
    h.dvLat100      = enc100(e.dvLatMps);
    h.dvVert100     = enc100(e.dvVertMps);
    h.airMs         = e.airMs;
    h.crc           = framCrc16(&h, sizeof(h) - 2);

    if (!framWriteAsync(headerAddr(s_recSlot, h.rev & 1), &h, sizeof(h)))
        return false;

    s_recRev       = h.rev;
    s_recHdrBlocks = s_recBlocks;
    s_slot[s_recSlot].valid    = true;
    s_slot[s_recSlot].id       = h.id;
    s_slot[s_recSlot].peakG100 = h.peakG100;
    return true;
    }

static void openCapture(const MotionEventView& v)
    {
    if (framEventsDumping())
        framEventsDumpAbort("ABORTED (a new impact is being recorded)");

    s_recSlot      = chooseSlot();
    s_recBlocks    = 0;
    s_recHdrBlocks = 0;
    s_recRev       = 0;
    s_recLost      = false;
    s_recOpen      = true;
    motionEventStoreBusy(true);

    const BoardData& b = g_data.board;
    s_recHdr = {};
    s_recHdr.magic        = EVENT_MAGIC;
    s_recHdr.id           = v.id;
    s_recHdr.logSeq       = g_data.fram.logSeq;
    s_recHdr.session      = g_data.fram.session;
    s_recHdr.hh           = b.clockValid ? b.hh : 0xFF;
    s_recHdr.mm           = b.clockValid ? b.mm : 0xFF;
    s_recHdr.ss           = b.clockValid ? b.ss : 0xFF;
    s_recHdr.blocksTotal  = IMU_EVENT_BLOCKS;
    s_recHdr.preBlocks    = v.preBlocks;
    s_recHdr.triggerFrame = v.triggerFrame;
    s_recHdr.hgRateHz     = IMU_HG_RATE_HZ;
    s_recHdr.decim        = IMU_EVENT_DECIM;
    s_recHdr.blockFrames  = IMU_EVENT_BLOCK_FRAMES;
    s_recHdr.fullScaleG   = IMU_HG_FULL_SCALE_G;
    s_recHdr.trigG100     = (uint16_t)lroundf(IMU_EVENT_TRIGGER_G * 100.0f);
    if (g_data.ecuOnline())
        {
        s_recHdr.rpm    = g_data.ecu.rpm;
        s_recHdr.tpsPct = (uint8_t)constrain(g_data.ecu.tpsPct, 0.0f, 100.0f);
        }

    // Both copies straight away: whatever was in this slot before, neither
    // of its headers survives to be mistaken for this capture's.
    writeHeader(v, EV_STATE_RECORDING);
    writeHeader(v, EV_STATE_RECORDING);

    Serial.printf("IMPACT #%lu: recording to slot %u (%.1f g so far)\n",
                  (unsigned long)v.id, s_recSlot, g_data.imu.event.peakG);
    }

// --- public API: mount / format / tick ------------------------------------------------

void framEventsMount()
    {
    s_mounted = false;
    if (!g_data.fram.present)
        return;

    uint32_t newest = 0, count = 0;
    float    biggest = 0.0f;
    for (uint8_t i = 0; i < FRAM_EVENT_SLOTS; i++)
        {
        EventHeader h;
        s_slot[i] = {};
        if (!readHeader(i, h)) continue;
        s_slot[i].valid    = true;
        s_slot[i].id       = h.id;
        s_slot[i].peakG100 = h.peakG100;
        count++;
        if (h.id > newest) newest = h.id;
        if (h.peakG100 / 100.0f > biggest) biggest = h.peakG100 / 100.0f;
        }
    motionEventSetNextId(newest + 1);
    s_mounted = true;

    if (count)
        Serial.printf("FRAM: %lu impact capture(s) on the chip, newest #%lu, hardest %.1f g ('e' lists them)\n",
                      (unsigned long)count, (unsigned long)newest, biggest);
    else
        Serial.printf("FRAM: no impact captures yet (%u slots, trigger %.1f g)\n",
                      FRAM_EVENT_SLOTS, IMU_EVENT_TRIGGER_G);
    }

void framEventsFormat()
    {
    if (!g_data.fram.present)
        return;
    const uint8_t zeros[sizeof(EventHeader)] = {};
    for (uint8_t i = 0; i < FRAM_EVENT_SLOTS; i++)
        {
        framWriteSync(headerAddr(i, 0), zeros, sizeof(zeros));
        framWriteSync(headerAddr(i, 1), zeros, sizeof(zeros));
        s_slot[i] = {};
        }
    s_recOpen = false;
    motionEventStoreBusy(false);
    }

void framEventsTick()
    {
    if (!s_mounted || !g_data.fram.present)
        {
        // No chip (or it stopped answering mid-capture): captures still
        // happen in RAM and on CAN, they just are not kept.
        s_recOpen = false;
        motionEventStoreBusy(false);
        return;
        }

    MotionEventView v;
    if (!motionEventView(v))
        return;

    if (!s_recOpen)
        {
        if (v.id == s_doneId)
            return; // the last capture is safely on the chip; nothing new
        if (framQueueFree() <= QUEUE_RESERVE + 1)
            return; // (never in practice) wait a tick for the queue to drain
        openCapture(v);
        }

    // Blocks: everything motion.cpp has finished and we have not yet queued.
    uint8_t budget = BLOCKS_PER_TICK;
    while (budget > 0 && s_recBlocks < v.blocksReady && framQueueFree() > QUEUE_RESERVE)
        {
        static const uint8_t lost[MOTION_BLOCK_BYTES] = {};
        const uint8_t* b = motionEventBlock(s_recBlocks);
        if (!b) s_recLost = true; // it left the RAM ring first: zeros keep the time axis whole
        if (!framWriteAsync(blockAddr(s_recSlot, s_recBlocks), b ? b : lost, MOTION_BLOCK_BYTES))
            break;
        s_recBlocks++;
        budget--;
        }

    if (v.complete && s_recBlocks >= v.blocksReady)
        {
        if (framQueueFree() > QUEUE_RESERVE && writeHeader(v, EV_STATE_COMPLETE))
            {
            const ImuEventData& e = g_data.imu.event;
            Serial.printf("IMPACT #%lu: stored. peak %.1f g (lon %+.1f lat %+.1f vert %+.1f), "
                          "dv %+.1f/%+.1f/%+.1f m/s, %u ms over %.0f g\n",
                          (unsigned long)v.id, e.peakG, e.peakLonG, e.peakLatG, e.peakVertG,
                          e.dvLonMps, e.dvLatMps, e.dvVertMps, e.overMs, IMU_EVENT_TRIGGER_G);
            s_doneId  = v.id;
            s_recOpen = false;
            motionEventStoreBusy(false);
            }
        }
    else if ((uint8_t)(s_recBlocks - s_recHdrBlocks) >= HEADER_EVERY_BLOCKS
             && framQueueFree() > QUEUE_RESERVE)
        writeHeader(v, EV_STATE_RECORDING);
    }

// --- serial console: list, dump, clear --------------------------------------------

static void printSummary(const EventHeader& h, uint8_t slot, const char* lead)
    {
    char clock[12];
    if (h.hh == 0xFF) strcpy(clock, "--:--:--");
    else              snprintf(clock, sizeof(clock), "%02u:%02u:%02u", h.hh, h.mm, h.ss);

    Serial.printf("%simpact %lu (slot %u): %s, %u/%u blocks, session %u at %s, black-box seq %lu\n",
                  lead, (unsigned long)h.id, slot,
                  (h.state == EV_STATE_COMPLETE) ? "complete" : "INCOMPLETE (power lost while recording?)",
                  h.blocksWritten, h.blocksTotal, h.session, clock, (unsigned long)h.logSeq);
    Serial.printf("%s  peak %.2f g (lon %+.2f, lat %+.2f, vert %+.2f), dv lon %+.2f lat %+.2f vert %+.2f m/s, "
                  "%u ms over %.1f g, air %u ms\n",
                  lead, h.peakG100 / 100.0f, h.peakLon100 / 100.0f, h.peakLat100 / 100.0f,
                  h.peakVert100 / 100.0f, h.dvLon100 / 100.0f, h.dvLat100 / 100.0f, h.dvVert100 / 100.0f,
                  h.overMs, h.trigG100 / 100.0f, h.airMs);
    Serial.printf("%s  rpm %u, tps %u%%, flags:%s%s%s%s\n", lead, h.rpm, h.tpsPct,
                  (h.flags & MOTION_EVF_LEVEL_CAL) ? " level-set" : " NOT-level-set",
                  (h.flags & MOTION_EVF_HG_TRIM)   ? "" : " trim-unsettled",
                  (h.flags & MOTION_EVF_DATA_GAP)  ? " SAMPLES-LOST" : "",
                  (h.flags & MOTION_EVF_CLIPPED)   ? " SENSOR-CLIPPED" : "");
    }

static void listCaptures()
    {
    uint8_t shown = 0;
    for (uint8_t i = 0; i < FRAM_EVENT_SLOTS; i++)
        {
        EventHeader h;
        if (!readHeader(i, h)) continue;
        printSummary(h, i, "");
        shown++;
        }
    if (!shown) Serial.println("impacts: none stored");
    else        Serial.printf("impacts: %u of %u slots used. 'E' dumps the waveforms as CSV.\n",
                              shown, FRAM_EVENT_SLOTS);
    }

// The dump prints every capture, oldest first, one block per superloop pass
// (12 CSV rows, one 39-byte read) so the gauge keeps running while it scrolls.
static bool        s_dumpOn;
static uint8_t     s_dumpSlots[FRAM_EVENT_SLOTS]; // slots to print, in id order
static uint8_t     s_dumpCount, s_dumpPos, s_dumpBlock;
static EventHeader s_dumpHdr;
static bool        s_dumpHdrLoaded;
static uint32_t    s_clearArmedMs;

static void dumpStart()
    {
    s_dumpCount = 0;
    for (uint8_t i = 0; i < FRAM_EVENT_SLOTS; i++)
        if (s_slot[i].valid) s_dumpSlots[s_dumpCount++] = i;
    // insertion sort by capture id: eight entries at most
    for (uint8_t a = 1; a < s_dumpCount; a++)
        for (uint8_t b = a; b > 0 && s_slot[s_dumpSlots[b]].id < s_slot[s_dumpSlots[b - 1]].id; b--)
            {
            const uint8_t t = s_dumpSlots[b];
            s_dumpSlots[b] = s_dumpSlots[b - 1];
            s_dumpSlots[b - 1] = t;
            }
    if (s_dumpCount == 0)
        {
        Serial.println("impacts: none stored - nothing to dump");
        return;
        }
    s_dumpOn        = true;
    s_dumpPos       = 0;
    s_dumpBlock     = 0;
    s_dumpHdrLoaded = false;
    Serial.printf("impacts: dumping %u capture(s), oldest first (any key aborts). Lines starting # are notes.\n",
                  s_dumpCount);
    Serial.println("impact_id,t_ms,lon_g,lat_g,vert_g,roll_deg,pitch_deg,yaw_dps");
    }

bool framEventsDumping()
    {
    return s_dumpOn;
    }

void framEventsDumpAbort(const char* why)
    {
    if (!s_dumpOn) return;
    s_dumpOn = false;
    Serial.printf("impacts: dump %s\n", why);
    }

void framEventsDumpPump()
    {
    if (!s_dumpOn)
        return;
    if (s_dumpPos >= s_dumpCount)
        {
        framEventsDumpAbort("done");
        return;
        }

    const uint8_t slot = s_dumpSlots[s_dumpPos];
    if (!s_dumpHdrLoaded)
        {
        if (!readHeader(slot, s_dumpHdr))
            {
            s_dumpPos++; // vanished (cleared mid-dump): next
            return;
            }
        printSummary(s_dumpHdr, slot, "# ");
        s_dumpHdrLoaded = true;
        s_dumpBlock     = 0;
        }

    const EventHeader& h = s_dumpHdr;
    const uint8_t frames = h.blockFrames;
    if (s_dumpBlock >= h.blocksWritten || frames == 0 || frames * 3u + 3u > MOTION_BLOCK_BYTES)
        {
        s_dumpPos++;
        s_dumpHdrLoaded = false;
        return;
        }

    uint8_t blk[MOTION_BLOCK_BYTES];
    if (!framRead(blockAddr(slot, s_dumpBlock), blk, (size_t)frames * 3u + 3u))
        {
        framEventsDumpAbort("ABORTED (read error)");
        return;
        }

    // Time zero is the trigger. Each row is one frame; the block's attitude
    // (sampled at its end) is repeated on every row of it.
    const float  frameMs   = 1000.0f * h.decim / (h.hgRateHz ? h.hgRateHz : 1);
    const int32_t trigger  = (int32_t)h.preBlocks * frames + h.triggerFrame;
    const float  fs        = h.fullScaleG;
    const int    rollDeg   = (int8_t)blk[frames * 3 + 0] * 2;
    const int    pitchDeg  = (int8_t)blk[frames * 3 + 1];
    const int    yawDps    = (int8_t)blk[frames * 3 + 2] * 4;
    for (uint8_t f = 0; f < frames; f++)
        {
        const int32_t n = (int32_t)s_dumpBlock * frames + f;
        Serial.printf("%lu,%.1f,%.2f,%.2f,%.2f,%d,%d,%d\n", (unsigned long)h.id,
                      (n - trigger) * frameMs,
                      imuExpand((int8_t)blk[f * 3 + 0], fs), imuExpand((int8_t)blk[f * 3 + 1], fs),
                      imuExpand((int8_t)blk[f * 3 + 2], fs), rollDeg, pitchDeg, yawDps);
        }
    s_dumpBlock++;
    }

void framEventsConsoleHelp()
    {
    Serial.println("               e = list impact captures   E = dump their waveforms as CSV");
    Serial.println("               C then Y = clear the impact captures (fuel record + log untouched)");
    }

bool framEventsConsoleKey(char c)
    {
    switch (c)
        {
        case 'e':
            if (s_mounted) listCaptures();
            else           Serial.println("impacts: FRAM not available");
            return true;
        case 'E':
            if (s_mounted) dumpStart();
            else           Serial.println("impacts: FRAM not available");
            return true;
        case 'C':
            s_clearArmedMs = millis() | 1; // never 0: 0 means "not armed"
            Serial.println("CLEAR ARMED: send 'Y' within 5 s to erase the impact captures");
            return true;
        default:
            return false;
        }
    }

void framEventsDisarmClear()
    {
    s_clearArmedMs = 0;
    }

bool framEventsConfirmClear()
    {
    if (!s_clearArmedMs)
        return false;
    const bool inTime = millis() - s_clearArmedMs < 5000;
    s_clearArmedMs = 0;
    if (!inTime)
        return false;
    if (!s_mounted)
        Serial.println("impacts: FRAM not available");
    else if (s_recOpen)
        Serial.println("impacts: busy recording one - try again in a few seconds");
    else
        {
        framEventsFormat();
        Serial.println("impacts: all captures cleared");
        }
    return true;
    }
