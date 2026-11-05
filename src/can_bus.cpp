// ---------------------------------------------------------------------------
// Rally CAN Gauge - CAN bus input implementation
// ---------------------------------------------------------------------------
// How frames get from the wire into g_data (three hand-offs):
//
//   1. The FlexCAN peripheral receives a frame entirely in hardware.
//   2. Its interrupt fires and FlexCAN_T4 copies the frame into a software
//      ring buffer (RX_SIZE_256 below = room for 256 frames). This is the
//      "fills at interrupt speed without intervention" part: no frame is ever
//      lost because loop() happened to be busy drawing a screen.
//   3. canBusPoll() calls s_can.events(), which drains that ring buffer and
//      calls onCanFrame() for each frame FROM LOOP CONTEXT. So all of our
//      decode code runs in the superloop, single-threaded - g_data needs no
//      locking anywhere.
//
// Decoding: the Megasquirt broadcast is a fixed table (see the PDF, section
// 2.1, or docs/ECU_CAN_SETUP.md for just the fields we use). Each enabled
// 8-byte "group" arrives at CAN id (1520 + group). All values big-endian.
// ---------------------------------------------------------------------------

#include "can_bus.h"
#include "vehicle_data.h"

#include <FlexCAN_T4.h>
#include <TimeLib.h> // wall-clock time for the PCS heartbeat payload

// CAN3 = pins 30/31 per the pinout table in the project plan.
// RX_SIZE_256: the interrupt-fed ring buffer. Worst case the ECU sends 6
// frames per 20 ms tick, so 256 is roughly 850 ms of headroom.
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> s_can;

// --- 16-bit field extraction ---------------------------------------------------
// CAN gives us raw bytes and the two protocols on this bus disagree about
// byte order: the MS broadcast packs values high-byte first ("big-endian"),
// the BMS low-byte first ("little-endian"). We never just cast a pointer -
// each value is assembled explicitly with the right helper. Using the wrong
// one produces spectacular garbage (e.g. 16.0 V -> 0x0640 read backwards is
// 0x4006 = 1638.4 V), which is also how you spot it instantly at bring-up.

static inline uint16_t be16u(const uint8_t* p)
    {
    return (uint16_t)((p[0] << 8) | p[1]);
    }

static inline int16_t be16s(const uint8_t* p)
    {
    // same bits, then reinterpreted as signed (two's complement)
    return (int16_t)be16u(p);
    }

static inline uint16_t le16u(const uint8_t* p)
    {
    return (uint16_t)(p[0] | (p[1] << 8));
    }

static inline int16_t le16s(const uint8_t* p)
    {
    return (int16_t)le16u(p);
    }

// MS broadcasts temperatures in Fahrenheit; the whole project thinks in C.
// Convert once, here, at the system boundary - nothing downstream should
// ever see a Fahrenheit number.
static inline float f2c(float f)
    {
    return (f - 32.0f) * (5.0f / 9.0f);
    }

// --- the ECU decode table ---------------------------------------------------
// One case per group we care about; returns false for groups we don't
// decode, so only ids this firmware explicitly claims get counted as ECU
// traffic. (That matters because the BMS's 0x600 SN frame happens to fall
// inside the MS id range - see docs/BMS_CAN_SETUP.md "id collision".)
// Enabling extra groups in TunerStudio still can't break the gauge; they
// just show up in the unknown-frame counter.

static bool decodeEcuGroup(uint8_t group, const uint8_t* d)
    {
    switch (group)
        {
        case 0: // seconds, pw1, pw2, rpm
            g_data.ecu.secondsOn = be16u(d + 0);
            g_data.ecu.pw1Ms     = be16u(d + 2) / 1000.0f; // broadcast unit: us
            g_data.ecu.pw2Ms     = be16u(d + 4) / 1000.0f;
            g_data.ecu.rpm       = be16u(d + 6);
            break;

        case 1: // spark advance, squirt/engine bitfields, AFR targets
            g_data.ecu.sparkAdvDeg = be16s(d + 0) / 10.0f;
            // d[2] = 'squirt' bitfield - batch-fire injector events, unused
            g_data.ecu.engineBits  = d[3];          // cranking/warmup/... flags
            g_data.ecu.afrTarget   = d[4] / 10.0f;  // 1 byte, AFR x10
            // d[5] = bank 2 AFR target, unused (single lambda target on the M54)
            break;

        case 2: // baro, MAP, MAT, CLT
            g_data.ecu.baroKpa = be16s(d + 0) / 10.0f;
            g_data.ecu.mapKpa  = be16s(d + 2) / 10.0f;
            g_data.ecu.matC    = f2c(be16s(d + 4) / 10.0f);
            g_data.ecu.cltC    = f2c(be16s(d + 6) / 10.0f);
            break;

        case 3: // TPS, battery voltage, AFR (bank1/bank2)
            g_data.ecu.tpsPct    = be16s(d + 0) / 10.0f;
            g_data.ecu.battVolts = be16s(d + 2) / 10.0f;
            g_data.ecu.afr       = be16s(d + 4) / 10.0f;
            // d[6..7] = bank 2 AFR, unused
            break;

        case 4: // knock, EGO corrections
            // d[0..1] = knock input %, unused for now
            g_data.ecu.egoCorPct = be16s(d + 2) / 10.0f;
            // d[4..5] = bank 2 EGO correction, unused
            break;

        case 13: // generic sensors 1-4: OUR oil pressure + oil temp channels.
                 // The ECU-side calibration that makes sensors1 mean "psi"
                 // and sensors2 mean "deg C" is documented in
                 // docs/ECU_CAN_SETUP.md.
            g_data.ecu.oilPressPsi  = be16s(d + 0) / 10.0f;
            g_data.ecu.oilTempC     = be16s(d + 2) / 10.0f;
            g_data.ecu.spareSensor3 = be16s(d + 4) / 10.0f;
            g_data.ecu.spareSensor4 = be16s(d + 6) / 10.0f;
            break;

        default:
            return false; // a group we don't decode
        }
    return true;
    }

// --- the BMS decode table ------------------------------------------------------
// One case per Deye-protocol frame we use (full layouts: the protocol PDF /
// docs/BMS_CAN_SETUP.md). Everything little-endian. Returns false for ids
// that aren't ours so onCanFrame() can count them as unknown traffic.

static bool decodeBmsFrame(uint32_t id, const uint8_t* d, uint8_t len)
    {
    if (len < 8) return false; // every Deye data frame carries 8 bytes

    BmsData& b = g_data.bms;
    uint8_t frame; // which BmsFrame slot to timestamp

    switch (id)
        {
        case BMS_ID_SOC: // 0x355
            b.socPct = le16u(d + 0); // unit: 1 %
            b.sohPct = le16u(d + 2);
            frame = BMS_F_SOC;
            break;

        case BMS_ID_PACK: // 0x356 - the BMS heartbeat frame
            b.packVolts    = le16s(d + 0) / 100.0f; // 0.01 V
            b.packAmps     = le16s(d + 2) / 10.0f;  // 0.1 A, discharge positive
            b.avgCellTempC = le16s(d + 4) / 10.0f;  // 0.1 C
            frame = BMS_F_PACK;
            break;

        case BMS_ID_CELLS: // 0x361
            b.maxCellVolts = le16u(d + 0) / 1000.0f; // 1 mV
            b.minCellVolts = le16u(d + 2) / 1000.0f;
            b.maxCellTempC = le16s(d + 4) / 10.0f;   // 0.1 C
            b.minCellTempC = le16s(d + 6) / 10.0f;
            frame = BMS_F_CELLS;
            break;

        case BMS_ID_PROTECT: // 0x359 - raw bitfields, tables 1-7 in the PDF
            b.protection[0] = d[0];
            b.protection[1] = d[1];
            b.protection[2] = d[2];
            b.protection[3] = d[3];
            b.alarm[0]      = d[4];
            b.alarm[1]      = d[5];
            b.sysError      = d[6];
            frame = BMS_F_FAULT;
            break;

        case BMS_ID_STATE: // 0x400
            b.opMode       = d[0];          // 0 standstill / 1 charge / 2 discharge
            b.failureLevel = d[1];          // 0 none / 1 minor / 2 major
            b.cycleCount   = le16u(d + 2);
            b.balanceBits  = (uint16_t)(d[4] | (d[5] << 8)); // cells 1-8, 9-16
            frame = BMS_F_STATE;
            break;

        case BMS_ID_FAULT_MOS: // 0x110 - bytes 0-6 duplicate the 0x359 tables,
                               // so we only take byte 7 (MOSFET states) here.
            b.mosStateBits = d[7];
            frame = BMS_F_MOS;
            break;

        default:
            return false; // not a BMS frame we decode
        }

    b.frameLastRxMs[frame] = millis();
    b.frameCount++;
    return true;
    }

// --- per-frame dispatch ----------------------------------------------------------
// Called by s_can.events() (i.e. in loop context) once per queued frame.
// Try the ECU decoder first, then the BMS decoder; whatever neither claims
// is counted as unknown traffic.

static void onCanFrame(const CAN_message_t& msg)
    {
    // Both protocols on this bus use standard 11-bit ids; extended-id
    // traffic is somebody else's conversation.
    if (!msg.flags.extended)
        {
        const int32_t group = (int32_t)msg.id - (int32_t)MS_CAN_BASE_ID;
        if (group >= 0 && group < MS_CAN_NUM_GROUPS && msg.len >= 8
            && decodeEcuGroup((uint8_t)group, msg.buf))
            {
            // Stamp AFTER decoding so a fresh timestamp always means fresh
            // data. millis() is never 0 by the time CAN traffic flows, so 0
            // stays reserved for "never received".
            g_data.ecu.groupLastRxMs[group] = millis();
            g_data.ecu.frameCount++;
            return;
            }

        if (decodeBmsFrame(msg.id, msg.buf, msg.len))
            return; // timestamping is handled inside (it knows the frame slot)
        }

    g_data.unknownFrameCount++;
    }

// --- public API -------------------------------------------------------------------

void canBusInit()
    {
    s_can.begin();
    s_can.setBaudRate(CAN_BITRATE);
    s_can.setMaxMB(16);            // plenty: FIFO + a few TX mailboxes
    s_can.enableFIFO();            // hardware receive FIFO, accepts all ids for
                                   // now (id filtering is a later optimization -
                                   // easier to debug when we can see everything)
    s_can.enableFIFOInterrupt();   // fill the ring buffer at interrupt speed
    s_can.onReceive(onCanFrame);   // ...and hand frames to us from events()

    Serial.printf("CAN3 up at %lu bps, listening for ECU broadcast at ids %lu-%lu (0x%lX+)\n",
                  (unsigned long)CAN_BITRATE,
                  (unsigned long)MS_CAN_BASE_ID,
                  (unsigned long)(MS_CAN_BASE_ID + MS_CAN_NUM_GROUPS - 1),
                  (unsigned long)MS_CAN_BASE_ID);
    }

void canBusPoll()
    {
    s_can.events(); // drain the ring buffer -> onCanFrame() per frame
    }

void canBusTick1s()
    {
    if (!BMS_SEND_PCS_HEARTBEAT) return;

    // The Deye protocol expects the inverter to answer the BMS broadcast with
    // a 0x305 heartbeat each second ("0x305: 00-00-00-00-00-00-00-00, add
    // time data"). No inverter exists in the car, so the gauge supplies it -
    // otherwise the BMS may raise its "PCS communication failure" bit.
    // All-zero payload is valid per the spec; when our RTC has a real time we
    // send it (bytes: sec, min, hour, day, month, year-2000, little-endian
    // single bytes), which lets the BMS keep a sensible clock too.
    CAN_message_t msg;
    msg.id  = BMS_ID_PCS_HEARTBEAT;
    msg.len = 8;
    for (uint8_t i = 0; i < 8; i++) msg.buf[i] = 0;

    if (g_data.board.clockValid)
        {
        msg.buf[0] = (uint8_t)second();
        msg.buf[1] = (uint8_t)minute();
        msg.buf[2] = (uint8_t)hour();
        msg.buf[3] = (uint8_t)day();
        msg.buf[4] = (uint8_t)month();
        msg.buf[5] = (uint8_t)(year() - 2000);
        }

    s_can.write(msg);
    }
