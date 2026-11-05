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
// Decoding: the ECU half follows rusEFI's own definitions - its DBC
// (firmware/controllers/can/rusEFI_CAN_verbose.dbc) for the native frames,
// can_dash_ms.cpp for the one MS Simplified byte we take. The BMS half
// follows the Deye protocol PDF. docs/ECU_CAN_SETUP.md and
// docs/BMS_CAN_SETUP.md carry the frame tables.
// ---------------------------------------------------------------------------

#include "can_bus.h"
#include "vehicle_data.h"

#include <FlexCAN_T4.h>
#include <TimeLib.h> // wall-clock time for the PCS heartbeat payload

// CAN3 = pins 30/31 per the pinout table in the project plan.
// RX_SIZE_256: the interrupt-fed ring buffer. The bus carries roughly 1000
// frames/s (rusEFI's MS frames alone are 500/s, plus its native frames, the
// wideband and the BMS), so 256 is ~250 ms of headroom - two orders of
// magnitude more than the superloop ever needs between drains.
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> s_can;

// --- 16-bit field extraction ---------------------------------------------------
// CAN gives us raw bytes. Everything this gauge decodes - both rusEFI
// broadcasts and the BMS - is little-endian (low byte first). We never just
// cast a pointer: each value is assembled explicitly, so the byte order is
// visible at every use and alignment never matters. Reading big-endian by
// mistake produces spectacular garbage (14.0 V = raw 14000 = 0x36B0 read
// backwards is 0xB036 = 45.1 V), which is how you spot it at bring-up.

static inline uint16_t le16u(const uint8_t* p)
    {
    return (uint16_t)(p[0] | (p[1] << 8));
    }

static inline int16_t le16s(const uint8_t* p)
    {
    // same bits, then reinterpreted as signed (two's complement)
    return (int16_t)le16u(p);
    }

static const float KPA_TO_PSI = 0.1450377f;

// --- the rusEFI native frames (BASE0..BASE7) -------------------------------------
// Field positions and scale factors straight from rusEFI_CAN_verbose.dbc.
// Returns false for frames we don't decode (BASE8..11) - the caller counts
// those as expected-but-unused traffic.

static bool decodeRusefiFrame(uint8_t index, const uint8_t* d)
    {
    EcuData& e = g_data.ecu;
    uint8_t  frame; // which EcuFrame slot to timestamp

    switch (index)
        {
        case 0: // BASE0 - status
            e.warningCount = le16u(d + 0);
            e.lastError    = le16u(d + 2);
            e.statusBits   = d[4];                 // ECU_STATUS_* (rev limit, CEL, ...)
            // d[5] gear, d[6..7] distance - unused
            frame = ECU_F_STATUS;
            break;

        case 1: // BASE1 - speeds
            e.rpm         = le16u(d + 0);
            e.sparkAdvDeg = le16s(d + 2) * 0.02f;
            e.injDutyPct  = d[4] * 0.5f;
            // d[5] coil duty, d[6] vehicle speed, d[7] ethanol % - unused
            frame = ECU_F_SPEEDS;
            break;

        case 2: // BASE2 - pedal + throttle, signed 0.01 % (-1 % = no sensor)
            e.pedalPct = le16s(d + 0) * 0.01f;
            e.tpsPct   = le16s(d + 2) * 0.01f;
            // d[4..5] TPS2, d[6..7] wastegate - unused
            frame = ECU_F_THROTTLE;
            break;

        case 3: // BASE3 - sensors 1
            e.mapKpa = le16u(d + 0) / 30.0f;      // 1/30 kPa per count
            e.cltC   = (int)d[2] - 40;             // deg C + 40
            e.matC   = (int)d[3] - 40;
            // d[4..5] aux temps, d[6] MCU temp, d[7] fuel level - unused
            frame = ECU_F_SENSORS1;
            break;

        case 4: // BASE4 - sensors 2 (d[0..1] are padding)
            {
            // rusEFI sends a dead oil pressure sensor as -1 kPa into an
            // UNSIGNED field. What a negative float becomes when cast to
            // uint16 is implementation-defined: 0 on some builds, 65506
            // (= 2183 kPa) on others. Map the wrapped form to 0 as well, so a
            // dead sensor always reads 0 psi - OIL LOW fires with the engine
            // running (fail loud), and never a fake 317 psi.
            const uint16_t raw = le16u(d + 2);
            e.oilPressPsi = (raw >= 0xFF00) ? 0.0f : raw / 30.0f * KPA_TO_PSI;
            e.oilTempC    = (int)d[4] - 40;        // deg C + 40
            // d[5] fuel temperature - unused
            e.battVolts   = le16u(d + 6) * 0.001f; // mV
            frame = ECU_F_SENSORS2;
            break;
            }

        case 5: // BASE5 - fueling
            // d[0..1] cylinder air mass, d[2..3] estimated airflow - unused
            e.injPwMs = le16u(d + 4) / 300.0f;     // 1/300 ms per count
            // d[6..7] knock count - unused
            frame = ECU_F_FUELING;
            break;

        case 6: // BASE6 - fueling 2
            // d[0..1] fuel used (g) - unused
            e.fuelFlowGps = le16u(d + 2) * 0.005f;
            e.egoCorPct   = le16s(d + 4) * 0.01f;  // bank 1 trim
            // d[6..7] bank 2 trim - unused (one bank on the M54)
            frame = ECU_F_FUELING2;
            break;

        case 7: // BASE7 - fueling 3
            e.lambda = le16u(d + 0) * 0.0001f;
            // d[2..3] lambda 2, d[4..7] fuel pressures - unused
            frame = ECU_F_LAMBDA;
            break;

        default:
            return false; // BASE8..11: cams, EGTs, per-cylinder knock, brake
        }

    // Stamp AFTER decoding so a fresh timestamp always means fresh data.
    // millis() is never 0 by the time CAN traffic flows, so 0 stays
    // reserved for "never received".
    e.frameLastRxMs[frame] = millis();
    e.frameCount++;
    return true;
    }

// --- the MS Simplified dash frame with the AFR target ---------------------------
// rusEFI fills 'afrtgt1' (byte 0 of id 1514) with target lambda x
// STOICH_RATIO (14.7), x 10, rounded into one byte. A single byte has no
// byte order, which matters here: rusEFI sends its MS frames' 16-bit fields
// little-endian, unlike a real MegaSquirt - one more reason we take nothing
// else from them.

static bool decodeMsDashFrame(uint32_t id, const uint8_t* d, uint8_t len)
    {
    if (id != MS_DASH_AFR_TARGET_ID || len < 1)
        return false;

    g_data.ecu.lambdaTarget = d[0] / 10.0f / AFR_STOICH;
    g_data.ecu.frameLastRxMs[ECU_F_AFR_TARGET] = millis();
    g_data.ecu.frameCount++;
    return true;
    }

// --- the BMS decode table ------------------------------------------------------
// One case per Deye-protocol frame we use (full layouts: the protocol PDF /
// docs/BMS_CAN_SETUP.md). Everything little-endian. Returns false for ids
// that aren't ours so onCanFrame() can classify them.

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

// --- expected traffic we deliberately don't decode -------------------------------
// Counting these as "unknown" would bury real surprises, so they get their
// own counter. The JK BMS's other Deye frames (docs/BMS_CAN_SETUP.md) and
// the rusEFI wideband controller's frames are listed here; the unused
// rusEFI/MS frames are recognised by id range in onCanFrame().

static const uint16_t k_bmsUnusedIds[] =
    {
    0x150, 0x200, 0x250,                        // per-module summaries
    0x351, 0x35C, 0x35E, 0x363, 0x364, 0x371,   // limits, request flags, name, ...
    0x500, 0x550, 0x600, 0x650, 0x700, 0x750,   // versions, serial number, statistics
    };

static bool isExpectedUnusedId(uint32_t id)
    {
    if (id >= RUSEFI_WBO_FIRST_ID && id <= RUSEFI_WBO_LAST_ID) return true;
    for (uint16_t known : k_bmsUnusedIds)
        if (id == known) return true;
    return false;
    }

// --- per-frame dispatch ----------------------------------------------------------
// Called by s_can.events() (i.e. in loop context) once per queued frame.
// Every frame ends up in exactly one bucket: decoded (stamped by its
// decoder), ignored (expected traffic we don't use), or unknown.

static void onCanFrame(const CAN_message_t& msg)
    {
    // Everything we talk to uses 11-bit ids. 29-bit frames on this bus are
    // other conversations (e.g. rusEFI's status messages to its wideband
    // controller): expected, never ours.
    if (msg.flags.extended)
        {
        g_data.ignoredFrameCount++;
        return;
        }

    const uint32_t id = msg.id;

    if (id >= RUSEFI_CAN_BASE_ID && id < RUSEFI_CAN_BASE_ID + RUSEFI_CAN_FRAME_COUNT)
        {
        const uint8_t index = (uint8_t)(id - RUSEFI_CAN_BASE_ID);
        if (msg.len >= 8 && decodeRusefiFrame(index, msg.buf))
            return;
        if (index >= 8) { g_data.ignoredFrameCount++; return; } // BASE8..11
        g_data.unknownFrameCount++; // a short BASE0..7 frame: not what rusEFI sends
        return;
        }

    if (id >= MS_DASH_BASE_ID && id < MS_DASH_BASE_ID + MS_DASH_FRAME_COUNT)
        {
        if (!decodeMsDashFrame(id, msg.buf, msg.len))
            g_data.ignoredFrameCount++; // the other four MS frames
        return;
        }

    if (decodeBmsFrame(id, msg.buf, msg.len))
        return; // timestamping is handled inside (it knows the frame slot)

    if (isExpectedUnusedId(id))
        g_data.ignoredFrameCount++;
    else
        g_data.unknownFrameCount++;
    }

// --- public API -------------------------------------------------------------------

void canBusInit()
    {
    s_can.begin();
    s_can.setBaudRate(CAN_BITRATE);
    s_can.setMaxMB(16);            // plenty: FIFO + a few TX mailboxes
    s_can.enableFIFO();            // hardware receive FIFO, accepts all ids
                                   // (no hardware filtering: the unknown /
                                   // ignored counters are how we see the bus)
    s_can.enableFIFOInterrupt();   // fill the ring buffer at interrupt speed
    s_can.onReceive(onCanFrame);   // ...and hand frames to us from events()

    Serial.printf("CAN3 up at %lu bps: rusEFI frames 0x%03lX-0x%03lX, AFR target 0x%03lX, BMS 0x%03lX/0x%03lX...\n",
                  (unsigned long)CAN_BITRATE,
                  (unsigned long)RUSEFI_CAN_BASE_ID,
                  (unsigned long)(RUSEFI_CAN_BASE_ID + RUSEFI_CAN_FRAME_COUNT - 1),
                  (unsigned long)MS_DASH_AFR_TARGET_ID,
                  (unsigned long)BMS_ID_SOC, (unsigned long)BMS_ID_PACK);
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
