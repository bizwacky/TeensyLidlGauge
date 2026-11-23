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
// follows the Deye protocol PDF. The PDM half follows dingoFW's
// boards/dingopdm_v7/msg.cpp through the PDM_FRAMES table in config.h.
// docs/ECU_CAN_SETUP.md, docs/BMS_CAN_SETUP.md and docs/PDM_CAN_SETUP.md
// carry the frame tables.
//
// TRANSMITTING. The gauge also talks: the 1 Hz PCS heartbeat for the BMS
// (canBusTick1s) and, since the roof switch panel moved onto the gauge, the
// two frames that carry its switches and pots to the PDMs (canBusTick20ms).
// Those two are the car's light switches - their layout is at the bottom of
// this file and in docs/PDM_CAN_SETUP.md, and it is a contract with each
// PDM's dingoConfig setup: change one side, change the other.
// The four IMU frames (also canBusTick20ms) are for whoever wants to log
// them - today a Lua script on the ECU (docs/rusefi_imu.lua); layout below
// the switch frames and in docs/IMU_SETUP.md.
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

// --- the PDM status frames ---------------------------------------------------------
// dingoFW sends every status frame every 100 ms, 8 bytes, little-endian.
// WHICH frame carries WHAT is not written here: it is the PDM_FRAMES table
// in config.h (one row per frame: id offset, kind, which outputs). This
// function only knows how each KIND packs its fields, so a firmware that
// moves or adds frames - the 14-output fork - needs a table edit, not code.

// Which g_data.pdm[] slot owns this id, or -1 if it is not a PDM's. Also
// hands back the id's offset from that box's Base ID.
//
// THIS IS THE ONE PLACE that ties a box to a role. Today the Base ID burned
// into a box IS its role. If "the role follows the plug" is adopted, decode
// by Base ID into a per-BOX slot and derive the role from the strap inputs
// (PdmData::digInBits) here.
static int8_t pdmSlotForId(uint32_t id, uint8_t& offset)
    {
    for (uint8_t box = 0; box < PDM_COUNT; box++)
        {
        if (id >= PDM_BASE_ID[box] && id < PDM_BASE_ID[box] + PDM_ID_BLOCK)
            {
            offset = (uint8_t)(id - PDM_BASE_ID[box]);
            return (int8_t)box;
            }
        }
    return -1;
    }

// Returns false when no row of PDM_FRAMES claims this offset (the box's
// settings channel and the status frames the gauge has no use for).
static bool decodePdmFrame(uint8_t box, uint8_t offset, const uint8_t* d, uint8_t len)
    {
    if (len < 8) return false; // every dingoFW status frame carries 8 bytes

    PdmData& p = g_data.pdm[box];

    for (uint8_t row = 0; row < PDM_FRAME_COUNT; row++)
        {
        const PdmFrameDef& def = PDM_FRAMES[row];
        if (def.idOffset != offset) continue;

        switch (def.kind)
            {
            case PDM_KIND_DEVICE:
                p.digInBits   = d[0];
                p.deviceState = d[1] & 0x0F;
                p.boardType   = d[1] >> 4;
                p.totalAmps   = le16u(d + 2) * PDM_AMPS_PER_COUNT;
                p.battVolts   = le16u(d + 4) * 0.1f;
                p.boardTempC  = le16s(d + 6) * 0.1f;
                break;

            case PDM_KIND_CURRENT: // one uint16 per output, four fit a frame
                for (uint8_t k = 0; k < def.outputs && k < 4; k++)
                    if (def.firstOutput + k < PDM_MAX_OUTPUTS)
                        p.out[def.firstOutput + k].amps = le16u(d + 2 * k) * PDM_AMPS_PER_COUNT;
                break;

            case PDM_KIND_STATE: // one nibble per output, low nibble first
                for (uint8_t k = 0; k < def.outputs && k < 16; k++)
                    if (def.firstOutput + k < PDM_MAX_OUTPUTS)
                        p.out[def.firstOutput + k].state =
                            (k & 1) ? (uint8_t)(d[k / 2] >> 4) : (uint8_t)(d[k / 2] & 0x0F);
                break;

            case PDM_KIND_OC_COUNT: // one byte per output
                for (uint8_t k = 0; k < def.outputs && k < 8; k++)
                    if (def.firstOutput + k < PDM_MAX_OUTPUTS)
                        p.out[def.firstOutput + k].ocCount = d[k];
                break;

            case PDM_KIND_DUTY: // one byte per output, percent
                for (uint8_t k = 0; k < def.outputs && k < 8; k++)
                    if (def.firstOutput + k < PDM_MAX_OUTPUTS)
                        p.out[def.firstOutput + k].dutyPct = d[k];
                break;
            }

        p.frameLastRxMs[row] = millis(); // after decoding, like the ECU frames
        p.frameCount++;
        return true;
        }
    return false;
    }

// --- expected traffic we deliberately don't decode -------------------------------
// Counting these as "unknown" would bury real surprises, so they get their
// own counter. The JK BMS's other Deye frames (docs/BMS_CAN_SETUP.md) and
// the rusEFI wideband controller's frames are listed here; the unused
// rusEFI/MS frames and the rest of each PDM's id block are recognised by
// id range in onCanFrame().

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

    uint8_t      pdmOffset = 0;
    const int8_t pdmBox    = pdmSlotForId(id, pdmOffset);
    if (pdmBox >= 0)
        {
        // Anything else in a PDM's block is its own business: the dingoConfig
        // settings channel (Base+0/+1) and status frames we take nothing from.
        if (!decodePdmFrame((uint8_t)pdmBox, pdmOffset, msg.buf, msg.len))
            g_data.ignoredFrameCount++;
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
    Serial.printf("      PDM-E base 0x%03lX, PDM-L base 0x%03lX; gauge sends 0x%03lX + 0x%03lX every %lu ms\n",
                  (unsigned long)PDM_BASE_ID[PDM_E], (unsigned long)PDM_BASE_ID[PDM_L],
                  (unsigned long)GAUGE_ID_SWITCHES, (unsigned long)GAUGE_ID_POTS,
                  (unsigned long)GAUGE_TX_PERIOD_MS);
    Serial.printf("      IMU frames 0x%03lX-0x%03lX (impact + motion every %lu ms, attitude %lu ms, event %lu ms)\n",
                  (unsigned long)IMU_CAN_BASE_ID, (unsigned long)IMU_CAN_ID_EVENT,
                  (unsigned long)IMU_CAN_FAST_MS, (unsigned long)IMU_CAN_ATTITUDE_MS,
                  (unsigned long)IMU_CAN_EVENT_MS);
    }

void canBusPoll()
    {
    s_can.events(); // drain the ring buffer -> onCanFrame() per frame
    }

// --- the gauge's own frames: roof switches + pots -> the PDMs -----------------------
// Two 8-byte frames, sent every GAUGE_TX_PERIOD_MS and immediately (this
// tick) whenever their content changes, so a switch reaches the PDMs within
// one debounce + 20 ms and a silent switch still refreshes the PDMs' timeouts.
//
// GAUGE_ID_SWITCHES
//   byte 0  switch ON bits    bit 0 = SW1 ... bit 5 = SW6   (1 = switch is on)
//   byte 1  switch OFF bits   bit 0 = SW1 ... bit 5 = SW6   (1 = switch is off)
//   byte 2  pot ACTIVE bits   bit 0 = pot 1 ... bit 4 = pot 5 (1 = turned up)
//   byte 3  pot FAULT bits    bit 0 = pot 1 ... bit 4 = pot 5 (1 = wire open,
//                             the pot's value is its failsafe)
//   byte 4  gauge state       bits 0-3 dial position, bit 4 night, bit 5
//                             silent, bit 6 displays off, bit 7 dial fault
//   byte 5  alarm summary     bit 0 = a red alarm is up, bit 1 = a yellow is
//   byte 6  top alarm         AlarmId of the banner alarm, 0xFF = nominal
//   byte 7  alive counter     +1 per transmission, wraps
//
// GAUGE_ID_POTS
//   bytes 0-4  pot 1..5 position, 0..100 %  (a PDM can use it directly as a
//              PWM duty: dingoFW "variable duty cycle", denominator 1)
//   bytes 5-7  fan PWM 1..3 duty the gauge is driving, 0..100 %
//
// WHY EVERY SWITCH IS SENT TWICE: dingoFW zeroes a timed-out CAN input, so
// "0" is what a PDM sees when the gauge is dead. Bytes 0 and 1 are always
// complements, which lets the PDM config choose the failure direction per
// load: drive it from the ON bit and it fails OFF; drive it from
// NOT(OFF bit) and it fails ON. The lamps use the second form - a dead gauge
// or a cut bus must not put the lights out on a night stage.

static void buildSwitchFrame(uint8_t* b)
    {
    const SwitchData& sw = g_data.sw;
    const ModeData&   m  = g_data.mode;
    const AlarmsData& a  = g_data.alarms;

    const uint8_t allSwitches = (uint8_t)((1u << ROOF_SWITCH_COUNT) - 1);

    uint8_t potActive = 0, potFault = 0;
    for (uint8_t i = 0; i < POT_COUNT; i++)
        {
        if (sw.pot[i].active) potActive |= (uint8_t)(1u << i);
        if (sw.pot[i].open)   potFault  |= (uint8_t)(1u << i);
        }

    b[0] = sw.roofBits & allSwitches;
    b[1] = (uint8_t)(~sw.roofBits) & allSwitches;
    b[2] = potActive;
    b[3] = potFault;
    b[4] = (uint8_t)((sw.rotaryPos & 0x0F)
                     | (m.night       ? 0x10 : 0)
                     | (m.silent      ? 0x20 : 0)
                     | (m.displaysOff ? 0x40 : 0)
                     | (sw.rotaryOpen ? 0x80 : 0));
    b[5] = (uint8_t)((a.anyRed ? 0x01 : 0) | (a.anyYellow ? 0x02 : 0));
    b[6] = a.top;
    b[7] = 0; // the alive counter is added at send time - it must not count
              // as a "change", or the frame would go out on every tick
    }

static void buildPotFrame(uint8_t* b)
    {
    for (uint8_t i = 0; i < 8; i++) b[i] = 0;
    for (uint8_t i = 0; i < POT_COUNT && i < 5; i++)
        b[i] = (uint8_t)constrain(g_data.sw.pot[i].pct + 0.5f, 0.0f, 100.0f);
    for (uint8_t i = 0; i < FAN_COUNT && i < 3; i++)
        b[5 + i] = g_data.fans.dutyPct[i];
    }

// --- the gauge's own frames: the IMU --------------------------------------------------
// Four 8-byte frames, little-endian, on ids chosen to lose every arbitration
// that matters (config.h, IMU_CAN_BASE_ID). Signs and axes as in
// vehicle_data.h: lon + = speeding up, lat + = turning left, vert + = up
// (1 g at rest); roll + = right side down, pitch + = nose up, yaw + = left.
//
// IMU_CAN_ID_IMPACT, every IMU_CAN_FAST_MS - the high-g channel, PEAK-HELD:
//   bytes 0-1  uint16  largest total acceleration of the last 0.2 s   0.01 g
//   bytes 2-3  int16   lon  \  per axis: the reading of largest size  0.01 g
//   bytes 4-5  int16   lat   } in the same 0.2 s, with its sign
//   bytes 6-7  int16   vert /
//   Held so that a logger sampling slower than the hit is long still sees
//   it at full height. At rest: 1.00, 0, 0, +1.00. All zero = no IMU data.
//
// IMU_CAN_ID_MOTION, every IMU_CAN_FAST_MS - the low-g channel, smoothed:
//   bytes 0-1  int16   lon                                            0.001 g
//   bytes 2-3  int16   lat                                            0.001 g
//   bytes 4-5  int16   vert                                           0.001 g
//   bytes 6-7  int16   yaw rate                                       0.1 deg/s
//
// IMU_CAN_ID_ATTITUDE, every IMU_CAN_ATTITUDE_MS:
//   bytes 0-1  int16   roll                                           0.1 deg
//   bytes 2-3  int16   pitch                                          0.1 deg
//   bytes 4-5  uint16  air time of the last jump                      1 ms
//   byte  6    uint8   jumps since boot (wraps)
//   byte  7    status  bit 0 IMU data is live     bit 1 level-calibrated
//                      bit 2 high-g trim settled  bit 3 airborne now
//                      bit 4 an impact capture is recording
//                      bit 5 standing still       bit 6 samples have been lost
//                                                       since boot (FIFO overrun)
//
// IMU_CAN_ID_EVENT, every IMU_CAN_EVENT_MS - the latest impact capture:
//   bytes 0-1  uint16  capture number (0 = none yet); it steps when a
//                      capture starts, and the black box carries the same
//                      number                                         1
//   bytes 2-3  uint16  its peak total acceleration                    0.01 g
//   bytes 4-5  uint16  its delta-v, all three axes combined           0.01 m/s
//   bytes 6-7  uint16  landing of the last jump, total acceleration   0.01 g

static inline void put16(uint8_t* b, int32_t v)
    {
    b[0] = (uint8_t)(v & 0xFF);
    b[1] = (uint8_t)((v >> 8) & 0xFF);
    }
static inline int32_t scaledS16(float v, float perUnit) // signed, saturating
    {
    return (int32_t)lroundf(constrain(v * perUnit, -32767.0f, 32767.0f));
    }
static inline int32_t scaledU16(float v, float perUnit) // unsigned, saturating
    {
    return (int32_t)lroundf(constrain(v * perUnit, 0.0f, 65535.0f));
    }

static_assert(IMU_CAN_FAST_MS % TICK_FAST_MS == 0 && IMU_CAN_ATTITUDE_MS % TICK_FAST_MS == 0
              && IMU_CAN_EVENT_MS % TICK_FAST_MS == 0,
              "the IMU frame periods must be multiples of the 20 ms tick");
static_assert(IMU_CAN_FAST_MS >= 2 * TICK_FAST_MS,
              "the impact and motion frames share a period and go out on alternate ticks");

static void sendImuFrames()
    {
    static uint32_t s_tick; // counts 20 ms ticks. The frames take turns: at
    s_tick++;               //  the default periods no tick sends more than two

    const ImuData& imu     = g_data.imu;
    const uint32_t fastN   = IMU_CAN_FAST_MS / TICK_FAST_MS;
    const uint32_t attN    = IMU_CAN_ATTITUDE_MS / TICK_FAST_MS;
    const uint32_t eventN  = IMU_CAN_EVENT_MS / TICK_FAST_MS;

    CAN_message_t msg;
    msg.len = 8;

    if (s_tick % fastN == 0)
        {
        msg.id = IMU_CAN_ID_IMPACT;
        put16(msg.buf + 0, scaledU16(imu.peakG,     100.0f));
        put16(msg.buf + 2, scaledS16(imu.peakLonG,  100.0f));
        put16(msg.buf + 4, scaledS16(imu.peakLatG,  100.0f));
        put16(msg.buf + 6, scaledS16(imu.peakVertG, 100.0f));
        s_can.write(msg);
        }
    else if (s_tick % fastN == 1)
        {
        msg.id = IMU_CAN_ID_MOTION;
        put16(msg.buf + 0, scaledS16(imu.lonG,       1000.0f));
        put16(msg.buf + 2, scaledS16(imu.latG,       1000.0f));
        put16(msg.buf + 4, scaledS16(imu.vertG,      1000.0f));
        put16(msg.buf + 6, scaledS16(imu.yawRateDps, 10.0f));
        s_can.write(msg);
        }

    if (s_tick % attN == attN / 2)
        {
        msg.id = IMU_CAN_ID_ATTITUDE;
        put16(msg.buf + 0, scaledS16(imu.rollDeg,  10.0f));
        put16(msg.buf + 2, scaledS16(imu.pitchDeg, 10.0f));
        put16(msg.buf + 4, imu.lastAirMs);
        msg.buf[6] = (uint8_t)imu.jumpCount;
        msg.buf[7] = (uint8_t)((g_data.imuOnline()  ? 0x01 : 0)
                             | (imu.levelCal        ? 0x02 : 0)
                             | (imu.hgTrimmed       ? 0x04 : 0)
                             | (imu.airborne        ? 0x08 : 0)
                             | (imu.event.active    ? 0x10 : 0)
                             | (imu.still           ? 0x20 : 0)
                             | (imu.fifoOverruns    ? 0x40 : 0));
        s_can.write(msg);
        }

    if (s_tick % eventN == eventN / 2)
        {
        const ImuEventData& e = imu.event;
        const float dv = sqrtf(e.dvLonMps * e.dvLonMps + e.dvLatMps * e.dvLatMps
                               + e.dvVertMps * e.dvVertMps);
        msg.id = IMU_CAN_ID_EVENT;
        put16(msg.buf + 0, (int32_t)(e.id & 0xFFFF));
        put16(msg.buf + 2, scaledU16(e.peakG,       100.0f));
        put16(msg.buf + 4, scaledU16(dv,            100.0f));
        put16(msg.buf + 6, scaledU16(imu.lastLandG, 100.0f));
        s_can.write(msg);
        }
    }

void canBusTick20ms()
    {
    static uint8_t  s_lastSwitch[8], s_lastPot[8];
    static uint32_t s_lastSwitchMs, s_lastPotMs;
    static bool     s_sentOnce;
    static uint8_t  s_alive;

    const uint32_t now = millis();
    CAN_message_t  msg;
    msg.len = 8;

    uint8_t b[8];

    buildSwitchFrame(b);
    if (!s_sentOnce || memcmp(b, s_lastSwitch, 8) != 0
        || now - s_lastSwitchMs >= GAUGE_TX_PERIOD_MS)
        {
        memcpy(s_lastSwitch, b, 8);
        s_lastSwitchMs = now;
        msg.id = GAUGE_ID_SWITCHES;
        memcpy(msg.buf, b, 8);
        msg.buf[7] = s_alive++;
        s_can.write(msg);
        }

    buildPotFrame(b);
    if (!s_sentOnce || memcmp(b, s_lastPot, 8) != 0
        || now - s_lastPotMs >= GAUGE_TX_PERIOD_MS)
        {
        memcpy(s_lastPot, b, 8);
        s_lastPotMs = now;
        msg.id = GAUGE_ID_POTS;
        memcpy(msg.buf, b, 8);
        s_can.write(msg);
        }

    s_sentOnce = true;

    sendImuFrames(); // after the switch frames: those are the ones that matter
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
