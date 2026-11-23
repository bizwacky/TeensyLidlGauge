// ---------------------------------------------------------------------------
// Rally CAN Gauge - LSM6DSV320X IMU driver implementation
// ---------------------------------------------------------------------------
// Design rationale in imu.h. Chip specifics this file encodes, from the
// LSM6DSV320X datasheet (ST DS14623 rev 3) and ST's own register driver
// (github.com/STMicroelectronics/lsm6dsv320x-pid, the source for every
// register address and bit position below):
//
//   * Three sensors, three clocks. The low-g accelerometer and the gyroscope
//     are configured in CTRL1/CTRL2, the high-g accelerometer in its own
//     CTRL1_XL_HG. The high-g channel only runs while the low-g one is in
//     high-performance (or high-accuracy) mode - datasheet table 21.
//   * The FIFO stores 7-byte WORDS: one tag byte saying which sensor the
//     word is from, then X, Y, Z as little-endian int16. The three sensors'
//     words arrive interleaved, in the order they were sampled.
//   * FIFO_STATUS1/2 give the number of unread words (9 bits) and an
//     overrun flag. Reading FIFO_DATA_OUT_TAG (78h) onward pops words.
//   * BURST READS. ST's application notes (AN6119 section 9.8, AN5922) say
//     the address pointer wraps from 7Eh back to 78h, so many words come out
//     of one multi-byte read. The datasheet itself does not spell this out,
//     so every word's tag is checked: if words after the first in a burst
//     come back with tags nothing here batches, tag= on the debug line
//     climbs and IMU_FIFO_BURST_WORDS = 1 is the fix (config.h).
//
// THE BUS. teensy4_i2c ships ready-made objects for LPI2C1/3/4 only; the
// IMU is on LPI2C2, which the Teensy 4.0 never brought out and the MicroMod
// does (pins 41 SDA / 40 SCL). So this file builds its own master object
// from the same pin table the Teensy core uses for Wire3 on a MicroMod
// (WireIMXRT.cpp, i2c2_hardware).
//
// THE PUMP is a five-state machine, one I2C transaction per trip round it:
//
//   IDLE --(every IMU_POLL_MS)--> ask for FIFO_STATUS --> read 2 bytes
//        --> words waiting? --> ask for FIFO_DATA_OUT --> read 7 x n bytes
//        --> hand the samples to motion.cpp --> more waiting? read again
//
// A read is two chained transfers, like the FRAM's: a one-byte write of the
// register address WITHOUT a stop, then a repeated-start read.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <i2c_driver.h>
#include <imx_rt1060/imx_rt1060_i2c_driver.h>

#include "imu.h"
#include "motion.h"
#include "config.h"
#include "vehicle_data.h"

// --- registers (main page) ----------------------------------------------------------

static const uint8_t REG_FIFO_CTRL1       = 0x07; // watermark (unused: nothing to interrupt)
static const uint8_t REG_FIFO_CTRL2       = 0x08;
static const uint8_t REG_FIFO_CTRL3       = 0x09; // BDR_GY[7:4] | BDR_XL[3:0]: batch rates
static const uint8_t REG_FIFO_CTRL4       = 0x0A; // FIFO_MODE[2:0]
static const uint8_t REG_COUNTER_BDR_REG1 = 0x0B; // bit 3 XL_HG_BATCH_EN
static const uint8_t REG_WHO_AM_I         = 0x0F;
static const uint8_t REG_CTRL1            = 0x10; // OP_MODE_XL[6:4] | ODR_XL[3:0]
static const uint8_t REG_CTRL2            = 0x11; // OP_MODE_G[6:4]  | ODR_G[3:0]
static const uint8_t REG_CTRL3            = 0x12; // BOOT | BDU | IF_INC | SW_RESET
static const uint8_t REG_CTRL6            = 0x15; // FS_G[2:0]
static const uint8_t REG_CTRL8            = 0x17; // FS_XL[1:0]
static const uint8_t REG_FIFO_STATUS1     = 0x1B; // DIFF_FIFO[7:0]; STATUS2 follows at 1Ch
static const uint8_t REG_CTRL1_XL_HG      = 0x4E; // REGOUT_EN | - | ODR_XL_HG[5:3] | FS_XL_HG[2:0]
static const uint8_t REG_FIFO_DATA_OUT    = 0x78; // tag, then X/Y/Z low+high to 7Eh

static const uint8_t CTRL3_SW_RESET = 0x01;
static const uint8_t CTRL3_IF_INC   = 0x04; // address auto-increment in multi-byte access
static const uint8_t CTRL3_BDU      = 0x40; // block data update

// Rate and range codes. The ODR codes are what IMU_HG_RATE_HZ and
// IMU_LG_RATE_HZ in config.h describe - change them together or not at all.
static const uint8_t ODR_120HZ            = 0x06; // ODR_XL / ODR_G / BDR_XL / BDR_GY, same code
static const uint8_t OP_MODE_HIGH_PERF    = 0x00; // OP_MODE_XL / OP_MODE_G
static const uint8_t ODR_XL_HG_960HZ      = 0x04;
static const uint8_t FS_XL_16G            = 0x03;
static const uint8_t FS_G_2000DPS         = 0x04;
static const uint8_t CTRL1_XL_HG_REGOUT   = 0x80; // also expose high-g in the output registers
static const uint8_t XL_HG_BATCH_EN       = 0x08;
static const uint8_t FIFO_MODE_BYPASS     = 0x00; // off, and empties it
static const uint8_t FIFO_MODE_CONTINUOUS = 0x06; // newest data overwrites the oldest

static_assert(IMU_HG_RATE_HZ == 960 && IMU_LG_RATE_HZ == 120,
              "the ODR codes in imu.cpp are for 960 Hz high-g and 120 Hz low-g/gyro");

// FS_XL_HG code for the configured full scale (datasheet table 150).
static constexpr uint8_t hgFsCode()
    {
    return IMU_HG_FULL_SCALE_G == 32  ? 0
         : IMU_HG_FULL_SCALE_G == 64  ? 1
         : IMU_HG_FULL_SCALE_G == 128 ? 2
         : IMU_HG_FULL_SCALE_G == 256 ? 3
         :                              4;
    }
static_assert(IMU_HG_FULL_SCALE_G == 32 || IMU_HG_FULL_SCALE_G == 64 || IMU_HG_FULL_SCALE_G == 128
              || IMU_HG_FULL_SCALE_G == 256 || IMU_HG_FULL_SCALE_G == 320,
              "IMU_HG_FULL_SCALE_G must be 32, 64, 128, 256 or 320");

// FIFO tags we batch (datasheet table 232). Anything else in the stream is
// counted as a tag error.
static const uint8_t TAG_GYRO   = 0x01;
static const uint8_t TAG_LOW_G  = 0x02;
static const uint8_t TAG_HIGH_G = 0x1D;

static const uint8_t  FIFO_WORD_BYTES   = 7;
static const uint8_t  FIFO_STATUS2_OVR  = 0x48; // FIFO_OVR_IA | FIFO_OVR_LATCHED
static const uint16_t FIFO_MAX_READ     = 252;  // 36 words: the I2C driver reads 256 bytes at most
static_assert(IMU_FIFO_BURST_WORDS >= 1 && IMU_FIFO_BURST_WORDS * FIFO_WORD_BYTES <= FIFO_MAX_READ,
              "IMU_FIFO_BURST_WORDS: 1..36 (one I2C read carries at most 256 bytes)");

// --- the bus: LPI2C2 on pins 41 (SDA) / 40 (SCL) ------------------------------------
// The same numbers as TwoWire::i2c2_hardware in the Teensy core: both pads on
// mux alternative 2 with the input path forced on (0x10), input select 1.

#if !defined(ARDUINO_TEENSY_MICROMOD)
#error "imu.cpp: the IMU bus (LPI2C2 on pins 40/41) only exists on the MicroMod Teensy"
#endif

static IMX_RT1060_I2CBase::Config s_busConfig =
    {
    CCM_CCGR2,
    CCM_CCGR2_LPI2C2(CCM_CCGR_ON),
    IMX_RT1060_I2CBase::PinInfo{41U, 2U | 0x10U, &IOMUXC_LPI2C2_SDA_SELECT_INPUT, 1U},
    IMX_RT1060_I2CBase::PinInfo{40U, 2U | 0x10U, &IOMUXC_LPI2C2_SCL_SELECT_INPUT, 1U},
    false,                                                     // no alternative pins
    IMX_RT1060_I2CBase::PinInfo{0U, 0U, nullptr, 0U},
    IMX_RT1060_I2CBase::PinInfo{0U, 0U, nullptr, 0U},
    IRQ_LPI2C2
    };

static void busIsr();
static IMX_RT1060_I2CMaster s_bus(&LPI2C2, s_busConfig, busIsr);
static void busIsr() { s_bus._interrupt_service_routine(); }

// --- module state -------------------------------------------------------------------

enum PumpState : uint8_t
    {
    PUMP_OFF = 0,     // no chip (or it stopped answering): retried every IMU_RETRY_MS
    PUMP_START,       // configured, FIFO still off: the next pass switches it on
    PUMP_STARTING,    //   ...that write is on the wire
    PUMP_IDLE,        // streaming; waiting for the next poll time
    PUMP_STATUS_ADDR, // wrote the FIFO_STATUS1 address
    PUMP_STATUS_READ, // reading the two status bytes
    PUMP_DATA_ADDR,   // wrote the FIFO_DATA_OUT_TAG address
    PUMP_DATA_READ,   // reading a burst of words
    };

static PumpState     s_state;
static elapsedMillis s_xferMs;        // since the transfer in flight started
static elapsedMillis s_sincePollMs;   // since the last status poll started
static elapsedMillis s_sinceRetryMs;  // since the last (re)configuration attempt
static uint32_t      s_streamStartMs; // millis() when the FIFO was switched on
static uint8_t       s_consecErrors;
static uint16_t      s_wordsWaiting;  // unread words per the last status read
static uint16_t      s_wordsInFlight; // words the burst in flight will deliver
static bool          s_expectOverrun; // the first poll after a (re)start may find a
                                      // stale overrun flag: not a loss worth counting

// Transfer buffers. The I2C interrupt reads/writes these while a transfer
// runs, so they are static and never touched until it has finished.
static uint8_t s_txBuf[2];
static uint8_t s_statusBuf[2];
static uint8_t s_fifoBuf[FIFO_MAX_READ];

// Sample-rate measurement for the debug line.
static uint32_t s_rateLastCount;
static uint32_t s_rateLastMs;

// --- blocking register access (setup and re-configuration only) ----------------------

static bool waitFinished(uint32_t budgetMs)
    {
    elapsedMillis waited;
    while (!s_bus.finished())
        {
        if (waited > budgetMs)
            {
            s_bus.begin(IMU_I2C_HZ); // wedged bus: re-initialising the peripheral clears it
            return false;
            }
        }
    return !s_bus.has_error();
    }

static bool regWrite(uint8_t reg, uint8_t value)
    {
    uint8_t buf[2] = { reg, value };
    s_bus.write_async(IMU_I2C_ADDR, buf, 2, true);
    return waitFinished(3);
    }

static bool regRead(uint8_t reg, uint8_t* dst, size_t len)
    {
    s_bus.write_async(IMU_I2C_ADDR, &reg, 1, false);
    if (!waitFinished(3)) return false;
    s_bus.read_async(IMU_I2C_ADDR, dst, len, true);
    return waitFinished(3);
    }

// Identify, reset and configure the chip, leaving the FIFO OFF. Blocks for
// about 10 ms on a healthy chip and 3 ms on a missing one. 'why' receives a
// short reason on failure.
static bool configureChip(const char*& why)
    {
    uint8_t id = 0;
    if (!regRead(REG_WHO_AM_I, &id, 1)) { why = "no answer at 0x6A"; return false; }
    if (id != IMU_WHO_AM_I)             { why = "wrong WHO_AM_I";    return false; }

    // Software reset, ST's sequence: everything to power-down first (after a
    // warm restart of the Teensy the chip is still running its old setup),
    // then SW_RESET, then wait for the bit to clear itself.
    bool ok = regWrite(REG_FIFO_CTRL4, FIFO_MODE_BYPASS)
           && regWrite(REG_CTRL1_XL_HG, 0)
           && regWrite(REG_CTRL1, 0)
           && regWrite(REG_CTRL2, 0)
           && regWrite(REG_CTRL3, CTRL3_SW_RESET);
    if (!ok) { why = "reset write failed"; return false; }

    uint8_t ctrl3 = CTRL3_SW_RESET;
    for (uint8_t tries = 0; tries < 20 && (ctrl3 & CTRL3_SW_RESET); tries++)
        {
        delay(1);
        if (!regRead(REG_CTRL3, &ctrl3, 1)) ctrl3 = CTRL3_SW_RESET;
        }
    if (ctrl3 & CTRL3_SW_RESET) { why = "reset did not complete"; return false; }

    // The setup. Each line is one register; the same values are read back
    // below, so a chip that silently refused one cannot pass as configured.
    struct RegValue { uint8_t reg, value; };
    static const RegValue setup[] =
        {
        { REG_CTRL3,            (uint8_t)(CTRL3_BDU | CTRL3_IF_INC) },       // the reset defaults, stated
        { REG_FIFO_CTRL4,       FIFO_MODE_BYPASS },                          // FIFO off until the pump runs
        { REG_FIFO_CTRL1,       0 },
        { REG_FIFO_CTRL2,       0 },                                         // no compression, no stop-on-watermark
        { REG_CTRL8,            FS_XL_16G },                                 // low-g +-16 g
        { REG_CTRL6,            FS_G_2000DPS },                              // gyro +-2000 dps
        { REG_CTRL1,            (uint8_t)((OP_MODE_HIGH_PERF << 4) | ODR_120HZ) }, // low-g on - BEFORE high-g
        { REG_CTRL2,            (uint8_t)((OP_MODE_HIGH_PERF << 4) | ODR_120HZ) }, // gyro on
        { REG_CTRL1_XL_HG,      (uint8_t)(CTRL1_XL_HG_REGOUT | (ODR_XL_HG_960HZ << 3) | hgFsCode()) },
        { REG_FIFO_CTRL3,       (uint8_t)((ODR_120HZ << 4) | ODR_120HZ) },   // batch gyro + low-g at 120 Hz
        { REG_COUNTER_BDR_REG1, XL_HG_BATCH_EN },                            // batch high-g (at its own rate)
        };

    for (const RegValue& rv : setup)
        if (!regWrite(rv.reg, rv.value)) { why = "setup write failed"; return false; }

    for (const RegValue& rv : setup)
        {
        uint8_t back = 0;
        if (!regRead(rv.reg, &back, 1)) { why = "setup read-back failed"; return false; }
        if (back != rv.value)
            {
            Serial.printf("IMU: register 0x%02X reads 0x%02X, wrote 0x%02X\n", rv.reg, back, rv.value);
            why = "setup did not stick";
            return false;
            }
        }
    return true;
    }

// Shared by boot and by the retry path.
static bool bringUp(bool announce)
    {
    const char* why = "";
    s_bus.begin(IMU_I2C_HZ);
    s_sinceRetryMs = 0;

    if (!configureChip(why))
        {
        g_data.imu.present = false;
        s_state = PUMP_OFF;
        if (announce)
            Serial.printf("IMU: %s - motion data disabled (retrying every %lu s)\n",
                          why, (unsigned long)(IMU_RETRY_MS / 1000));
        return false;
        }

    g_data.imu.present = true;
    s_consecErrors     = 0;
    s_state            = PUMP_START;
    Serial.printf("IMU: LSM6DSV320X online, high-g +-%u g at %u Hz, low-g + gyro at %u Hz, I2C at %lu kHz\n",
                  IMU_HG_FULL_SCALE_G, IMU_HG_RATE_HZ, IMU_LG_RATE_HZ,
                  (unsigned long)(IMU_I2C_HZ / 1000));
    return true;
    }

// --- error accounting ---------------------------------------------------------------

static void transferFailed(const char* what)
    {
    g_data.imu.busErrors++;
    if (++s_consecErrors >= IMU_MAX_CONSEC_ERRS)
        {
        // The chip stopped answering. Say so once and stand down; the retry
        // timer in imuPump() will try a full re-configuration.
        Serial.printf("IMU: %u consecutive bus errors (last: %s) - standing down\n",
                      s_consecErrors, what);
        g_data.imu.present = false;
        s_state        = PUMP_OFF;
        s_sinceRetryMs = 0;
        motionOnDataGap();
        return;
        }
    // Try again: the FIFO switch-on if that was what failed, else at the
    // next poll.
    s_state = (s_state == PUMP_STARTING) ? PUMP_START : PUMP_IDLE;
    }

// --- FIFO word decoding -------------------------------------------------------------

static inline int16_t le16(const uint8_t* p)
    {
    return (int16_t)(p[0] | (p[1] << 8));
    }

static void decodeWords(const uint8_t* buf, uint16_t words)
    {
    // The first samples after power-up are the sensors settling (the gyro
    // needs 40 ms): pop them, don't use them.
    const bool settling = (millis() - s_streamStartMs) < IMU_SETTLE_MS;

    uint16_t hg = 0;
    for (uint16_t w = 0; w < words; w++, buf += FIFO_WORD_BYTES)
        {
        const uint8_t tag = buf[0] >> 3; // TAG_SENSOR[4:0]; the low bits are a counter + parity
        const int16_t v[3] = { le16(buf + 1), le16(buf + 3), le16(buf + 5) };

        switch (tag)
            {
            case TAG_HIGH_G: hg++; if (!settling) motionFeedHighG(v); break;
            case TAG_LOW_G:        if (!settling) motionFeedLowG(v);  break;
            case TAG_GYRO:         if (!settling) motionFeedGyro(v);  break;
            default:               g_data.imu.tagErrors++;            break;
            }
        }

    g_data.imu.hgSamples += hg;
    if (!settling && words) g_data.imu.lastDataMs = millis();
    }

// --- public API ---------------------------------------------------------------------

bool imuInit()
    {
    s_rateLastMs = millis();
    return bringUp(true);
    }

void imuPump()
    {
    ImuData& imu = g_data.imu;

    switch (s_state)
        {
        case PUMP_OFF:
            // No chip. A retry costs a few ms of blocking I2C, so it is rare.
            if (s_sinceRetryMs >= IMU_RETRY_MS)
                bringUp(false);
            return;

        case PUMP_START:
            // Switch the FIFO on. Done here rather than in imuInit() so the
            // FIFO starts empty at the moment the superloop starts draining.
            s_txBuf[0] = REG_FIFO_CTRL4;
            s_txBuf[1] = FIFO_MODE_CONTINUOUS;
            s_bus.write_async(IMU_I2C_ADDR, s_txBuf, 2, true);
            s_xferMs = 0;
            s_state  = PUMP_STARTING;
            return;

        case PUMP_IDLE:
            // Streaming but silent for too long: the chip lost its setup (a
            // supply dip only it noticed). Reconfigure, at most every
            // IMU_RETRY_MS.
            if (millis() - s_streamStartMs > IMU_STALL_MS + IMU_SETTLE_MS
                && millis() - imu.lastDataMs > IMU_STALL_MS
                && s_sinceRetryMs >= IMU_RETRY_MS)
                {
                Serial.println("IMU: no samples - reconfiguring");
                motionOnDataGap();
                bringUp(true);
                return;
                }
            if (s_sincePollMs < IMU_POLL_MS)
                return;
            s_sincePollMs = 0;
            s_txBuf[0] = REG_FIFO_STATUS1;
            s_bus.write_async(IMU_I2C_ADDR, s_txBuf, 1, false);
            s_xferMs = 0;
            s_state  = PUMP_STATUS_ADDR;
            return;

        default:
            break; // a transfer is in flight: handled below
        }

    // --- a transfer is in flight ---
    if (!s_bus.finished())
        {
        if (s_xferMs > IMU_XFER_TIMEOUT_MS)
            {
            s_bus.begin(IMU_I2C_HZ);
            transferFailed("timeout");
            }
        return;
        }
    if (s_bus.has_error())
        {
        transferFailed("nack/bus error");
        return;
        }
    s_consecErrors = 0;

    switch (s_state)
        {
        case PUMP_STARTING:
            s_streamStartMs = millis();
            s_expectOverrun = true;
            s_sincePollMs   = 0;
            s_state         = PUMP_IDLE;
            break;

        case PUMP_STATUS_ADDR:
            s_bus.read_async(IMU_I2C_ADDR, s_statusBuf, 2, true);
            s_xferMs = 0;
            s_state  = PUMP_STATUS_READ;
            break;

        case PUMP_STATUS_READ:
            s_wordsWaiting = (uint16_t)(s_statusBuf[0] | ((s_statusBuf[1] & 0x01) << 8));
            if (s_wordsWaiting > imu.fifoPeakWords) imu.fifoPeakWords = s_wordsWaiting;
            if (s_statusBuf[1] & FIFO_STATUS2_OVR)
                {
                // The FIFO filled and dropped its oldest samples: the stream
                // has a hole in it. motion.cpp must not integrate across it.
                if (!s_expectOverrun)
                    {
                    imu.fifoOverruns++;
                    motionOnDataGap();
                    }
                }
            s_expectOverrun = false;
            if (s_wordsWaiting == 0)
                {
                s_state = PUMP_IDLE;
                break;
                }
            s_txBuf[0] = REG_FIFO_DATA_OUT;
            s_bus.write_async(IMU_I2C_ADDR, s_txBuf, 1, false);
            s_xferMs = 0;
            s_state  = PUMP_DATA_ADDR;
            break;

        case PUMP_DATA_ADDR:
            s_wordsInFlight = (s_wordsWaiting < IMU_FIFO_BURST_WORDS) ? s_wordsWaiting
                                                                      : IMU_FIFO_BURST_WORDS;
            s_bus.read_async(IMU_I2C_ADDR, s_fifoBuf, (size_t)s_wordsInFlight * FIFO_WORD_BYTES, true);
            s_xferMs = 0;
            s_state  = PUMP_DATA_READ;
            break;

        case PUMP_DATA_READ:
            decodeWords(s_fifoBuf, s_wordsInFlight);
            s_wordsWaiting -= s_wordsInFlight;
            if (s_wordsWaiting > 0)
                {
                // More were waiting than one burst carries: go straight back
                // for them instead of waiting out the poll interval.
                s_txBuf[0] = REG_FIFO_DATA_OUT;
                s_bus.write_async(IMU_I2C_ADDR, s_txBuf, 1, false);
                s_xferMs = 0;
                s_state  = PUMP_DATA_ADDR;
                }
            else
                s_state = PUMP_IDLE;
            break;

        default:
            s_state = PUMP_IDLE;
            break;
        }
    }

void imuTick1s()
    {
    ImuData& imu = g_data.imu;
    const uint32_t now = millis();
    const uint32_t dt  = now - s_rateLastMs;
    if (dt >= 500)
        {
        imu.hgRateHz    = (imu.hgSamples - s_rateLastCount) * 1000.0f / dt;
        s_rateLastCount = imu.hgSamples;
        s_rateLastMs    = now;
        }
    }
