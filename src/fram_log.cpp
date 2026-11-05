// ---------------------------------------------------------------------------
// Rally CAN Gauge - FRAM persistence implementation
// ---------------------------------------------------------------------------
// THE ONE PROBLEM THIS FILE IS SHAPED AROUND: the kill switch. Power can
// vanish between any two bytes, so nothing here ever overwrites the only
// copy of anything. The pattern, used three times over:
//
//   * records carry a SEQUENCE NUMBER (which copy is newest?) and a CRC
//     (is this copy complete?),
//   * consecutive writes land in DIFFERENT slots (fuel: 16 rotating slots,
//     log head pointer: 2 ping-pong slots, log: the ring itself),
//   * recovery = "read the candidates, keep the newest one whose CRC
//     passes." A record that was mid-write when power died just loses the
//     CRC lottery and the previous one wins - costing one second of data.
//
// ON-CHIP LAYOUT (addresses in config.h, byte-level detail in
// docs/FRAM_BLACKBOX.md):
//
//   0x00000  header       32 B   magic + version + geometry; sanity gate
//   0x00020  fuel ring    16 x 32 B  slot = seq % 16
//   0x00220  log head ptr  2 x 16 B  slot = seq % 2 - "where does the log
//                                    resume?", updated after every record
//   0x00400  black box    4064 x 32 B circular, to the end of the chip
//
// WHY A HEAD POINTER RECORD AT ALL: the ring's write position must survive
// reboots or every boot would restart at index 0 and chew up the oldest
// data. The alternatives were (a) scan all 4064 record headers at boot
// (~1.3 s of I2C) or (b) binary-search the sequence discontinuity (clever,
// fragile around corrupt records). Spending 16 bytes per second on a
// pointer record makes recovery two reads and dead simple. F-RAM endurance
// makes the "same two slots, once a second, forever" pattern free.
//
// BLACK-BOX GATING: a record is written on the 1 s tick when the second is
// "worth remembering" - engine turning, or a yellow/red alarm active (CAN-
// link alarms excluded: a gauge powered with the car off would otherwise
// log "NO CAN" in a loop until the ring's real content was gone). Cadence
// is every LOG_NOMINAL_EVERY_S normally, every LOG_ALARM_EVERY_S while an
// alarm is up - the boring laps stretch the ring to ~2.2 h, the spicy parts
// get full resolution. After the trigger clears, logging runs LOG_GRACE_MS
// longer: the 30 s AFTER a stall or alarm is exactly the evidence a black
// box exists to keep.
//
// FIELD ENCODING: 8-bit scaled integers, chosen so range and resolution
// beat the sensors' own honesty (1 kPa, 0.1 V, 1 C at +40 offset, 0.1 AFR,
// 10 mV/cell). A float per field would triple the record and halve the
// history for precision nobody would ever read back.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <math.h>

#include "fram_log.h"
#include "fram.h"
#include "config.h"
#include "vehicle_data.h"
#include "alarms.h"    // alarmColor()/alarmName() for gating + CSV decode
#include "fuel_calc.h" // fuelCalcGetState() - the accumulators to persist

// --- record formats ---------------------------------------------------------------
// All little-endian (the Teensy's native order), packed so the in-memory
// struct IS the on-chip byte layout. Any change to any of these must bump
// FRAM_LAYOUT_VERSION: the header check will then treat the chip as foreign
// and reformat rather than misread old-layout records.

static const uint32_t FRAM_MAGIC          = ((uint32_t)'G' << 24) | ((uint32_t)'0' << 16)
                                          | ((uint32_t)'3' << 8) | (uint32_t)'E'; // "E30G"
// v2: DIALFAIL joined the alarm catalog between NO CAN and BATTCOLD, which
// shifted the bit numbers of every alarm below it in LogRecord.alarmBits.
// Old chips reformat on first boot rather than decode blue alarms as
// their neighbors.
static const uint16_t FRAM_LAYOUT_VERSION = 2;

struct __attribute__((packed)) FramHeader          // 32 B at 0x0000
    {
    uint32_t magic;       // FRAM_MAGIC, else the chip is virgin/foreign
    uint16_t version;     // FRAM_LAYOUT_VERSION at format time
    uint16_t logRecSize;  // geometry snapshot: if a future firmware changes
    uint32_t logCapacity; // these, the mismatch is detectable
    uint32_t formatCount; // times this chip has been formatted (curiosity)
    uint8_t  pad[14];
    uint16_t crc;
    };
static_assert(sizeof(FramHeader) == 32, "header must be exactly 32 bytes");

struct __attribute__((packed)) FuelRecord          // 32 B x 16 slots at 0x0020
    {
    uint32_t seq;            // newest valid slot wins at boot
    double   remainingGal;   // the fuel_calc accumulators, verbatim - doubles
    double   burnedTotalGal; // so a restore is bit-exact (see fuel_calc.cpp
                             // for why these are doubles at all)
    uint8_t  flags;          // bit0: calcValid when saved (always 1 today)
    uint8_t  pad[9];
    uint16_t crc;
    };
static_assert(sizeof(FuelRecord) == 32, "fuel record must be exactly 32 bytes");

struct __attribute__((packed)) LogPtrRecord        // 16 B x 2 slots at 0x0220
    {
    uint32_t nextSeq;  // sequence number the NEXT log record will carry
    uint32_t nextIdx;  // ring index it will be written to
    uint8_t  session;  // session of the run that last logged (boot bumps it)
    uint8_t  pad[5];
    uint16_t crc;
    };
static_assert(sizeof(LogPtrRecord) == 16, "log pointer must be exactly 16 bytes");

struct __attribute__((packed)) LogRecord           // 32 B, the black box entry
    {
    uint32_t seq;          // monotonic forever (never resets except format)
    uint8_t  session;      // which run this belongs to - splits the CSV
    uint8_t  hh, mm, ss;   // RTC wall clock; 0xFF = clock wasn't set
    uint16_t ecuSecondsOn; // the ECU's own uptime counter - the key that
                           // lines this log up with the ECU's SD log
    uint16_t rpm;
    uint8_t  oilPressPsi;  // 1 psi/count
    uint8_t  oilTempC40;   // deg C + 40 (covers -40..+215)
    uint8_t  cltC40;       // deg C + 40
    uint8_t  mapKpa;       // 1 kPa/count
    uint8_t  tpsPct;       // %
    uint8_t  afr10;        // AFR x 10
    uint8_t  afrTgt10;     // AFR target x 10
    uint8_t  battV10;      // volts x 10 (ECU-measured 12V system)
    uint8_t  injDutyPct;   // %
    uint8_t  fuelSendPct;  // slosh-filtered sender level, %
    uint8_t  fuelCalcPct;  // calculated fuel, %
    int8_t   packAmps;     // BMS pack current, 1 A, + = discharge, clamped
    uint8_t  cellMin100;   // (min cell V - 1.0) x 100 -> 1.00..3.55 V
    uint8_t  battMaxC40;   // hottest BMS cell, deg C + 40
    uint32_t alarmBits;    // bit n = AlarmId n active this second
    uint16_t crc;
    };
static_assert(sizeof(LogRecord) == 32, "log record must be exactly 32 bytes");

// Ring capacity: every 32-byte slot that fits between the bookkeeping area
// and the end of the chip. (131072 - 1024) / 32 = 4064 records.
static const uint32_t LOG_CAPACITY = (FRAM_SIZE_BYTES - FRAM_ADDR_LOG) / sizeof(LogRecord);

// --- module state -----------------------------------------------------------------

static bool     s_mounted;             // header verified, safe to write
static uint32_t s_logSeq  = 1;         // seq for the NEXT log record
static uint32_t s_logIdx;              // ring index for the NEXT log record
static uint8_t  s_session = 1;         // this boot's run number
static uint32_t s_fuelSeq = 1;         // seq for the NEXT fuel record

// Change detection for the fuel save: NAN never compares equal to anything,
// so the very first valid state always saves.
static double s_lastSavedRemaining = NAN;
static double s_lastSavedBurned    = NAN;

static uint32_t s_worthyUntilMs;       // black box records until millis() passes this
static uint8_t  s_ticksSinceRec = 255; // big, so the first worthy tick logs immediately

// --- small helpers ----------------------------------------------------------------

static uint32_t logSlotAddr(uint32_t idx)  { return FRAM_ADDR_LOG + idx * sizeof(LogRecord); }
static uint32_t fuelSlotAddr(uint32_t seq) { return FRAM_ADDR_FUEL + (seq % FRAM_FUEL_SLOTS) * sizeof(FuelRecord); }
static uint32_t ptrSlotAddr(uint32_t seq)  { return FRAM_ADDR_LOGPTR + (seq % 2) * sizeof(LogPtrRecord); }

// Enqueue + pump until it's on the chip. Boot/console use only - the 1 s
// tick never calls this (it fires and forgets via framWriteAsync).
static bool writeSync(uint32_t addr, const void* src, uint8_t len)
    {
    if (!framWriteAsync(addr, src, len))
        return false;
    elapsedMillis guard;
    while (!framIdle())
        {
        framPump();
        if (guard > 50) return false;
        }
    return true;
    }

// Scaled-integer encoders. Everything clamps: a sensor spiking outside its
// encoding range must saturate, not wrap around into a plausible lie.
static uint8_t encU8(float v)    { return (uint8_t)constrain(v, 0.0f, 255.0f); }
static uint8_t encTempC(float c) { return encU8(c + 40.0f); }
static uint8_t encX10(float v)   { return encU8(v * 10.0f); }
static int8_t  encAmps(float a)  { return (int8_t)constrain(a, -127.0f, 127.0f); }
static uint8_t encCell(float v)  { return encU8((v - 1.0f) * 100.0f); }

// --- black-box gating -------------------------------------------------------------

// "Spicy": any yellow/red alarm EXCEPT the CAN-link pair. NO CAN / BATT CAN
// mean the data sources are gone - there is nothing worth recording, and a
// gauge left powered with everything else off would log them forever.
static bool spicyAlarmActive()
    {
    for (uint8_t i = 0; i < ALARM_COUNT; i++)
        {
        if (!g_data.alarms.active[i]) continue;
        if (i == ALARM_NO_CAN || i == ALARM_BATT_CAN) continue;
        if (alarmColor(i) >= ALARM_YELLOW) return true;
        }
    return false;
    }

static bool engineTurning()
    {
    return g_data.ecuOnline() && g_data.ecu.rpm > 0;
    }

// --- fuel record persistence ------------------------------------------------------

bool framFuelLoad(double& remainingGal, double& burnedTotalGal)
    {
    if (!s_mounted)
        return false;

    // Read all 16 slots, keep the newest one that is both intact (CRC) and
    // sane (a valid CRC on insane numbers would mean a firmware bug wrote
    // garbage - refuse it rather than gauge off a lie).
    bool found = false;
    FuelRecord best = {};
    for (uint8_t slot = 0; slot < FRAM_FUEL_SLOTS; slot++)
        {
        FuelRecord rec;
        if (!framRead(FRAM_ADDR_FUEL + slot * sizeof(FuelRecord), &rec, sizeof(rec)))
            continue;
        if (framCrc16(&rec, sizeof(rec) - 2) != rec.crc) continue;
        if (!(rec.flags & 0x01)) continue;
        if (!isfinite(rec.remainingGal) || rec.remainingGal < 0.0
            || rec.remainingGal > FUEL_TANK_GAL * 1.25) continue;
        if (!isfinite(rec.burnedTotalGal) || rec.burnedTotalGal < 0.0
            || rec.burnedTotalGal > 100000.0) continue;
        if (!found || rec.seq > best.seq) { best = rec; found = true; }
        }

    if (!found)
        return false;

    s_fuelSeq            = best.seq + 1;
    remainingGal         = best.remainingGal;
    burnedTotalGal       = best.burnedTotalGal;
    // Prime change detection so an unchanged value doesn't re-save at boot.
    s_lastSavedRemaining = best.remainingGal;
    s_lastSavedBurned    = best.burnedTotalGal;
    return true;
    }

static void fuelSaveTick()
    {
    double rem, burned;
    bool   valid;
    fuelCalcGetState(rem, burned, valid);
    if (!valid)
        return;
    // Bit-exact compare is the right test here: these are copies of the same
    // doubles, so "unchanged" means literally identical - and engine-off
    // means unchanged, which is what makes the bus go quiet in the pits.
    if (rem == s_lastSavedRemaining && burned == s_lastSavedBurned)
        return;

    FuelRecord rec    = {};
    rec.seq            = s_fuelSeq;
    rec.remainingGal   = rem;
    rec.burnedTotalGal = burned;
    rec.flags          = 0x01;
    rec.crc            = framCrc16(&rec, sizeof(rec) - 2);

    if (!framWriteAsync(fuelSlotAddr(s_fuelSeq), &rec, sizeof(rec)))
        return; // queue hiccup: don't advance, same state retries next second

    s_fuelSeq++;
    s_lastSavedRemaining = rem;
    s_lastSavedBurned    = burned;
    g_data.fram.fuelSaves++;
    }

// --- the black box ----------------------------------------------------------------

static bool s_dumpSuspendsLog; // console dump in progress - hold new records

static void blackBoxTick()
    {
    // Update the "worth remembering" window.
    if (engineTurning() || spicyAlarmActive())
        s_worthyUntilMs = millis() + LOG_GRACE_MS;
    const bool inWindow = (int32_t)(millis() - s_worthyUntilMs) < 0;
    g_data.fram.logging = inWindow && !s_dumpSuspendsLog;
    if (!g_data.fram.logging)
        return;

    // Cadence: full rate while an alarm is up, relaxed when nominal.
    const uint8_t everyN = spicyAlarmActive() ? LOG_ALARM_EVERY_S : LOG_NOMINAL_EVERY_S;
    if (s_ticksSinceRec < 254) s_ticksSinceRec++; // saturate, don't wrap
    if (s_ticksSinceRec < everyN)
        return;

    const EcuData&  e = g_data.ecu;
    const BmsData&  m = g_data.bms;
    const FuelData& f = g_data.fuel;

    LogRecord rec   = {};
    rec.seq          = s_logSeq;
    rec.session      = s_session;
    rec.hh           = g_data.board.clockValid ? g_data.board.hh : 0xFF;
    rec.mm           = g_data.board.clockValid ? g_data.board.mm : 0xFF;
    rec.ss           = g_data.board.clockValid ? g_data.board.ss : 0xFF;
    rec.ecuSecondsOn = e.secondsOn;
    rec.rpm          = e.rpm;
    rec.oilPressPsi  = encU8(e.oilPressPsi);
    rec.oilTempC40   = encTempC(e.oilTempC);
    rec.cltC40       = encTempC(e.cltC);
    rec.mapKpa       = encU8(e.mapKpa);
    rec.tpsPct       = encU8(e.tpsPct);
    rec.afr10        = encX10(e.afr);
    rec.afrTgt10     = encX10(e.afrTarget);
    rec.battV10      = encX10(e.battVolts);
    rec.injDutyPct   = encU8(e.injDutyPct);
    rec.fuelSendPct  = encU8(f.levelPct);
    rec.fuelCalcPct  = encU8(f.calcPct);
    rec.packAmps     = encAmps(m.packAmps);
    rec.cellMin100   = encCell(m.minCellVolts);
    rec.battMaxC40   = encTempC(m.maxCellTempC);
    for (uint8_t i = 0; i < ALARM_COUNT; i++)
        if (g_data.alarms.active[i])
            rec.alarmBits |= (1UL << i);
    rec.crc = framCrc16(&rec, sizeof(rec) - 2);

    if (!framWriteAsync(logSlotAddr(s_logIdx), &rec, sizeof(rec)))
        return; // queue full (shouldn't happen) - retry next tick

    s_logSeq++;
    s_logIdx = (s_logIdx + 1) % LOG_CAPACITY;
    s_ticksSinceRec = 0;
    if (g_data.fram.logRecords < LOG_CAPACITY) g_data.fram.logRecords++;
    g_data.fram.logSeq = s_logSeq;

    // The head pointer follows its record into the queue. If power dies
    // between the two, next boot's pointer is one behind and the record
    // above gets overwritten - one second of log, the accepted loss.
    LogPtrRecord ptr = {};
    ptr.nextSeq = s_logSeq;
    ptr.nextIdx = s_logIdx;
    ptr.session = s_session;
    ptr.crc     = framCrc16(&ptr, sizeof(ptr) - 2);
    framWriteAsync(ptrSlotAddr(ptr.nextSeq), &ptr, sizeof(ptr));
    }

// --- mount / format ---------------------------------------------------------------

static void formatChip(uint32_t formatCount)
    {
    Serial.println("FRAM: formatting (header + fuel slots + log pointer)...");

    FramHeader h  = {};
    h.magic        = FRAM_MAGIC;
    h.version      = FRAM_LAYOUT_VERSION;
    h.logRecSize   = sizeof(LogRecord);
    h.logCapacity  = LOG_CAPACITY;
    h.formatCount  = formatCount;
    h.crc          = framCrc16(&h, sizeof(h) - 2);
    writeSync(FRAM_ADDR_HEADER, &h, sizeof(h));

    // Blank the fuel slots so stale records can't win the seq race. The log
    // ring itself is NOT wiped (127 KB): resetting the head pointer makes
    // old records unreachable - the dump walks sequence continuity backward
    // from the head, and the fresh sequence numbering never matches them.
    uint8_t zeros[sizeof(FuelRecord)] = {};
    for (uint8_t slot = 0; slot < FRAM_FUEL_SLOTS; slot++)
        writeSync(FRAM_ADDR_FUEL + slot * sizeof(FuelRecord), zeros, sizeof(zeros));

    LogPtrRecord ptr = {};
    ptr.nextSeq = 1;
    ptr.nextIdx = 0;
    ptr.session = 0; // the first run to log will call itself session 1
    ptr.crc     = framCrc16(&ptr, sizeof(ptr) - 2);
    writeSync(ptrSlotAddr(0), &ptr, sizeof(ptr)); // seq 0 -> slot 0
    writeSync(ptrSlotAddr(1), &ptr, sizeof(ptr)); // both slots start valid

    s_logSeq  = 1;
    s_logIdx  = 0;
    s_session = 1;
    s_fuelSeq = 1;
    s_lastSavedRemaining = NAN; // force the next valid fuel state to save
    s_lastSavedBurned    = NAN;
    g_data.fram.logRecords = 0;
    g_data.fram.logSeq     = 1;
    g_data.fram.session    = 1;
    }

void framLogInit()
    {
    g_data.fram.logCapacity = LOG_CAPACITY;
    if (!g_data.fram.present)
        return; // framInit already announced the bad news

    FramHeader h;
    const bool headerOk = framRead(FRAM_ADDR_HEADER, &h, sizeof(h))
                       && framCrc16(&h, sizeof(h) - 2) == h.crc
                       && h.magic == FRAM_MAGIC
                       && h.version == FRAM_LAYOUT_VERSION
                       && h.logRecSize == sizeof(LogRecord)
                       && h.logCapacity == LOG_CAPACITY;
    s_mounted = true; // needed so format/mount below may write
    if (!headerOk)
        {
        // Virgin chip, or a layout change made the old contents unreadable.
        formatChip(1);
        Serial.printf("FRAM: formatted - log holds %lu records (%lu min at 1 rec/%lus)\n",
                      (unsigned long)LOG_CAPACITY,
                      (unsigned long)(LOG_CAPACITY * LOG_NOMINAL_EVERY_S / 60),
                      (unsigned long)LOG_NOMINAL_EVERY_S);
        }
    else
        {
        // Recover the log head: two pointer slots, newest valid one wins.
        LogPtrRecord best  = {};
        bool         found = false;
        for (uint8_t slot = 0; slot < 2; slot++)
            {
            LogPtrRecord p;
            if (!framRead(FRAM_ADDR_LOGPTR + slot * sizeof(LogPtrRecord), &p, sizeof(p)))
                continue;
            if (framCrc16(&p, sizeof(p) - 2) != p.crc) continue;
            if (p.nextIdx >= LOG_CAPACITY || p.nextSeq == 0) continue;
            if (!found || p.nextSeq > best.nextSeq) { best = p; found = true; }
            }
        if (found)
            {
            s_logSeq  = best.nextSeq;
            s_logIdx  = best.nextIdx;
            s_session = best.session + 1; // ptr only advances when records are
                                          // written, so idle power cycles don't
                                          // burn session numbers
            g_data.fram.logRecords = min(s_logSeq - 1, LOG_CAPACITY);
            }
        else
            {
            // Both pointer slots corrupt: pathological (each write alternates
            // slots and is CRC-checked). Restart the numbering; old records
            // become unreachable but the chip stays usable.
            Serial.println("FRAM: WARNING both log pointers corrupt - restarting log");
            formatChip(h.formatCount + 1);
            }
        g_data.fram.logSeq  = s_logSeq;
        g_data.fram.session = s_session;
        Serial.printf("FRAM: mounted - log resumes at seq %lu (%lu/%lu records), session %u\n",
                      (unsigned long)s_logSeq, (unsigned long)g_data.fram.logRecords,
                      (unsigned long)LOG_CAPACITY, s_session);
        }
    g_data.fram.mounted = true;
    }

void framLogTick1s()
    {
    if (!g_data.fram.present || !s_mounted)
        return;
    fuelSaveTick();
    blackBoxTick();
    }

// --- serial console + chunked CSV dump --------------------------------------------
// The dump runs as a state machine pumped from framLogPoll(), a few records
// per superloop pass, so the 20 ms tick (and the watchdog feed inside it)
// never waits on 4000 I2C reads. Two phases:
//
//   COUNTING: walk BACKWARD from the head while records are intact and the
//     sequence numbers run contiguously - that finds where readable history
//     begins (and stops cold at pre-format garbage, which can't match the
//     current numbering).
//   PRINTING: walk forward from that start, decode each record to CSV.
//
// New black-box records are held off during a dump so the ring can't
// overwrite the tail out from under the walk (fuel saves continue - fuel
// beats forensics).

enum DumpPhase : uint8_t { DUMP_IDLE = 0, DUMP_COUNTING, DUMP_PRINTING };
static DumpPhase s_dumpPhase;
static uint32_t  s_dumpStep;   // records examined/printed so far this phase
static uint32_t  s_dumpCount;  // COUNTING's answer: readable records

static const uint8_t DUMP_RECS_PER_PASS = 4; // ~4 x 0.33 ms of I2C per pass, max

static uint32_t s_formatArmedMs; // 'X' pressed; 'Y' within 5 s formats

static void dumpStart()
    {
    if (g_data.fram.logRecords == 0)
        {
        Serial.println("log: empty - nothing to dump");
        return;
        }
    s_dumpPhase        = DUMP_COUNTING;
    s_dumpStep         = 0;
    s_dumpCount        = 0;
    s_dumpSuspendsLog  = true;
    Serial.println("log: scanning ring (send 'q' to abort)...");
    }

static void dumpStop(const char* why)
    {
    s_dumpPhase       = DUMP_IDLE;
    s_dumpSuspendsLog = false;
    Serial.printf("log: dump %s\n", why);
    }

// One backward step of the COUNTING walk. Returns true once the start of
// readable history is found (a record that is missing, corrupt, or from a
// different numbering era breaks the chain).
static bool dumpCountStep()
    {
    if (s_dumpStep >= LOG_CAPACITY || s_dumpStep >= s_logSeq - 1)
        return true; // walked the whole ring / reached record #1

    const uint32_t idx      = (s_logIdx + LOG_CAPACITY - 1 - s_dumpStep) % LOG_CAPACITY;
    const uint32_t expected = s_logSeq - 1 - s_dumpStep;
    LogRecord      rec;
    if (!framRead(logSlotAddr(idx), &rec, sizeof(rec))
        || framCrc16(&rec, sizeof(rec) - 2) != rec.crc
        || rec.seq != expected)
        return true;

    s_dumpStep++;
    return false;
    }

static void dumpPump()
    {
    for (uint8_t n = 0; n < DUMP_RECS_PER_PASS && s_dumpPhase != DUMP_IDLE; n++)
        {
        if (s_dumpPhase == DUMP_COUNTING)
            {
            if (!dumpCountStep())
                continue;
            s_dumpCount = s_dumpStep;
            Serial.printf("log: %lu readable records, oldest first:\n",
                          (unsigned long)s_dumpCount);
            Serial.println("seq,session,time,ecu_s,rpm,oil_psi,oil_C,clt_C,map_kPa,tps,"
                           "afr,afr_tgt,batt_V,inj_duty,fuel_sender,fuel_calc,"
                           "pack_A,cell_min_V,batt_C,alarm_hex,alarms");
            s_dumpPhase = DUMP_PRINTING;
            s_dumpStep  = 0;
            if (s_dumpCount == 0) dumpStop("done (no contiguous history)");
            continue;
            }

        // DUMP_PRINTING
        if (s_dumpStep >= s_dumpCount)
            {
            dumpStop("done");
            return;
            }
        const uint32_t idx = (s_logIdx + LOG_CAPACITY - s_dumpCount + s_dumpStep) % LOG_CAPACITY;
        LogRecord      r;
        if (!framRead(logSlotAddr(idx), &r, sizeof(r)))
            {
            dumpStop("ABORTED (read error)");
            return;
            }
        char clock[10];
        if (r.hh == 0xFF) strcpy(clock, "--:--:--");
        else              snprintf(clock, sizeof(clock), "%02u:%02u:%02u", r.hh, r.mm, r.ss);

        // Decode the alarm bitmask into names: "OIL LOW+FUEL LOW".
        char names[256] = "";
        for (uint8_t i = 0; i < ALARM_COUNT; i++)
            {
            if (!(r.alarmBits & (1UL << i))) continue;
            char one[10];
            strncpy(one, alarmName(i), 9);
            one[9] = 0;
            for (int8_t t = strlen(one) - 1; t >= 0 && one[t] == ' '; t--) one[t] = 0;
            if (names[0]) strlcat(names, "+", sizeof(names));
            strlcat(names, one, sizeof(names));
            }

        Serial.printf("%lu,%u,%s,%u,%u,%u,%d,%d,%u,%u,%.1f,%.1f,%.1f,%u,%u,%u,%d,%.2f,%d,%08lX,%s\n",
                      (unsigned long)r.seq, r.session, clock, r.ecuSecondsOn, r.rpm,
                      r.oilPressPsi, (int)r.oilTempC40 - 40, (int)r.cltC40 - 40,
                      r.mapKpa, r.tpsPct,
                      r.afr10 / 10.0f, r.afrTgt10 / 10.0f, r.battV10 / 10.0f,
                      r.injDutyPct, r.fuelSendPct, r.fuelCalcPct,
                      (int)r.packAmps, 1.0f + r.cellMin100 / 100.0f,
                      (int)r.battMaxC40 - 40,
                      (unsigned long)r.alarmBits, names);
        s_dumpStep++;
        }
    }

static void printFuelSlots()
    {
    Serial.println("fuel slots (slot: seq remaining/burned):");
    for (uint8_t slot = 0; slot < FRAM_FUEL_SLOTS; slot++)
        {
        FuelRecord rec;
        if (!framRead(FRAM_ADDR_FUEL + slot * sizeof(FuelRecord), &rec, sizeof(rec)))
            { Serial.printf("  %2u: read error\n", slot); continue; }
        if (framCrc16(&rec, sizeof(rec) - 2) != rec.crc)
            { Serial.printf("  %2u: empty/invalid\n", slot); continue; }
        Serial.printf("  %2u: seq %-8lu %7.3f gal remaining, %8.3f burned%s\n",
                      slot, (unsigned long)rec.seq,
                      rec.remainingGal, rec.burnedTotalGal,
                      (rec.seq + 1 == s_fuelSeq) ? "  <- newest" : "");
        }
    Serial.printf("next fuel seq %lu -> slot %lu\n",
                  (unsigned long)s_fuelSeq, (unsigned long)(s_fuelSeq % FRAM_FUEL_SLOTS));
    }

static void printHelp()
    {
    Serial.println("FRAM console:  l = dump black box as CSV   f = show fuel slots");
    Serial.println("               X then Y = FORMAT (erases fuel record + log)   q = abort dump");
    }

void framLogPoll()
    {
    if (s_dumpPhase != DUMP_IDLE)
        dumpPump();

    while (Serial.available())
        {
        const char c = Serial.read();
        if (c == '\r' || c == '\n') continue;

        if (s_dumpPhase != DUMP_IDLE) // any key aborts a running dump
            {
            dumpStop("ABORTED");
            continue;
            }
        if (!g_data.fram.present || !s_mounted)
            {
            Serial.println("FRAM: not available");
            continue;
            }

        switch (c)
            {
            case 'l': case 'L': dumpStart();      break;
            case 'f': case 'F': printFuelSlots(); break;
            case 'X':
                s_formatArmedMs = millis();
                Serial.println("FORMAT ARMED: send 'Y' within 5 s to erase the FRAM");
                break;
            case 'Y':
                if (s_formatArmedMs && millis() - s_formatArmedMs < 5000)
                    {
                    FramHeader h;
                    uint32_t   count = 1;
                    if (framRead(FRAM_ADDR_HEADER, &h, sizeof(h))
                        && framCrc16(&h, sizeof(h) - 2) == h.crc && h.magic == FRAM_MAGIC)
                        count = h.formatCount + 1;
                    formatChip(count);
                    Serial.println("FRAM: format complete");
                    }
                else Serial.println("format not armed (send 'X' first)");
                s_formatArmedMs = 0;
                break;
            default: printHelp(); break;
            }
        }
    }
