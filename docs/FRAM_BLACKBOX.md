# FRAM: fuel persistence, black-box flight recorder, impact captures

The gauge carries an Infineon **FM24V10** F-RAM: 128 KB, I2C, nonvolatile.
It does three jobs:

1. **Fuel record** - the calculated-fuel accumulators survive the kill
   switch. Saved every second they change, restored at boot bit-exactly.
2. **Black box** - a circular log of one snapshot per second-ish (all gauge
   fields + active warnings + g). The ECU's own log is the primary record;
   this is the backup that survives when that log is lost with the car in a
   hedge.
3. **Impact captures** - five seconds of high-g waveform around each hard
   hit, in eight slots of their own. See the section near the end.

It also keeps the IMU's level calibration (`IMU_SETUP.md`).

Code: `src/fram.h/.cpp` (chip driver, non-blocking write queue),
`src/fram_log.h/.cpp` (record formats, gating, console),
`src/fram_events.h/.cpp` (the impact captures). Constants: the FRAM section
of `src/config.h`.

## Why F-RAM (and why the design looks like this)

F-RAM writes commit **at bus speed** - no page buffers, no write-cycle
polling, no EEPROM-style 5 ms stalls - and endurance is 10^14 cycles, so
once-a-second writes for the car's whole life consume a rounding error of
its life. Two consequences shape everything here:

* **Wear leveling is unnecessary.** Slots rotate for *atomicity*, not wear.
* **The only failure that matters is power dying mid-write** (the kill
  switch). Every record carries a sequence number + CRC-16, consecutive
  writes land in different slots, and recovery is always "newest copy whose
  CRC passes." A record that was mid-write when power died loses the CRC
  check and costs exactly one second of data.

## Hardware

| FM24V10 pin | Connect to | Note |
|---|---|---|
| SDA | Teensy 18 | external 2.2 kΩ pull-up to 3.3 V required |
| SCL | Teensy 19 | external 2.2 kΩ pull-up to 3.3 V required |
| A1, A2 | GND | device address 0x50 (internal pulldowns agree) |
| WP | GND | write protect off (internal pulldown agrees) |
| VDD | 3.3 V | 100 nF ceramic decoupling at the pin |
| VSS | GND | |

Bus speed is 1 MHz ("Fast-mode Plus") - the chip's fastest mode short of
Hs-mode, which the Teensy's I2C peripheral doesn't speak. At 1 MHz a byte
costs ~9 µs; a full 44-byte record plus addressing ~0.4 ms of *bus* time,
and ~0 CPU time (see next section).

## The non-blocking write path

The stock Wire library spins the CPU until the last bit is clocked out.
Instead, `fram.cpp` uses the **teensy4_i2c** library, which runs the I2C
peripheral from interrupts - the same "hand the hardware a buffer and walk
away" pattern as the display DMA and the WS2812 LEDs. On top of it sits an
8-entry write queue:

```
framWriteAsync(addr, data, len)      framPump()  (every superloop pass)
   copies into the queue, ~1 us  ->     bus idle? start next transfer
                                        I2C interrupts clock bytes out
```

The superloop never waits on the bus. Worst case per 1 s tick is three
queued writes (log record + head pointer + fuel record) ≈ 0.8 ms of bus
time ≈ 0.08% duty. While an impact capture is being recorded it adds one
39-byte write every 50 ms (three per 20 ms tick for the first 0.15 s), and
it only queues while at least five of the eight queue entries are free, so
the once-a-second records never find the queue full. Reads *are* blocking
(boot recovery, console dump only) - simplicity wins where latency doesn't
matter.

If the chip stops answering (failed, wiring shook loose), the driver counts
consecutive errors, announces once, and stands down: the gauge runs exactly
as it did before FRAM existed - fuel seeds from the sender, no black box.

## Memory map

```
0x00000   header          32 B          magic "E30G", layout version, geometry
0x00020   fuel ring       16 x 32 B     slot = seq % 16
0x00220   log head ptr     2 x 16 B     slot = seq % 2 (ping-pong)
0x00240   IMU level cal    2 x 32 B     slot = seq % 2 (ping-pong)
0x00400   black box     2210 x 44 B     circular ring (95 KB)
0x18000   impact captures  8 x 4096 B   to the end of the chip (32 KB)
```

**The trade between the last two.** The capture slots are carved out of what
used to be ring. Each slot costs the black box 93 records: 3 minutes at the
nominal cadence, 1.5 minutes at the alarm cadence. With eight slots the ring
holds 74 minutes of nominal running, 37 minutes if a warning is up the whole
time. The slot count is `FRAM_EVENT_SLOTS` in `config.h`; four slots would
give 86 / 43 minutes, twelve 61 / 31. The captures can never grow beyond
their slots, so a rough stage cannot shorten the engine log.

Any change to a record layout must bump `FRAM_LAYOUT_VERSION` in
`fram_log.cpp`; the header check then treats the chip as foreign and
reformats instead of misreading old bytes.

**Fuel record** (32 B): seq, the two `double` accumulators verbatim
(gallons remaining + lifetime gallons burned), flags, CRC. Sixteen rotating
slots means a mid-write power cut always leaves the previous second's
record intact. Loading takes the newest slot that passes both CRC *and*
sanity checks (finite, plausible range) - a corrupt-but-lucky record must
never gauge the car off a lie.

**Log head pointer** (16 B): where the ring resumes after a reboot
(next seq + next index + session), rewritten after every log record,
alternating between two slots. This is what makes boot recovery two reads
instead of a 3000-record scan. If power dies between a record and its
pointer update, the pointer is one behind and that record gets overwritten
next boot: one second of log, the accepted loss.

**IMU level calibration** (32 B): seq, the direction of "up" measured by
the LEVEL SET gesture as three floats, CRC. Two alternating slots, like the
log pointer.

**Black-box record** (44 B, encodings chosen to out-resolve the sensors):

| field | size | encoding |
|---|---|---|
| seq | 4 | monotonic, never resets (except format) |
| session | 1 | run number, bumps each boot that logs |
| hh mm ss | 3 | RTC wall clock, 0xFF = clock not set - **the key that lines this log up with the ECU's own log** |
| ecuLastError | 2 | rusEFI's most recent warning/error code (what the ECU itself was complaining about) |
| rpm | 2 | as broadcast |
| oil psi / oil C / clt C / map kPa / tps % | 5 | 1:1 (temps offset +40) |
| lambda / lambda target | 2 | x100 (0 = no reading) |
| batt V | 1 | x10 |
| inj duty % / fuel sender % / fuel calc % | 3 | 1:1 |
| pack A / min cell V / max cell C | 3 | 1 A signed; (V-1.0)x100; +40 |
| PDM-E / PDM-L board C | 2 | +40; **0 = that box was not reporting** |
| PDM-E / PDM-L total A | 2 | 1 A |
| pdmEvent | 1 | last PDM output event since the previous record, 0 = none: bit 6 box (0 PDM-E, 1 PDM-L), bits 5-2 output (0 = output 1), bits 1-0 verdict (1 TRP, 2 OPN, 3 FLT) |
| pdmEventAmps | 1 | 1 A. For a trip: the highest current seen while the output was on. Otherwise the current when the verdict changed |
| roofBits | 1 | roof switches, bit 0 = SW1 on … bit 5 = SW6 on |
| alarmBits | 5 | bit n = `AlarmId` n active (byte n/8, bit n%8) |
| gLon50 | 1 | **average** longitudinal g since the previous record × 50 (0.02 g steps, ±2.5 g), + = speeding up. −128 = no IMU data |
| gLat50 | 1 | average lateral g × 50, + = turning left. −128 = no IMU data |
| gPeak | 1 | hardest hit since the previous record (total acceleration), square-root companded: g = 320 × (byte / 255)². Steps of 0.14 g at 1 g, 0.4 g at 10 g, 1.4 g at 100 g |
| impactId | 1 | low byte of the latest impact capture's number, 0 = none yet |
| crc | 2 | CRC-16/CCITT |

Why averages for lon and lat: next to the oil pressure byte they are the
oil-surge detector. Pressure that sags only while `g_lat` sits at 1 g for
seconds is the pickup uncovering, not the pump or the bearings.

Layout version 2 (Sept 2026): `DIALFAIL` joined the alarm catalog between
`NO CAN` and `BATTCOLD`, renumbering every `alarmBits` bit below it.

Layout version 3 (Nov 2026): the ECU moved from the Megasquirt broadcast to
rusEFI's own. rusEFI broadcasts no uptime counter, so bytes 8-9 now carry
its last error code, and the mixture bytes carry lambda x 100 instead of
AFR x 10.

Layout version 4 (Nov 2026): the alarm catalogue grew from 26 rows to 36
and was re-ordered (the PDM rows), which a 32-bit mask cannot hold. The
record grew from 32 to 40 bytes: a 40-bit alarm mask plus the PDM and roof
switch fields above. The ring holds 3251 records instead of 4064.

Layout version 5 (Nov 2026): the IMU. The record grew from 40 to 44 bytes
(the four g fields above), the top 32 KB of the chip became the impact
capture slots, and the level calibration got two slots in the bookkeeping
area. The ring holds 2210 records instead of 3251.

A chip formatted by older firmware reformats on first boot (fuel record +
log lost - nothing else changed). **Before flashing v5 onto a gauge that has
been in the car, note the fuel on board:** the calculated-fuel record is
erased with the format and re-seeds from the sender.

## When the black box records

A second is logged when it is *worth remembering*: **engine turning**, or a
**yellow/red alarm active** (the CAN-link alarms - NO CAN, BATT CAN,
PDME/PDML CAN - don't count: a gauge left powered with the car off would
log them until the ring's real content was gone). After the trigger clears, logging continues 30 s (`LOG_GRACE_MS`)
- the aftermath of a stall is exactly what a black box is for - then halts.

Cadence: every 2 s nominal, every 1 s while an alarm is up
(`LOG_NOMINAL_EVERY_S` / `LOG_ALARM_EVERY_S`). The 2210-record ring
therefore holds **~74 min of nominal running**, ~37 min of continuous-alarm
running, oldest overwritten first. A realistic engine failure - an hour of
nominal running, then a minute or two of alarms - fits with room to spare.

The g fields cover the time since the previous record: two seconds
normally, one while an alarm is up. Impacts do not change the cadence or
start the recorder on their own; they have their own store.

A PDM output event (a trip, an open load, a fault) rides in the next record
written. Each raises a yellow or red alarm, so the 1 s cadence is already
running when it lands. If several happen inside one second the worst one is
kept: this is a backup log, not a sequence-of-events recorder.

## Reading it: the serial console

Open the monitor (`pio device monitor`, 115200) and press a key:

* `l` - dump the log as **CSV**, oldest record first (decoded units +
  alarm names). Runs chunked through the superloop so the gauge keeps
  working during the few-second dump; new records pause, fuel saves continue.
  Any key aborts. Copy the output into a `.csv` file and open it in any
  spreadsheet; the `session` column splits runs, the `time` column (RTC)
  aligns rows with the ECU's own log, and `ecu_err` shows any warning code
  rusEFI was raising. `pdm_event` reads like `PDME:PUMP:TRP`. The PDM alarm
  rows have no fixed text, so the `alarms` column prints them as whatever
  they say at dump time, or a placeholder such as `(ENGINE)`; `pdm_event`
  is the record of which load it was. `g_lon`, `g_lat` and `g_peak` are in
  g (empty when the IMU sent nothing), and `impact` is the latest capture's
  number (low byte).
* `f` - show all 16 fuel slots (the persistence machinery, visible).
* `e` - list the impact captures; `E` - dump their waveforms as CSV.
* `C` then `Y` (within 5 s) - clear the impact captures only.
* `X` then `Y` (within 5 s) - **format**: erases the fuel record, the log,
  the IMU level calibration and the impact captures. For bench use; a
  layout-version bump formats automatically.
* anything else - help.

The 1 Hz `[FRAM]` debug line shows health at a glance:
`REC`/`idle` (gating verdict), `log 123/2210` (records in ring), `s2`
(session), `w=`/`err=`/`drop=` (write/error/queue-drop counters - the
latter two should sit at 0 forever).

## Impact captures

When the IMU's high-g channel stays at or above the trigger level
(`IMU_EVENT_TRIGGER_G`, 6 g to start with) for two samples in a row,
`motion.cpp` declares an impact and `fram_events.cpp` stores five seconds
around it: one second from before the trigger, four after. What triggers,
how to tune it and how to read captures out are in `IMU_SETUP.md`; this
section is the storage.

**One slot is one capture**, 4096 bytes:

```
+0      header, copy A    64 B
+64     header, copy B    64 B
+128    100 blocks x 39 B   the waveform, oldest first
+4028   unused              68 B
```

**A block** is 50 ms: 12 frames of (lon, lat, vert), one byte each, then
three attitude bytes for the end of the block (roll / 2 °, pitch °, yaw
rate / 4 °/s). A frame is four high-g samples (4.2 ms); per axis it keeps
the sample of largest size, so the decimation loses no peak. The bytes are
square-root companded: g = fullScale × (byte / 127)², signed, with the full
scale in the header. A decoded sample is within 0.35 g at 6 g and 1.4 g at
100 g. That is a waveform to look at; the exact peaks are in the header as
16-bit values.

**The header** (64 B, CRC'd): capture number, state (recording / complete),
blocks on the chip, black-box sequence number, session and RTC time at the
trigger, the geometry needed to decode the blocks (sample rate, decimation,
frames per block, full scale, where in the capture the trigger fell), the
trigger level, rpm and throttle, and the results: peak total acceleration
and where it occurred, per-axis peaks, time above the trigger level,
delta-v per axis, and the air time if the hit was a landing.

**Written as it happens.** The capture that matters most is the one that
ends with the power gone, so nothing waits for the end of the window:

- At the trigger both header copies are written, then the lead-in blocks
  already in RAM, three per tick.
- Each further block is written as it completes, 50 ms after the previous.
- Every 0.5 s one header copy is rewritten with the block count and the
  peaks so far. The two copies alternate, so the one being written when the
  power dies is never the only one.
- A last header marks the capture complete.

Reading back uses the same rule as the rest of the chip: of the two header
copies, the newest whose CRC passes wins. A capture cut short by the kill
switch lists as `INCOMPLETE` with the blocks it has, and has lost at most
the last half second.

**Which slot a new capture takes.** An empty one if there is one. Otherwise
the oldest capture, except that the `IMU_EVENT_KEEP_BIGGEST` (two) hardest
hits on the chip are never the victim. So the chip always holds the two
biggest hits it has ever seen and the six most recent others. The crash is
not pushed out by the bumps of the drive home. The keepers stay until the
captures are cleared (`C`, `Y`): do that after reading out anything worth
keeping.

Captures are numbered from 1 and the numbering continues across power
cycles. The same number goes out on CAN (`Lua: Gauge 8` in the ECU's log)
and, as its low byte, into every black-box record, so a capture can be
found in either log and the other way round.

If the FRAM is missing, impacts are still detected and reported on CAN; they
are just not kept.

## Bench checklist

1. Flash, open the monitor. Boot should print `FRAM: FM24V10 online`, then
   `FRAM: formatted` (first boot) or `FRAM: mounted - log resumes at ...`.
2. `f` should show empty slots; once CAN data arrives (or the sender seeds)
   watch `fuelSaves` climb, then `f` again - slots filling in rotation.
3. Kill power mid-run, reboot: boot prints
   `Fuel calc restored from FRAM: X gal` and the log resumes at the next
   sequence number. Do it ten times; the fuel number never regresses more
   than one second's burn.
4. Fake an alarm (unplug the sender -> FUEL LOW after its persistence
   delay) and watch the `[FRAM]` line flip to `REC` at 1 s cadence.
5. `l`, paste the CSV somewhere, admire.
