# FRAM: fuel persistence + black-box flight recorder

The gauge carries an Infineon **FM24V10** F-RAM: 128 KB, I2C, nonvolatile.
It does two jobs:

1. **Fuel record** - the calculated-fuel accumulators survive the kill
   switch. Saved every second they change, restored at boot bit-exactly.
2. **Black box** - a circular log of one snapshot per second-ish (all gauge
   fields + active warnings). The ECU's own log is the primary record; this
   is the backup that survives when that log is lost with the car in a
   hedge.

Code: `src/fram.h/.cpp` (chip driver, non-blocking write queue),
`src/fram_log.h/.cpp` (record formats, gating, console). Constants:
the FRAM section of `src/config.h`.

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
costs ~9 µs; a full 32-byte record plus addressing ~0.3 ms of *bus* time,
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
time ≈ 0.08% duty. Reads *are* blocking (boot recovery, console dump only)
- simplicity wins where latency doesn't matter.

If the chip stops answering (failed, wiring shook loose), the driver counts
consecutive errors, announces once, and stands down: the gauge runs exactly
as it did before FRAM existed - fuel seeds from the sender, no black box.

## Memory map

```
0x00000   header        32 B          magic "E30G", layout version, geometry
0x00020   fuel ring     16 x 32 B     slot = seq % 16
0x00220   log head ptr   2 x 16 B     slot = seq % 2 (ping-pong)
0x00400   black box   4064 x 32 B     circular ring to the end of the chip
```

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
instead of a 4000-record scan. If power dies between a record and its
pointer update, the pointer is one behind and that record gets overwritten
next boot: one second of log, the accepted loss.

**Black-box record** (32 B, encodings chosen to out-resolve the sensors):

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
| alarmBits | 4 | bit n = `AlarmId` n active |
| crc | 2 | CRC-16/CCITT |

Layout version 2 (Sept 2026): `DIALFAIL` joined the alarm catalog between
`NO CAN` and `BATTCOLD`, renumbering every `alarmBits` bit below it.

Layout version 3 (Nov 2026): the ECU moved from the Megasquirt broadcast to
rusEFI's own. rusEFI broadcasts no uptime counter, so bytes 8-9 now carry
its last error code, and the mixture bytes carry lambda x 100 instead of
AFR x 10.

A chip formatted by older firmware reformats on first boot (fuel record +
log lost - nothing else changed).

## When the black box records

A second is logged when it is *worth remembering*: **engine turning**, or a
**yellow/red alarm active** (the CAN-link alarms don't count - a gauge left
powered with the car off would log "NO CAN" until the ring's real content
was gone). After the trigger clears, logging continues 30 s (`LOG_GRACE_MS`)
- the aftermath of a stall is exactly what a black box is for - then halts.

Cadence: every 2 s nominal, every 1 s while an alarm is up
(`LOG_NOMINAL_EVERY_S` / `LOG_ALARM_EVERY_S`). The 4064-record ring
therefore holds **~2.2 h of nominal running**, ~68 min of continuous-alarm
running, oldest overwritten first.

## Reading it: the serial console

Open the monitor (`pio device monitor`, 115200) and press a key:

* `l` - dump the log as **CSV**, oldest record first (decoded units +
  alarm names). Runs chunked through the superloop so the gauge keeps
  working during the few-second dump; new records pause, fuel saves continue.
  Any key aborts. Copy the output into a `.csv` file and open it in any
  spreadsheet; the `session` column splits runs, the `time` column (RTC)
  aligns rows with the ECU's own log, and `ecu_err` shows any warning code
  rusEFI was raising.
* `f` - show all 16 fuel slots (the persistence machinery, visible).
* `X` then `Y` (within 5 s) - **format**: erases the fuel record and
  restarts the log numbering. For bench use; a layout-version bump formats
  automatically.
* anything else - help.

The 1 Hz `[FRAM]` debug line shows health at a glance:
`REC`/`idle` (gating verdict), `log 123/4064` (records in ring), `s2`
(session), `w=`/`err=`/`drop=` (write/error/queue-drop counters - the
latter two should sit at 0 forever).

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
