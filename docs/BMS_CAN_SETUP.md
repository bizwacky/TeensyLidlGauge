# BMS CAN Setup (JK BMS, Deye protocol)

The gauge decodes the **"PCS CAN-Bus-protocol-DY-low-voltage"** — the protocol
the JK BMS app calls **Deye, protocol number 001**. Reference: `PCS
CAN-Bus-protocol-DY-low-voltage V3.3.zh-CN.en.pdf` in the project root.
This file is the contract between the BMS configuration and the firmware:
**change one, change the other** (`src/config.h` + `src/can_bus.cpp`).

## BMS app settings

| Setting | Value |
|---|---|
| CAN protocol | **Deye (001)** |
| Bitrate | **500 kbps** — same bus as the ECU, everyone must match |

Nothing else to configure: the protocol broadcasts a fixed frame set on a
fixed **1-second cycle** (vs. the ECU's 20 Hz — hence the separate, much
longer staleness window `BMS_OFFLINE_MS`).

## Byte order

Deye frames are **little-endian** (low byte first), the same as both rusEFI
broadcasts. `can_bus.cpp` assembles every field explicitly with its
`le16u`/`le16s` helpers. If BMS values ever look absurd (a 16 V pack reading
as ~1638 V, temperatures of thousands of degrees), byte order is the first
suspect.

## Frames the gauge decodes

| CAN id | Cycle | Fields used | What for |
|---|---|---|---|
| 0x356 | 1 s | pack voltage (0.01 V), current (0.1 A, discharge +), avg cell temp (0.1 °C) | voltage/current display; this frame is also the "BMS online" heartbeat |
| 0x355 | 1 s | SOC (1 %), SOH (1 %) | battery state of charge |
| 0x361 | 1 s | max/min cell voltage (1 mV), max/min cell temp (0.1 °C) | **all cell-level alarms**: balance = max−min > 0.1 V (warning), any cell < 2 V (critical), temp > 50/65 °C (hottest cell) |
| 0x359 | 1 s | protection bytes 0–3, alarm bytes 4–5, system-error byte 6 | raw fault bitfields (bit meanings: Tables 1–7 in the PDF), for the alarm phase |
| 0x400 | 1 s | operation mode, failure level, cycle count, balance bits | status display; `failureLevel` is the BMS's own 0/1/2 severity summary |
| 0x110 | 1 s | byte 7: MOSFET states | is the pack actually connected (charge/discharge FET on)? |

Frames **not** decoded (recognised as expected traffic and counted as `ign=`
in the ECU debug line, so they never inflate the `unk=` counter): 0x351
charge parameters, 0x35C request flags, 0x35E manufacturer name,
0x363/0x364/0x371, 0x150/0x200/0x250 per-module summaries (duplicate the
above at different resolutions), 0x500/0x550/0x600/0x650/0x700/0x750
versions/SN/statistics. The list lives in `k_bmsUnusedIds` in `can_bus.cpp`.

### Per-cell voltages

The Deye protocol reports only **max and min** cell voltage, not all cells
individually (the project plan originally hoped for per-cell values). Max/min
is sufficient for every planned alarm; if per-cell detail is ever wanted, it
would need the JK RS-485 protocol or a different CAN dialect.

## The PCS heartbeat (gauge → BMS)

The protocol expects the inverter ("PCS") to send **0x305 once per second**;
without it the BMS may raise its *PCS communication failure* bit (Table 4,
bit 6). There is no inverter in the car, so **the gauge transmits this
heartbeat** on its 1 s tick, carrying real time-of-day from the RTC when the
clock is set (all-zeros otherwise, which the spec shows as valid). Kill
switch: `BMS_SEND_PCS_HEARTBEAT` in `config.h`.

## id collision warning

The BMS's per-module summary frame **0x200** is also the **default base
address of rusEFI's native broadcast** (0x200-0x20B), which the Super uaEFI
enables out of the box. Two nodes transmitting the same id corrupt each
other's frames, so the ECU's base address is moved to **0x680 (1664)**; see
`ECU_CAN_SETUP.md`. Every id on the bus, for reference:

| Ids | Sender |
|---|---|
| 0x110, 0x150, 0x200, 0x250, 0x351, 0x355, 0x356, 0x359, 0x35C, 0x35E, 0x361, 0x363, 0x364, 0x371, 0x400, 0x500, 0x550, 0x600, 0x650, 0x700, 0x750 | JK BMS (this file) |
| 0x305 | the gauge (PCS heartbeat) |
| 0x190-0x19F (+ 29-bit frames) | the ECU's on-board wideband controller |
| 0x5E8-0x5EC | rusEFI MS Simplified Broadcast |
| 0x680-0x68B | rusEFI native broadcast (moved from 0x200) |

Before adding anything to the bus, check its ids against this table.

## Things to verify at first bring-up

1. **Byte order**: pack voltage should read ~13–17 V for the 6s LTO pack,
   cells ~2.2–2.7 V. Garbage numbers → see byte-order section.
2. **Current sign**: spec says discharge positive, charge negative (protocol
   v2.5+). Confirm with a known load.
3. **MOS state bits**: spec (translated) says 1 = "close" = conducting. The
   debug print shows `mosC`/`mosD` — confirm they read 1 when the pack is
   connected. If inverted, fix `chargeMosOn()`/`dischargeMosOn()` in
   `vehicle_data.h`.
4. **Heartbeat accepted**: after a minute on the bus, the 0x359/0x110
   protection bytes should NOT show the PCS-communication-failure bit
   (Table 4 bit 6, i.e. `prot=..40..` in the fourth byte).

## Debug print reference

```
[BMS ] ONLINE 15.84V -3.2A soc=87% cells 2.641-2.655V (d=0.014) temp 22.1..24.9C avg=23.4C
       mode=DIS fail=0 cyc=42 bal=0x0000 mosC=1 mosD=1 prot=00.00.00.00 alrm=00.00 err=00 frames=360
```

- `OFFLINE` with `frames=0` → check bitrate, wiring, and that the JK app is
  set to protocol 001/Deye.
- `prot`/`alrm`/`err` bytes are the raw 0x359 bitfields (hex); all zeros is
  a healthy pack. Look bits up in Tables 1–7 of the PDF.
- `bal` shows which cells are being balanced (bit 0 = cell 1).
