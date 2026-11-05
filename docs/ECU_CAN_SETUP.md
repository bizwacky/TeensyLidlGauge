# ECU CAN Broadcast Setup (TunerStudio)

The gauge firmware decodes the **Megasquirt "Advanced Real-Time Data
Broadcast"** protocol (see `Megasquirt_CAN_Broadcast.pdf`, section 2). This
file is the contract between the ECU configuration and the firmware: **if you
change anything in TunerStudio, update this file and `src/config.h` /
`src/can_bus.cpp` to match.**

## Bus settings (every node must agree)

| Setting | Value |
|---|---|
| Bitrate | **500 kbps** (`CAN_BITRATE` in `config.h`) |
| Identifiers | standard 11-bit |
| Byte order | big-endian (per the MS protocol; handled in `can_bus.cpp`) |
| Base identifier | **1520 decimal / 0x5F0** (`MS_CAN_BASE_ID` in `config.h`) |

The JK BMS shares this bus, so it must also be configured for 500 kbps.
(BMS decoding is a future firmware phase.)

## TunerStudio steps

Under **CAN-Bus/Testmodes**:

1. **CAN Parameters** → set *Master enable* to **On**.
2. **CAN Realtime Data Broadcasting** → set *Enable realtime data
   broadcasting over CAN* to **On**.
3. Leave the base identifier at the default **1520**.
4. Enable **exactly the groups below** at the listed rates. Each enabled
   group is transmitted as its own 8-byte frame at CAN id `1520 + group`.

Extra groups beyond these are harmless (the firmware timestamps and ignores
them) but waste bus bandwidth.

## Required groups

| Group | CAN id | Rate | Fields the gauge uses | What for |
|---|---|---|---|---|
| 0 | 1520 (0x5F0) | **50 Hz** | `pw1`, `pw2`, `rpm` (+`seconds`) | tach, LED tach, injector duty, fuel-burn integration; group 0 is also the "ECU online" heartbeat |
| 1 | 1521 (0x5F1) | **10 Hz** | `engine` bitfield, `afrtgt1`, `adv_deg` | CRANKING detection, lambda-vs-target alarms |
| 2 | 1522 (0x5F2) | **50 Hz** | `map`, `mat`, `clt` (+`baro`) | water temp gauge/alarms; MAP > 90 kPa is part of the AFR-critical rule |
| 3 | 1523 (0x5F3) | **50 Hz** | `tps`, `batt`, `afr1_old` | battery voltage alarms, AFR display |
| 4 | 1524 (0x5F4) | **10 Hz** | `egocor1` | EGO correction display |
| 13 | 1533 (0x5FD) | **50 Hz** | `sensors1`–`sensors4` | **oil pressure & oil temp** (below); 50 Hz because low oil pressure is the most time-critical alarm in the car |

Total bus load: ~4×50 Hz + 2×10 Hz = 220 frames/s ≈ **6% of a 500 kbps bus**.
Plenty of headroom for the BMS.

**Never enable group 16 (0x600):** that id collides with the BMS's SN-code
frame (see `BMS_CAN_SETUP.md`, "id collision warning").

## Generic sensor mapping (group 13) — IMPORTANT

The MS protocol has no native oil-pressure or oil-temp channels; they ride in
the generic sensor slots. The broadcast sends each as a signed 16-bit value
that the gauge divides by 10, so **calibrate each channel in TunerStudio to
output the engineering unit directly**:

| Slot | Must be calibrated as | Gauge decodes it as |
|---|---|---|
| `sensors1` | **oil pressure in psi** | psi (value/10, e.g. raw 543 → 54.3 psi) |
| `sensors2` | **oil temperature in °C** | °C |
| `sensors3` | spare (decoded + shown in debug print) | value/10 |
| `sensors4` | spare (decoded + shown in debug print) | value/10 |

Warning thresholds in the project plan are psi for oil pressure and °C for
temperatures — getting these units right in the ECU calibration is what makes
the alarm thresholds mean what they say.

Note: `clt` and `mat` arrive in °F per the MS protocol; the firmware converts
to °C at decode time. Don't "help" by calibrating them in °C.

## Values the gauge computes itself (nothing to configure)

- **Injector duty cycle** = `pw1 × rpm / 1200` (sequential, one squirt per
  cycle). No duty channel needs broadcasting.
- **Lambda vs target** = from `afr1_old` and `afrtgt1`.
- **Fuel burned** (future) = integrated from `pw1` + `rpm`.

## If the ECU can't speak this exact dialect

All knowledge of the protocol lives in two places: the id/base constants in
`src/config.h` and the decode table in `src/can_bus.cpp`
(`decodeEcuGroup()`). Re-pointing the gauge at a different broadcast format
(e.g. rusEFI's native broadcast) means editing only those — `g_data` and
everything downstream stay untouched.

Related: the MS "Simplified Dash Broadcasting" mode (base id 1512, one
switch, fixed 50 Hz set) carries almost everything we need but **not** the
`engine` bitfield, so no CRANKING indication — that's why we use the Advanced
broadcast instead.

## Verifying the link

Flash the gauge, open the serial monitor (115200 baud), and watch the 1 Hz
snapshot:

```
[ECU ] ONLINE  rpm=843 map=32.1kPa clt=61.2C ... frames=5023 other=0
```

- `ONLINE` + climbing `frames=` → broadcast is arriving and decoding.
- `OFFLINE` with `frames=0` → check bitrate, transceiver, termination, and
  that Master enable + broadcasting are both On.
- `unk=` counts frames nobody decodes (neither this ECU table nor the BMS
  decoder). It should sit near 0; climbing steadily means an id/config
  mismatch somewhere on the bus.
