# ECU CAN Setup (rusEFI, Super uaEFI)

The gauge reads two broadcasts that are built into rusEFI. Nothing to
script, nothing to flash on the ECU, just settings in TunerStudio:

1. **rusEFI's native broadcast**: twelve fixed frames, defined by rusEFI's
   own DBC file. Almost everything the gauge shows and judges comes from
   here.
2. **"MS Simplified Broadcast"**: rusEFI's MegaSquirt-dash emulation. The
   gauge takes exactly **one byte** from it, the target AFR, because the
   native broadcast doesn't carry a target.

This file is the contract between the ECU configuration and the firmware:
**if you change anything in TunerStudio, update this file and
`src/config.h` / `src/can_bus.cpp` to match.**

Everything below was checked against rusEFI's source (master, Nov 2026), the
most reliable documentation there is:

| What | Where in the rusEFI repo |
|---|---|
| Frame contents | `firmware/controllers/can/can_verbose.cpp` |
| Bit layouts, scales, units | `firmware/controllers/can/rusEFI_CAN_verbose.dbc` |
| The MS Simplified frames | `firmware/controllers/can/can_dash_ms.cpp` |
| TunerStudio field labels | `firmware/tunerstudio/tunerstudio.template.ini` (dialog `canBus`) |
| Super uaEFI defaults | `firmware/config/boards/hellen/super-uaefi/board_configuration.cpp` |

If a rusEFI update ever changes the broadcast, the DBC is the file to diff.

## The bus

| Setting | Value |
|---|---|
| Bitrate | **500 kbps** (`CAN_BITRATE` in `config.h`) |
| Identifiers | standard 11-bit |
| Byte order | little-endian (both rusEFI broadcasts *and* the BMS) |

The Super uaEFI has **one** CAN port, so the gauge, the JK BMS and the ECU's
**on-board wideband controller** all share this bus. The wideband talks on
0x190-0x19F plus some 29-bit frames; the gauge expects that traffic and
ignores it.

## TunerStudio settings

All in the **CAN Bus Communication** dialog:

| Field | Set to | Why |
|---|---|---|
| CAN dash type | **MS Simplified Broadcast** | the target AFR (frame 0x5EA) |
| Enable rusEFI CAN broadcast | **yes** (the Super uaEFI default) | everything else |
| rusEFI CAN data bus | the gauge's bus (the Super uaEFI has only one) | |
| rusEFI CAN data base address | **1664** (= 0x680) | see below: the default collides with the BMS |
| rusEFI CAN data address type | **11 bit** | |
| rusEFI CAN data period | **50 ms** (20 Hz) | 20 ms works too, at about twice the bus load |

**Base address: why 1664.** rusEFI's default base is 0x200, but the JK BMS
in Deye mode lists a frame of its own at 0x200 (`BMS_CAN_SETUP.md`). Two
nodes sending the same id corrupt each other's frames, so the ECU moves.
0x680-0x68B is clear of everything else on the bus. The gauge listens there
(`RUSEFI_CAN_BASE_ID`).

**CAN write enabled** must be on. rusEFI transmits nothing without it. Some
TunerStudio projects hide the field. If the gauge's debug line shows ECU
frames arriving, it's on.

## Frames the gauge decodes

Ids assume base 0x680. "BASEn" names match rusEFI's DBC.

| CAN id | rusEFI name | Fields used (scale) | What for |
|---|---|---|---|
| 0x680 | BASE0 | warning count, last error code, status bits: rev-limit/fuel cut, CEL, lambda protection, ... | mixture gating (rev-limit), debug, black box |
| 0x681 | BASE1 | RPM (1), timing (0.02°, signed), injector duty (0.5 %) | tach, engine running, **the "ECU online" heartbeat** |
| 0x682 | BASE2 | pedal, TPS1 (0.01 %, signed; **−1 % = no sensor**) | mixture gating (is the driver asking for power?) |
| 0x683 | BASE3 | MAP (1/30 kPa), coolant, intake (°C + 40) | CLT gauge/alarms, AFR LEAN red load rule |
| 0x684 | BASE4 | oil pressure (1/30 kPa), oil temp (°C + 40), battery (1 mV) | oil gauges/alarms, VOLT alarms |
| 0x685 | BASE5 | injector pulse width (1/300 ms, **includes dead time**) | fuel calculation, mixture gating |
| 0x686 | BASE6 | fuel trim bank 1 (0.01 %), rusEFI fuel flow (0.005 g/s) | debug; flow cross-checks the gauge's fuel calc |
| 0x687 | BASE7 | lambda 1 (0.0001) | lambda display, AFR alarms |
| 0x5EA | MS Simplified frame 3 | byte 0 only: target AFR × 10 (= target λ × 14.7) | AFR alarms |

Expected but not decoded (counted as `ign=` in the debug line): 0x688-0x68B
(cams, EGTs, knock, brake), the other four MS frames 0x5E8/0x5E9/0x5EB/0x5EC,
the wideband's 0x190-0x19F, every 29-bit frame, and the BMS frames listed in
`BMS_CAN_SETUP.md`.

A quirk worth knowing: rusEFI's MS Simplified frames send their 16-bit
fields **little-endian**, unlike a real MegaSquirt (big-endian). A one-byte
field has no byte order, which is one reason the gauge takes only the AFR
target from them.

## Sensor setup in rusEFI

- **Oil pressure and oil temperature** must be configured as rusEFI's own
  *Oil Pressure* and *Oil Temperature* sensors; the native broadcast sends
  exactly those. Calibrate them normally in rusEFI's units (kPa, °C). The
  gauge converts oil pressure to psi itself.
- **How rusEFI broadcasts a missing/failed sensor** (it has no separate
  "invalid" flag):
  - Coolant, oil and intake temperature read **0 °C**.
  - Pedal and TPS read **−1 %**. No pedal sensor means a cable throttle, and
    the gauge then uses TPS.
  - Lambda reads **0** while the wideband warms up. The display shows `--`
    and the AFR alarms stay quiet.
  - Oil pressure: rusEFI sends −1 kPa into an unsigned field, which arrives
    as 0 or as 65506 depending on its compiler. The gauge decodes both as
    **0 psi**, so a dead oil sensor raises OIL LOW with the engine running.
    It fails loudly on purpose.
- **Injector dead time:** the broadcast pulse width includes rusEFI's own
  dead-time correction, and the gauge's fuel calc subtracts it back out
  using `INJ_DEAD_TIME` in `config.h`. Keep that table the same as the
  dead-time (battery lag) table in the tune.
- **DFCO threshold:** rusEFI only cuts fuel on overrun while the pedal (TPS
  on a cable throttle) is below the tune's coasting fuel-cut TPS threshold.
  Keep the gauge's `LAMBDA_MIN_THROTTLE_PCT` above that number, so every
  fuel-cut lift is kept out of the AFR alarms (`src/mixture.cpp`).

## Bus load

| Source | Frames/s | Share of 500 kbps |
|---|---|---|
| MS Simplified (5 frames every 10 ms, fixed) | 500 | ~13 % |
| rusEFI native at 50 ms (12 frames) | 240 | ~6 % |
| wideband, BMS, gauge heartbeat | a few hundred | a few % |

That's comfortably under the ~50 % where CAN gets crowded. The MS broadcast
is the expensive part, and it exists here for one byte. If bus load ever
matters, a few lines of rusEFI Lua (`txCan`) could send the target alone,
and the gauge would need only the id change in `config.h`.

## Verifying the link

Flash the gauge, open the serial monitor (115200 baud), and watch the 1 Hz
snapshot:

```
[ECU ] OK      rpm=843 map=34.2kPa clt=61C iat=24C batt=14.21V lam=1.004(tgt 1.000) trim=+0.6%
       oilP=38.4psi oilT=58C tps=1.8% pedal=0.0% pw=2.41ms duty=1.7% adv=12.0deg st=0x06 warn=0 err=0
       mix gated err2s=+0.000 frames=21233 unk=0 ign=53188
```

- `OK`: every frame the gauge decodes is arriving.
- `PARTIAL (no X)`: frame X is missing, and **NO CAN** is raised for it.
  `MS-AFRTGT` means CAN dash type isn't *MS Simplified Broadcast*; a `BASEn`
  name means the native broadcast is off, or on another base/bus.
- `OFFLINE`: not even the rpm frame arrives. Check bitrate, wiring,
  termination, *CAN write enabled*, and the base address (1664).
- `unk=` counts frames nothing expects; it should stay at **0**. If it
  climbs, the ECU is probably still on base 0x200, or something new joined
  the bus. `ign=` climbs steadily, and that's normal.
- `mix judging` means the AFR alarms have a full window of valid samples.
  `gated` means samples are being excluded (closed throttle, fuel cut, rev
  limiter, sensor warming, holdoff). Expect `gated` at idle; mixture is only
  judged on throttle.

Bench checks once it's talking:

1. Press the pedal (or open the throttle): `pedal=`/`tps=` track it. A cable
   throttle shows `pedal=-1.0%`.
2. Lambda reads 0 for the first seconds after start, then tracks the
   wideband. The target matches TunerStudio's target-lambda gauge.
3. `pw=` is non-zero at idle (the fuel calc and the mixture gate both rely
   on it).
4. Engine off, unplug the oil pressure sensor: `oilP=0.0psi`.
5. On the rev limiter `st=` shows ` REVLIM`.

## If you change the broadcast

All knowledge of the ECU protocol lives in two places: the ids in
`src/config.h` (`RUSEFI_CAN_BASE_ID`, `MS_DASH_*`) and the decoders in
`src/can_bus.cpp`. Re-pointing the gauge (a new base address, a Lua frame
instead of the MS broadcast) means editing only those; `g_data` and
everything downstream stay untouched.
