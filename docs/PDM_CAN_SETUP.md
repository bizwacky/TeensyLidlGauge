# PDM CAN Setup (dingoFW, PDM-E and PDM-L)

The gauge and the two PDMs talk in both directions:

1. **PDM → gauge:** each box's cyclic status frames. The gauge decodes
   device state, board temperature and, per output, state, current and trip
   count. Those feed the PDM alarm rows, the lamp icons and night mode.
2. **Gauge → PDMs:** two frames carrying the roof panel's six switches and
   five pots. The roof panel is wired to the gauge, so these frames **are
   the car's light switches**.

This file is the contract between `dingoConfig` and the firmware: **change a
Base ID, a CAN input or a frame layout on a PDM, and update this file and
`src/config.h` to match.**

Status as of Nov 2026: the PDM boards are not built and nothing here has
been run against hardware. The dingoFW facts were read from its source
(commit `d594cd7`, `boards/dingopdm_v7/msg.cpp`, `functions/can_input.cpp`,
`functions/profet.cpp`) and its CAN documentation.

## Ids

Everything is 500 kbps, 11-bit ids, little-endian.

| Ids | Sender | `config.h` |
|---|---|---|
| 0x440-0x47F | PDM-E (Base ID 0x440) | `PDM_BASE_ID[PDM_E]` |
| 0x480-0x4BF | PDM-L (Base ID 0x480) | `PDM_BASE_ID[PDM_L]` |
| 0x4C0 | gauge: switches, flags, alive counter | `GAUGE_ID_SWITCHES` |
| 0x4C1 | gauge: pot positions, fan duty | `GAUGE_ID_POTS` |
| 0x4C2 | gauge: light show lamp requests | `GAUGE_ID_SHOW` |

**These five numbers are placeholders** picked to be clear of every other id
on the bus (table in `BMS_CAN_SETUP.md`). Replace them with the CAN id
plan's numbers, in `config.h` and in each box.

- A dingoPDM uses Base+0 and Base+1 for dingoConfig and Base+2 upward for
  status. Stock firmware uses 32 ids; 0x40 are reserved per box so the
  fork's extra frames fit.
- Neither box sits on dingoFW's factory default (0x0DE). An unconfigured
  replacement box then shows up as `unk=` traffic on the gauge's debug line
  instead of impersonating a PDM.
- **The gauge maps Base ID → role.** If "the role follows the plug" is
  adopted, the Base ID stays with the box while the role moves, and the
  gauge has to read the role from the strap inputs instead
  (`pdmSlotForId()` in `can_bus.cpp` is the one place).

## What the gauge decodes

The frame table is `PDM_FRAMES` in `config.h`: one row per frame, as an
offset from the Base ID. Rows above the line are stock dingoFW; rows below
it **do not exist in any firmware yet**.

| Id | Kind | Carries | Needed |
|---|---|---|---|
| Base+2 | device | digital inputs, device state, total A, battery V ×10, board °C ×10 | always |
| Base+3 | current | outputs 1-4, one uint16 each | always |
| Base+4 | current | outputs 5-8 | always |
| Base+5 | state | outputs 1-8, one nibble each (0 Off, 1 On, 2 Overcurrent, 3 Fault) | always |
| Base+6 | trip count | outputs 1-8, one byte each | always |
| Base+25 | duty | outputs 1-8, % (only sent with PWM enabled) | never |
| **Base+32** | current | **outputs 9-12** | once seen |
| **Base+33** | current | **outputs 13-14** | once seen |
| **Base+34** | state | **outputs 9-14** | once seen |
| **Base+35** | trip count | **outputs 9-14** | once seen |
| **Base+36** | duty | **outputs 9-14** | never |

"Once seen" means the frame is optional until it first arrives and required
from then on. So the same gauge firmware runs against a stock 8-output
dingoPDM on the bench and against the 14-output fork.

**Until the fork sends the bold rows, outputs 9-14 have no status.** On the
rev A pin map that is yellow-high, light bar wide, brake lights and horn on
PDM-E, and wiper high, washer, defrost, seat fans and both coolshirt pumps
on PDM-L. They raise no alarms and the brights icon cannot see yellow-high
or the wide bar. Build the fork to this layout, or edit the rows to match
what it sends.

**Current is in whole amps.** dingoFW `d594cd7` casts a float in amps
straight to `uint16_t`, and its CAN docs say "A". The `.dbc` file in the
same repo still says 0.1 A/bit. `PDM_AMPS_PER_COUNT` follows the source; if
the fork restores 0.1 A, change that one constant. At whole-amp resolution
the open-load checks (pump, fans: "on and under 2 A") still work.

## What the gauge sends

All three frames go out every 100 ms, and on the next 20 ms tick whenever
their content changes.

### 0x4C0, switches and flags

| Byte | Content |
|---|---|
| 0 | switch **ON** bits: bit 0 = SW1 … bit 5 = SW6 (1 = on) |
| 1 | switch **OFF** bits: bit 0 = SW1 … bit 5 = SW6 (1 = off) |
| 2 | pot **active** bits: bit 0 = pot 1 … bit 4 = pot 5 (1 = turned up) |
| 3 | pot **fault** bits: bit 0 = pot 1 … bit 4 = pot 5 (1 = wire open) |
| 4 | bits 0-3 dial position, bit 4 night, bit 5 silent, bit 6 displays off, bit 7 dial fault |
| 5 | bit 0 = a red alarm is up, bit 1 = a yellow alarm is up |
| 6 | `AlarmId` of the banner alarm, 0xFF = nominal |
| 7 | alive counter, +1 per transmission |

### 0x4C1, pots and fans

| Byte | Content |
|---|---|
| 0-4 | pot 1…5 position, 0-100 % |
| 5-7 | fan PWM 1…3 duty the gauge is driving, 0-100 % |

A dingoFW start bit is `byte × 8 + bit`: SW1 ON is start bit 0, SW1 OFF is
start bit 8, pot 4 is start bit 24 with length 8.

### 0x4C2, light show lamp requests

All zero except while a light show is playing
([`LIGHTSHOW.md`](LIGHTSHOW.md)). Byte 0 is PDM-L's lamps and byte 1 is
PDM-E's, so each box reads one byte.

| Byte | Bit | Start bit | Lamp | Output |
|---|---|---|---|---|
| 0 | 0 | 0 | low beam | PDM-L 1 |
| 0 | 1 | 1 | high beam | PDM-L 2 |
| 0 | 2 | 2 | light bar, spot | PDM-L 3 |
| 0 | 3 | 3 | turn left | PDM-L 4 |
| 0 | 4 | 4 | turn right | PDM-L 5 |
| 0 | 5 | 5 | tail + plate | PDM-L 6 |
| 0 | 6 | 6 | reverse | PDM-L 7 |
| 1 | 0 | 8 | yellow lamps, low | PDM-E 8 |
| 1 | 1 | 9 | yellow lamps, high | PDM-E 9 |
| 1 | 2 | 10 | light bar, wide | PDM-E 10 |
| 1 | 3 | 11 | brake lights | PDM-E 11 |
| 2 | | 16 (length 8) | show state: 0 idle, 1 countdown, 2 playing, 3 over | |

These are plain ON bits with no complement: a show lamp must fail OFF.

## Light show: letting the gauge flash a lamp

Nothing in this section is needed for the car to work. Skip it and the
show still runs on the gauge's own LEDs and screens.

For each lamp that should join the show, on the box that drives it:

1. Add a CAN input for its bit:

   | Setting | Value |
   |---|---|
   | ID | 0x4C2 |
   | Start Bit | from the table above |
   | Bit Length | 1 |
   | Operator / Operand | Equal / 1 |
   | Mode | Momentary |
   | Timeout Enabled / Timeout | yes / 500 ms |

2. Make the output's input the lamp's normal request **OR** that CAN
   input. Where the lamp already has a virtual input for its inverted OFF
   bit (the recipe below), add the show input to it as a second term with
   an OR condition.

What this gives:

| Roof switch | Show bit | Gauge alive | Lamp |
|---|---|---|---|
| off | 0 | yes | off |
| off | 1 | yes | **on** |
| on | either | yes | on |
| either | either | no | exactly as without a show: fail-on lamps on, show bit timed out to 0 |

The show can add light and cannot remove it, so no state of the show or
fault in it can darken a lamp the crew has switched on.

Things to settle on the bench, none of them checked:

- **Inrush.** The show restarts a lamp as often as once a beat (2.3 times
  a second at 136 bpm). Each restart is an inrush event; an output whose
  inrush limit or time is tight will trip, and the gauge will report `TRP`.
- **Brake and turn outputs** presumably have their own inputs already
  (pedal switch, flasher). The show bit is one more OR term on each.
- **Input budget.** Seven CAN inputs on PDM-L and four on PDM-E, on top of
  the switch inputs. Count them against what the firmware allows.
- **The virtual-input fields** (three terms, AND/OR/NOR between them, a
  NOT per term) are written here from memory of stock dingoPDM, not read
  from the fork's source. Check them in dingoConfig.

## Lights fail ON: why every switch is sent twice

With its timeout enabled, dingoFW zeroes a CAN input (value and result)
when the frame stops arriving. So `0` is what a PDM sees when the gauge is
dead or the bus is cut. Bytes 0 and 1 are always complements, which lets
each load pick its failure direction in dingoConfig:

| Driven from | Gauge alive | Gauge dead / bus cut |
|---|---|---|
| the **ON** bit | follows the switch | **OFF** |
| NOT (the **OFF** bit) | follows the switch | **ON** |

Lamps use the second form. A CAN input for one lamp switch:

| Setting | Value |
|---|---|
| ID | 0x4C0 |
| Start Bit | 8 + (switch number − 1) |
| Bit Length | 1 |
| Operator / Operand | Equal / 1 |
| Mode | Momentary |
| Timeout Enabled / Timeout | yes / 500 ms |

Then a virtual input with that CAN input **inverted** drives the lamp
output. Check on the bench that pulling the gauge's plug turns the lamps on.

A broken switch wire is a separate failure: the inputs are ground-switched,
so an open wire reads as "off". `ROOF_SWITCH_ON_WHEN_CLOSED` in `config.h`
flips a switch's sense if a function should read "on" with the wire open.

## Pots: fans and coolshirt pumps

Assumed assignment (`config.h`, change it if the panel differs):

| Pot | Function | Gauge output | PDM |
|---|---|---|---|
| 1 | defrost fan | PWM1 | PDM-L output 11 on while pot 1 active |
| 2 | driver seat fan | PWM2 | PDM-L output 12 on while pot 2 **or** 3 active |
| 3 | co-driver seat fan | PWM3 | (same output) |
| 4 | driver coolshirt | none | PDM-L output 13, PWM duty from pot 4 |
| 5 | co-driver coolshirt | none | PDM-L output 14, PWM duty from pot 5 |

- The fans take their speed from the gauge's PWM pins and their power from
  PDM-L. A PWM-input fan usually idles at 0 % rather than stopping, so the
  PDM must drop the feed when the pot's **active** bit is 0.
- For the pumps, read the pot byte as a CAN input value and use it as a
  variable duty cycle with denominator 1 (dingoFW: duty = input ÷
  denominator, 0-100).
- A pot whose wire is open reports its failsafe value (`POT_CAL`: 100 % for
  defrost, 0 % for the rest) and sets its fault bit.

## Alarms built on the status frames

Full table: `src/alarms.cpp`. Thresholds: `config.h`.

| Banner | Color | Raised when |
|---|---|---|
| `PUMP`/`COIL`/`MAIN` + `FLT`/`OPN` | red | an engine-class output is in Fault, spent 2 s of the last 5 s in Overcurrent, or (pump) is on and under 2 A for 1 s |
| `PDME ERR` | red | PDM-E reports device Error |
| `PDME HOT` | red | PDM-E board at 75 °C (latch − 5 K) |
| `PUMP`/`COIL`/`MAIN` + `TRP` | yellow | an engine-class output tripped; held 10 s |
| `FAN1`/`FAN2` + `TRP`/`FLT`/`OPN` | yellow | fan output tripped, faulted, or on and under 2 A for 2 s |
| `PDME HOT` / `PDML HOT` | yellow | either board at 65 °C (latch − 15 K) |
| lamp / wiper / defrost tag + `TRP`/`FLT` | yellow | a vision-class output tripped or faulted |
| `PDML ERR` | yellow | PDM-L reports device Error |
| `PDME CAN` / `PDML CAN` | yellow | a required status frame is 1 s stale |
| body tag + `TRP`/`FLT` | yellow | a body-class output tripped or faulted |

Things that follow from how dingoFW behaves:

- **A trip can be shorter than a status period.** The gauge watches the trip
  counter as well as the state, so set Reset Time ≥ 200 ms where possible
  and it will catch either.
- **A stale box raises only its `CAN` banner.** Every output verdict from it
  drops to OK, like the ECU alarms behind NO CAN.
- **If a box in device Error stops transmitting,** the gauge shows
  `PDME CAN` (yellow) and never sees `PDME ERR` (red). Not checked against a
  real board.
- **The 80 °C thermal latch is stock dingoFW** (`BOARD_TEMP_CRIT`). If the
  fork changes it, change `PDM_LATCH_TEMP_C`.

## Verifying the link

The 1 Hz debug print has a line per box and two for the roof panel:

```
[PDME] ONLINE  RUN 34A 13.8V 41.5C ext=no frames=5210 | 5:PUMP=On/8A 6:COIL=On/0A 7:MAIN=On/6A
[PDML] ONLINE  RUN 12A 13.8V 38.0C ext=yes frames=9120 | 1:LOBM=On/4A 6:TAIL=On/1A
[SW  ] rotary=0=BRIGHT (held 42s, 0ohm raw=3) roof=100000 lamps: head=1 brights=0 turnL=0 turnR=0
[POT ] 1=40%*(4012ohm raw=1886) 2=0%(3ohm raw=2) ... | fan duty 52/0/0%
```

- `never heard`: no frame from that Base ID. Check the id burned into the
  box against `PDM_BASE_ID`.
- `STALE`: frames stopped. dingoFW pauses its cyclic frames for up to 4 s
  while dingoConfig reads or writes all parameters.
- `ext=no`: the frames for outputs 9-14 have never arrived.
- Outputs print only when they are not simply off; a verdict (`TRP`, `OPN`,
  `FLT`) and a trip count (`x3`) follow when there is one.
- `roof=` is SW1 first. Pots print percent, ohms and raw counts; calibrate
  `POT_CAL` from the ohms at each end stop.

Bench checks once a box is talking:

1. Flip each roof switch: its bit moves in `roof=`, and the right PDM
   output follows.
2. Unplug the gauge with the lamps off: the lamps come on.
3. Short an output through a load above its limit: `TRP` appears, and the
   banner holds for 10 s after it recovers.
4. Unplug the fuel pump with the pump output on: `PUMP OPN` (red) after 1 s.
5. Turn each pot end to end: 0 % and 100 % are reached, the `*` appears just
   off the stop.
