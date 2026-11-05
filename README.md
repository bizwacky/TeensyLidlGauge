# Rally CAN Gauge firmware

Firmware for the E30 rally car gauge cluster: Teensy 4.0 driving three
ER-TFT2.79-1 displays (NV3007 controller) over SPI, fed by CAN data from the
rusEFI ECU (a Super uaEFI) and the JK BMS. The full hardware/software plan
(`Rally CAN Gauge Project Plan.md`) lives in the local project folder
alongside the firmware and is not part of this repository.

## Opening the project

This is a standard PlatformIO project: in VS Code with the PlatformIO
extension, use "Open Folder" on the root of this repository. Or from a
terminal:

```
pio run                        # build the main gauge firmware
pio run -t upload              # flash the Teensy
pio device monitor             # serial output (115200)
pio run -e display_test        # build the display bring-up test instead
pio run -e display_test -t upload
```

## Architecture

```
CAN interrupt -> ring buffer -> canBusPoll() ---+
ADC pins  ------> inputsPoll()  (20 ms tick) ---+--> g_data (raw + derived:
RTC + die temp -/                                |    mixture window, fuel calc)
                                                 alarmsUpdate()
                                                       v
                                                g_data.alarms ---> audio (1 Hz),
                                                (verdicts)     LED tach + screens
                                                               (50 Hz)
```

Two layers by design: input modules establish *what is* (temperatures,
voltages, link freshness) in the global `VehicleData g_data` struct; the
alarm state machine judges *what matters* and publishes verdicts to
`g_data.alarms`. Consumers (audio, screens, LED tach) read only verdicts -
no display or audio code ever touches a threshold. The screens repeat the
same split one level down: `gauge_render.cpp` decides what a frame *shows*
(pure g_data -> pixels), `screens.cpp` decides how it *reaches the glass*
(framebuffers, diffs, DMA). The superloop in `src/main.cpp` is a cooperative
scheduler: CAN drains every pass, inputs + mixture window + fuel calc +
alarms + LED strip + screens run on a 20 ms tick (which also feeds the
hardware watchdog), housekeeping runs at 1 Hz. A hung firmware stops feeding
the watchdog and the chip hard-resets within ~1 s.

## Layout

- `src/main.cpp` - the superloop (scheduler + 1 Hz debug print).
- `src/config.h` - every pin, constant, threshold and tunable in one place.
- `src/vehicle_data.h/.cpp` - the central data model (`g_data`), including
  the alarm catalog enum (ordered by the plan's priorities).
- `src/alarms.h/.cpp` - the warning/alarm state machine: one table row per
  alarm (name, color, priority, persistence, audio clip, condition).
- `src/audio.h/.cpp` - voice-callout arbitration (priority, repeat intervals,
  no talking over itself) plus the DFPlayer Mini driver (Makuna/DFMiniMp3 on
  Serial7, pins 28/29, watchdog-safe NoAck mode - design notes at the top
  of the .cpp).
  SD card layout is documented on the clip enum in `audio.h`; the rotary
  dial's silent position (`MODE_SILENT`) mutes all callouts.
- `src/backlight.h/.cpp` - display backlight dimming: 20 kHz PWM into three
  low-side N-FET switches (pins 2/3/22 - all on FlexPWM4 so the three
  windows can be locked a third of a period apart: the 5V rail sees one
  100 mA step at a time instead of 300 mA), brightness from the rotary mode
  with per-screen binning trim (`BL_TRIM`) and a 250 ms fade between
  levels. Hardware, frequency and phase rationale in `config.h`; the
  FlexPWM register mechanics and the bench check at the top of the .cpp.
- `src/watchdog.h/.cpp` - minimal register driver for the i.MX RT's WDOG1.
- `src/can_bus.h/.cpp` - CAN3 + the decoders: rusEFI's native broadcast
  (frames BASE0..7, layouts from rusEFI's own DBC), the one target-AFR byte
  from rusEFI's MS Simplified broadcast, and the JK BMS Deye protocol (1 Hz)
  - all little-endian - plus the 1 Hz PCS heartbeat the BMS expects from an
  "inverter". Every frame is counted as decoded, ignored (expected traffic
  we don't use) or unknown, so a misconfigured bus shows up as `unk=`.
- `src/mixture.h/.cpp` - the gated two-second lambda window the AFR alarms
  judge: samples only count while the driver is on the throttle, rusEFI
  isn't cutting fuel, and the sensor has recovered from the last time that
  wasn't true - so overrun and fuel cut can't raise AFR LEAN.
- `src/inputs.h/.cpp` - fuel sender (ratiometric 150R divider, including
  the two-stage slosh filter: median window + slow EMA), rotary switch
  (2.2k pull-up into an 11 x 220R ladder, decoded in ohms, with open-wiper
  detection feeding the DIALFAIL alarm and a fail-safe fallback to Bright),
  12V switches, chip temp, RTC. Both divider circuits are drawn in `config.h`.
- `src/fuel_calc.h/.cpp` - calculated fuel: integrates the ECU's injector
  pulse width (dead-time corrected) into a gallons-remaining estimate, and
  runs the rotary FUEL FILL gesture (hold 30 s = +5 gal, 60 s = full tank).
  Physics and design rationale at the top of the .cpp; constants in
  `config.h`. Seeds from the FRAM fuel record at boot (sender fallback).
- `src/fram.h/.cpp` - FM24V10 F-RAM driver: non-blocking I2C write queue
  (interrupt-driven via teensy4_i2c - the superloop never waits on the
  bus), blocking reads for boot/console, shared CRC-16.
- `src/fram_log.h/.cpp` - what the FRAM stores: the persistent fuel record
  (16 rotating CRC'd slots - kill-switch-proof) and the black-box flight
  recorder (~2 h circular log of every gauge field + active alarms), plus
  the serial console (`h` in the monitor: CSV dump, fuel slots, format).
  **See `docs/FRAM_BLACKBOX.md`** for the memory map, record formats,
  wiring, and how to read the log out.
- `src/gauge_render.h/.cpp` - what the three panels show: the bench-tested
  prototype layouts (big value left, secondary stack right), the semantic
  color Theme (night mode = every blue element turns amber, resolved in one
  place), per-readout coloring from alarm verdicts, the warning banner with
  its 1 Hz critical flash, and the MODE_DEBUG dense pages. Stateless: every
  frame is composed from scratch from g_data ("compose then show", like the
  LED strip). Design story at the top of the .cpp.
- `src/screens.h/.cpp` - how frames reach the glass: three ILI9341_T4
  drivers (one per SPI bus), one shared 121 KB draw buffer + three
  per-display internal buffers in OCRAM, differential DMA updates, the
  busy-skip rule that keeps the superloop from ever blocking on a transfer,
  and displays-off handling. Memory map + pipeline at the top of the .cpp.
- `src/fonts/` - the custom Iosevka faces (tgx font exports, PROGMEM):
  Regular 30 px for labels, Heavy 30 px for the banner, Extended 55/114 px
  for values. Monospaced so a changing digit dirties only its own cell
  (small diffs). Regenerate with vindar's tgx font tool from the Iosevka
  TTFs; the PC prototype (the local `Graphics/` folder, not in this
  repository) is the layout source of truth.
- `src/examples/display_bringup.cpp` - display test pattern, built by the
  `display_test` environment (wiring documented at the top of the file).
- `docs/FRAM_BLACKBOX.md` - the FM24V10 wiring, memory map, record
  formats, black-box gating rules, and the serial console commands.
- `docs/ECU_CAN_SETUP.md` - **the rusEFI / TunerStudio settings the firmware
  expects** (broadcast base address, dash type, period), the frame map, and
  the bring-up checklist. Keep it in sync with `config.h`/`can_bus.cpp`.
- `docs/BMS_CAN_SETUP.md` - the JK BMS protocol selection (Deye/001), the
  frames decoded, BMS bring-up checks, and the table of every CAN id on the
  bus.
- `lib/ILI9341_T4/` - vindar's ILI9341_T4 DMA display library, modified for
  the NV3007 controller. **Read `lib/ILI9341_T4/PORTING_NV3007.md`** for a
  walkthrough of every change and why it was needed. It is vendored from
  upstream v1.7.1 (commit `bee2957`, https://github.com/vindar/ILI9341_T4);
  diff against that commit to see the exact modifications.

## Display driving in one paragraph

Each display gets its own SPI bus (SPI0/1/2 on the Teensy). We draw into a
428x142 RGB565 framebuffer in RAM, then `tft.update(fb)` hands it to the
library, which diffs it against what is already on screen and streams only
the changed pixels out via DMA - the CPU is free almost the entire time.
VSync is not available (the display has no MISO line to poll the scanline),
but the TE pin is broken out on the module and reserved in the pinout for a
possible future tear-free sync mod.

## Memory budget (as built)

Each display keeps a 121 KB internal framebuffer inside the 512 KB
OCRAM/DMAMEM region (3 x 121 = 363 KB), plus ONE shared 121 KB draw
framebuffer = 486 KB of the 512 KB used; the ~25 KB remainder is heap
headroom (nothing in this firmware mallocs). The 20 ms tick draws each
screen into the shared buffer sequentially: draw 1 -> update(1) -> draw 2 ->
update(2) -> draw 3 -> update(3). The DMA transfers overlap the following
draws because update() copies the changed pixels into the per-display
internal buffer before returning. The six 8 KB diff buffers and all other
variables live in the separate 512 KB DTCM bank. Fonts (~145 KB) stay in
flash. Watch the 1 Hz `[SCRN]` debug line: worst-case draw time and the
skipped-frame counter are the two health numbers for this pipeline.
