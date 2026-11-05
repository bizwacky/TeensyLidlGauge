# Porting ILI9341_T4 to the NV3007 (ER-TFT2.79-1)

This is [vindar/ILI9341_T4](https://github.com/vindar/ILI9341_T4) modified to drive
the EastRising **ER-TFT2.79-1** bar display (2.79", 142x428 pixels, **NV3007**
controller, 4-wire SPI). The folder is vendored into the gauge repository (it is
no longer a git clone of its own), starting from upstream **v1.7.1, commit
`bee2957`**. To see every modification against the original, check that commit
out of the upstream repo and diff the two trees:

```
git clone https://github.com/vindar/ILI9341_T4 /tmp/ILI9341_T4-upstream
git -C /tmp/ILI9341_T4-upstream checkout bee2957
diff -ru /tmp/ILI9341_T4-upstream/src lib/ILI9341_T4/src
```

## Why this library works for the NV3007 at all

The NV3007 speaks the same MIPI-DCS command language as the ILI9341 for
everything this library does at runtime:

| Operation           | Command | Same on both? |
|---------------------|---------|---------------|
| Set column window   | 0x2A    | yes           |
| Set row window      | 0x2B    | yes           |
| Write pixel memory  | 0x2C    | yes           |
| Memory access ctrl  | 0x36    | yes           |
| Pixel format        | 0x3A    | yes           |
| Sleep out / disp on | 0x11 / 0x29 | yes       |
| Invert on/off       | 0x21 / 0x20 | yes       |

After initialization, the library *only* uses the commands above, so the entire
high-performance machinery (differential redraws, DMA, double buffering) works
unmodified. The differences are all in setup and geometry, described below.

## The changes, one by one

### 1. No MISO line -> no VSync (`ILI9341Driver.cpp: begin()`)

The module's SDA pin is bidirectional but the Teensy's LPSPI hardware + DMA
pipeline needs a dedicated MISO line to read, so the display is write-only for
us. Upstream already handles `miso = 255` by disabling everything that reads:
VSync (which polls the scanline register 0x45), status queries, refresh-rate
measurement, and XPT2046 touch support. `begin()` now **forces** `miso = 255`
so no read path can ever run. Consequences:

- `setVSyncSpacing()` only accepts 0 (wait for previous frame) or -1 (drop
  frames); requesting vsync reverts to 0 with a warning.
- `begin()` cannot detect a dead/miswired display: it always returns true.
- Screen tearing is possible in principle. In practice, differential updates
  redraw so few pixels per frame that visible tearing is rare.
- The module DOES have a **TE (tearing effect) output pin** (enabled in our
  init sequence: commands 0x35/0x44). The library cannot use it today, but a
  future modification could watch that pin with a GPIO interrupt to time
  uploads and recover tear-free updates. This matches the "SPI1 TE"/"SPI2 TE"
  pins reserved in the project pinout.

### 2. Native orientation is landscape 428x142 (`DiffBuff.h`, `ILI9341Driver.h`)

The glass itself is a **portrait** bar: 142 columns wide x 428 rows tall
(active area 21.3 x 66.1 mm - the source driver spans the short axis, the
gate driver the long axis). The controller's power-on addressing is therefore
142x428 portrait, and the **only** way to address it as a landscape display
is the MADCTL row/column-exchange bit (MV) - this is not a workaround, it is
the controller's built-in landscape mode, the same one the vendor demo uses
for its "horizontal screen" settings (DERECTION 2/3 in `lcd.h`).

Since the gauges are mounted landscape in the car, MV is set during init and
the driver's native orientation 0 is landscape 428x142: the framebuffer you
draw into matches the display as mounted, with no per-frame cost anywhere.

Happily this is also what the diff engine needs: its 32-bit fast paths
require the native width to be divisible by 4 (`static_assert` in
`DiffBuffTemplate.h`) - 142 fails, 428 passes. Running the panel in portrait
natively would mean patching the diff engine, and drawing landscape content
on it would then pay `setRotation(1)`'s slower strided-diff path every frame,
for an identical picture on the glass. All cost, no benefit - so: MV stays.

`setRotation(0..3)` is implemented entirely in software (the diff/upload
always runs in native order), so all four orientations remain available:

| rotation | size    | use for                                  |
|----------|---------|------------------------------------------|
| 0        | 428x142 | landscape, as mounted (native = fastest) |
| 1        | 142x428 | portrait                                 |
| 2        | 428x142 | landscape rotated 180 degrees            |
| 3        | 142x428 | portrait flipped                         |

`MAX_WRITE_LINE` in `DiffBuff.h` was reduced 120 -> 64: it caps the pixels in
one DMA transfer, and the Teensy DMA major-loop counter is 15 bits (32767
pixel max), so with 428-pixel lines: 64 x 428 = 27392 < 32767.

### 3. GRAM window offset (`ILI9341Driver.h` + every 0x2A/0x2B write)

The NV3007's internal memory is **168 columns** wide but the glass shows only
142, starting at physical column 12 (12 + 142 + 14 = 168). Every address the
driver sends must therefore be shifted to land on visible pixels. The vendor
demo (`LCD_SetPos()` in `lcd.h`) confirms the offsets: +12 or +14 depending on
whether MADCTL mirrors that axis.

With our MADCTL value (0x60 = MV|MX), the panel's column axis is addressed by
the *row* command (0x2B) and is mirrored, so:

- `ILI9341_T4_XOFFSET = 0`  (added to every 0x2A address)
- `ILI9341_T4_YOFFSET = 14` (added to every 0x2B address)

The offsets are applied at the last moment - only when bytes are written to
the controller - so all internal bookkeeping stays in clean 0-based screen
coordinates. If the image appears 180 degrees rotated on your unit, either use
`setRotation(2)` (software, free) or change `ILI9341_T4_MADCTL_VALUE` to 0xA0
and `ILI9341_T4_YOFFSET` to 12 (hardware).

### 4. NV3007 initialization sequence (`ILI9341Driver.cpp: begin()`)

The ILI9341 init table was replaced with the NV3007 sequence transcribed
byte-for-byte from the vendor demo. Almost all of it writes an undocumented
vendor register page (selected by `0xFF 0xA5`): power pump, gate driving (GOA),
and gamma settings specific to this glass. Treat it as a black box - it must be
sent exactly as EastRising specifies. The meaningful tail end:

- `0x35 0x00`, `0x44 ...` - enable the TE output pin (unused for now, free)
- `0xFF 0x00` - switch back to the standard command page
- `0x3A 0x05` - 16-bit RGB565 pixels (what this library uses)
- `0x36 0x60` - MADCTL: row/column exchange (see change 2)

The vendor sequence contains one stray data byte (0x17 after command 0xF1,
where a `0xF9` command was commented out); it is reproduced faithfully since
init sequences should match the manufacturer's exactly.

### 5. Refresh-rate control disabled (`setRefreshMode()`)

Upstream writes the ILI9341 frame-rate register 0xB1 - that register does not
exist on the NV3007's public command page (its frame timing lives in the
undocumented vendor page). `setRefreshMode()` no longer sends anything; the
panel runs at its power-on refresh rate (assumed ~60Hz, and unmeasurable
without MISO). The internal `_period` bookkeeping is kept so timers behave.

### 6. Hardware scroll disabled (`setScroll()`)

The controller scrolls along its physical 428-row axis, which after
row/column exchange is the driver's X axis - the library's "scroll the
framebuffer lines" semantic can't be implemented with it. The method now just
prints a warning. (Scrolling in software by drawing shifted content works
fine; the diff engine turns it into an efficient update anyway.)

### 7. Font-struct interop guard (`font_ILI9341_T4.h`)

`ILI9341_t3_font_t` (the standard PJRC font struct) is defined by half the
Teensy display ecosystem - ILI9341_t3.h, ILI9341_t3n.h and tgx all wrap
their identical copy in `#ifndef _ILI9341_FONTS_H_` so that any two of them
can appear in the same translation unit. Upstream ILI9341_T4 defines the
struct without that guard, so including `<ILI9341_T4.h>` and `<tgx.h>`
together (which the gauge firmware does: ILI9341_T4 drives the panels, tgx
draws the frames) failed to compile. The typedef is now wrapped in the same
interop guard; the layout is identical everywhere, so whichever library gets
included first provides the definition.

## Performance expectations

The NV3007 datasheet rates SPI **writes at 100 MHz** (10 ns min clock cycle) -
far above the ILI9341's 10 MHz rating. The practical limit will be the Teensy
LPSPI + wiring, typically 40-60 MHz. At 40 MHz a full-frame upload is
60776 px x 16 bits / 40 MHz = ~24 ms (~41 fps), and differential updates on
mostly-static gauge graphics redraw a small fraction of that, so the 50 fps
target is realistic for gauge-style content.

## What was deliberately NOT changed

- Class/namespace/macro names (`ILI9341_T4::ILI9341Driver`, ...) - keeping the
  upstream names keeps the diff against upstream small and future upstream
  merges easy.
- The `examples/` folder - still the upstream ILI9341 examples; sizes and pins
  do not match this display. See the project's `src/main.cpp` instead.
- The XPT2046 touch code - dead code for us (disabled when miso=255).
