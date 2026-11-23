# Light Show (`MODE_LIGHTSHOW`)

Dial position 9 plays a music track from the DFPlayer while the LED strip,
the three screens and their backlights, and the car's exterior lamps flash
in time with it. It is for car parks. Everything about it is built so that
it cannot happen on a stage.

Status as of Nov 2026: **a prototype that has never run on hardware.** The
firmware builds, the logic passes its host tests, and the choreography can
be watched in the simulator. The audio sync, the lamp behaviour and the 5 V
load are unmeasured; the bench list at the end says what to check.

## Using it

1. Turn the dial to position 9. The screens show the track name and a
   3-second countdown; the tach strip burns down like a fuse.
2. To pick another track, flick the dial one click away and back within
   4 seconds. Each flick steps to the next row of `SHOW_TRACKS`. Staying
   away longer starts again at track 1.
3. At zero the track starts. Turning the dial anywhere else stops it at
   once.
4. When the track ends the gauge is a gauge again (at Medium brightness)
   until the dial leaves and returns. `SHOW_AUTO_ADVANCE` makes it play the
   whole list on a loop instead.

The show will not start, and ends immediately if it is running, when:

| Condition | Why |
|---|---|
| a red alarm is active | the gauge has something more important to say |
| engine speed above `SHOW_MAX_RPM` (1500) with the ECU talking | the car is being driven. Idling to hold the battery up is allowed |
| the gauge powered up already on position 9 | a reset mid-show must come back as a gauge, not restart the show |

Once ended this way it stays ended until the dial is moved off and back.
While a show runs, the alarm voice is quiet. A red alarm ends the show and
then speaks; a yellow one that came up during the show speaks after it.

## The SD card

Show tracks go in the same `/mp3` folder as the voice clips, numbered from
`0100` so they can never collide with a clip:

```
/mp3/0001-nominal.mp3 ... 0023-check-messages.mp3   voice clips (audio.h)
/mp3/0100-click120.wav                              calibration click
/mp3/0101-sandstorm.mp3                             track 1
/mp3/0102-....mp3                                   track 2 ...
```

Only the four digits matter to the module. The speaker is the 3 W voice
speaker, so expect it to sound like one.

## How it stays in time

The DFPlayer reports nothing about where in a track it is. The gauge sends
the play command, starts a clock, and computes the position in the music
from elapsed time. Two constants in `config.h` line the two up:

| Constant | Meaning | Shipped value |
|---|---|---|
| `SHOW_AUDIO_LATENCY_MS` | play command to first sound | 150, **a guess**. DFRobot publishes no figure; forum reports say "hundreds of ms" |
| `SHOW_TEMPO_TRIM` | DFPlayer clock speed ÷ Teensy clock speed | 1.0 |

A beat at 136 bpm is 441 ms. As a rule of thumb (broadcast lip-sync limits
sit in the same range), light and sound less than about 50 ms apart read
as together and more than about 150 ms apart read as wrong. So the latency
has to be found to a few tens of ms, and over a 3:45 track the two clocks
have to agree to a few hundredths of a percent. Whether a DFPlayer is that
accurate, or that stable from cold to hot, is not known. If it is not, the
fix is hardware (see "If the sync will not hold").

### Calibrating

```
python3 tools/lightshow/make_click_track.py     # writes 0100-click120.wav
```

Copy it to `/mp3` and select `CLICK 120` (the last track). Every light
flashes on every click.

1. **Latency.** Watch the first few seconds. Lights early: raise
   `SHOW_AUDIO_LATENCY_MS`. Lights late: lower it.
2. **Trim.** If the start is right and the end has drifted, the clocks
   differ. When a track finishes the serial monitor prints
   `[SHOW] "…" ended after N ms`. With the file's true length `L` ms,
   `SHOW_TEMPO_TRIM = L / (N - SHOW_AUDIO_LATENCY_MS)`. Use the longest
   track you have for this: the uncertainty in `N` is a fixed few tens of
   ms, so a longer track gives a better ratio.

The click is a WAV because an MP3 encoder adds a few tens of ms of silence
at the front. If your module will not play a WAV from `/mp3`, convert it
and allow for that.

## Adding a track

One row in `SHOW_TRACKS` (`config.h`):

```cpp
{ 102, "MY TRACK", 128.0f, 420, 201, nullptr },
//  |       |        |      |    |     └ map (nullptr = the default arc)
//  |       |        |      |    └ length in seconds
//  |       |        |      └ ms into the file of the first downbeat
//  |       |        └ tempo
//  |       └ name on the countdown screen (capitals and digits)
//  └ /mp3/0102*.mp3
```

The tempo has to be right to four figures: 0.1 bpm off is a tenth of a
beat after one minute and over a third of a beat by the end of a 3:45
track. `beat1Ms` belongs to the file, not the song: two rips
of the same track differ by their leading silence.

**The simulator finds both.** Open `tools/lightshow/lightshow_sim.html`,
choose "Your audio file", type a rough tempo, press "Find tempo and first
beat", and copy the row it prints. The file is decoded in the page and goes
nowhere.

### The map

One letter per four-bar phrase. If the track outlasts the map, the map
repeats.

| Letter | Look | Strip and screens | Exterior lamps |
|---|---|---|---|
| `p` | pulse | one colour, flashing on every beat and fading | tail lights on the beat, both turn signals on every second beat |
| `c` | chase | a comet crosses the tach strip each beat; the screens light in turn | turn signals and the two light bars alternate, left/spot then right/wide |
| `b` | build | the strip fills left to right; white strobe at 1, then 2, then 4 flashes a beat. A run of `b`s is one build and its last beat is dark | brake and reverse lights alternate; turn signals join for the last third |
| `d` | drop | white on the kick cooling to a new colour each beat; backlights strobe | everything, on the beat: PDM-L's forward lamps on even beats, PDM-E's on odd ones |
| `k` | break | colours drift slowly along the strip; the screens breathe | tail lights on, steady |
| `-` | dark | off | off |

`nullptr` uses `SHOW_DEFAULT_MAP` (`ppccbbddddkkbbdddd`), which fits most
dance tracks well enough that a new track needs only its tempo and first
beat. The simulator's "Suggest a map" drafts one from the bass level of
each phrase.

**The Sandstorm row is a placeholder.** 136 bpm and 3:45 are the published
figures for the radio edit. The map was written from the track's general
shape without a file to measure, and `beat1Ms` is 0. Align both against
your own file in the simulator.

## What limits what

| Source | Limit | Where |
|---|---|---|
| LED strip | every frame is scaled to fit `SHOW_LED_MAX_MA` (800 mA). Full white on 27 LEDs would be 1.6 A on a 3 A rail that also runs the backlights and the audio amplifier | `lightshow_fx.cpp`, end of `showRender` |
| Screens | a panel changes colour at most once a beat (a full repaint is ~24 ms of SPI). Its brightness is the backlight PWM, which is what strobes | `lightshow_fx.cpp` |
| Strip and backlight strobe rate | `SHOW_MAX_FLASH_DIV` flashes per beat: 4 is 9 Hz at 136 bpm | `config.h` |
| Exterior lamps | each lamp changes at most once per half beat and flashes at most once per beat. The two PDMs' forward lamps are never on together | `lightshow_fx.cpp`; enforced by the host tests |

On flash rates: epilepsy charities put the risk band at roughly 3 to 30 Hz
(some say up to 60), and report the UK safety regulator's advice for
strobes at public venues as 4 flashes a second or fewer. That guidance is about bright, large sources.
The exterior lamps are the bright, large sources here and stay under 3 Hz
at any tempo below 180 bpm. The strip and backlights are small, but at the
shipped setting they do enter the band; `SHOW_MAX_FLASH_DIV = 1` keeps
them out of it.

The lamps have their own physics, none of it measured on this car:

- A filament takes around 200 ms to reach most of its light, so bulbs
  pulse rather than strobe. LED lamps and bars follow the bits exactly.
- Every flash is a cold start. If a PDM output's inrush settings do not
  allow a restart every beat it will trip, and the gauge will show `TRP`
  and say "light fault" after the show.
- HID lamps must not be flashed. If the car has any, leave their bits out
  of the PDM configuration.

## The exterior lamps

The gauge cannot switch a lamp. It sends `GAUGE_ID_SHOW` (0x4C2) with one
bit per lamp group, and each PDM output is configured in dingoConfig to
come on for its roof switch **or** its show bit. The recipe is in
[`PDM_CAN_SETUP.md`](PDM_CAN_SETUP.md). Until that is done the bits do
nothing, and `SHOW_DRIVE_LAMPS = false` stops them being sent at all.

The show can only add light. It has no way to turn off a lamp the roof
switches have on, and the fail-on logic for a dead gauge is untouched.

## If the sync will not hold

In order of effort:

1. **Shorter tracks.** Drift grows with time; a 60 to 90 second edit has a
   third of the exposure of a full track.
2. **Wire the DFPlayer's BUSY pin** to a free input. It goes low when
   sound starts, which removes the latency guess. It does nothing for
   drift. BUSY is unconnected on the v1.0 board.
3. **A sync channel.** Put the music on the left channel and a click on
   the right, and route `DAC_R` to a Teensy pin. The show then follows the
   file itself and both constants disappear. `DAC_L`/`DAC_R` are
   unconnected on the v1.0 board, and which channel the module's own
   amplifier plays would need checking first.

## Bench list

1. Dial to 9: countdown, title, then lights. Dial away: everything stops
   within one tick.
2. Power up with the dial on 9: nothing happens.
3. Raise a red alarm mid-show, by whatever the bench allows: the show
   ends and the alarm speaks.
4. Calibrate latency and trim as above. Then play the longest track and
   judge the last 20 seconds.
5. Scope the 5 V rail during a drop at full volume. The strip budget, the
   backlights and the amplifier peak together there.
6. With the PDMs configured: watch each lamp group for trips through a
   whole track, and check the lamp currents against the output limits.
7. Unplug the gauge mid-show: every show lamp goes out within the PDM
   input timeout and the fail-on lamps behave exactly as they do without a
   show.

## Tools

`tools/lightshow/` holds everything that runs off the car:

| File | What |
|---|---|
| `build.sh` | runs the host tests, builds the simulator |
| `test_lightshow.cpp` | the tests: every track rendered start to finish against the limits above, and the state machine against a hand-driven clock |
| `lightshow_sim.html` | the simulator (generated; open it in a browser) |
| `sim_template.html`, `sim_api.cpp` | its page and its hook into the firmware code |
| `Arduino.h` | stand-in for the Teensy core, for PC builds only |
| `make_click_track.py` | the calibration track |

The simulator is not a second implementation. `build.sh` compiles
`src/lightshow_fx.cpp` itself to WebAssembly and then to JavaScript, and
checks a few thousand frames against the native build before writing the
page. Change the choreography, re-run `build.sh`, and the page shows the
new behaviour.
