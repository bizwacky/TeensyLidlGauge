// ---------------------------------------------------------------------------
// Rally CAN Gauge - central configuration
// ---------------------------------------------------------------------------
// Every pin number, electrical constant, and tunable value lives in this one
// file. If the wiring or the ECU configuration changes, this should be the
// only file that needs editing (the .cpp files just consume these names).
//
// THE BOARD: a SparkFun MicroMod Teensy in an M.2 socket (it replaced the
// Teensy 4.0 in Nov 2026). Pin numbers below are Teensyduino pin numbers,
// which is also how the schematic names the MicroMod symbol's pins
// ("5_Switch_1", "39_PWM_3"...). The whole map, as wired on the rev 2 PCB
// (the one with the M.2 socket turned 180 degrees):
//
//    0  disp 2 DC   (SPI1 CS0)  16  pot 4 (A2)             32  disp 2 RESET
//    1  disp 3 TE   (reserved)  17  pot 5 (A3)             33  backlight 2 PWM
//    2  disp 2 CS               18  FRAM SDA               34  disp 1 TE (reserved)
//    3  roof switch 2           19  FRAM SCL               35  disp 1 MOSI (SPI2)
//    4  backlight 3 PWM         20  pot 3 (A6)             36  disp 1 DC   (SPI2 CS0)
//    5  roof switch 1           21  pot 1 (A7)             37  disp 1 SCK  (SPI2)
//    6  disp 1 CS               22  fan PWM 2              38  roof switch 6
//    7  fan PWM 1               23  pot 2 (A9)             39  fan PWM 3
//    8  LED strip data          24  disp 3 RESET           40  IMU SCL
//    9  backlight 1 PWM         25  disp 3 CS              41  IMU SDA
//   10  disp 3 DC   (SPI CS0)   26  disp 2 MOSI (SPI1)     42  roof switch 3
//   11  disp 3 MOSI (SPI)       27  disp 2 SCK  (SPI1)     43  roof switch 4
//   12  disp 2 TE   (reserved)  28  MP3 RX (Serial7)       44  roof switch 5
//   13  disp 3 SCK  (SPI)       29  MP3 TX (Serial7)       45  disp 1 RESET
//   14  fuel sender (A0)        30  CAN3 RX
//   15  rotary dial (A1)        31  CAN3 TX
//
// 31 of these 46 moved with the rev 2 board. What stayed: display 2's bus
// (0/26/27), the LED strip, both I2C buses, MP3 serial, CAN and roof
// switches 3-5. Three rules of the hardware constrain any future shuffle:
//   * the three backlights must share ONE FlexPWM module (backlight.cpp
//     phase-locks them) - they are all on FlexPWM2;
//   * the LED strip must sit on a UART TX pin (WS2812Serial);
//   * each display's DC sits on its SPI bus's hardware CS0 (ILI9341_T4's
//     fast path needs it).
// The three TE lines are inputs parked on the SPI MISO pads (1, 12, 34);
// nothing reads them yet. The IMU on 40/41 is not driven by this firmware
// yet either.
//
// Checked Nov 2026 against the KiCad netlist, the routed PCB and the Teensy
// core's MicroMod tables (pwm.c, analog.c, SPI.cpp, HardwareSerial7.cpp,
// WS2812Serial, FlexCAN_T4): every pin carries the peripheral function the
// code asks of it.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>

// --- superloop timing ---------------------------------------------------------

static const uint32_t TICK_FAST_MS = 20;   // 50 Hz: inputs, mixture window, alarms, LEDs, screens, backlights
static const uint32_t TICK_SLOW_MS = 1000; // 1 Hz: BMS heartbeat, audio arbitration, FRAM, debug print

// --- CAN bus ------------------------------------------------------------------
// One physical bus shared by the rusEFI ECU (a Super uaEFI - it has a single
// CAN port, so its on-board wideband controller talks on this bus too), the
// JK BMS and this gauge. Everyone must agree on the bitrate. The ECU side is
// documented in docs/ECU_CAN_SETUP.md - keep that file and these constants
// in sync!

static const uint32_t CAN_BITRATE = 500000; // 500 kbps (rusEFI default)

// rusEFI's native broadcast ("Enable rusEFI CAN broadcast" in TunerStudio):
// twelve fixed 8-byte frames at base + 0..11, LITTLE-endian, all sent
// together once per "rusEFI CAN data period". Their layouts are defined by
// rusEFI itself (firmware/controllers/can/rusEFI_CAN_verbose.dbc, which
// calls them BASE0..BASE11); can_bus.cpp decodes BASE0..BASE7.
//
// rusEFI's default base is 0x200 - the same id as one of the JK BMS's
// frames (docs/BMS_CAN_SETUP.md). Two nodes transmitting one id corrupt
// each other, so the ECU is moved: set "rusEFI CAN data base address" to
// 1664 (= 0x680) in TunerStudio. 0x680..0x68B is clear of everything else
// on this bus.
static const uint32_t RUSEFI_CAN_BASE_ID     = 0x680;
static const uint8_t  RUSEFI_CAN_FRAME_COUNT = 12; // BASE0..BASE11

// The one value the native broadcast lacks is the TARGET lambda, which the
// mixture alarms are judged against. rusEFI's "MS Simplified Broadcast"
// (TunerStudio: CAN dash type) carries it in its third frame as 'afrtgt1':
// one byte, AFR x 10, = target lambda x 14.7. That byte is all we take from
// the five MS frames; the other four are ignored. rusEFI fixes their ids
// (1512..1516) and sends them every 10 ms.
static const uint32_t MS_DASH_BASE_ID       = 0x5E8; // 1512
static const uint8_t  MS_DASH_FRAME_COUNT   = 5;
static const uint32_t MS_DASH_AFR_TARGET_ID = 0x5EA; // 1514: afrtgt1 in byte 0

// rusEFI wideband controllers - including the Super uaEFI's on-board one -
// broadcast their own lambda frames from 0x190 up. Expected traffic, but not
// decoded: the ECU already re-broadcasts the lambda it uses in BASE7.
static const uint32_t RUSEFI_WBO_FIRST_ID = 0x190;
static const uint32_t RUSEFI_WBO_LAST_ID  = 0x19F;

// Freshness: how old an ECU frame may be before its values stop counting
// (display shows "--", alarms gate off, NO CAN raises). Several broadcast
// periods, so a lost frame never trips anything. Sized for a rusEFI CAN
// data period of 50 ms or faster; rusEFI slows its broadcast 5x while
// TunerStudio talks to it OVER CAN, and at 50 ms that (250 ms) still fits.
static const uint32_t ECU_FRESH_MS = 500;

// JK BMS speaking the "Deye" protocol (PCS CAN-Bus-protocol-DY-low-voltage,
// select protocol 001 in the JK app). Standard 11-bit ids, LITTLE-endian
// payloads (opposite of the ECU!), everything broadcast on a 1 s cycle.
// Frame layouts: docs/BMS_CAN_SETUP.md.
static const uint8_t  BMS_ADDR = 1; // single pack; per-module ids are base+(addr-1)

static const uint32_t BMS_ID_SOC       = 0x355;                  // SOC / SOH
static const uint32_t BMS_ID_PACK      = 0x356;                  // pack V / A / avg temp
static const uint32_t BMS_ID_PROTECT   = 0x359;                  // fault + alarm bitfields
static const uint32_t BMS_ID_CELLS     = 0x361;                  // max/min cell V + temp
static const uint32_t BMS_ID_STATE     = 0x400 + (BMS_ADDR - 1); // mode/fail/cycles/balance
static const uint32_t BMS_ID_FAULT_MOS = 0x110 + (BMS_ADDR - 1); // MOSFET states (byte 7)

// The protocol expects the inverter ("PCS") to send a 0x305 heartbeat every
// second; without one the BMS may raise its "PCS communication failure" bit.
// There is no inverter in the car, so the gauge plays that role. Set false
// if it ever causes trouble on the bench.
static const bool     BMS_SEND_PCS_HEARTBEAT = true;
static const uint32_t BMS_ID_PCS_HEARTBEAT   = 0x305;

// 1 s broadcast cycle -> allow a few missed frames before declaring it gone.
static const uint32_t BMS_OFFLINE_MS = 5000;

// --- the two PDMs (dingoFW) ---------------------------------------------------------
// Two identical 14-channel boxes running dingoFW (the "Shrimp" fork of
// dingoPDM) switch every load in the car. PDM-E carries the engine: fans,
// starter, fuel pump, coils, engine-switched feed. PDM-L carries lighting,
// wipers and cabin loads. Forward lighting is split across both. Pin maps:
// PDM_OUTPUTS further down; contract with dingoConfig: docs/PDM_CAN_SETUP.md.
//
// Each box owns a block of 11-bit ids starting at its Base ID (burned into
// the box with dingoConfig): Base+0/+1 are dingoConfig's settings channel,
// Base+2 upward are the cyclic status frames, all sent every 100 ms.
//
// THESE TWO BASE IDS ARE PLACEHOLDERS - set them to whatever the CAN id plan
// assigns and burn the same numbers into the boxes. Neither sits on dingoFW's
// factory default (0x0DE) on purpose: a replacement box that was never
// configured then shows up as unk= traffic instead of impersonating a PDM.
// 0x40 ids are reserved per box (stock dingoFW uses 32; the fork's frames
// for outputs 9-14 need a few more).
//
// The index into g_data.pdm[] is the box's ROLE. Today role = box (the id
// is burned in). If "the role follows the plug" is adopted, the role must
// come from the strap inputs instead - can_bus.cpp maps id -> slot in one
// place (pdmSlotForId) so that is the only code to change.

enum PdmId : uint8_t
    {
    PDM_E = 0, // engine box: its device faults can stop the car -> red
    PDM_L,     // lighting/cabin box: its device faults are yellow
    PDM_COUNT
    };

static const uint32_t PDM_BASE_ID[PDM_COUNT] = { 0x440, 0x480 }; // PLACEHOLDERS, see above
static const uint32_t PDM_ID_BLOCK           = 0x40;             // ids reserved per box

static const uint8_t  PDM_MAX_OUTPUTS = 14; // the Shrimp board; stock dingoPDM has 8

// Status older than this is not trusted: the PDM alarms gate off and
// PDME CAN / PDML CAN raises. Ten missed 100 ms frames.
static const uint32_t PDM_FRESH_MS = 1000;

// Current scale of the status frames. dingoFW d594cd7 sends WHOLE AMPS
// ((uint16_t) cast of a float in msg.cpp, and the CAN docs say "A"); the
// .dbc file in the same repo still says 0.1 A/bit. Source wins. If the
// fork restores 0.1 A resolution, change this one number.
static const float PDM_AMPS_PER_COUNT = 1.0f;

// THE STATUS FRAME TABLE. One row per cyclic frame the gauge decodes, as an
// offset from the box's Base ID - so the decoder follows whatever layout
// the firmware defines without touching can_bus.cpp. Field packing per kind
// (all little-endian, from dingoFW boards/dingopdm_v7/msg.cpp):
//
//   DEVICE    b0 digital inputs, b1 low nibble device state (0 Run, 1 Sleep,
//             2 OverTemp, 3 Error) / high nibble board type, b2-3 total
//             current, b4-5 battery V x10, b6-7 board temp C x10
//   CURRENT   one uint16 per output (PDM_AMPS_PER_COUNT)
//   STATE     one nibble per output, low nibble first (0 Off, 1 On,
//             2 Overcurrent, 3 Fault). Only whole output-carrying bytes are
//             read: the stock frame's wiper/flasher bytes are not outputs.
//   OC_COUNT  one byte per output: overcurrent trips since the request went on
//   DUTY      one byte per output: PWM duty %, only sent when PWM is enabled
//
// "need" says how the frame counts toward the box being ONLINE:
//   PDM_NEED_ALWAYS     must keep arriving (stock frames dingoFW always sends)
//   PDM_NEED_ONCE_SEEN  optional until it has arrived once, required after.
//                       This is how the same gauge firmware runs against a
//                       stock 8-output board on the bench (which never sends
//                       the rows below the line) and against the fork.
//   PDM_NEED_NEVER      decoded when present, never required
//
// THE ROWS FOR OUTPUTS 9-14 ARE A PROPOSAL: those frames do not exist in any
// firmware yet. They are what this gauge will decode, so either build the
// fork to this layout or edit the rows to match what the fork sends.

enum PdmFrameKind : uint8_t
    {
    PDM_KIND_DEVICE = 0,
    PDM_KIND_CURRENT,
    PDM_KIND_STATE,
    PDM_KIND_OC_COUNT,
    PDM_KIND_DUTY,
    };

enum PdmFrameNeed : uint8_t
    {
    PDM_NEED_ALWAYS = 0,
    PDM_NEED_ONCE_SEEN,
    PDM_NEED_NEVER,
    };

struct PdmFrameDef
    {
    uint8_t      idOffset;    // CAN id = PDM_BASE_ID[box] + idOffset
    PdmFrameKind kind;
    uint8_t      firstOutput; // 0-based index of the first output in the frame
    uint8_t      outputs;     // how many outputs the frame carries
    PdmFrameNeed need;
    };

static const PdmFrameDef PDM_FRAMES[] =
    {
    //  id    kind               first  n   need
    {  2, PDM_KIND_DEVICE,    0, 0, PDM_NEED_ALWAYS    }, // msg 0
    {  3, PDM_KIND_CURRENT,   0, 4, PDM_NEED_ALWAYS    }, // msg 1: outputs 1-4
    {  4, PDM_KIND_CURRENT,   4, 4, PDM_NEED_ALWAYS    }, // msg 2: outputs 5-8
    {  5, PDM_KIND_STATE,     0, 8, PDM_NEED_ALWAYS    }, // msg 3: outputs 1-8
    {  6, PDM_KIND_OC_COUNT,  0, 8, PDM_NEED_ALWAYS    }, // msg 4: outputs 1-8
    { 25, PDM_KIND_DUTY,      0, 8, PDM_NEED_NEVER     }, // msg 23: outputs 1-8
    // ---- PROPOSED fork frames for outputs 9-14 (see the note above) ----
    { 32, PDM_KIND_CURRENT,   8, 4, PDM_NEED_ONCE_SEEN }, // outputs 9-12
    { 33, PDM_KIND_CURRENT,  12, 2, PDM_NEED_ONCE_SEEN }, // outputs 13-14
    { 34, PDM_KIND_STATE,     8, 6, PDM_NEED_ONCE_SEEN }, // outputs 9-14
    { 35, PDM_KIND_OC_COUNT,  8, 6, PDM_NEED_ONCE_SEEN }, // outputs 9-14
    { 36, PDM_KIND_DUTY,      8, 6, PDM_NEED_NEVER     }, // outputs 9-14
    };
static const uint8_t PDM_FRAME_COUNT = sizeof(PDM_FRAMES) / sizeof(PDM_FRAMES[0]);

// --- what the gauge transmits to the PDMs ----------------------------------------------
// The roof panel's six switches and five pots are wired to the gauge, not to
// the PDMs. The gauge reads them and broadcasts two frames; each PDM picks
// the bits it wants with dingoFW "CAN inputs". Layouts: docs/PDM_CAN_SETUP.md
// and the top of can_bus.cpp. Like the PDM ids these two are PLACEHOLDERS.
//
// FAIL-ON LIGHTING: with its timeout enabled, dingoFW zeroes a CAN input
// when the frame stops arriving. So the switch frame carries every switch
// TWICE: an "on" bit and an "off" bit that is always its complement. A PDM
// lamp output driven from NOT(off bit) comes ON when the gauge or the bus
// dies; a load driven from the on bit goes OFF. Which behaviour each load
// gets is a dingoConfig choice - the gauge always sends both.
static const uint32_t GAUGE_ID_SWITCHES = 0x4C0; // switches, flags, alive counter
static const uint32_t GAUGE_ID_POTS     = 0x4C1; // pot positions + fan PWM duty
static const uint32_t GAUGE_ID_SHOW     = 0x4C2; // light show lamp requests (all zero
                                                 // unless a show is playing)

// Both frames go out every GAUGE_TX_PERIOD_MS, and at once (next 20 ms tick)
// when a switch or a pot moves. Set each PDM CAN input's timeout to a few
// periods (500 ms suggested).
static const uint32_t GAUGE_TX_PERIOD_MS = 100;

// --- analog input pins ----------------------------------------------------------

static const uint8_t PIN_FUEL_SENDER = 14; // A0 - fuel sender divider node
static const uint8_t PIN_ROTARY      = 15; // A1 - 12-position rotary divider

// (The four 12V trigger inputs - headlights, brights, blinkers - are gone
// from the board. The lamp states the screens show come from the PDMs over
// CAN, see LAMP_* below.)

// --- ADC configuration ----------------------------------------------------------

static const uint8_t  ADC_RESOLUTION_BITS = 12;   // 0..4095 counts
static const int      ADC_MAX_COUNTS      = 4095;
static const uint8_t  ADC_AVERAGING       = 16;   // hardware averaging: ~16x less noise,
                                                  // still well under 0.1 ms per pin

// --- fuel sender ----------------------------------------------------------------
// Circuit (KiCad sheet "Analog Inputs"):
//
//   +3V3 --[R14 150R precision]--+-- FUEL_IN --> 2 x E30 sender in series --> GND
//                                |
//                          [R13 2.2k]--+-- FUEL_READ (pin 14)
//                                      |
//                                 [C12 100n]
//                                      |
//                                     GND
//
// R14 and the senders form the divider; R13/C12 are only an RC filter on
// the ADC pin (720 Hz corner). The ADC draws no DC current, so R13 drops
// nothing and the pin sees the divider node exactly. Divider and ADC share
// the 3.3V rail, so the reading is RATIOMETRIC - the rail cancels out:
//
//   f = counts / 4095 = Rs / (Rs + 150)    =>    Rs = 150 * f / (1 - f)
//
// Only R14's tolerance matters, which is why it is a precision part. It
// dissipates 67 mW with the tank full (Rs = 6R -> 21 mA): the schematic's
// "up to 70 mW". Expected readings, each sender being 3R full .. 72R empty:
//
//   full   Rs =   6R   f = 0.038   0.13 V    157 counts
//   empty  Rs = 144R   f = 0.490   1.62 V   2006 counts   (~13 counts/ohm)
//   open   Rs = inf    f = 1.000   3.30 V   4095 counts
//   short  Rs =   0R   f = 0.000   0.00 V      0 counts

static const float FUEL_REF_OHMS = 150.0f; // R14, the precision reference

// Dual E30 senders in series: 2 x (3R full .. 72R empty) per the schematic
// note. STILL CALIBRATE ON THE CAR: fill with known amounts and record the
// slosh-filtered ohms the debug print shows - senders are rarely truly
// linear. If the gauge reads backwards, just swap these two numbers.
static const float FUEL_OHMS_EMPTY = 144.0f; // 2 x 72R
static const float FUEL_OHMS_FULL  = 6.0f;   // 2 x 3R

// Fast debug filter: a short exponential moving average that tracks the
// sender closely. NOT used for display or alarms - it exists so the serial
// print shows what the sender is physically doing (slosh and all), which is
// how you sanity-check the slosh filter below against reality.
static const float FUEL_FILTER_TAU_S = 2.0f;

// Fault detection, in ohms (the same math as the reading itself): a broken
// wire lets R14 pull the node to the rail (Rs -> infinite), a wire chafed
// onto the chassis reads ~0R. Both limits sit far outside the 6..144R
// sender range while staying well clear of the fault readings themselves.
static const float FUEL_OHMS_OPEN  = 400.0f; // above this = wiring open
static const float FUEL_OHMS_SHORT = 1.0f;   // below this = shorted to ground

// --- fuel slosh filter ------------------------------------------------------------
// The display/alarm level runs through a TWO-STAGE filter (inputs.cpp):
//
//   stage 1: keep the last FUEL_SLOSH_MEDIAN_N raw readings, sampled every
//            FUEL_SLOSH_SAMPLE_MS, and take their MEDIAN. Why a median and
//            not more averaging? Slosh is not symmetric noise: in a long
//            corner the fuel piles up on one side and the float can PIN at an
//            end stop for seconds. An average gets dragged toward that lie;
//            a median simply ignores it as long as it lasts for less than
//            half the window.
//   stage 2: a slow exponential average (time constant FUEL_SLOSH_TAU_S)
//            over the median, which irons out what remains.
//
// Net effect: honest step response of ~1 minute. That is fine - real fuel
// level only moves at ~1%/minute at race pace. The fill gesture bypasses the
// lag by resetting the chain (fuelSloshReset()) when fuel is actually added.
static const uint32_t FUEL_SLOSH_SAMPLE_MS = 500;   // one median sample per 0.5 s
static const uint8_t  FUEL_SLOSH_MEDIAN_N  = 9;     // 9 samples = 4.5 s window
static const float    FUEL_SLOSH_TAU_S     = 30.0f; // stage-2 smoothing

// --- calculated fuel: injector-pulse integration (fuel_calc.cpp) -------------------
// rusEFI broadcasts the injector pulse width it is commanding (BASE5, once
// per CAN data period). Injectors are precision metering devices: open time
// x flow rate = fuel delivered. Integrating that gives a fuel estimate far
// steadier than any float in a sloshing tank.

static const uint8_t ENGINE_CYLINDERS = 6; // M54: one injector per cylinder

// Siemens A2C59514053 (M54B30 OEM): 254 cm3/min at 3.5 bar fuel pressure
// (from "M54 Injector Spec.png" in the project root). If rail pressure ever
// changes, flow scales with sqrt(new bar / 3.5).
static const float INJ_FLOW_CC_PER_MIN = 254.0f;

// Injector dead time vs battery voltage, from the same spec sheet
// ("ip_ti_add_dly__vb"). An injector is an electromagnet: it takes this long
// to physically open after the pulse starts, during which NO fuel flows.
// rusEFI adds its own dead-time correction onto the pulse it broadcasts
// (InjectorModelBase::getInjectionDuration returns base + dead time), so the
// fuel math must subtract it back out - at a 2 ms idle pulse the ~0.4 ms
// dead time is 20% of the reading, so skipping this correction would
// overcount idle burn by a fifth. Keep this table the same as the injector
// dead-time (battery lag) table in the rusEFI tune - the subtraction is only
// right if both sides agree. Interpolated linearly, clamped at the ends.
struct InjDeadTime
    {
    float volts;
    float ms;
    };

static const InjDeadTime INJ_DEAD_TIME[] =
    {
    {  6.0f, 3.328f },
    {  8.0f, 1.408f },
    { 10.0f, 0.864f },
    { 12.0f, 0.576f },
    { 14.0f, 0.384f }, // normal running voltage sits right about here
    { 16.0f, 0.256f },
    { 25.0f, 0.032f },
    };
static const uint8_t INJ_DEAD_TIME_N = sizeof(INJ_DEAD_TIME) / sizeof(INJ_DEAD_TIME[0]);

// Calibration trim for systematic error (real injectors are not exactly
// nominal, fuel temp varies, etc). CALIBRATE: run a few tanks, compare
// gallons the gauge says were burned against gallons the pump says went
// back in, then set trim = actual / calculated. Expect within a few %.
static const float FUEL_CALC_TRIM = 1.000f;

static const float CC_PER_GALLON = 3785.41f; // US gallon

// MEASURE ON THE CAR: usable tank capacity. Late E30 tank is nominally
// 16.4 gal (62 L) - verify what a full fill actually takes.
static const float FUEL_TANK_GAL = 16.4f;

// Smoothing for the displayed burn rate (gal/h). Display nicety only - the
// integration itself always uses the raw per-tick values.
static const float FUEL_BURN_GPH_TAU_S = 2.0f;

// --- fuel fill gesture --------------------------------------------------------------
// Refueling is told to the gauge with the rotary dial: sit on the FUEL FILL
// detent and wait. Crossing 30 s credits one 5-gal can; staying to 60 s
// declares a full tank (an absolute reset, so it also recovers a lost/silly
// calculated value). Two cans = do the 30 s hold twice (leave the detent and
// come back). Progress is published in g_data.fuel for the screens phase.
static const float    FUEL_FILL_CAN_GAL = 5.0f;
static const uint32_t FUEL_FILL_CAN_MS  = 30000;
static const uint32_t FUEL_FILL_FULL_MS = 60000;

// --- rotary mode switch ----------------------------------------------------------
// Circuit (KiCad sheet "Analog Inputs" - the same input cell as the roof
// switches and pots): the 12-position switch selects one of 11 resistors
// to PCB ground (position 0 is a dead short). On the PCB a 4.7k pull-up
// feeds the wiper node, with the filter cap AT the connector and a 22k
// series resistor protecting the ADC pin:
//
//   +3V3 --[R66 4.7k]--+-- DIAL_IN --> wiper --> Rk --> GND
//                      |
//                 [C15 100n]      [R67 22k]
//                      |      +------/\/\/------ DIAL_READ (pin 15)
//                     GND     |
//                  (DIAL_IN) -+
//
// The pin sees 3.3V x Rk / (Rk + 4700). The eleven resistor values are
// chosen so those VOLTAGES are roughly evenly spaced (250-330 counts
// apart), which means the resistances are not: the ladder runs 0R to 15k.
// So inputs.cpp decides in the voltage domain - it compares the measured
// divider ratio f = counts/4095 against each position's expected ratio
// Rk/(Rk + 4700) and takes the nearest. Every boundary then sits halfway
// between two neighbours, at least +-125 counts of margin each:
//
//   pos  0     0R  0.000 V     0 counts    pos  6  3.3k  1.361 V  1689
//   pos  1   360R  0.235 V   291           pos  7  4.3k  1.577 V  1957
//   pos  2   750R  0.454 V   564           pos  8  5.6k  1.794 V  2226
//   pos  3  1.2k   0.671 V   833           pos  9  7.5k  2.029 V  2518
//   pos  4  1.8k   0.914 V  1134           pos 10   10k  2.245 V  2786
//   pos  5  2.4k   1.115 V  1384           pos 11   15k  2.513 V  3118
//   open (wiper lifted / broken wire)     3.300 V  4095
//
// Margin budget: 1% resistors move a tap by at most ~20 counts, and the 22k
// series resistor turns ADC input leakage into an offset of 27 counts per
// microamp (the pad's worst case is about 1 uA hot; typical is far less).
// Against a 125-count half-step that is comfortable.
//
// (History: until Nov 2026 this was a 2.2k pull-up into a chain of
// 11 x 220R, decoded by rounding ohms. The input cell and the ladder both
// changed with the MicroMod board; ROTARY_PULLUP_OHMS and ROTARY_OHMS are
// the two things to edit if the ladder is ever rebuilt.)
//
// FAULT DETECTION (DIALFAIL): an open wiper - broken wire, dead switch, or
// a non-shorting switch caught between detents - reads far above the last
// tap. Anything past ROTARY_OPEN_OHMS is called open. inputs.cpp debounces
// that like any position, holds the last good position while it lasts, and
// after ROTARY_OPEN_MS parks the dial on ROTARY_FAILSAFE_POS: position 0 =
// BRIGHT, i.e. "behave as if the dial weren't there" - screens on, full
// brightness, audio unmuted - so the DIALFAIL banner and callout can
// actually reach the driver instead of being trapped behind DISPLAYS OFF
// or SILENT. The alarm (alarms.cpp) uses the same ROTARY_OPEN_MS, so the
// banner and the fallback appear together. A short to ground is
// indistinguishable from position 0, which is the fail-safe mode anyway.

static const uint8_t  ROTARY_POSITIONS      = 12;
static const float    ROTARY_PULLUP_OHMS    = 4700.0f; // R66
// Resistance to ground at each detent, position 0 first (the note on the
// schematic). Must stay in rising order.
static const float    ROTARY_OHMS[ROTARY_POSITIONS] =
    {
       0.0f,  360.0f,  750.0f, 1200.0f, 1800.0f,  2400.0f,
    3300.0f, 4300.0f, 5600.0f, 7500.0f, 10000.0f, 15000.0f,
    };
static const float    ROTARY_OPEN_OHMS      = 35000.0f; // 3610 counts: halfway (in volts)
                                                        // between the last tap (3118)
                                                        // and open (4095)
static const uint8_t  ROTARY_DEBOUNCE_TICKS = 3;       // 3 x 20 ms = stable for 60 ms
static const uint16_t ROTARY_OPEN_MS        = 2000;    // open this long = DIALFAIL + fail-safe
static const uint8_t  ROTARY_FAILSAFE_POS   = 0;       // = MODE_BRIGHT in ROTARY_MODE_MAP

// --- gauge modes: what each detent of the dial means ------------------------------
// The dial is the gauge's ONLY control, one mode per detent. Every tick,
// modesUpdate() (src/modes.cpp) reads the debounced position, looks it up
// in this table and publishes the RESOLVED decisions (silent? night colors?
// what brightness? displays off?) to g_data.mode. Consumers read only those
// flags, never the raw position - so remapping the dial is an edit to this
// table and nothing else.

enum GaugeMode : uint8_t
    {
    MODE_BRIGHT = 0,   // full brightness
    MODE_MEDIUM,       // mid brightness
    MODE_DARK,         // minimum brightness + night color scheme
    MODE_DEBUG,        // dense all-parameters pages on the three screens
                       // (gauge_render.cpp, gaugeRenderDebug)
    MODE_SILENT,       // no voice, no flashing - "I know, stop telling me":
                       // for a known-bad sensor that would nag all stage
    MODE_FUEL_RESET,   // the FUEL FILL detent: hold 30 s = +5 gal,
                       // hold 60 s = full-tank reset (fuel_calc.cpp)
    MODE_DISPLAYS_OFF, // every LED dark, screens black, backlights off.
                       // Audio alarms STILL play - only the light goes away.
    MODE_LIGHTSHOW,    // the car-park light show: music + every light on the
                       // car, in time (lightshow.cpp, "light show" below)
    MODE_UNASSIGNED,   // spare detents - fun themes and clock-set from the
                       // plan land on these one day
    };

static const GaugeMode ROTARY_MODE_MAP[ROTARY_POSITIONS] =
    {
    MODE_BRIGHT,       // position 0  (assumed = the 0V end of the ladder)
    MODE_MEDIUM,       // position 1
    MODE_DARK,         // position 2
    MODE_DEBUG,        // position 3
    MODE_SILENT,       // position 4
    MODE_FUEL_RESET,   // position 5
    MODE_DISPLAYS_OFF, // position 6
    MODE_UNASSIGNED,   // position 7
    MODE_UNASSIGNED,   // position 8
    MODE_LIGHTSHOW,    // position 9: two clicks from any working mode in
                       // either direction, even on a dial without end stops
    MODE_UNASSIGNED,   // position 10
    MODE_UNASSIGNED,   // position 11
    };

// --- roof switch panel: six ground-switched inputs -----------------------------------
// Circuit, per switch (KiCad sheet "Analog Inputs", J2 pins SW1..SW6, out
// to the roof switch panel; each switch closes its wire to sensor ground):
//
//   +3V3 --[4.7k]--+-- SWn_IN --> switch --> GND
//                  |
//             [100n]      [22k]
//                  |   +---/\/\/--- SWn_READ (Teensy pin)
//                 GND -+
//
// Open switch = pin HIGH (3.3 V), closed = LOW (0 V). The 22k protects the
// pin if the wire ever finds 12 V. Two firmware consequences:
//   * NO internal pull-up. The chip's pull-up is ~22k; against the 22k
//     series resistor a closed switch would sit at 1.65 V, right on the
//     logic threshold. The external 4.7k is the only pull-up.
//   * The input needs its Schmitt trigger: releasing a switch is a slow
//     edge (4.7k x 100n = 0.47 ms) that a plain CMOS input may chatter on.
//     pinMode(INPUT) leaves hysteresis off, so inputs.cpp sets the pad's
//     HYS bit itself.
//
// These six switches request the lights. The gauge does not decide what any
// of them DOES - it debounces them and broadcasts their states, and each
// PDM's dingoConfig maps bits to outputs (docs/PDM_CAN_SETUP.md).

static const uint8_t ROOF_SWITCH_COUNT = 6;
static const uint8_t PIN_ROOF_SWITCH[ROOF_SWITCH_COUNT] = { 5, 3, 42, 43, 44, 38 }; // SW1..SW6

// true = the switch is ON when it closes to ground (pin LOW). Flip an entry
// for a switch wired the other way round (ON = open), e.g. to make a broken
// wire read as "on" for that function.
static const bool ROOF_SWITCH_ON_WHEN_CLOSED[ROOF_SWITCH_COUNT] =
    { true, true, true, true, true, true };

static const uint8_t ROOF_SWITCH_DEBOUNCE_TICKS = 3; // 3 x 20 ms = stable for 60 ms

// --- roof switch panel: five pots ----------------------------------------------------
// Each pot input is the same cell as a switch input (4.7k pull-up, 100n,
// 22k into an ADC pin) with ONE wire per pot on J2 - so each pot is wired
// as a two-terminal rheostat to sensor ground, and the pin sees
// 3.3V x Rpot / (Rpot + 4700). Like the fuel sender that is ratiometric and
// linear in RESISTANCE, so inputs.cpp converts counts -> ohms -> percent of
// travel. (The schematic's note "pot channels can read switches": a switch
// on a pot input reads 0R closed and open-circuit open - see POT_OPEN_OHMS.)
//
// What each pot is FOR (assumed from the harness: PWM1 -> defrost fan,
// PWM2/3 -> seat fans, PDM-L outputs 13/14 -> coolshirt pumps). CHANGE THIS
// if the panel is laid out differently - nothing else depends on the order:
//
//   pot 1  defrost fan speed          -> fan PWM 1 + CAN
//   pot 2  driver seat fan speed      -> fan PWM 2 + CAN
//   pot 3  co-driver seat fan speed   -> fan PWM 3 + CAN
//   pot 4  driver coolshirt pump      -> CAN only (PDM-L PWMs output 13)
//   pot 5  co-driver coolshirt pump   -> CAN only (PDM-L PWMs output 14)

static const uint8_t POT_COUNT = 5;
static const uint8_t PIN_POT[POT_COUNT] = { 21, 23, 20, 16, 17 }; // POT1..POT5 = A7, A9, A6, A2, A3

static const float POT_PULLUP_OHMS = 4700.0f; // R57/R59/R61/R63/R65

// Per-pot calibration: the resistance at each end of travel. 0% is where the
// load should be OFF. MEASURE ON THE PANEL (the 1 Hz debug print shows each
// pot's ohms): 10k linear pots are ASSUMED here, wired so that fully
// counter-clockwise is 0R. If a knob works backwards, swap its two numbers.
// With a 10k pot the pin spans 0..2.24 V (0..2786 counts), ~28 counts per
// percent - plenty.
struct PotCal
    {
    float   ohmsAt0;      // resistance at the 0 % (off) end of travel
    float   ohmsAt100;    // resistance at the 100 % end
    uint8_t failsafePct;  // what to report while the pot reads open-circuit
    };

static const PotCal POT_CAL[POT_COUNT] =
    {
    //  0 %      100 %    failsafe
    { 0.0f, 10000.0f, 100 }, // pot 1 defrost: a broken wire leaves the screen clearing
    { 0.0f, 10000.0f,   0 }, // pot 2 seat fan, driver
    { 0.0f, 10000.0f,   0 }, // pot 3 seat fan, co-driver
    { 0.0f, 10000.0f,   0 }, // pot 4 coolshirt, driver
    { 0.0f, 10000.0f,   0 }, // pot 5 coolshirt, co-driver
    };

// Above this the wire is open (broken, unplugged): the pull-up drags the
// node to the rail. Three times a 10k pot's full travel, 3540 counts.
static const float POT_OPEN_OHMS = 30000.0f;
static const uint16_t POT_OPEN_MS = 500; // open this long = fault + failsafe value

// Pots are noisy, mechanical things. Three small filters keep the CAN value
// and the fan duty from dithering:
static const float POT_FILTER_TAU_S   = 0.10f; // EMA on the percent reading
static const float POT_END_DEADBAND   = 3.0f;  // within this of either end = exactly 0 / 100
static const float POT_ON_PCT         = 6.0f;  // "active" (load wanted) above this...
static const float POT_OFF_PCT        = 3.0f;  // ...and inactive again below this
static const float POT_REPORT_STEP    = 1.0f;  // the published percent only moves once the
                                               // filtered value is this far from it

// --- cabin fan PWM outputs -----------------------------------------------------------
// Three logic-level PWM signals leave the board on J2 (PWM1..PWM3) for the
// speed-control inputs of the 12V cabin fans: PWM1 -> defrost fan, PWM2 ->
// driver seat fan, PWM3 -> co-driver seat fan. The fans' POWER comes from
// PDM-L (outputs 11 and 12); the gauge only tells them how fast. So a fan
// is off because the PDM switched it off (the gauge sends each pot's
// "active" bit for exactly that), not because its PWM is at zero - most
// PWM-input fans idle at minimum speed on 0 % rather than stop.
//
// As drawn (Nov 2026) the three pins go STRAIGHT from the MicroMod to J2:
// 3.3 V push-pull, no buffer, no series resistor. High time = speed, the
// 4-wire fan convention. If an open-drain stage is ever added (the unused
// pwm.kicad_sch has three 2N7002s), the FET inverts the signal - flip
// FAN_PWM_ACTIVE_HIGH and nothing else.
//
// Each pin has a timebase to itself: pin 7 is FlexPWM1 submodule 3, pin 22
// FlexPWM4 submodule 0, pin 39 FlexPWM1 submodule 2. None is shared with
// another fan or with the backlights (FlexPWM2), so fans.cpp sets the
// frequency per pin and the 20 kHz backlight timebase is never disturbed.
// 25 kHz is the 4-wire fan standard (21-28 kHz) and sits above hearing.

static const uint8_t FAN_COUNT = 3;
static const uint8_t PIN_FAN_PWM[FAN_COUNT] = { 7, 22, 39 }; // PWM1..PWM3

// Which pot sets each fan's speed (index into PIN_POT, 0 = pot 1).
static const uint8_t FAN_POT[FAN_COUNT] = { 0, 1, 2 };

static const float FAN_PWM_HZ         = 25000.0f;
static const bool  FAN_PWM_ACTIVE_HIGH = true;

// Duty sent for pot travel just above "active" and at full travel; the pot
// maps linearly between them. Most fans stall or ignore duty below ~20 %.
// TUNE per fan on the bench. While a pot is inactive its fan gets 0 %.
static const float FAN_MIN_DUTY_PCT = 20.0f;
static const float FAN_MAX_DUTY_PCT = 100.0f;

// --- PDM output catalogue ------------------------------------------------------------
// What hangs on each output of each box (vehicle harness rev A). The alarm
// system classes PDM faults BY CONSEQUENCE, not by channel: 28 outputs
// collapse to four classes, each with one alarm row per severity. The
// 4-character tag is what the banner shows next to the status:
//
//   TRP  tripped on overcurrent and retrying
//   FLT  off and staying off (dingoFW "Fault", or stuck in overcurrent)
//   OPN  commanded on but drawing nothing (blown fuse link, unplugged load)
//
// OPN needs a load whose normal current is far above the BTS7002's ~0.5 A
// measurement floor, so only the pump and the fans get one (openAmps > 0).
// Coils idle near the floor; LED lamps sit under it.

enum LoadClass : uint8_t
    {
    LOAD_NONE = 0, // spare output: never judged
    LOAD_ENGINE,   // losing it stops the car: pump, coils, engine-switched feed
    LOAD_COOLING,  // radiator fans
    LOAD_VISION,   // what the crew sees with: lamps, wipers, defrost
    LOAD_BODY,     // everything that can wait for service
    };

struct PdmOutputDef
    {
    const char* tag;      // exactly 4 characters
    LoadClass   cls;
    float       openAmps; // OPN when On and below this many amps; 0 = no OPN check
    uint16_t    openMs;   // ...for this long
    };

static const PdmOutputDef PDM_OUTPUTS[PDM_COUNT][PDM_MAX_OUTPUTS] =
    {
    // ---- PDM-E ----
        {
        { "FAN1", LOAD_COOLING, 2.0f, 2000 }, //  1 radiator fan A (expect 20-22 A)
        { "FAN2", LOAD_COOLING, 2.0f, 2000 }, //  2 radiator fan B
        { "STRT", LOAD_BODY,    0.0f,    0 }, //  3 starter solenoid A
        { "STRT", LOAD_BODY,    0.0f,    0 }, //  4 starter solenoid B
        { "PUMP", LOAD_ENGINE,  2.0f, 1000 }, //  5 fuel pump (expect 7.9-9.1 A)
        { "COIL", LOAD_ENGINE,  0.0f,    0 }, //  6 ignition coils
        { "MAIN", LOAD_ENGINE,  0.0f,    0 }, //  7 engine switched: injectors, VANOS, ECU
        { "YLLO", LOAD_VISION,  0.0f,    0 }, //  8 yellow lamps, low
        { "YLHI", LOAD_VISION,  0.0f,    0 }, //  9 yellow lamps, high
        { "WIDE", LOAD_VISION,  0.0f,    0 }, // 10 light bar, wide
        { "BRAK", LOAD_BODY,    0.0f,    0 }, // 11 brake lights
        { "HORN", LOAD_BODY,    0.0f,    0 }, // 12 horn
        { "SPR1", LOAD_NONE,    0.0f,    0 }, // 13 spare
        { "SPR2", LOAD_NONE,    0.0f,    0 }, // 14 spare
        },
    // ---- PDM-L ----
        {
        { "LOBM", LOAD_VISION,  0.0f,    0 }, //  1 low beam
        { "HIBM", LOAD_VISION,  0.0f,    0 }, //  2 high beam
        { "SPOT", LOAD_VISION,  0.0f,    0 }, //  3 light bar, spot
        { "TRNL", LOAD_BODY,    0.0f,    0 }, //  4 turn left
        { "TRNR", LOAD_BODY,    0.0f,    0 }, //  5 turn right
        { "TAIL", LOAD_BODY,    0.0f,    0 }, //  6 tail + plate
        { "REVS", LOAD_BODY,    0.0f,    0 }, //  7 reverse
        { "WIPR", LOAD_VISION,  0.0f,    0 }, //  8 wiper low + module power
        { "WIPH", LOAD_VISION,  0.0f,    0 }, //  9 wiper high select
        { "WASH", LOAD_BODY,    0.0f,    0 }, // 10 washer
        { "DFRS", LOAD_VISION,  0.0f,    0 }, // 11 defrost fan
        { "SEAT", LOAD_BODY,    0.0f,    0 }, // 12 seat fans
        { "SHRD", LOAD_BODY,    0.0f,    0 }, // 13 coolshirt pump, driver
        { "SHRC", LOAD_BODY,    0.0f,    0 }, // 14 coolshirt pump, co-driver
        },
    };

// --- lamp indicators: which PDM outputs light which icon -------------------------------
// The four icons on the PSI/lambda screen (and night mode) used to read 12V
// trigger wires. They now read the PDMs' own output states, so an icon shows
// what the box is actually driving - a tripped lamp output reads as off.
// A turn output carries the flasher's on/off wave, so the raw state IS the
// blink (at the 100 ms resolution of the status frames).

struct PdmOutputRef
    {
    uint8_t pdm;    // PDM_E / PDM_L
    uint8_t output; // 0-based: output 1 = 0
    };

// "Headlights" = any dipped set. Also what flips the gauge to night colors.
static const PdmOutputRef LAMP_HEADLIGHTS[] = { { PDM_L, 0 },   // low beam
                                                { PDM_E, 7 } }; // yellow low
// "Brights" = any main-beam or auxiliary set.
static const PdmOutputRef LAMP_BRIGHTS[]    = { { PDM_L, 1 },   // high beam
                                                { PDM_L, 2 },   // light bar, spot
                                                { PDM_E, 8 },   // yellow high
                                                { PDM_E, 9 } }; // light bar, wide
static const PdmOutputRef LAMP_TURN_L[]     = { { PDM_L, 3 } };
static const PdmOutputRef LAMP_TURN_R[]     = { { PDM_L, 4 } };

// --- injection mode ----------------------------------------------------------------
// Sequential injection (rusEFI on the M54): each injector fires once per
// engine cycle = once per two crank revolutions -> 1 event per cycle. The
// fuel calculation (fuel_calc.cpp) needs this to turn a pulse width into a
// flow rate. (Injector DUTY is not computed by the gauge - rusEFI broadcasts
// its own figure in BASE1.)

static const float INJ_EVENTS_PER_ENGINE_CYCLE = 1.0f;

// --- alarm thresholds ---------------------------------------------------------------
// Transcribed from the "Warning Thresholds" section of the project plan -
// keep the two in sync. Values marked TUNE have no number in the plan and
// were picked as sane starting points.
//
// Each threshold has a matching hysteresis constant: an alarm that turned ON
// at a threshold only turns OFF once the value has retreated past the
// threshold by the hysteresis amount. Without this, a value hovering exactly
// at the limit would flick the alarm on and off every tick.

// The engine counts as "running" above this rpm. Gates the alarms that are
// meaningless with the engine off (oil pressure, alternator voltage, AFR) -
// otherwise you would sit in the pits staring at a red OIL LOW.
static const uint16_t ENGINE_RUNNING_RPM = 400;

// Petrol stoichiometric AFR. Only used to turn the MS dash frame's target AFR
// back into lambda, so it must equal the factor rusEFI multiplied by when it
// sent it: rusEFI's STOICH_RATIO, 14.7f (rusefi_types.h) - NOT the "stoich
// ratio" setting in the tune.
static const float AFR_STOICH = 14.7f;

// oil pressure (psi). "Ratio" = rpm / psi: pressure demand scales with rpm,
// so 4000 rpm at 30 psi (ratio 133) is marginal while idle at 15 psi is fine.
static const float OILP_LOW_WARN_PSI  = 10.0f;
static const float OILP_LOW_CRIT_PSI  = 8.0f;
static const float OILP_HIGH_WARN_PSI = 80.0f;
static const float OILP_RATIO_WARN    = 133.0f; // rpm per psi
static const float OILP_RATIO_CRIT    = 250.0f;
static const float OILP_HYST_PSI      = 2.0f;
static const float OILP_RATIO_HYST    = 15.0f;

// oil temperature (deg C, rusEFI's Oil Temperature sensor via BASE4)
static const float OILT_COLD_C = 75.0f;
static const float OILT_WARN_C = 130.0f;
static const float OILT_CRIT_C = 140.0f;

// coolant temperature (deg C)
static const float CLT_COLD_C = 70.0f;
static const float CLT_WARN_C = 100.0f;
static const float CLT_CRIT_C = 115.0f;

static const float TEMP_HYST_C = 3.0f; // shared by all temperature alarms

// lambda vs target (judged on a TWO-SECOND average of lambda - target per the
// plan; + = lean). The average and the rules for which samples may enter it
// live in mixture.cpp - see the "mixture window" block right below.
static const float LAMBDA_RICH_DELTA    = 0.10f; // this much below target -> blue info
static const float LAMBDA_LEAN_WARN     = 0.03f; // above target -> yellow
static const float LAMBDA_LEAN_CRIT     = 0.05f; // above target AND under load for the
                                                 // whole window -> red. "Load" =
static const float LAMBDA_CRIT_MIN_MAP  = 90.0f; //   MAP above this (kPa, the plan's rule)
static const float LAMBDA_CRIT_MIN_THROTTLE_PCT = 70.0f; // OR pedal/TPS at least this: a
                                                 // naturally aspirated engine at altitude
                                                 // never sees 90 kPa (barometric pressure
                                                 // is ~84 kPa at 1500 m), so MAP alone would
                                                 // disarm the red on mountain stages. Set
                                                 // above 100 to restore the MAP-only rule.
static const float LAMBDA_HYST          = 0.01f;

// --- mixture window (mixture.cpp) ---------------------------------------------------
// A wideband only tells the truth while the engine burns what the ECU
// meters. On overrun it doesn't: a fuel cut (DFCO, rev limiter) shows the
// sensor free air, and closed-throttle misfires push unburnt oxygen past it.
// An average that swallows a lift-off reads LEAN for seconds afterward - the
// old averaging turned every overrun into a yellow AFR LEAN, and into a RED
// one the moment the throttle reopened (MAP > 90 kPa with the average still
// full of overrun). So a sample only enters the window while the driver is
// asking for power, the ECU isn't cutting fuel, and the sensor has had time
// to recover from the last time that wasn't true. Full rules: mixture.cpp.
//
// Price of that honesty: once the throttle opens, a real lean condition can
// reach the alarm table no sooner than HOLDOFF + WINDOW (3 s here) plus the
// alarm's own set time.
static const uint32_t LAMBDA_AVG_WINDOW_MS    = 2000; // the plan's two seconds; a multiple of
                                                      // TICK_FAST_MS (one sample per tick)
static const float    LAMBDA_MIN_THROTTLE_PCT = 5.0f; // pedal (TPS without a pedal sensor) at or
                                                      // above this = asking for power. TUNE: keep it
                                                      // above the tune's DFCO TPS threshold
                                                      // (coastingFuelCutTps), so every fuel-cut
                                                      // lift falls below it
static const uint32_t LAMBDA_HOLDOFF_MS       = 1000; // after any excluded sample, ignore this long:
                                                      // exhaust transport + sensor lag + the intake
                                                      // fuel film refilling after a cut. TUNE from a
                                                      // log of a lift-and-reapply
static const float    LAMBDA_SENSOR_MIN       = 0.5f; // below this is not a reading: rusEFI sends
                                                      // lambda 0 while the wideband warms up or has
                                                      // failed (and a target of 0 before it has one)

// 12V system voltage (as the ECU measures it)
static const float VOLT_LOW_WARN_V  = 14.0f; // alternator should hold >= this when running
static const float VOLT_LOW_CRIT_V  = 13.2f;
static const float VOLT_HIGH_WARN_V = 15.5f; // TUNE: plan lists VLT HIGH but no number;
                                             // set vs the LTO pack's max charge voltage
static const float VOLT_HYST_V      = 0.2f;

// Second VOLT LOW trigger: the pack is DISCHARGING with the engine above
// idle. A dead alternator shows here first - the LTO pack holds the bus
// above 13.2 V for a long while as it drains, so the voltage rule alone
// would stay quiet until most of the runtime is gone. The BMS frame is
// 1 Hz, so the persistence below is about ten samples. TUNE all three from
// a log: the amps must sit above the worst brief discharge a healthy
// alternator allows (fans + lights kicking in at low rpm).
static const float    VOLT_DISCHARGE_AMPS    = 10.0f; // pack amps, + = discharge
static const uint32_t VOLT_DISCHARGE_MS      = 10000; // ...for this long
static const uint16_t VOLT_DISCHARGE_MIN_RPM = 1500;  // "above idle"

// --- PDM alarms ------------------------------------------------------------------------
// dingoFW reports its board temperature; at BOARD_TEMP_CRIT (80 C, stock)
// it latches EVERY output off until a power cycle. The gauge warns on the
// way there: yellow at latch - 15 K, red (PDM-E only: that box can stop
// the car) at latch - 5 K. If the fork moves or replaces the latch (staged
// shedding), PDM_LATCH_TEMP_C is the one number to follow it with.
// UNTESTED against a real board: nobody has measured the Shrimp board's
// temperature rise at stage load. If it runs hotter than ~65 C in a hot
// cabin the yellow will nag - bench it at 60 A before trusting these.
static const float PDM_LATCH_TEMP_C      = 80.0f;
static const float PDM_HOT_WARN_MARGIN_C = 15.0f; // yellow at 65 C
static const float PDM_HOT_CRIT_MARGIN_C = 5.0f;  // red at 75 C (PDM-E)
static const float PDM_TEMP_HYST_C       = 2.0f;

// A trip is not red; losing an engine-class output for about two seconds is.
// "Stuck" = the output spent PDM_OC_STUCK_MS of the last PDM_OC_WINDOW_MS in
// dingoFW's Overcurrent state (off, waiting out its reset time). A single
// trip with a ~1 s reset time stays under the line; an output that trips
// again the moment it retries crosses it on the third attempt.
static const uint32_t PDM_OC_WINDOW_MS = 5000;
static const uint32_t PDM_OC_STUCK_MS  = 2000;

// A trip only lasts the PDM's reset time (could be 200 ms). The TRP banner
// is held this long after each trip so the crew can actually read it.
static const uint32_t PDM_TRIP_HOLD_MS = 10000;

// battery pack cells (from the BMS)
static const float CELL_BAL_WARN_V = 0.10f; // max-min cell spread
static const float CELL_LOW_CRIT_V = 2.0f;  // any (= the minimum) cell below this
static const float CELL_V_HYST     = 0.03f;
static const float BATT_HOT_WARN_C = 50.0f; // hottest cell
static const float BATT_HOT_CRIT_C = 65.0f;
static const float BATT_COLD_C     = 5.0f;  // TUNE: informational blue only

// fuel level (percent). There are TWO estimates: the slosh-filtered sender
// and the injector-integration calculation. Either one below the warn line
// raises yellow. The red NO GAS is stricter about the calculated value: a
// calc-only critical is VETOED while a healthy sender reads above
// FUEL_SENDER_VETO_PCT - if the float insists a quarter tank is left, the
// most likely story is a missed fill gesture, not an empty tank, and a
// false red teaches the driver to ignore red. (A sender-driven critical is
// never vetoed by the calc - the sender is the physical measurement.)
static const float FUEL_WARN_PCT        = 20.0f;
static const float FUEL_CRIT_PCT        = 10.0f;
static const float FUEL_PCT_HYST        = 3.0f;
static const float FUEL_SENDER_VETO_PCT = 25.0f;

// --- audio: DFPlayer Mini -----------------------------------------------------------
// The module speaks 9600-baud serial on Serial7: pin 29 (TX7 -> module RX)
// and pin 28 (RX7 <- module TX), per the pinout table ("MP3 Serial TX/RX").
// It moved off Serial3 (pins 14/15) in the Sept 2026 pin shuffle to free
// those two analog-capable pins (they read the fuel sender and the dial
// today).
// Driver in audio.cpp.

static HardwareSerial& AUDIO_SERIAL = Serial7;

// Volume on the DFPlayer's 0..30 scale: baseline (boot greeting + yellow
// warnings) at 70%, red alarms at 85%. TUNE on the car against exhaust
// drone at stage pace.
static const uint8_t AUDIO_VOL_NORMAL = 21; // 70% of 30
static const uint8_t AUDIO_VOL_RED    = 26; // 85% of 30

// The module needs ~1.5-3 s after power-up to mount its SD card. It
// announces readiness itself (an "online" frame the driver listens for);
// this deadline is the fallback after which arbitration runs regardless,
// so a missing module still lets the serial prints show what WOULD play.
static const uint32_t AUDIO_BOOT_GRACE_MS = 4000;

// Clip end is normally the module's "track finished" notification; this
// timeout is the fallback when that frame never arrives (lost byte, or no
// module wired at all). Long enough that no real clip gets its tail talked
// over, short enough that a lost frame doesn't gag the voice for long.
static const uint32_t AUDIO_CLIP_TIMEOUT_MS = 6000;

// Ignore a "finished" frame arriving sooner than this after a play command:
// clone chips report the track a play command CUT OFF as finished, right
// after the new clip starts - that must not count as the new clip ending.
// No real voice clip is shorter than this.
static const uint32_t AUDIO_FINISH_GUARD_MS = 400;

// --- LED tachometer: WS2812B strip ---------------------------------------------------
// One 15-LED WS2812B strip (96/m, "v6" silicon = accepts our 3.3V data line
// directly), reflected onto the windshield. Driven by the WS2812Serial
// library on pin 8 ("LED Serial" in the pinout = LPUART4's TX). The library
// picks the UART from the PIN NUMBER, so it is untouched by the MicroMod
// core numbering its SerialN objects differently from the Teensy 4.0's
// (there, Serial2 is no longer the port on pins 7/8). It
// encodes the LED bitstream into serial framing and a DMA engine clocks it
// out on its own, so repainting the strip costs the superloop microseconds.
// Repainted on the 20 ms tick = 50 Hz (rpm itself updates at the rusEFI CAN
// data period - see docs/ECU_CAN_SETUP.md).
//
// Strip layout (logical index 0 = LEFT end):
//
//    [ 0  1  2 ]   [ 3  4  5  6  7  8  9 10 | 11 12 13 14 ]
//     status LEDs    tach: 8 green            4 orange
//
// All behavior/priority logic lives in led_tach.cpp; every number it uses
// lives here.

static const uint8_t PIN_LED_STRIP = 8;

static const uint8_t LED_COUNT       = 15; // the tach strip: status + tach blocks
static const uint8_t LED_STATUS_LEDS = 3;  // left block: vehicle status
static const uint8_t LED_TACH_LEDS   = 12; // right block: the tachometer

// If the strip ends up physically mounted with its input wire on the RIGHT,
// flip this instead of rewiring - it mirrors the tach strip in software.
// (Only the 15 tach LEDs mirror; the label backlights below keep their
// place at the end of the chain either way.)
static const bool LED_STRIP_REVERSED = false;

// LABEL BACKLIGHTS. More WS2812s are wired onto the END of the tach strip's
// data chain (the 16th LED onward) to light the labels under the switches
// on the roof panel and the dash. They are not part of the tach: one fixed
// color, a brightness that follows the dial, and they never flash - not
// for the shift light, not for a red alarm. A flashing label is a label
// you cannot read at the moment you are reaching for the switch.
//
// SET LED_BACKLIGHT_COUNT TO THE NUMBER ACTUALLY WIRED (0 = none). Too high
// costs nothing but 30 us of frame per LED; too low leaves the extra LEDs
// showing whatever they powered up with.
//
// Because they share the tach's data line they are re-sent with every tach
// frame (50 Hz), but their VALUES only change when the dial or night mode
// does. They must be the same protocol as the tach strip (3-byte, GRB
// order): an RGBW or RGB-order part will show wrong colors.
//
// 5V budget: one color at these levels is 3-12 mA per LED, so a dozen
// labels cost under 150 mA. Full white would be 60 mA each - the reason the
// colors below are single-channel-dominant.
static const uint8_t LED_BACKLIGHT_COUNT = 12; // PLACEHOLDER - set to the build
static const uint8_t LED_TOTAL_COUNT     = LED_COUNT + LED_BACKLIGHT_COUNT;

static const uint32_t LED_COLOR_BACKLIGHT_DAY   = 0xFFFFFF; // white
static const uint32_t LED_COLOR_BACKLIGHT_NIGHT = 0xFF5000; // the tach's night orange:
                                                            // no blue for dark-adapted eyes

// Backlight brightness per dial mode (0..255), separate from the tach's:
// labels want to be readable, not eye-catching. Every other mode (Silent,
// Fuel Reset, Debug, unassigned) uses MEDIUM; Displays Off turns them off
// with everything else. TUNE on the car at night.
static const uint8_t LED_BACKLIGHT_BRIGHT = 120;
static const uint8_t LED_BACKLIGHT_MEDIUM = 50;
static const uint8_t LED_BACKLIGHT_DARK   = 12;

// Tach scale: tach LED k (k = 0..11) lights once rpm reaches
//     TACH_START_RPM + k * TACH_STEP_RPM
// -> first LED at 3500, last LED at 3500 + 11*375 = 7625 rpm.
static const uint16_t TACH_START_RPM = 3500;
static const uint16_t TACH_STEP_RPM  = 375;

// The first this-many tach LEDs are green, the rest orange.
static const uint8_t TACH_GREEN_LEDS = 8;

// Shift light: at this rpm the whole 12-LED tach flashes - blue by day,
// orange at night (a blue flare off the windshield wrecks night vision).
static const uint16_t SHIFT_FLASH_RPM = 7800;

// Staged warmup rev limits, keyed on OIL temperature - viscosity is what
// we are protecting, and oil warms far slower than coolant. Tach LEDs whose
// rpm sits above the current stage's limit show solid blue: the "cold band"
// of a factory BMW tachometer, shrinking as the oil warms. Display-only -
// the ECU enforces nothing from here.
//
// The table must stay ordered coldest -> warmest. A stage is entered the
// moment oil temp reaches minOilTempC, but only left downward again once
// the temp has fallen WARMUP_STAGE_HYST_C below it, so a reading dithering
// on a boundary can't flick an LED between blue and live.
struct WarmupStage
    {
    float    minOilTempC; // stage applies from this oil temperature upward
    uint16_t rpmLimit;    // LEDs above this rpm show the blue block
    };

static const WarmupStage WARMUP_STAGES[] =
    {
    { -999.0f, 4000 }, // cold (below 50 C)
    {   50.0f, 5000 }, // warmup
    {   70.0f, 6500 }, // nearly there
    {   80.0f, 8000 }, // full send - above the top LED, so no blue at all
    };
static const uint8_t WARMUP_STAGE_COUNT  = sizeof(WARMUP_STAGES) / sizeof(WARMUP_STAGES[0]);
static const float   WARMUP_STAGE_HYST_C = 3.0f;

// Colors as 0xRRGGBB. Note the two oranges are red-heavy mixes: an even
// red+green "orange" reads as yellow once it bounces off glass.
static const uint32_t LED_COLOR_TACH_GREEN  = 0x00FF00;
static const uint32_t LED_COLOR_TACH_ORANGE = 0xFF5000;
static const uint32_t LED_COLOR_WARMUP_BLUE = 0x0000FF;
static const uint32_t LED_COLOR_SHIFT_DAY   = 0x0000FF; // blue
static const uint32_t LED_COLOR_SHIFT_NIGHT = 0xFF5000; // orange
static const uint32_t LED_COLOR_STATUS_OK   = 0x00FF00; // the single green
static const uint32_t LED_COLOR_STATUS_WARN = 0xFFD000; // yellow, x3
static const uint32_t LED_COLOR_ALERT_RED   = 0xFF0000;

// Global brightness (0..255), applied to every color above at paint time.
// One level per brightness mode on the dial; every OTHER mode (Silent,
// Fuel Reset, Debug, unassigned) defaults to MEDIUM. TUNE all three on the
// car - and note perception is nonlinear: 140/255 looks much brighter than
// "half", which is why DARK sits so low.
static const uint8_t LED_BRIGHTNESS_BRIGHT = 255;
static const uint8_t LED_BRIGHTNESS_MEDIUM = 140;
static const uint8_t LED_BRIGHTNESS_DARK   = 60;

// Flash half-periods (light is ON this long, then OFF this long). Keep them
// multiples of TICK_FAST_MS: the strip repaints on the 20 ms tick, so a
// half-period that isn't a tick multiple flashes visibly unevenly.
static const uint32_t LED_RED_FLASH_HALF_MS   = 240; // ~2 Hz whole-gauge red
static const uint32_t LED_SHIFT_FLASH_HALF_MS = 60;  // ~8 Hz shift light

// --- display backlights: low-side N-FET switches, phase-staggered PWM -----------------
// Circuit, per screen (Schematic Ref.pdf, sheet 2; the schematic's AO3400A
// and the BOM's AOSS32334C are interchangeable SOT-23 logic-level parts):
//
//   +5V --> backlight LED (3.0 V, 5 chips in parallel) --> [R8 21R] --> drain
//   DisplayN.PWM (Teensy pin) --[R3 470R]--+-- gate           source --> GND
//                                          |
//                                    [R2 100k]
//                                          |
//                                         GND
//
// A HIGH pin turns the FET on and the LED lights: (5 - 3.0 - ~0) / 21R =
// ~95 mA, the panel's rated current, set by the ballast resistor. A fixed
// current while on means light output tracks duty cycle linearly - the
// same physics as the WS2812 strip, so the three SCREEN_BRIGHTNESS_*
// levels can mirror the LED ones. The 100k keeps the gate low (backlights
// OFF) while the Teensy is resetting, until backlightInit() takes over.
//
// 3.3 V gate drive is plenty: the AOSS32334C's Vgs(th) is 1.3-2.3 V and
// its transfer curve carries amps at Vgs = 3 V (datasheet fig. 2); we ask
// for 0.1 A, so the FET sits deep in its ohmic region and drops millivolts.
// Ciss ~600 pF / Qg ~6 nC through 470R gives edges of a microsecond or two.
//
// PIN TOPOLOGY - why these three pins. The i.MX RT's FlexPWM modules are
// split into four submodules, each with its own counter and an A/B output
// pair, and a counter can only be locked to another submodule of the SAME
// module. So all three backlights sit on FlexPWM2: pin 9 is submodule 2
// channel B, pins 33 and 4 are submodule 0 channels B and A. Submodule 0
// is the one that can broadcast its sync to the others; backlight.cpp
// slaves submodule 2 to it, so both counters run in lockstep and each
// window can be placed at a chosen phase. (Before the rev 2 board the
// backlights were pins 2/3/22 on FlexPWM4, the same arrangement one module
// over. The QuadTimer pins - 10-15, 18, 19 - can't do any of this.)
// IF THESE PINS EVER CHANGE, the k_chan table and BL_PWM in backlight.cpp
// must change with them.

static const uint8_t PIN_BACKLIGHT[3] = { 9, 33, 4 }; // Display 1, 2, 3

// PHASE STAGGER. With analogWrite() every channel's ON window starts at
// counter zero, so all three FETs switch on in the same nanosecond and the
// 5V rail sees a 300 mA step per cycle. Offsetting the windows by a third
// of a period each turns that into three separate 100 mA steps: below 33%
// duty the loads never overlap at all, above it the overlap grows, but the
// STEP the rail sees is always one backlight's worth. That is what "lower
// bus capacitance" is about - the bulk cap has to ride out I x t_on / dV,
// and both the I (100 vs 300 mA) and the t_on (see the frequency note)
// just got smaller. Fractions of the PWM period; set all three to 0 to get
// the old aligned behavior back for an A/B comparison on the scope.
static const float BL_PHASE[3] = { 0.0f, 1.0f / 3.0f, 2.0f / 3.0f };

// PWM FREQUENCY. The BCR405 driver this design replaced wanted 1 kHz (a
// 10 us turn-off delay ate into every cycle); the FET switches in ~1 us and
// has no such constraint, and everything else argues for going well up:
//   * Rail ripple: the 5V bulk cap sees each backlight as I x t_on. At
//     1 kHz that is 100 mA for up to 1 ms = 4.5 V of droop across 22 uF,
//     i.e. the regulator has to supply every pulse itself. At 20 kHz it is
//     50 us = 0.23 V on the same cap, so the 3 x 22 uF already on the
//     board absorb the switching and the TSR 2-2450 sees a DC load. (Its
//     600 uF load-capacitance limit means bulk was never the fix at 1 kHz.)
//   * Peripheral vision: 1 kHz PWM is invisible to a fixed stare but shows
//     as a dotted trail ("phantom array") whenever the eye sweeps across
//     the panel - exactly how this gauge is read, from a vibrating car.
//     Above a few kHz the effect is gone.
//   * Audible noise: X7R caps sing at the PWM rate under a 100 mA chopped
//     load; 1 kHz is the ear's most sensitive band, 20 kHz is outside it.
//   * Cost: ~10 mW of switching loss per FET, 7500 duty steps per period
//     at the 150 MHz bus clock, and a 2 us edge is 4% of a 50 us period
//     (rise and fall cancel to first order). DARK mode's 23% is 12 us of
//     on-time - nowhere near the edges.
// Anything from ~5 to ~30 kHz would work; 20 kHz is the audibility line.
static const float BL_PWM_HZ = 20000.0f;

// PWM polarity. true = a HIGH pin turns the backlight ON - the low-side
// N-FET above. If a board revision ever moves to a high-side P-FET (pin LOW
// = on), flip this instead of respinning: it inverts in software.
static const bool BL_ACTIVE_HIGH = true;

// Brightness per rotary mode, 0..255 = fraction of full output - the SAME
// perceptual scale as the LED tach above, on purpose: both the WS2812s and
// a PWMed constant-current backlight are linear duty-to-light, so the same
// numbers produce the same perceived bright/medium/dark steps on both.
// Displays-off mode forces 0 (handled in backlight.cpp, not a level here).
static const uint8_t SCREEN_BRIGHTNESS_BRIGHT = 255;
static const uint8_t SCREEN_BRIGHTNESS_MEDIUM = 140;
static const uint8_t SCREEN_BRIGHTNESS_DARK   = 60;

// Per-screen trim, 0.00..1.00, multiplied onto the mode level. This is the
// binning knob: LED batches differ in efficiency, so three "identical"
// panels at identical duty can visibly mismatch. TUNE ON THE BENCH: set
// all three to 1.00, put the gauge in Bright, and dim the brighter screens
// down until all three match by eye (the 1 Hz debug print shows the final
// duty each screen is actually getting). Trim only ever REDUCES a screen -
// the dimmest panel stays at 1.00 and defines full brightness.
static const float BL_TRIM[3] = { 1.00f, 1.00f, 1.00f };

// Mode changes fade over this long rather than snapping (a 0->full step at
// night is a flashbang to dark-adapted eyes). Full-scale traverse time;
// smaller steps take proportionally less. Also gives a gentle fade-in on
// boot. 0 = snap instantly.
static const uint32_t BL_FADE_MS = 250;

// --- screens: 3x ER-TFT2.79-1 (NV3007) over SPI ---------------------------------------
// Each display sits alone on its own SPI bus, so all three DMA uploads can
// run at the same time. Display 1 is on SPI2, display 2 on SPI1, display 3
// on SPI (displays 1 and 3 traded buses on the rev 2 board).
// Note every DC pin is a HARDWARE CS pin of its bus - that is deliberate:
// the ILI9341_T4 driver toggles DC mid-stream via the SPI peripheral itself
// (no CPU interrupt per command) only when DC is on a hardware CS pin, and
// falls back to a much slower interrupt-per-toggle mode otherwise. The CS
// pins themselves carry no such requirement - any GPIO works.
//
// MISO is 255 everywhere: the NV3007 has no MISO line at all (write-only
// display), and the port forces this in begin() - see PORTING_NV3007.md.

struct ScreenPins
    {
    uint8_t cs, dc, sck, mosi, rst;
    };

static const ScreenPins SCREEN_PINS[3] =
    {
    {  6, 36, 37, 35, 45 }, // display 1 on SPI2 (LPSPI1, the SD pads)
    {  2,  0, 27, 26, 32 }, // display 2 on SPI1 (LPSPI3)
    { 25, 10, 13, 11, 24 }, // display 3 on SPI  (LPSPI4)
    };

// SPI clock. The NV3007 datasheet rates writes to 100 MHz; the practical
// ceiling is the Teensy + wiring, typically 40-60 MHz. 40 MHz is the proven
// safe start (full frame = ~24 ms; typical differential gauge frames are a
// few percent of that). Try 60 MHz once the real harness exists - if the
// image corrupts, come back down.
static const uint32_t SCREEN_SPI_HZ = 40000000;

// Native landscape geometry (rotation 0), as mounted. See PORTING_NV3007.md
// for why 428x142 is the native orientation and not a rotation.
static const int SCREEN_LX = 428;
static const int SCREEN_LY = 142;

// Diff buffers: the library compares each new frame against what is already
// on the glass and uploads only changed pixels. 8 KB per buffer is generous
// for gauge content (mostly-static frames); two per display lets the next
// diff compute while the previous upload still streams. If printStats()
// ever shows diff overflows, raise this.
static const int SCREEN_DIFF_BUF_BYTES = 8192;

// Diff gap: pixel runs shorter than this between two changed regions get
// merged into one upload. Small = tighter uploads but more command
// overhead; the library docs and the bring-up test both land on ~6-10 for
// this class of content.
static const int SCREEN_DIFF_GAP = 6;

// What each physical display shows. Swap entries if the cluster ends up
// mounted in a different order - nothing else in the firmware cares.
enum ScreenContent : uint8_t
    {
    SCREEN_OIL_VOLTS = 0, // oil temp + volts + the warning banner
    SCREEN_CLT_FUEL,      // coolant temp + fuel % / bar / clock
    SCREEN_PSI_LAMBDA,    // oil pressure + lambda + turn/light icons
    };

static const ScreenContent SCREEN_CONTENT[3] =
    {
    SCREEN_OIL_VOLTS,  // display 1
    SCREEN_CLT_FUEL,   // display 2
    SCREEN_PSI_LAMBDA, // display 3
    };

// The fuel filler arrow (the "Moylan arrow") beside the FUEL label: the
// little arrow every OEM fuel gauge tucks next to the pump symbol so you
// never pull up on the wrong side of the pump. true = filler on the
// passenger/right side (the E30's flap is on the right rear quarter) - the
// arrow sits to the right of the label and points right; false = left.
static const bool FUEL_FILLER_ARROW_RIGHT = true;

// The warning banner flashes on any CRITICAL (red) alarm: 500 ms on,
// 500 ms off = the plan's 1 Hz eye-catcher. Must stay a multiple of
// TICK_FAST_MS (frames repaint on the tick, so a non-multiple half-period
// renders unevenly). Silent mode disables flashing (banner goes solid).
static const uint32_t SCREEN_RED_FLASH_HALF_MS = 500;

// --- screen colors, 0xRRGGBB (same convention as the LED colors above) ----------------
// These get packed to RGB565 (5 bits red / 6 green / 5 blue) at theme-build
// time, once per tick. Values match the bench-tested PC prototype in
// ../Graphics/GaugeClusterPrototype.h; anything marked TUNE was added for
// the alarm-coloring feature and has not met real eyeballs yet.
//
// "Info blue" is SEMANTIC: it is whatever color means "cold / informational"
// right now - dodger blue by day, amber at night (night vision: a bright
// blue element is the single worst color for dark-adapted eyes, same
// reasoning as the LED shift light's night color). Nothing below ever asks
// for "blue" - it asks for COL_INFO and the theme answers.

static const uint32_t COL_SCREEN_BG      = 0x42314A; // = raw565(8,12,9), the prototype bg
static const uint32_t COL_SCREEN_GREEN   = 0x2CFF7A; // nominal readouts + labels
static const uint32_t COL_SCREEN_YELLOW  = 0xFFBE00; // TUNE: warning-state values
static const uint32_t COL_SCREEN_RED     = 0xFF4650; // TUNE: critical-state values
static const uint32_t COL_SCREEN_WHITE   = 0xFFFFFF; // text on blue/red banner fills
static const uint32_t COL_SCREEN_DARKTXT = 0x0A0F0A; // text on yellow/green banner fills
static const uint32_t COL_SCREEN_DIVIDER = 0x14321E; // separator lines + off segments/icons

static const uint32_t COL_INFO_DAY    = 0x1E90FF; // dodger blue
static const uint32_t COL_INFO_NIGHT  = 0xFF7800; // TUNE: amber replacement at night
// The brights icon keeps a dim ghost of its color when off (it sits alone
// in a row of green icons, so a green "off" state would read as ON).
static const uint32_t COL_BRIGHTS_OFF_DAY   = 0x0C2337; // dim blue
static const uint32_t COL_BRIGHTS_OFF_NIGHT = 0x3C1E00; // TUNE: dim amber

// Banner fills per alarm color (text color pairs with each: white on
// blue/red, dark on yellow/green - dark-on-yellow beats white-on-yellow
// for contrast, and red keeps white text because red + dark text muddies).
static const uint32_t COL_BANNER_YELLOW = 0xFFA500; // amber fill, dark text
static const uint32_t COL_BANNER_RED    = 0xEB232D; // red fill, white text
// (blue banner fill = COL_INFO_*, green banner fill = COL_SCREEN_GREEN)

// --- light show (lightshow.cpp, lightshow_fx.cpp) ---------------------------------------
// MODE_LIGHTSHOW: the DFPlayer plays a music track while the LED strip, the
// three screens and their backlights, and (over CAN, through the PDMs) the
// car's exterior lamps flash in time with it. For car parks, not stages.
// How it works, how to add a track and how to calibrate: docs/LIGHTSHOW.md.
//
// THE SYNC IS OPEN-LOOP. The DFPlayer reports nothing about where in a track
// it is, so the gauge starts the track, starts its own clock, and trusts
// both to run at the same speed. Two numbers line them up, both TUNE ON THE
// BENCH with the calibration click track (tools/lightshow/make_click_track.py):
static const int16_t SHOW_AUDIO_LATENCY_MS = 150;  // play command -> first sound. A GUESS:
                                                   // DFRobot publishes no figure
static const float   SHOW_TEMPO_TRIM       = 1.0f; // DFPlayer clock speed / ours. Play a file
                                                   // of known length; the [SHOW] line at the
                                                   // end prints how long the module took

// When the show may run. It starts only when the dial ARRIVES on the detent
// (never because the gauge booted there - a brownout mid-show must not loop),
// after a countdown, and ends at once on any red alarm or when the engine
// leaves idle: nothing may strobe the headlights on a moving car.
static const uint32_t SHOW_ARM_MS         = 3000; // the countdown: title on screen, dial still free
static const uint16_t SHOW_MAX_RPM        = 1500; // idling to hold the battery up is fine
static const uint32_t SHOW_BOOT_IGNORE_MS = 1000; // a dial position accepted this soon after
                                                  // boot was not an arrival
static const uint32_t SHOW_SKIP_WINDOW_MS = 4000; // off the detent and back within this = next
                                                  // track; longer away = back to track 1
static const bool     SHOW_AUTO_ADVANCE   = false;// true = play the list on a loop

static const uint8_t  SHOW_VOLUME = 26; // DFPlayer 0..30 (= AUDIO_VOL_RED)

// WS2812 current limit. A full-white strip is 60 mA per LED - 1.6 A for the
// 27 on the chain - on a 3 A rail that also feeds the backlights and the
// audio amplifier. Every show frame is scaled down to fit this budget, which
// is about what the red alarm flash plus the label backlights draw today.
static const uint16_t SHOW_LED_MAX_MA    = 800;
static const uint8_t  SHOW_LED_MA_PER_CH = 20; // one colour channel at full

// Fastest strobe on the strip and backlights, in flashes per beat (1, 2 or
// 4). 4 = sixteenth notes, 9 Hz at 136 bpm - inside the 3-30 Hz band that
// photosensitive-epilepsy guidance warns about. Set 1 for a public event.
// (The exterior lamps never exceed one flash per beat per lamp regardless.)
static const uint8_t SHOW_MAX_FLASH_DIV = 4;

// EXTERIOR LAMPS. One bit per lamp group, sent in GAUGE_ID_SHOW bytes 0-1.
// The bits do nothing until each PDM output ORs its bit in (the dingoConfig
// recipe is in docs/PDM_CAN_SETUP.md), so the show can only ever ADD light:
// the roof switches and the fail-on lamp logic are untouched by it.
// false = never send a lamp bit, whatever the PDMs are configured to do.
static const bool SHOW_DRIVE_LAMPS = true;

enum ShowLamp : uint16_t
    {
    SHOW_LAMP_LOW      = 1 << 0,  // PDM-L  1 low beam
    SHOW_LAMP_HIGH     = 1 << 1,  // PDM-L  2 high beam
    SHOW_LAMP_BAR_SPOT = 1 << 2,  // PDM-L  3 light bar, spot
    SHOW_LAMP_TURN_L   = 1 << 3,  // PDM-L  4 turn left
    SHOW_LAMP_TURN_R   = 1 << 4,  // PDM-L  5 turn right
    SHOW_LAMP_TAIL     = 1 << 5,  // PDM-L  6 tail + plate (running lights)
    SHOW_LAMP_REVERSE  = 1 << 6,  // PDM-L  7 reverse
    SHOW_LAMP_YEL_LO   = 1 << 8,  // PDM-E  8 yellow lamps, low
    SHOW_LAMP_YEL_HI   = 1 << 9,  // PDM-E  9 yellow lamps, high
    SHOW_LAMP_BAR_WIDE = 1 << 10, // PDM-E 10 light bar, wide
    SHOW_LAMP_BRAKE    = 1 << 11, // PDM-E 11 brake lights
    };

// THE TRACK LIST. Track 1 is what the detent plays; the rest are reached by
// flicking the dial off and back during the countdown. One row per track:
//
//   file     the DFPlayer plays /mp3/NNNN*.mp3 (0101 up: clear of the voice clips)
//   name     shown during the countdown (the banner font: capitals and digits)
//   bpm      tempo. Must be right to four figures - 0.1 bpm off is a tenth
//            of a beat after one minute and over a third by the end of 3:45
//   beat1Ms  where in THE FILE the first downbeat falls (depends on the rip)
//   seconds  track length: the fallback end if the "finished" frame is lost
//   map      what the lights do, one letter per 4-bar phrase, repeated if the
//            track outlasts it:
//              p pulse   colour flash on every beat          (verse, intro)
//              c chase   comet on the strip, wig-wag lamps   (groove)
//              b build   strobe that accelerates; a run of b's is one build,
//                        and its last beat goes dark         (riser)
//              d drop    everything, on the beat             (the drop)
//              k break   slow colour drift, nothing flashes  (breakdown)
//              - dark
//            nullptr = SHOW_DEFAULT_MAP, a generic arc that suits most dance
//            tracks - so a new track needs only its tempo and first beat.
//
// tools/lightshow/lightshow_sim.html plays a row against the real audio file
// and finds bpm and beat1Ms for it.
struct ShowTrack
    {
    uint16_t    file;
    const char* name;
    float       bpm;
    int16_t     beat1Ms;
    uint16_t    seconds;
    const char* map;
    };

static const char* const SHOW_DEFAULT_MAP = "ppccbbddddkkbbdddd";

static const ShowTrack SHOW_TRACKS[] =
    {
    // Darude - Sandstorm, radio edit (3:45, 136 bpm). THE MAP AND beat1Ms ARE
    // PLACEHOLDERS: written from the track's general shape, not measured
    // against a file. Align them in the simulator before trusting them.
    { 101, "SANDSTORM", 136.0f, 0, 225, "bbddddccddkkbbddddccddddddccppk-" },
    // The bench calibration click (tools/lightshow/make_click_track.py).
    { 100, "CLICK 120", 120.0f, 0,  60, "dddddddd" },
    };
static const uint8_t SHOW_TRACK_COUNT = sizeof(SHOW_TRACKS) / sizeof(SHOW_TRACKS[0]);

static const uint8_t SHOW_PHRASE_BEATS = 16; // 4 bars of 4

// --- FRAM: FM24V10 1-Mbit I2C F-RAM ---------------------------------------------------
// Nonvolatile memory for two jobs: the persistent fuel record (survives the
// kill switch) and the black-box flight recorder (backup to the ECU's own
// log). On the Wire pins per the pinout table: 18 = SDA, 19 = SCL.
// A1/A2/WP all have internal pulldowns and are left grounded -> device
// address 0x50 (+1 for the upper 64 KB page). SDA/SCL each need an external
// pull-up to 3.3V: 2.2k is right for 1 MHz operation.
//
// The chip is 128 KB, writes at full bus speed with NO write delays (F-RAM,
// not EEPROM), and endures 10^14 writes - writing every second for 30 years
// uses 0.000001% of its life. Wear is a non-problem; the design worries
// about exactly one thing: POWER DYING MID-WRITE (kill switch). Every record
// therefore carries a sequence number + CRC, and nothing is ever the only
// copy of itself. See docs/FRAM_BLACKBOX.md for the full memory map.

static const uint32_t FRAM_I2C_HZ      = 1000000; // FM24V10 F/S-mode tops out at 1 MHz
static const uint8_t  FRAM_I2C_ADDR    = 0x50;    // 1010 A2 A1 (A2=A1=0); bit0 selects the page
static const uint32_t FRAM_SIZE_BYTES  = 131072;  // 128 KB

// Async write-queue geometry (fram.cpp). Worst case per 1 s tick is three
// writes (log record + head pointer + fuel record), so depth 8 is generous.
static const uint8_t  FRAM_QUEUE_DEPTH     = 8;
static const uint8_t  FRAM_MAX_WRITE       = 40;  // largest single record (the log's)
static const uint32_t FRAM_XFER_TIMEOUT_MS = 5;   // one 42-byte transfer takes ~0.4 ms;
                                                  // 5 ms of silence = the bus is wedged
static const uint8_t  FRAM_MAX_CONSEC_ERRS = 8;   // give up (mark chip absent) after this
                                                  // many back-to-back failures

// Memory map. Everything below FRAM_ADDR_LOG is bookkeeping (~1 KB); the
// rest of the chip - about 127 KB - is the black-box ring.
static const uint32_t FRAM_ADDR_HEADER = 0x0000; // 32 B  format marker + geometry
static const uint32_t FRAM_ADDR_FUEL   = 0x0020; // 16 x 32 B fuel record ring
static const uint8_t  FRAM_FUEL_SLOTS  = 16;
static const uint32_t FRAM_ADDR_LOGPTR = 0x0220; // 2 x 16 B log head pointer (ping-pong)
static const uint32_t FRAM_ADDR_LOG    = 0x0400; // black-box ring, to end of chip

// Black-box cadence. Records are written on the 1 s tick while the engine
// runs (or a real alarm is active - see fram_log.cpp for the gating), every
// LOG_NOMINAL_EVERY_S seconds normally and every LOG_ALARM_EVERY_S once any
// yellow/red alarm is up. At 40 B/record the ~127 KB ring holds 3251
// records: ~1.8 h of nominal running, ~54 min if an alarm is active the
// whole time. After the engine stops (or the alarm clears) logging continues
// for LOG_GRACE_MS - the aftermath of a stall is exactly the data you want -
// then halts so a car left sitting with the ignition on doesn't grind away
// the interesting part of the ring.
static const uint32_t LOG_NOMINAL_EVERY_S = 2;
static const uint32_t LOG_ALARM_EVERY_S   = 1;
static const uint32_t LOG_GRACE_MS        = 30000;

// --- watchdog -------------------------------------------------------------------------
// Hardware watchdog timeout. The 20 ms tick feeds it; if the firmware ever
// hangs for this long the chip hard-resets and the gauge is back in ~1 s.
// Must be a multiple of 500 ms (hardware granularity).

static const uint32_t WATCHDOG_TIMEOUT_MS = 1000;
