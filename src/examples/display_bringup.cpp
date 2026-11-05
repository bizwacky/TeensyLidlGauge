// ---------------------------------------------------------------------------
// Rally CAN Gauge - display bring-up test
// ---------------------------------------------------------------------------
// First-light sketch for ONE ER-TFT2.79-1 (NV3007) on the Teensy 4.0, using
// the modified ILI9341_T4 library in lib/ILI9341_T4 (see PORTING_NV3007.md).
//
// Wiring (display 1 / "SPI0" from the project plan pinout):
//
//   display pin        Teensy 4.0 pin
//   -----------        --------------
//   1  SDA      ->     11  (SPI0 MOSI)
//   2  SCL      ->     13  (SPI0 SCK)
//   3  RS  (DC) ->     10  (hardware CS pin -> big speedup, see library docs)
//   4  RSTB     ->      6
//   5  CS       ->      9
//   6  TE       ->     (unconnected for now - future tear-free sync)
//   7  VDD      ->     3.3V
//   8  LEDK     ->     21R ballast -> N-FET drain; FET gate on pin 2 (PWM)
//   9  LEDA     ->     +5V
//                      (the gauge PCB's backlight switch - circuit drawn at
//                       PIN_BACKLIGHT in src/config.h)
//   10 GND      ->     GND
//
// What you should see, and what each element verifies:
//
//   1. A 1-pixel white border around the whole screen.
//        -> verifies the GRAM window offsets: if any edge of the border is
//           missing or you see a stripe of garbage, the offsets are off.
//   2. Three bars labeled by position: RED (left), GREEN (middle), BLUE (right).
//        -> verifies color order: if red and blue are swapped, the panel wants
//           the BGR bit - change ILI9341_T4_MADCTL_VALUE 0x60 -> 0x68.
//   3. A small box bouncing around with a moving gradient.
//        -> verifies the differential-update + DMA path: motion should be
//           smooth and the FPS counter (top-right) should sit at ~50.
//
// Open the serial monitor (115200) for init messages and upload statistics.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <ILI9341_T4.h>

// --- pins -------------------------------------------------------------------

static const uint8_t PIN_CS = 9;
static const uint8_t PIN_DC = 10;   // on a hardware CS pin -> accelerated DC
static const uint8_t PIN_SCK = 13;
static const uint8_t PIN_MOSI = 11;
static const uint8_t PIN_MISO = 255; // the NV3007 has no MISO line
static const uint8_t PIN_RST = 6;
static const uint8_t PIN_BACKLIGHT = 2; // PWM

// --- display geometry ---------------------------------------------------------
// Rotation 0 is native landscape 428x142, matching the mounted orientation of
// the gauges (the glass is physically a portrait bar; the controller's MADCTL
// MV bit, set during init, is its hardware landscape mode - see
// lib/ILI9341_T4/PORTING_NV3007.md).

static const int LX = 428; // width in rotation 0
static const int LY = 142; // height in rotation 0

// --- the driver and its memory ----------------------------------------------

ILI9341_T4::ILI9341Driver tft(PIN_CS, PIN_DC, PIN_SCK, PIN_MOSI, PIN_MISO, PIN_RST);

// The framebuffer we draw into (121 KB). DMAMEM places it in the 512 KB OCRAM
// region instead of the fast-but-small DTCM, keeping DTCM free for variables.
DMAMEM uint16_t fb[LX * LY];

// Internal framebuffer for the library: enables double buffering, i.e.
// update() copies our fb and returns immediately while DMA streams it out.
DMAMEM uint16_t internal_fb[LX * LY];

// Two diff buffers: the library compares the new frame against what is already
// on screen and uploads only the pixels that changed. 8 KB each is generous.
ILI9341_T4::DiffBuffStatic<8192> diff1;
ILI9341_T4::DiffBuffStatic<8192> diff2;

// --- tiny drawing helpers (a real graphics lib like tgx comes later) ---------

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
    {
    // pack 8-bit channels into 16-bit 565: RRRRRGGGGGGBBBBB
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }

void fillRect(int x0, int y0, int w, int h, uint16_t color)
    {
    if (x0 < 0) { w += x0; x0 = 0; }
    if (y0 < 0) { h += y0; y0 = 0; }
    if (x0 + w > LX) w = LX - x0;
    if (y0 + h > LY) h = LY - y0;
    for (int y = y0; y < y0 + h; y++)
        {
        uint16_t* p = fb + x0 + y * LX;
        for (int x = 0; x < w; x++) *p++ = color;
        }
    }

// --- test pattern state -------------------------------------------------------

int boxX = 40, boxY = 40;   // bouncing box position
int velX = 3, velY = 2;     // and velocity
uint32_t frameCount = 0;
elapsedMillis statsTimer;

void setup()
    {
    Serial.begin(115200);
    delay(500); // give the serial monitor a moment to attach (not required)

    // backlight on, full brightness (will be PWM-dimmed in the real gauge)
    pinMode(PIN_BACKLIGHT, OUTPUT);
    analogWrite(PIN_BACKLIGHT, 255);

    tft.output(&Serial);          // send driver debug/info output to USB serial
    while (!tft.begin(30000000))  // 30 MHz SPI: safe bring-up speed. The NV3007
        {                         // is rated to 100 MHz writes; try 40-60 MHz
        delay(1000);              // once the wiring is proven.
        }

    tft.setRotation(0);                    // native landscape 428x142, as mounted
    tft.setFramebuffer(internal_fb);       // enable double buffering
    tft.setDiffBuffers(&diff1, &diff2);    // enable differential updates
    tft.setDiffGap(6);                     // see PORTING_NV3007.md / library docs
    tft.setVSyncSpacing(0);                // no vsync possible (no MISO): 0 means
                                           // "never drop frames, wait if needed"
    }

void loop()
    {
    // ---- redraw the whole test pattern into our framebuffer ----
    // (Redrawing everything each frame is fine: the diff engine finds out
    //  by itself that only the moving parts actually need uploading.)

    // background
    fillRect(0, 0, LX, LY, rgb565(0, 0, 0));

    // (1) 1-pixel white border to verify the GRAM window offsets
    fillRect(0, 0, LX, 1, 0xFFFF);
    fillRect(0, LY - 1, LX, 1, 0xFFFF);
    fillRect(0, 0, 1, LY, 0xFFFF);
    fillRect(LX - 1, 0, 1, LY, 0xFFFF);

    // (2) color-order bars: must read RED, GREEN, BLUE from left to right
    fillRect(30, 20, 100, 30, rgb565(255, 0, 0));
    fillRect(164, 20, 100, 30, rgb565(0, 255, 0));
    fillRect(298, 20, 100, 30, rgb565(0, 0, 255));

    // (3) moving gradient strip (exercises many changing pixels)
    for (int x = 0; x < LX; x++)
        {
        uint8_t v = (uint8_t)((x * 255) / LX + frameCount * 2);
        uint16_t c = rgb565(v, 255 - v, 128);
        for (int y = 70; y < 82; y++) fb[x + y * LX] = c;
        }

    // (4) bouncing box (exercises the diff on a small moving region)
    boxX += velX;
    boxY += velY;
    if (boxX < 2 || boxX > LX - 22) { velX = -velX; boxX += velX; }
    if (boxY < 2 || boxY > LY - 22) { velY = -velY; boxY += velY; }
    fillRect(boxX, boxY, 20, 20, rgb565(255, 200, 0));

    // FPS counter drawn onto the framebuffer by the library (top-right)
    tft.overlayFPS(fb);

    // push the frame: returns quickly, upload continues via DMA in background
    tft.update(fb);

    frameCount++;

    // print upload statistics every 5 seconds
    if (statsTimer > 5000)
        {
        statsTimer = 0;
        tft.printStats();
        }

    // ~50 fps pacing, same cadence the real gauge loop will use (20 ms tick)
    static elapsedMicros frameTimer;
    while (frameTimer < 20000) { /* wait */ }
    frameTimer -= 20000;
    }
