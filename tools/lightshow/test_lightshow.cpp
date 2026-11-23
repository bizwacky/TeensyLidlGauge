// ---------------------------------------------------------------------------
// Light show - host tests
// ---------------------------------------------------------------------------
// The show's two source files compiled for the PC, with this folder's
// Arduino.h standing in for the Teensy core and a clock the tests move by
// hand. Two halves:
//
//   choreography   every track in SHOW_TRACKS is rendered start to finish
//                  in 5 ms steps and checked against the promises made in
//                  src/lightshow_fx.cpp - the LED current budget, how fast
//                  an exterior lamp may flash, that the two PDMs' forward
//                  lamps never switch on together, how often a screen may
//                  be repainted.
//   state machine  the dial gesture, countdown, track skip and every way a
//                  show ends (src/lightshow.cpp), against stubbed audio.
//
// Run by build.sh. Two more modes feed the simulator build:
//   --json     the track table and constants, for the page
//   --frames   a few thousand rendered frames, which build.sh replays
//              through the JavaScript engine to prove the two agree
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Arduino.h"
#include "audio.h"
#include "config.h"
#include "lightshow.h"
#include "lightshow_fx.h"
#include "modes.h"
#include "vehicle_data.h"

// --- the stand-ins ---------------------------------------------------------------

HardwareSerial Serial, Serial7;

static uint32_t s_now = 0;
uint32_t millis() { return s_now; }

static int      s_armCalls, s_playCalls, s_stopCalls;
static uint16_t s_lastFile;
static bool     s_finished, s_audioReady = true;
bool audioMusicReady()             { return s_audioReady; }
void audioMusicArm()               { s_armCalls++; }
void audioMusicPlay(uint16_t file) { s_playCalls++; s_lastFile = file; }
void audioMusicStop()              { s_stopCalls++; }
bool audioMusicFinished()          { return s_finished; }

static int s_failures = 0;
#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        if (!(cond)) {                                                        \
            s_failures++;                                                     \
            printf("  FAIL %s:%d  %s  - ", __FILE__, __LINE__, #cond);        \
            printf(__VA_ARGS__);                                              \
            printf("\n");                                                     \
        }                                                                     \
    } while (0)

// --- choreography ----------------------------------------------------------------

static const uint16_t k_lampBits[] =
    {
    SHOW_LAMP_LOW, SHOW_LAMP_HIGH, SHOW_LAMP_BAR_SPOT, SHOW_LAMP_TURN_L, SHOW_LAMP_TURN_R,
    SHOW_LAMP_TAIL, SHOW_LAMP_REVERSE, SHOW_LAMP_YEL_LO, SHOW_LAMP_YEL_HI,
    SHOW_LAMP_BAR_WIDE, SHOW_LAMP_BRAKE,
    };
static const char* const k_lampNames[] =
    {
    "low", "high", "barSpot", "turnL", "turnR", "tail", "reverse", "yelLo", "yelHi",
    "barWide", "brake",
    };
static const int k_lampCount = sizeof(k_lampBits) / sizeof(k_lampBits[0]);

static const uint16_t k_forwardL = SHOW_LAMP_LOW | SHOW_LAMP_HIGH | SHOW_LAMP_BAR_SPOT;
static const uint16_t k_forwardE = SHOW_LAMP_YEL_LO | SHOW_LAMP_YEL_HI | SHOW_LAMP_BAR_WIDE;

static uint32_t ledMilliamps(const ShowFrame& f)
    {
    uint32_t sum = 0;
    for (uint32_t c : f.led) sum += (c >> 16) + ((c >> 8) & 0xFF) + (c & 0xFF);
    return sum * SHOW_LED_MA_PER_CH / 255;
    }

static void testChoreography(const ShowTrack& trk)
    {
    const int   stepMs   = 5;
    const float beatMs   = 60000.0f / trk.bpm;
    const float halfBeat = beatMs / 2 - stepMs - 1; // shortest a lamp may stay in one state

    ShowFrame f, prev = {};
    uint32_t peakMa = 0, lampEdges = 0, repaints = 0;
    int      lastChange[k_lampCount], lastRise[k_lampCount], lastPaint[3];
    for (int& v : lastChange) v = -100000;
    for (int& v : lastRise)   v = -100000;
    for (int& v : lastPaint)  v = -100000;
    bool lit = false;

    for (int t = -500; t < trk.seconds * 1000; t += stepMs)
        {
        showRender(trk, t, f);

        const uint32_t mA = ledMilliamps(f);
        if (mA > peakMa) peakMa = mA;
        CHECK(mA <= SHOW_LED_MAX_MA, "%s t=%d: strip draws %u mA", trk.name, t, mA);

        if (t < trk.beat1Ms)
            CHECK(mA == 0 && f.lamps == 0 && !(f.backlight[0] | f.backlight[1] | f.backlight[2]),
                  "%s t=%d: lit before the first beat", trk.name, t);
        if (mA || f.lamps) lit = true;

        CHECK(!((f.lamps & k_forwardL) && (f.lamps & k_forwardE)),
              "%s t=%d: both PDMs' forward lamps on (0x%03X)", trk.name, t, f.lamps);

        for (int i = 0; i < k_lampCount; i++)
            {
            const bool on = f.lamps & k_lampBits[i], was = prev.lamps & k_lampBits[i];
            if (on == was) continue;
            lampEdges++;
            CHECK(t - lastChange[i] >= halfBeat, "%s t=%d: %s changed again after %d ms",
                  trk.name, t, k_lampNames[i], t - lastChange[i]);
            if (on)
                {
                CHECK(t - lastRise[i] >= beatMs - stepMs - 1, "%s t=%d: %s flashed twice in %d ms",
                      trk.name, t, k_lampNames[i], t - lastRise[i]);
                lastRise[i] = t;
                }
            lastChange[i] = t;
            }

        for (int i = 0; i < 3; i++)
            {
            if (f.screen[i] == prev.screen[i]) continue;
            repaints++;
            CHECK(t - lastPaint[i] >= beatMs - stepMs - 1, "%s t=%d: screen %d repainted after %d ms",
                  trk.name, t, i, t - lastPaint[i]);
            lastPaint[i] = t;
            }
        prev = f;
        }
    CHECK(lit, "%s: the show never lit anything", trk.name);
    printf("  %-10s %3u s  peak strip %3u mA  %5u lamp edges  %4u screen repaints\n",
           trk.name, trk.seconds, peakMa, lampEdges, repaints);
    }

// A build must end in one dark beat, and a '-' phrase must be dark throughout.
static void testLooks()
    {
    const ShowTrack trk = { 0, "looks", 120.0f, 0, 0, "bbd-" }; // 500 ms beats, 8 s phrases
    ShowFrame f;
    showRender(trk, 15250, f); // beat 30: the second-to-last beat of the build
    CHECK(ledMilliamps(f) > 0, "the build should still be lit on its second-to-last beat");
    showRender(trk, 15750, f); // beat 31: the last one
    CHECK(ledMilliamps(f) == 0 && f.lamps == 0 && f.backlight[1] == 0, "last beat of a build not dark");
    showRender(trk, 16010, f); // beat 32: the drop lands
    CHECK(ledMilliamps(f) > 0 && f.lamps != 0, "the drop after a build is not lit");
    for (int t = 24000; t < 32000; t += 20)
        {
        showRender(trk, t, f);
        CHECK(ledMilliamps(f) == 0 && f.lamps == 0, "'-' phrase lit at t=%d", t);
        }
    // An unknown tempo-only track uses the default map, and loops it.
    const ShowTrack generic = { 0, "generic", 128.0f, 0, 0, nullptr };
    showRender(generic, 10 * 60 * 1000, f); // ten minutes in: far past one pass of the map
    CHECK(ledMilliamps(f) > 0, "the default map did not loop");
    }

// --- state machine -----------------------------------------------------------------

static uint8_t showDetent()
    {
    for (uint8_t i = 0; i < ROTARY_POSITIONS; i++)
        if (ROTARY_MODE_MAP[i] == MODE_LIGHTSHOW) return i;
    return 255;
    }

static void dial(uint8_t pos)
    {
    if (g_data.sw.rotaryPos == pos) return;
    g_data.sw.rotaryPos       = pos;
    g_data.sw.rotaryChangedMs = s_now;
    }

// One 20 ms tick of the parts of the superloop the show depends on.
static void tick(int n = 1)
    {
    for (int i = 0; i < n; i++)
        {
        s_now += TICK_FAST_MS;
        modesUpdate();
        lightshowUpdate();
        }
    }

static void reboot(uint8_t dialPos)
    {
    // lightshow.cpp keeps file-scope state; walking the dial away and back
    // with a long pause puts it where a fresh boot would be, bar s_selected.
    g_data = VehicleData{};
    s_now += 60000;
    g_data.sw.rotaryPos = 0;
    tick();
    s_armCalls = s_playCalls = s_stopCalls = 0;
    s_finished = false;
    dial(dialPos);
    }

static void testStateMachine()
    {
    const uint8_t show = showDetent();
    CHECK(show != 255, "no dial position is mapped to MODE_LIGHTSHOW");
    const ShowData& s = g_data.show;
    const int armTicks = SHOW_ARM_MS / TICK_FAST_MS;

    // Powered up already on the detent: an ordinary gauge.
    g_data = VehicleData{};
    s_now  = 60; // the dial's debounce has just accepted its position
    g_data.sw.rotaryPos       = show;
    g_data.sw.rotaryChangedMs = s_now;
    tick(armTicks * 3);
    CHECK(s.state == SHOW_OVER && !s.active && s_playCalls == 0, "a show started at boot");

    // Away and back: countdown, then the first track.
    dial(0); tick(500);
    dial(show); tick();
    CHECK(s.state == SHOW_ARMING && s.active && s.track == 0, "arriving did not arm track 1");
    CHECK(s.countdown == SHOW_ARM_MS / 1000, "countdown starts at %u", s.countdown);
    CHECK(s_armCalls == 1 && s_playCalls == 0, "the music must wait for the countdown");
    CHECK(s.frame.lamps == 0, "lamps requested during the countdown");
    tick(armTicks - 2);
    CHECK(s.state == SHOW_ARMING && s.countdown == 1, "countdown ended early");
    tick(2);
    CHECK(s.state == SHOW_PLAYING && s_playCalls == 1 && s_lastFile == SHOW_TRACKS[0].file,
          "the track did not start after the countdown");
    bool lamps = false;
    for (int i = 0; i < 3000; i++) { tick(); if (s.frame.lamps) lamps = true; }
    CHECK(s.state == SHOW_PLAYING && lamps, "one minute in: not playing, or no lamp ever asked for");

    // Leaving the detent stops everything at once.
    dial(0); tick();
    CHECK(s.state == SHOW_IDLE && !s.active && s.frame.lamps == 0 && s_stopCalls == 1,
          "leaving the detent did not stop the show");

    // Back within the skip window: the next track. Back later: track 1 again.
    tick(10); dial(show); tick();
    CHECK(s.state == SHOW_ARMING && s.track == 1 % SHOW_TRACK_COUNT, "the skip gesture gave track %u", s.track);
    dial(0); tick(SHOW_SKIP_WINDOW_MS / TICK_FAST_MS + 5); dial(show); tick();
    CHECK(s.track == 0, "a slow return should start over at track 1, got %u", s.track);
    CHECK(s_stopCalls == 1, "stop sent although nothing was playing");

    // A red alarm ends the show and it stays ended.
    tick(armTicks + 50);
    CHECK(s.state == SHOW_PLAYING, "not playing before the alarm test");
    g_data.alarms.anyRed = true; tick();
    CHECK(s.state == SHOW_OVER && !s.active && s.frame.lamps == 0 && s_stopCalls == 2,
          "a red alarm did not end the show");
    g_data.alarms.anyRed = false; tick(armTicks * 2);
    CHECK(s.state == SHOW_OVER, "the show came back by itself after the alarm cleared");

    // Engine above idle: refused at arrival, and ended mid-show. An ECU that
    // is not talking (ignition off at a show) does not block.
    reboot(show);
    g_data.ecu.rpm = 3000; // stale: no frame has ever arrived
    tick(armTicks + 5);
    CHECK(s.state == SHOW_PLAYING, "a silent ECU blocked the show");
    g_data.ecu.frameLastRxMs[ECU_F_SPEEDS] = s_now; tick();
    CHECK(s.state == SHOW_OVER && s_stopCalls == 1, "3000 rpm did not end the show");
    reboot(show);
    g_data.ecu.rpm = 900;
    for (int i = 0; i < armTicks + 5; i++) { g_data.ecu.frameLastRxMs[ECU_F_SPEEDS] = s_now; tick(); }
    CHECK(s.state == SHOW_PLAYING, "an idling engine blocked the show");

    // Dialled in while the DFPlayer is still booting: the countdown waits for it.
    reboot(show);
    s_audioReady = false; tick(100);
    CHECK(s.state == SHOW_IDLE && s_armCalls == 0, "armed before the audio module was ready");
    s_audioReady = true; tick();
    CHECK(s.state == SHOW_ARMING && s_armCalls == 1, "did not arm once the module was ready");
    tick(armTicks + 5);

    // The track ends: by the module's report, or by the clock if that is lost.
    s_finished = true; tick();
    CHECK(s.state == SHOW_OVER && !s.active, "\"finished\" did not end the show");
    reboot(show);
    tick(armTicks + 5);
    tick((SHOW_TRACKS[0].seconds + 4) * 1000 / TICK_FAST_MS - 5);
    CHECK(s.state == SHOW_PLAYING, "the length fallback fired early");
    tick(2 * 1000 / TICK_FAST_MS);
    CHECK(s.state == SHOW_OVER && s.frame.lamps == 0, "a track with no \"finished\" frame ran on");
    }

// --- output for the simulator build ---------------------------------------------------

static void printJson()
    {
    printf("{\n \"ledCount\": %u, \"ledTotal\": %u, \"phraseBeats\": %u, \"armMs\": %u,\n",
           LED_COUNT, LED_TOTAL_COUNT, SHOW_PHRASE_BEATS, (unsigned)SHOW_ARM_MS);
    printf(" \"ledMaxMa\": %u, \"ledMaPerCh\": %u, \"maxFlashDiv\": %u, \"maxRpm\": %u,\n",
           SHOW_LED_MAX_MA, SHOW_LED_MA_PER_CH, SHOW_MAX_FLASH_DIV, SHOW_MAX_RPM);
    printf(" \"latencyMs\": %d, \"defaultMap\": \"%s\",\n", SHOW_AUDIO_LATENCY_MS, SHOW_DEFAULT_MAP);
    printf(" \"frame\": { \"led\": %zu, \"screen\": %zu, \"backlight\": %zu, \"lamps\": %zu },\n",
           offsetof(ShowFrame, led), offsetof(ShowFrame, screen),
           offsetof(ShowFrame, backlight), offsetof(ShowFrame, lamps));
    printf(" \"lamps\": {");
    for (int i = 0; i < k_lampCount; i++)
        printf("%s \"%s\": %u", i ? "," : "", k_lampNames[i], k_lampBits[i]);
    printf(" },\n \"tracks\": [\n");
    for (uint8_t i = 0; i < SHOW_TRACK_COUNT; i++)
        {
        const ShowTrack& t = SHOW_TRACKS[i];
        printf("  { \"file\": %u, \"name\": \"%s\", \"bpm\": %g, \"beat1Ms\": %d, \"seconds\": %u, \"map\": %s%s%s }%s\n",
               t.file, t.name, t.bpm, t.beat1Ms, t.seconds, t.map ? "\"" : "",
               t.map ? t.map : "null", t.map ? "\"" : "", i + 1 < SHOW_TRACK_COUNT ? "," : "");
        }
    printf(" ]\n}\n");
    }

// "bpm beat1Ms songMs map" then the frame as hex, one line each.
static void printFrames()
    {
    static const char* const maps[] = { "pcbdk-", "bbbd", "", "ddkkccpp" };
    static const float bpms[]       = { 136.0f, 120.0f, 174.0f, 99.5f };
    for (int m = 0; m < 4; m++)
        for (int t = -300; t < 200000; t += 173)
            {
            const ShowTrack trk = { 0, "", bpms[m], (int16_t)(m * 37), 0, maps[m][0] ? maps[m] : nullptr };
            ShowFrame f;
            memset(&f, 0, sizeof(f)); // padding bytes too: the frame is compared as raw memory
            showRender(trk, t, f);
            // showRender assigns a fresh ShowFrame{}; copy field by field so
            // padding stays zero whatever the compiler did with it.
            ShowFrame clean;
            memset(&clean, 0, sizeof(clean));
            memcpy(clean.led, f.led, sizeof(f.led));
            memcpy(clean.screen, f.screen, sizeof(f.screen));
            memcpy(clean.backlight, f.backlight, sizeof(f.backlight));
            clean.lamps = f.lamps;
            printf("%g %d %d %s ", bpms[m], m * 37, t, maps[m][0] ? maps[m] : "-default-");
            for (size_t i = 0; i < sizeof(clean); i++) printf("%02x", ((const uint8_t*)&clean)[i]);
            printf("\n");
            }
    }

int main(int argc, char** argv)
    {
    if (argc > 1 && !strcmp(argv[1], "--json"))   { printJson();   return 0; }
    if (argc > 1 && !strcmp(argv[1], "--frames")) { printFrames(); return 0; }

    printf("choreography, every track start to finish:\n");
    for (const ShowTrack& t : SHOW_TRACKS) testChoreography(t);
    const ShowTrack generic = { 0, "(default)", 128.0f, 250, 240, nullptr };
    testChoreography(generic);
    const ShowTrack fast = { 0, "(174 bpm)", 174.0f, 0, 240, nullptr };
    testChoreography(fast);
    testLooks();

    printf("state machine:\n");
    testStateMachine();

    printf(s_failures ? "\n%d CHECK(S) FAILED\n" : "\nall checks passed\n", s_failures);
    return s_failures ? EXIT_FAILURE : EXIT_SUCCESS;
    }
