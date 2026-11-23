// ---------------------------------------------------------------------------
// Rally CAN Gauge - the light show: when it runs
// ---------------------------------------------------------------------------
// The life of a show:
//
//   IDLE --dial arrives--> ARMING --SHOW_ARM_MS--> PLAYING --track ends--> OVER
//     ^                      |                        |                      |
//     +---------------- the dial leaves the detent (from any state) ---------+
//
//   ARMING   the countdown. The screens show the track's name; flicking the
//            dial off and back within SHOW_SKIP_WINDOW_MS steps to the next
//            track, so the countdown is also the track selector.
//   PLAYING  the play command has gone to the DFPlayer and every tick asks
//            lightshow_fx.cpp for the frame at "now" in the music.
//   OVER     an ordinary gauge again, until the dial leaves and comes back.
//
// THE RULES THAT KEEP IT A CAR-PARK TOY:
//
//   * A show starts only when the dial MOVES onto the detent. Powering up
//     already there does nothing - so a gauge that resets mid-show (the
//     strobing is the heaviest load the 5 V rail ever sees) comes back as a
//     gauge, not as the show again.
//   * A red alarm, or engine speed above SHOW_MAX_RPM, ends it at once and
//     it stays ended (OVER). The car is in use; this is no time for strobes.
//   * Ending it zeroes the frame, so the next CAN frame withdraws every lamp
//     request. The lamp bits only ever ask for MORE light than the roof
//     switches do (docs/PDM_CAN_SETUP.md), and a gauge that dies mid-show
//     stops sending them: the PDMs' input timeout drops the requests.
//
// THE CLOCK. Nothing comes back from the DFPlayer but "finished", so the
// position in the music is dead reckoning: time since the play command,
// less the module's start-up latency, times the trim between its clock and
// ours (config.h: SHOW_AUDIO_LATENCY_MS, SHOW_TEMPO_TRIM). When a track
// ends, the [SHOW] line prints how long the module actually took - compare
// it with the file's true length to set the trim.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "lightshow.h"
#include "lightshow_fx.h"
#include "audio.h"
#include "config.h"
#include "vehicle_data.h"

static uint32_t s_t0;       // millis() the countdown began / the play command left
static uint32_t s_leftMs;   // millis() the dial last left the detent
static bool     s_selected; // a track has been chosen since boot (arms the skip gesture)

static void endShow(ShowState next)
    {
    ShowData& s = g_data.show;
    if (s.state == SHOW_PLAYING)
        {
        Serial.printf("[SHOW] \"%s\" ended after %lu ms\n", SHOW_TRACKS[s.track].name,
                      (unsigned long)(millis() - s_t0));
        audioMusicStop();
        }
    s.state  = next;
    s.active = false;
    s.frame  = ShowFrame{}; // lamps = 0: the next CAN frame withdraws every request
    }

static void arm(uint8_t track, uint32_t now)
    {
    ShowData& s = g_data.show;
    s.track = track;
    s.state = SHOW_ARMING;
    s_t0    = now;
    audioMusicArm(); // volume now, so the play command travels alone later
    Serial.printf("[SHOW] armed: \"%s\"\n", SHOW_TRACKS[track].name);
    }

void lightshowUpdate()
    {
    ShowData&      s   = g_data.show;
    const uint32_t now = millis();

    if (g_data.mode.mode != MODE_LIGHTSHOW)
        {
        if (s.state != SHOW_IDLE)
            {
            endShow(SHOW_IDLE);
            s_leftMs = now;
            }
        return;
        }

    const ShowTrack& trk = SHOW_TRACKS[s.track];
    const bool blocked = g_data.alarms.anyRed
                         || (g_data.ecuOnline() && g_data.ecu.rpm > SHOW_MAX_RPM);

    switch (s.state)
        {
        case SHOW_IDLE: // the dial has just arrived
            if (!audioMusicReady()) break; // dialled in while the DFPlayer was still
                                           // booting: wait for it rather than lose
                                           // the volume and play commands
            if (g_data.sw.rotaryChangedMs < SHOW_BOOT_IGNORE_MS || blocked)
                {
                s.state = SHOW_OVER; // booted on the detent, or the car is busy
                break;
                }
            arm((s_selected && now - s_leftMs < SHOW_SKIP_WINDOW_MS)
                    ? (uint8_t)((s.track + 1) % SHOW_TRACK_COUNT) : 0, now);
            s_selected = true;
            [[fallthrough]]; // draw the first countdown frame this tick

        case SHOW_ARMING:
            {
            if (blocked) { endShow(SHOW_OVER); break; }
            const uint32_t waited = now - s_t0;
            if (waited >= SHOW_ARM_MS)
                {
                audioMusicPlay(SHOW_TRACKS[s.track].file);
                s.state = SHOW_PLAYING;
                s_t0    = now;
                break; // this tick keeps the last countdown frame
                }
            // Countdown frame: the tach strip burns down like a fuse; the
            // screens (black, medium backlight) carry the title and seconds.
            s.active    = true;
            s.countdown = (uint8_t)((SHOW_ARM_MS - waited + 999) / 1000);
            s.frame     = ShowFrame{};
            for (uint32_t i = 0; i <= (SHOW_ARM_MS - waited) * (LED_COUNT - 1) / SHOW_ARM_MS; i++)
                s.frame.led[i] = 0x181818;
            for (uint8_t& b : s.frame.backlight) b = SCREEN_BRIGHTNESS_MEDIUM;
            break;
            }

        case SHOW_PLAYING:
            {
            const uint32_t played = now - s_t0;
            if (blocked) { endShow(SHOW_OVER); break; }
            if (audioMusicFinished() || played > (trk.seconds + 5u) * 1000u)
                {
                endShow(SHOW_OVER);
                if (SHOW_AUTO_ADVANCE) arm((uint8_t)((s.track + 1) % SHOW_TRACK_COUNT), now);
                break;
                }
            showRender(trk, (int32_t)(((float)played - SHOW_AUDIO_LATENCY_MS) * SHOW_TEMPO_TRIM),
                       s.frame);
            break;
            }

        case SHOW_OVER:
            break;
        }
    }
