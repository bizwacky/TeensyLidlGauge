// ---------------------------------------------------------------------------
// Rally CAN Gauge - audio arbitration + DFPlayer Mini driver
// ---------------------------------------------------------------------------
// The arbitration rules, in the order they matter:
//
//   1. One clip at a time. A clip "plays" until the DFPlayer's "track
//      finished" notification arrives (AUDIO_CLIP_TIMEOUT_MS is the fallback
//      if that frame is lost, or no module is wired at all).
//   2. Priority: scan the alarm list from priority 1 down, take the first
//      ACTIVE alarm whose clip is DUE. Red alarms therefore always win.
//   3. Due = never announced this activation (one-shot alarms), or its
//      repeat interval has elapsed (repeating alarms - GPWS style).
//   4. Preemption: a RED alarm's clip may cut off a non-red clip mid-play.
//      Nothing cuts off a red clip. (Starting a new track on the DFPlayer
//      inherently stops the old one, so preemption costs nothing extra.)
//   5. Boot: "All Systems Nominal" plays at the first quiet 1 s tick. If
//      something is already wrong at boot, the alarm wins and the nominal
//      call is cancelled entirely (saying "all systems nominal" five
//      minutes into a stage would be a lie and a distraction).
//   6. Silent mode (g_data.mode.silent, resolved from the dial by
//      modes.cpp) cuts the current clip and starts nothing new.
//      Announcements are NOT credited while muted, so everything still
//      active gets its say on unmute.
//
// Driver design notes (Makuna/DFMiniMp3, used for RX only):
//
//   * TX is FIRE-AND-FORGET, sendCmd() below. The packet (built with the
//     library's own public generator, checksum included) sets the datasheet's
//     "feedback" byte to 0, so the module owes us no reply and we never wait
//     for one: write() lands in Serial7's interrupt-driven TX buffer and
//     returns in MICROSECONDS; the ~10 ms of wire time at 9600 baud happens
//     in the background. This is why the library's own command methods are
//     BANNED here (never call s_mp3.setVolume()/play*()/get*()): each one
//     requests a reply and then blocks in readBytes() for 25-900 ms waiting
//     for it - stalling the 20 ms tick, and with a missing module risking
//     the 1 s watchdog. (The Mp3ChipIncongruousNoAck template variant is
//     kept as a belt-and-suspenders cap: if such a call ever sneaks in, it
//     times out at 50 ms instead of 900 ms x 3 retries.)
//   * RX is where the library earns its keep: s_mp3.loop(), pumped by
//     audioLoop() every superloop pass, parses the module's asynchronous
//     frames - "track finished" (datasheet 3.3.2; our clip-end detection,
//     since no BUSY pin is wired), the 0x3F power-up frame (readiness),
//     and error frames. It only starts reading once a complete 10-byte
//     frame is already buffered, so it is effectively non-blocking; the
//     setTimeout(20) in audioInit() bounds the one rare exception (a noise
//     byte desyncing a read mid-frame - Stream's default timeout is
//     1000 ms, sitting exactly at the watchdog limit).
//   * The AUDIO_FINISH_GUARD_MS timestamp check rejects the bogus
//     finished-frame clone chips emit for a track that a new play command
//     just cut off.
//   * Volume (AUDIO_VOL_NORMAL baseline / AUDIO_VOL_RED for reds) is primed
//     once at readiness and then only re-sent when the level changes or the
//     module reboots (a reboot resets it). Keeps the volume command off the
//     play path most of the time: the datasheet advises against back-to-back
//     commands, and two queued packets are still serialized ~10 ms apart on
//     the wire - if a clip after a red/non-red switch ever fails to start on
//     real hardware, this spacing is the first suspect.
// ---------------------------------------------------------------------------

#include "audio.h"
#include "alarms.h"
#include "config.h"
#include "vehicle_data.h"

#include <DFMiniMp3.h>

// --- driver state -------------------------------------------------------------
// Declared before Mp3Notify because the notification hooks write these.

static bool     s_dfpReady    = false; // module reported in (or grace expired)
static uint32_t s_finishAtMs  = 0;     // millis() of the last "track finished" frame
static uint8_t  s_lastVolSent = 255;   // 255 = unknown -> next clip sends setVolume

// The library takes the notification hook class as a template parameter and
// passes the driver object back into every hook, so the type dance goes:
// forward-declare the hooks, name the driver type, then define the hooks.
class Mp3Notify;
using DfMp3 = DFMiniMp3<HardwareSerial, Mp3Notify, Mp3ChipIncongruousNoAck>;

// Hooks the library calls from inside s_mp3.loop(). That runs in superloop
// context (not an interrupt), so plain statics and Serial prints are safe.
class Mp3Notify
    {
public:
    static void OnPlayFinished(DfMp3&, DfMp3_PlaySources, uint16_t track)
        {
        s_finishAtMs = millis();
        Serial.printf("[AUD ] clip finished (track %u)\n", track);
        }

    // The module announces its play sources after power-up/reset: this is
    // the "DFPlayer is alive and its SD card is readable" signal.
    static void OnPlaySourceOnline(DfMp3&, DfMp3_PlaySources)   { sourceReady("online"); }
    static void OnPlaySourceInserted(DfMp3&, DfMp3_PlaySources) { sourceReady("SD inserted"); }
    static void OnPlaySourceRemoved(DfMp3&, DfMp3_PlaySources)
        {
        Serial.println("[AUD ] DFPlayer: SD card removed!");
        }

    static void OnError(DfMp3&, uint16_t code)
        {
        const char* what = "";
        switch (code)
            {
            case DfMp3_Error_Busy:         what = " (busy / SD not ready)"; break;
            case DfMp3_Error_FileIndexOut: what = " (track number out of range)"; break;
            case DfMp3_Error_FileMismatch: what = " (track not on SD card)"; break;
            case DfMp3_Error_SdReadFail:   what = " (SD read failed)"; break;
            }
        Serial.printf("[AUD ] DFPlayer error %u%s\n", code, what);
        }

private:
    static void sourceReady(const char* how)
        {
        s_dfpReady    = true;
        s_lastVolSent = 255; // a (re)booted module forgot its volume - resend
        Serial.printf("[AUD ] DFPlayer %s, SD ready\n", how);
        }
    };

static DfMp3 s_mp3(AUDIO_SERIAL); // RX parsing only - see design notes above

// Fire-and-forget transmit: build the 10-byte packet (feedback byte = 0, so
// no reply comes) and hand it to the TX buffer. Costs microseconds. The
// Mp3_Commands_* values come with the library (internal/Mp3Packet.h).
static void sendCmd(uint8_t command, uint16_t arg = 0)
    {
    const auto packet =
        Mp3ChipIncongruousNoAck::generatePacket(command, arg, /*requestAck=*/false);
    AUDIO_SERIAL.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    }

// --- arbitration state --------------------------------------------------------

static uint8_t  s_playingClip  = CLIP_NONE;
static bool     s_playingIsRed = false;
static uint32_t s_playStartMs  = 0;
static uint32_t s_initMs       = 0;

static uint32_t s_lastPlayMs[ALARM_COUNT]; // per ALARM (two alarms may share a clip)
static bool     s_saidOnce[ALARM_COUNT];   // one-shot latch, reset when the alarm clears
static bool     s_bootNominalPending = true;

static const char* k_clipNames[CLIP_COUNT] =
    {
    "(none)",
    "ALL SYSTEMS NOMINAL",
    "OIL PRESSURE",
    "OIL TEMPERATURE",
    "COOLANT TEMPERATURE",
    "MIXTURE LEAN",
    "ALTERNATOR",
    "OVERVOLTAGE",
    "BATTERY CELL CRITICAL",
    "BATTERY UNBALANCED",
    "BATTERY TEMPERATURE",
    "FUEL LOW",
    "FUEL CRITICAL",
    "ECU LINK LOST",
    "BATTERY LINK LOST",
    "MODE DIAL FAULT",
    };

// --- the driver ----------------------------------------------------------------

static void startClip(uint8_t clip, bool isRed)
    {
    s_playingClip  = clip;
    s_playingIsRed = isRed;
    s_playStartMs  = millis();

    // Volume first when it needs to change (red clips are louder); the wire
    // serializes the two packets ~10 ms apart, module executes them in order.
    const uint8_t vol = isRed ? AUDIO_VOL_RED : AUDIO_VOL_NORMAL;
    if (vol != s_lastVolSent)
        {
        sendCmd(Mp3_Commands_SetVolume, vol);
        s_lastVolSent = vol;
        }

    // Track number = clip enum value: plays /mp3/000N*.mp3 (see audio.h for
    // the SD card layout). Starting a track cuts off any current one, which
    // is exactly the preemption behavior rule 4 wants.
    sendCmd(Mp3_Commands_PlayMp3FolderTrack, clip);

    Serial.printf("[AUD ] play: \"%s\" (track %u, vol %u)\n",
                  k_clipNames[clip], clip, vol);
    }

// --- public API -------------------------------------------------------------------

void audioInit()
    {
    s_initMs = millis();

    s_mp3.begin(); // AUDIO_SERIAL at 9600 baud

    // Bound every read the RX parser can make. Stream's default readBytes
    // timeout is 1000 ms - one noise byte desyncing a frame would stall the
    // loop right at the watchdog limit. With a live module the rest of a
    // frame is at most ~10 ms away, so 20 ms is already generous.
    AUDIO_SERIAL.setTimeout(20);

    sendCmd(Mp3_Commands_Reset); // cleans up after a watchdog reboot (a clip
                                 // could still be speaking); the module's
                                 // 0x3F power-up frame or the boot grace
                                 // deadline then signals readiness.

    Serial.println("Audio: DFPlayer driver up on Serial7 (pins 28/29, waiting for module)");
    }

void audioLoop()
    {
    // Pump the library: read pending frames off the serial port and fire the
    // Mp3Notify hooks. Cheap when the port is idle, so every pass is fine.
    s_mp3.loop();
    }

void audioTick1s()
    {
    const uint32_t now = millis();

    // (0) hold off until the module reported in, or the grace deadline
    // passed (module absent/broken - arbitration then runs anyway, so the
    // serial prints still show what WOULD have played).
    if (!s_dfpReady)
        {
        if (now - s_initMs < AUDIO_BOOT_GRACE_MS) return;
        s_dfpReady = true;
        }

    // Prime the volume at readiness, and re-prime after a module reboot
    // (sourceReady() resets the cache because the chip forgets its setting).
    // Doing it here - usually a quiet tick - keeps the volume command spaced
    // away from play commands, which the datasheet prefers.
    if (s_lastVolSent == 255)
        {
        sendCmd(Mp3_Commands_SetVolume, AUDIO_VOL_NORMAL);
        s_lastVolSent = AUDIO_VOL_NORMAL;
        }

    // (1) has the current clip finished? Trust a "track finished" frame only
    // if it arrived AUDIO_FINISH_GUARD_MS into THIS clip - comparing
    // timestamps (rather than latching a flag) makes stale frames from a
    // preempted track self-invalidating. Timeout is the lost-frame fallback.
    if (s_playingClip != CLIP_NONE)
        {
        const bool finished =
            (int32_t)(s_finishAtMs - s_playStartMs) >= (int32_t)AUDIO_FINISH_GUARD_MS;
        if (finished || now - s_playStartMs >= AUDIO_CLIP_TIMEOUT_MS)
            s_playingClip = CLIP_NONE;
        }

    // Re-arm one-shot announcements for alarms that have cleared, so the
    // same alarm speaks again if it comes back later.
    for (uint8_t i = 0; i < ALARM_COUNT; i++)
        if (!g_data.alarms.active[i])
            s_saidOnce[i] = false;

    // (6) silent mode: the dial is the driver saying "I know".
    const bool muted = g_data.mode.silent;
    static bool mutedLast = false;
    if (muted != mutedLast)
        {
        mutedLast = muted;
        Serial.printf("[AUD ] %s (rotary dial)\n", muted ? "MUTED" : "unmuted");
        }
    if (muted)
        {
        s_bootNominalPending = false; // a muted boot cancels the greeting too
        if (s_playingClip != CLIP_NONE)
            {
            sendCmd(Mp3_Commands_Stop); // mute means NOW - cut mid-sentence
            s_playingClip = CLIP_NONE;
            }
        return; // before crediting: still-active alarms speak on unmute
        }

    // (2)+(3) find the highest-priority alarm with something to say.
    // Enum order IS priority order, so the first hit wins.
    int16_t cand = -1;
    for (uint8_t i = 0; i < ALARM_COUNT; i++)
        {
        if (!g_data.alarms.active[i]) continue;
        if (alarmClip(i) == CLIP_NONE) continue;

        const uint32_t repeat = alarmRepeatMs(i);
        const bool due = (repeat > 0)
                         ? (s_lastPlayMs[i] == 0 || now - s_lastPlayMs[i] >= repeat)
                         : !s_saidOnce[i];
        if (due) { cand = i; break; }
        }

    if (cand < 0)
        {
        // (5) nothing to report - the boot greeting gets its chance.
        if (s_bootNominalPending && s_playingClip == CLIP_NONE)
            {
            s_bootNominalPending = false;
            startClip(CLIP_NOMINAL, false);
            }
        return;
        }

    // Something was wrong before the greeting got out - cancel it for good.
    s_bootNominalPending = false;

    // (4) never talk over a red clip; only a red alarm may preempt at all.
    const bool candIsRed = (alarmColor((uint8_t)cand) == ALARM_RED);
    if (s_playingClip != CLIP_NONE && !(candIsRed && !s_playingIsRed))
        return; // wait for the current clip to end; we'll still be due next tick

    const AudioClip clip = alarmClip((uint8_t)cand);

    // Credit the announcement to EVERY active alarm that uses this clip, not
    // just the winner - e.g. VOLT LOW critical and VOLT LOW warning are both
    // active below 13.2 V and both say "ALTERNATOR"; without this the driver
    // would hear it twice back-to-back.
    for (uint8_t i = 0; i < ALARM_COUNT; i++)
        if (g_data.alarms.active[i] && alarmClip(i) == clip)
            {
            s_lastPlayMs[i] = now;
            s_saidOnce[i]   = true;
            }

    startClip(clip, candIsRed);
    }
