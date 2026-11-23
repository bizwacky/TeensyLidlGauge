#!/usr/bin/env python3
"""Light show - the bench calibration track.

Writes a WAV of kick-drum clicks on an exact tempo grid, with a higher
"tick" on the first beat of every bar. Nothing but the standard library.

    python3 tools/lightshow/make_click_track.py            # 0100-click120.wav
    python3 tools/lightshow/make_click_track.py --bpm 127.3 --first-beat-ms 420 --seconds 30 -o test.wav

Copy the default file into the DFPlayer card's /mp3 folder. It is the
"CLICK 120" row of SHOW_TRACKS (src/config.h), which flashes every light on
every beat - so the two sync constants can be set by eye and ear:

  SHOW_AUDIO_LATENCY_MS  lights early -> raise it, lights late -> lower it.
                         Judge it in the first few seconds.
  SHOW_TEMPO_TRIM        if the lights are right at the start and wrong by
                         the end, the module's clock differs from the
                         Teensy's. The serial monitor prints
                         "[SHOW] "CLICK 120" ended after N ms" when the track
                         finishes; the file is exactly 60000 ms long, so
                         trim = 60000 / (N - SHOW_AUDIO_LATENCY_MS).

WAV rather than MP3 on purpose: an MP3 encoder adds a few tens of ms of
silence at the front, and this file's whole job is to have a known start.
"""

import argparse
import math
import struct
import wave

RATE = 44100


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bpm", type=float, default=120.0)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--first-beat-ms", type=float, default=0.0)
    ap.add_argument("-o", "--out", default="0100-click120.wav")
    args = ap.parse_args()

    total = int(args.seconds * RATE)
    samples = [0.0] * total
    beat = 60.0 / args.bpm
    n = 0
    while True:
        start = int((args.first_beat_ms / 1000.0 + n * beat) * RATE)
        if start >= total:
            break
        # A kick: a sine that falls from 150 Hz to 50 Hz with a fast decay.
        phase = 0.0
        for i in range(min(int(0.18 * RATE), total - start)):
            t = i / RATE
            phase += 2 * math.pi * (50 + 100 * math.exp(-t * 30)) / RATE
            samples[start + i] += 0.8 * math.sin(phase) * math.exp(-t * 18)
        # Bar marker: a short 1.6 kHz tick on beat 1 of each bar.
        if n % 4 == 0:
            for i in range(min(int(0.03 * RATE), total - start)):
                t = i / RATE
                samples[start + i] += 0.25 * math.sin(2 * math.pi * 1600 * t) * math.exp(-t * 120)
        n += 1

    with wave.open(args.out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(b"".join(struct.pack("<h", int(max(-1.0, min(1.0, s)) * 32767)) for s in samples))
    print(f"{args.out}: {n} beats at {args.bpm:g} bpm, {total / RATE:.3f} s, first beat at {args.first_beat_ms:g} ms")


if __name__ == "__main__":
    main()
