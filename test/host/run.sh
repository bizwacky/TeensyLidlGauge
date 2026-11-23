#!/bin/sh
# PC-side checks of the IMU code: the real motion.cpp, fram_events.cpp,
# fram_log.cpp and gauge_render.cpp, built with g++ against a stand-in
# Arduino.h, a RAM model of the FRAM and synthetic sensor samples.
#
#   ./run.sh           logic tests (+ the Lua script test if 'lupa' is installed)
#   ./run.sh render    also draw the inclinometer pages to build/pages.png
#                      (borrows tgx from .pio/libdeps: run 'pio run' once first)
#
# Several checks assume the defaults in src/config.h (6 g trigger, 8 capture
# slots, 100-block captures): after changing those, expect to update the
# expectations in test_main.cpp too.
set -e
cd "$(dirname "$0")"
SRC=../../src
mkdir -p build

g++ -std=gnu++17 -O1 -Wall -Wextra -Wformat=2 -Wno-unused-parameter -Wno-missing-field-initializers \
    -I fake -I . -I $SRC \
    $SRC/motion.cpp $SRC/fram_events.cpp $SRC/fram_log.cpp $SRC/vehicle_data.cpp \
    fram_fake.cpp test_main.cpp -o build/test_host
./build/test_host

if python3 -c "import lupa.lua54" 2>/dev/null; then
    python3 test_lua.py
else
    echo "(skipping the Lua script test: pip install lupa to run it)"
fi

if [ "$1" = "render" ]; then
    TGX=../../.pio/libdeps/teensymm/tgx/src
    [ -d "$TGX" ] || { echo "tgx not found at $TGX - run 'pio run' once first"; exit 1; }
    g++ -std=gnu++17 -O1 -w -I fake -I $SRC -I $TGX \
        $SRC/gauge_render.cpp $SRC/vehicle_data.cpp $TGX/Color.cpp $TGX/Fonts.cpp \
        render_main.cpp -o build/render_host
    (cd build && ./render_host && K=2 python3 ../raw2png.py pages.png tilt night alarm dead)
    echo "wrote build/pages.png"
fi
