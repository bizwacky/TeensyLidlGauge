-- ---------------------------------------------------------------------------
-- Teensy Lidl Gauge -> rusEFI: log the gauge's IMU in the ECU's SD card log
-- ---------------------------------------------------------------------------
-- Paste this into TunerStudio's Lua Script dialog and burn. It needs nothing
-- else configured: it only LISTENS on CAN and never transmits.
--
-- The gauge broadcasts four IMU frames (layouts: docs/IMU_SETUP.md). This
-- script copies eight of their values into rusEFI's eight "Lua gauges",
-- which rusEFI writes to its SD card log like any other channel. They show
-- up in the log (and in TunerStudio) as "Lua: Gauge 1" .. "Lua: Gauge 8":
--
--   1  PeakG      hardest total acceleration of the last 0.2 s        g
--   2  PeakLon    \  per axis: the reading of largest size in the     g
--   3  PeakLat     } same 0.2 s, with its sign. + = speeding up,      g
--   4  PeakVert   /  + = turning left, + = up (reads 1 at rest)       g
--   5  Roll       + = right side down                                 deg
--   6  Pitch      + = nose up                                         deg
--   7  AirTime    how long the last jump was airborne                 s
--   8  Impact     number of the gauge's latest impact capture; it steps
--                 up by one at every hit hard enough to be captured
--
-- WHY PEAKS AND NOT "THE ACCELERATION NOW": a landing or a bash lasts a few
-- milliseconds and this log samples every 10-50 ms. The gauge samples its
-- high-g sensor at 960 Hz and sends the largest reading of the last 0.2 s,
-- so every hit is in the log at its full height at any SD log rate of
-- 10 Hz or more. At rest the four peak channels read 1, 0, 0, 1. PeakG = 0 means the
-- gauge has no IMU data (a working sensor never reads zero g for 0.2 s
-- unless the car is in the air - and then gauge 7 says so afterwards).
--
-- To log something else instead (the gauge also sends smoothed low-g lon /
-- lat / vert, yaw rate, jump count, landing g, the capture's peak and
-- delta-v), change the setLuaGauge lines below: a handler is there for
-- every frame. Eight is all rusEFI has.
--
-- Checked against rusEFI master, Nov 2026 (firmware/controllers/lua/
-- lua_hooks.cpp: canRxAdd, setLuaGauge; lua.cpp: setTickRate).
-- ---------------------------------------------------------------------------

-- Must equal IMU_CAN_BASE_ID in the gauge's src/config.h.
IMU_BASE_ID = 0x6A0

-- CAN payloads arrive as a table of bytes, data[1] = byte 0. Everything the
-- gauge sends is little-endian: low byte first.
local function u16(data, i)
    return data[i] + data[i + 1] * 256
end

local function s16(data, i)
    local v = data[i] + data[i + 1] * 256
    if v >= 32768 then v = v - 65536 end
    return v
end

-- Frame 0: the high-g channel, peak-held. 25 Hz.
function onImuImpact(bus, id, dlc, data)
    if dlc < 8 then return end
    setLuaGauge(1, u16(data, 1) * 0.01)     -- PeakG
    setLuaGauge(2, s16(data, 3) * 0.01)     -- PeakLon
    setLuaGauge(3, s16(data, 5) * 0.01)     -- PeakLat
    setLuaGauge(4, s16(data, 7) * 0.01)     -- PeakVert
end

-- Frame 1: the low-g channel, smoothed, and yaw rate. 25 Hz. Not subscribed
-- by default (see the canRxAdd lines at the bottom).
--   s16(data, 1) * 0.001   lon g          s16(data, 5) * 0.001   vert g
--   s16(data, 3) * 0.001   lat g          s16(data, 7) * 0.1     yaw rate, deg/s (+ = left)
function onImuMotion(bus, id, dlc, data)
    if dlc < 8 then return end
    -- setLuaGauge(2, s16(data, 1) * 0.001)
    -- setLuaGauge(3, s16(data, 3) * 0.001)
    -- setLuaGauge(7, s16(data, 7) * 0.1)
end

-- Frame 2: attitude, jumps, status. 10 Hz.
--   data[7]   jumps since the gauge booted (wraps at 256)
--   data[8]   status bits: 1 IMU live, 2 level-calibrated, 4 high-g trim
--             settled, 8 airborne, 16 capture recording, 32 standing still,
--             64 samples have been lost since boot
function onImuAttitude(bus, id, dlc, data)
    if dlc < 8 then return end
    setLuaGauge(5, s16(data, 1) * 0.1)      -- Roll
    setLuaGauge(6, s16(data, 3) * 0.1)      -- Pitch
    setLuaGauge(7, u16(data, 5) * 0.001)    -- AirTime
end

-- Frame 3: the latest impact capture. 5 Hz.
--   u16(data, 3) * 0.01   its peak, g
--   u16(data, 5) * 0.01   its delta-v, m/s (all axes combined)
--   u16(data, 7) * 0.01   landing of the last jump, g
function onImuEvent(bus, id, dlc, data)
    if dlc < 8 then return end
    setLuaGauge(8, u16(data, 1))            -- Impact
end

canRxAdd(IMU_BASE_ID + 0, onImuImpact)
-- canRxAdd(IMU_BASE_ID + 1, onImuMotion)   -- uncomment if onImuMotion sets a gauge
canRxAdd(IMU_BASE_ID + 2, onImuAttitude)
canRxAdd(IMU_BASE_ID + 3, onImuEvent)

-- Received frames are handed to the functions above once per tick. 50 Hz
-- keeps a gauge at most 20 ms behind the bus; the work per tick is one or
-- two tiny functions.
setTickRate(50)

function onTick()
end
