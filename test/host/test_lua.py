# Run docs/rusefi_imu.lua in a real Lua 5.4 with rusEFI's hooks mocked, feed
# it frames packed the way can_bus.cpp packs them, and check the gauges.
# Needs the 'lupa' package (pip install lupa), which embeds Lua 5.4.
import os, sys, struct
import lupa.lua54 as lua54

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'docs', 'rusefi_imu.lua')

lua = lua54.LuaRuntime()
gauges, filters, tick = {}, {}, {}
g = lua.globals()
g.setLuaGauge = lambda i, v: gauges.__setitem__(int(i), float(v))
def canRxAdd(*a):
    assert len(a) == 2, "script uses canRxAdd(id, callback)"
    filters[int(a[0])] = a[1]
g.canRxAdd = canRxAdd
g.setTickRate = lambda hz: tick.__setitem__('hz', hz)
lua.execute(open(SCRIPT).read())

def deliver(can_id, payload):
    if can_id not in filters: return False
    t = lua.table_from({i + 1: b for i, b in enumerate(payload)})   # 1-indexed, like rusEFI builds it
    filters[can_id](1, can_id, len(payload), t)
    return True

BASE = 0x6A0
fails = 0
def near(name, got, want, tol=1e-6):
    global fails
    ok = abs(got - want) <= tol
    if not ok: fails += 1
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {got:.3f} (want {want:.3f})")

assert tick['hz'] == 50
assert sorted(filters) == [BASE, BASE + 2, BASE + 3], sorted(filters)
assert g.onTick is not None

# impact: peak 41.27 g, lon -22.00, lat +6.01, vert +34.00
assert deliver(BASE, struct.pack('<Hhhh', 4127, -2200, 601, 3400))
near('PeakG', gauges[1], 41.27, 1e-3); near('PeakLon', gauges[2], -22.00, 1e-3)
near('PeakLat', gauges[3], 6.01, 1e-3); near('PeakVert', gauges[4], 34.00, 1e-3)
# extremes of the encoding
deliver(BASE, struct.pack('<Hhhh', 32150, -32000, 32000, -1))
near('PeakG 321.5', gauges[1], 321.5, 1e-3); near('PeakLon -320', gauges[2], -320.0, 1e-3)
near('PeakLat +320', gauges[3], 320.0, 1e-3); near('PeakVert -0.01', gauges[4], -0.01, 1e-3)
# attitude: roll -33.4, pitch +12.5, air 420 ms, 3 jumps, status 0x07
assert deliver(BASE + 2, struct.pack('<hhHBB', -334, 125, 420, 3, 0x07))
near('Roll', gauges[5], -33.4, 1e-3); near('Pitch', gauges[6], 12.5, 1e-3); near('AirTime', gauges[7], 0.42, 1e-6)
# event: capture 258, peak 23.3 g, dv 4.5 m/s, landing 9.0 g
assert deliver(BASE + 3, struct.pack('<HHHH', 258, 2330, 450, 900))
near('Impact', gauges[8], 258)
# the motion frame is not subscribed by default, and a short frame is ignored
assert not deliver(BASE + 1, struct.pack('<hhhh', 100, -800, 1000, 250))
gauges[1] = -99
deliver(BASE, bytes([1, 2, 3]))
near('short frame ignored', gauges[1], -99)
print('script length:', len(open(SCRIPT).read()), 'chars (rusEFI default limit 8000; Super uaEFI 25000)')
print('LUA TEST', 'FAILED' if fails else 'passed')
sys.exit(1 if fails else 0)
