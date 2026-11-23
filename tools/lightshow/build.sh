#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Light show - host tests + simulator build
# ---------------------------------------------------------------------------
#   tools/lightshow/build.sh
#
# 1. Compiles src/lightshow*.cpp for this PC and runs test_lightshow.cpp.
# 2. Compiles src/lightshow_fx.cpp to WebAssembly, then to plain JavaScript
#    (binaryen's wasm2js): the simulator page runs the firmware's own
#    choreography, and plain JS loads anywhere a page does.
# 3. Replays a few thousand frames through that JavaScript and compares them
#    byte for byte with the native build.
# 4. Writes lightshow_sim.html = sim_template.html + engine + track table.
#
# Needs g++, clang++ with the wasm32 target, wasm-ld, node and npm (which
# fetches binaryen once into ./.cache). Re-run after any change to the
# choreography, the track list or the template; commit the regenerated page.
# ---------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"
SRC=../../src
OUT=.cache
mkdir -p "$OUT"

echo "== host tests"
g++ -std=c++17 -O1 -Wall -Wextra -I. -I"$SRC" -o "$OUT/test_lightshow" \
    test_lightshow.cpp "$SRC/lightshow.cpp" "$SRC/lightshow_fx.cpp" \
    "$SRC/modes.cpp" "$SRC/vehicle_data.cpp"
"$OUT/test_lightshow"

echo "== engine -> wasm -> js"
clang++ --target=wasm32 -std=c++17 -O2 -nostdlib -ffreestanding -fno-builtin \
    -fno-exceptions -fno-rtti -I. -I"$SRC" -Wl,--no-entry -Wl,--strip-all \
    -o "$OUT/engine.wasm" sim_api.cpp "$SRC/lightshow_fx.cpp"
[ -x "$OUT/node_modules/.bin/wasm2js" ] || npm install --silent --prefix "$OUT" binaryen@132.0.0
"$OUT/node_modules/.bin/wasm2js" -O2 "$OUT/engine.wasm" -o "$OUT/engine.mjs"
# wasm2js writes an ES module; the page wants a plain script. Drop the
# export lines - the page reads retasmFunc itself.
grep -v '^export ' "$OUT/engine.mjs" > "$OUT/engine.js"

"$OUT/test_lightshow" --json   > "$OUT/config.json"
"$OUT/test_lightshow" --frames > "$OUT/frames.txt"

echo "== js engine vs native"
node - "$OUT" <<'JS'
const fs = require('fs'), dir = process.argv[2];
const cfg = JSON.parse(fs.readFileSync(dir + '/config.json'));
const eng = new Function(fs.readFileSync(dir + '/engine.js', 'utf8') + '; return retasmFunc;')();
const size = cfg.frame.lamps + 2, pad = [cfg.frame.backlight + 3];
let n = 0, bad = 0;
for (const line of fs.readFileSync(dir + '/frames.txt', 'utf8').trim().split('\n')) {
  const [bpm, beat1, t, map, hex] = line.split(' ');
  const mem = new Uint8Array(eng.memory.buffer), m = eng.sim_map();
  const text = map === '-default-' ? '' : map;
  for (let i = 0; i <= text.length; i++) mem[m + i] = i < text.length ? text.charCodeAt(i) : 0;
  const p = eng.sim_render(+bpm, +beat1, +t);
  let got = '';
  for (let i = 0; i < size; i++) got += (pad.includes(i) ? 0 : mem[p + i]).toString(16).padStart(2, '0');
  n++;
  if (got !== hex.slice(0, size * 2) && bad++ < 5) console.log('MISMATCH', line.slice(0, 40), '\n  js    ', got, '\n  native', hex);
}
console.log(`  ${n} frames compared, ${bad} differ`);
process.exit(bad ? 1 : 0);
JS

echo "== page"
node - "$OUT" <<'JS'
const fs = require('fs'), dir = process.argv[2];
let page = fs.readFileSync('sim_template.html', 'utf8');
for (const [mark, file] of [['/*@ENGINE@*/', 'engine.js'], ['/*@CONFIG@*/', 'config.json']]) {
  if (!page.includes(mark)) throw new Error('template lacks ' + mark);
  page = page.replace(mark, () => fs.readFileSync(dir + '/' + file, 'utf8'));
}
fs.writeFileSync('lightshow_sim.html', page);
console.log('  lightshow_sim.html', (page.length / 1024).toFixed(0) + ' KB');
JS
