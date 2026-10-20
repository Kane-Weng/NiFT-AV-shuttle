// File: perf.mjs
//
// Description:
//   Perf and memory numbers of the playground in Node + Pyodide 0.27.7 against native Python.
//
//   - First load: the bytes a browser downloads (Pyodide core, numpy, pyyaml, our bundle, the map),
//     raw, gzip and brotli (jsDelivr and GitHub Pages compress on the wire).
//   - Boot: Pyodide, packages, bundle and map, session start-up (all nodes built, map parsed).
//   - Speed: ms per 50 ms tick and times real time on route 04 (goal, start, arrive), natively and
//     in Pyodide, with waypoint_node's map markers on (the repo default, every 1 s) and off
//     (map_publish_period_s 3600, ARCH.md section 6.3). A profile gives the markers' share.
//   - Memory: the WebAssembly heap after each stage and after many resets in one interpreter, and
//     the Node process RSS.
//
//   node perf.mjs [--json OUT] [--resets N]

import { execFileSync } from 'node:child_process'
import { readFileSync, writeFileSync, existsSync, readdirSync, mkdtempSync } from 'node:fs'
import { createRequire } from 'node:module'
import { tmpdir } from 'node:os'
import path from 'node:path'
import zlib from 'node:zlib'
import { HERE, MAP_FILE, PLAYGROUND, PYTHON, buildBundle, loadPyodideNode } from './node_env.mjs'

const args = process.argv.slice(2)
const jsonOut = args.includes('--json') ? args[args.indexOf('--json') + 1] : null
const RESETS = args.includes('--resets') ? Number(args[args.indexOf('--resets') + 1]) : 20
const require = createRequire(import.meta.url)
const pyodideDir = path.dirname(require.resolve('pyodide/package.json'))

const mb = (b) => Math.round((b / 1e6) * 100) / 100
const report = { date: new Date().toISOString(), node: process.version }

// ---------------------------------------------------------------- the scenario

const base = JSON.parse(readFileSync(path.join(HERE, 'scenarios/route04_arrive.json'), 'utf8'))
const variants = {
  // The launch set overrides the period to 3600 s by default (ARCH 6.3); 1.0 is waypoint_params.yaml's value.
  markers_on: { ...base, launch: { ...base.launch, params: { waypoint_node: { map_publish_period_s: 1.0000001 } } } }, // not 1.0: JSON writes 1.0 as 1, an int
  markers_off: base, // the launch set default, 3600 s (a 3600.0 from JSON would arrive as an int)
}

// ---------------------------------------------------------------- native

const work = mkdtempSync(path.join(tmpdir(), 'nift-perf-'))
const PROFILE_PY = `
import cProfile, pstats, io, json, sys
sys.path.insert(0, PYO_DIR)
import drive

def profile(scenario, repo):
    pr = cProfile.Profile()
    pr.enable()
    res = drive.run(scenario, repo)
    pr.disable()
    st = pstats.Stats(pr)
    total = st.total_tt
    share = {}
    for (file, line, fn), (cc, nc, tt, ct, callers) in st.stats.items():
        key = None
        if fn == 'publish_map_markers':
            key = 'waypoint_node.publish_map_markers'
        elif fn == '_tick_once':
            key = 'kinematic_sim._tick_once'
        elif fn == '_control_loop' or fn == 'control_loop':
            key = 'trajectory_controller.control_loop'
        elif fn == 'findNearest':
            key = 'lanelet2.geometry.findNearest'
        elif fn == 'fix_callback':
            key = 'waypoint_node.fix_callback'
        elif fn == 'publish' and 'fakeros' in file:
            key = 'fakeros publish (copy + deliver)'
        if key:
            share[key] = max(share.get(key, 0.0), ct)
    top = sorted(st.stats.items(), key=lambda kv: -kv[1][2])[:12]
    return {'summary': res['summary'], 'profile_total_s': total,
            'share': {k: round(v / total, 4) for k, v in share.items()},
            'top_self': [[f"{fn} ({file.split('/')[-1]}:{line})", round(tt / total, 4)] for (file, line, fn), (cc, nc, tt, ct, _) in top]}
`

function nativeRun(name, scenario, prof) {
  const f = path.join(work, `${name}.json`)
  writeFileSync(f, JSON.stringify(scenario))
  if (!prof) {
    const out = execFileSync(PYTHON, [path.join(HERE, 'drive.py'), f], { encoding: 'utf8' })
    return JSON.parse(out.trim().split('\n').pop())
  }
  const code = `PYO_DIR = ${JSON.stringify(HERE)}\n${PROFILE_PY}\nimport json\nprint(json.dumps(profile(json.load(open(${JSON.stringify(f)})), ${JSON.stringify(path.resolve(PLAYGROUND, '..'))})))`
  const out = execFileSync(PYTHON, ['-c', code], { encoding: 'utf8' })
  return JSON.parse(out.trim().split('\n').pop())
}

report.native = {}
for (const [k, sc] of Object.entries(variants)) {
  const runs = [0, 1, 2].map(() => nativeRun(k, sc, false))
  const best = runs.reduce((a, b) => (a.run_s < b.run_s ? a : b))
  report.native[k] = { ...best, ms_per_tick: (best.run_s * 1000) / best.ticks }
  console.log(`native  ${k}: ${report.native[k].ms_per_tick.toFixed(3)} ms/tick, ${best.x_realtime}x real time, init ${best.init_s} s`)
}
report.native.profile = nativeRun('profile', variants.markers_off, true)
console.log(`native  profile shares: ${JSON.stringify(report.native.profile.share)}`)

// ---------------------------------------------------------------- Pyodide

const bundleBase = buildBundle()
const zip = readFileSync(path.join(bundleBase, 'py/nift_bundle.zip'))
const heap = (py) => mb(py._module.HEAP8.length)
const rss = () => mb(process.memoryUsage().rss)
const mem = { node_baseline_rss: rss() }

let t = performance.now()
const py = await loadPyodideNode()
const boot = { pyodide_s: (performance.now() - t) / 1e3 }
mem.after_boot = { heap: heap(py), rss: rss() }
t = performance.now()
await py.loadPackage(['numpy', 'pyyaml'])
boot.packages_s = (performance.now() - t) / 1e3
mem.after_packages = { heap: heap(py), rss: rss() }
t = performance.now()
py.unpackArchive(new Uint8Array(zip), 'zip', { extractDir: '/nift' })
py.FS.mkdirTree('/nift/maps')
py.FS.writeFile('/nift/maps/lanelet2_odr2024.osm', new Uint8Array(readFileSync(MAP_FILE)))
py.FS.mkdirTree('/nift/nift_playground/tests/pyodide')
py.FS.writeFile('/nift/nift_playground/tests/pyodide/drive.py', new Uint8Array(readFileSync(path.join(HERE, 'drive.py'))))
boot.bundle_and_map_s = (performance.now() - t) / 1e3
py.runPython(`PYO_DIR = '/nift/nift_playground/tests/pyodide'\n${PROFILE_PY}`)
const runPy = (sc) => {
  py.globals.set('_sc', JSON.stringify(sc))
  return JSON.parse(py.runPython(`json.dumps(drive.run(json.loads(_sc), '/nift')['summary'])`))
}

report.pyodide = { version: py.version }
const first = runPy(variants.markers_on)
boot.session_init_first_s = first.init_s
mem.after_first_mission = { heap: heap(py), rss: rss() }
for (const [k, sc] of Object.entries(variants)) {
  const runs = [0, 1, 2].map(() => runPy(sc))
  const best = runs.reduce((a, b) => (a.run_s < b.run_s ? a : b))
  report.pyodide[k] = { ...best, ms_per_tick: (best.run_s * 1000) / best.ticks }
  console.log(`pyodide ${k}: ${report.pyodide[k].ms_per_tick.toFixed(3)} ms/tick, ${best.x_realtime}x real time, init ${best.init_s} s`)
}
py.globals.set('_sc', JSON.stringify(variants.markers_off))
report.pyodide.profile = JSON.parse(py.runPython(`json.dumps(profile(json.loads(_sc), '/nift'))`))
console.log(`pyodide profile shares: ${JSON.stringify(report.pyodide.profile.share)}`)
report.boot = boot
console.log(`boot    ${JSON.stringify(boot)}`)

// Many missions in one interpreter, as the page's reset does.
const heaps = []
for (let i = 0; i < RESETS; i++) {
  runPy({ ...variants.markers_off, ticks: 200 })
  heaps.push(heap(py))
}
mem.after_resets = { n: RESETS, heap_first: heaps[0], heap_last: heaps.at(-1), rss: rss() }
report.memory = mem
console.log(`memory  ${JSON.stringify(mem)}`)

// ---------------------------------------------------------------- first load

const sizeOf = (buf) => ({ raw: mb(buf.length), gzip: mb(zlib.gzipSync(buf, { level: 9 }).length),
                           brotli: mb(zlib.brotliCompressSync(buf).length) })
const files = {}
for (const f of ['pyodide.mjs', 'pyodide.asm.js', 'pyodide.asm.wasm', 'python_stdlib.zip', 'pyodide-lock.json'])
  files[`pyodide/${f}`] = sizeOf(readFileSync(path.join(pyodideDir, f)))
// Only the wheels the page loads (ARCH section 9). Other tests may leave more in the cache.
for (const f of readdirSync(pyodideDir).filter((x) => /^(numpy|pyyaml)-.*\.whl$/.test(x)))
  files[`pyodide/${f}`] = { raw: mb(readFileSync(path.join(pyodideDir, f)).length), gzip: null, brotli: null }
files['py/nift_bundle.zip'] = sizeOf(zip)
files['maps/lanelet2_odr2024.osm'] = sizeOf(readFileSync(MAP_FILE))
const dist = path.join(PLAYGROUND, 'web/dist/assets')
if (existsSync(dist)) {
  const js = Buffer.concat(readdirSync(dist).filter((f) => /\.(js|css)$/.test(f) && !/mockWorker/.test(f)).map((f) => readFileSync(path.join(dist, f))))
  files['page js+css (web/dist)'] = sizeOf(js)
}
// Wheels are zip files already, so the wire size is the raw size.
const wire = Object.values(files).reduce((s, v) => s + (v.brotli ?? v.raw), 0)
report.first_load = { files, wire_mb_brotli: Math.round(wire * 100) / 100 }
console.log('first load (MB raw / gzip / brotli):')
for (const [k, v] of Object.entries(files)) console.log(`  ${k.padEnd(60)} ${v.raw} / ${v.gzip ?? '-'} / ${v.brotli ?? '-'}`)
console.log(`  total on the wire (brotli where it applies): ${report.first_load.wire_mb_brotli} MB`)

// ---------------------------------------------------------------- the marker question

const on = report.pyodide.markers_on.ms_per_tick
const off = report.pyodide.markers_off.ms_per_tick
report.markers = {
  pyodide_ms_per_tick_on: on,
  pyodide_ms_per_tick_off: off,
  pyodide_share: (on - off) / on,
  native_share: (report.native.markers_on.ms_per_tick - report.native.markers_off.ms_per_tick) / report.native.markers_on.ms_per_tick,
  budget_10x_ms: 5.0,
}
console.log(`markers: ${(report.markers.pyodide_share * 100).toFixed(1)} % of Pyodide tick time (${on.toFixed(3)} vs ${off.toFixed(3)} ms/tick); native ${(report.markers.native_share * 100).toFixed(1)} %`)

if (jsonOut) writeFileSync(jsonOut, JSON.stringify(report, null, 1))
