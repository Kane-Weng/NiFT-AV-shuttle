// File: run_compare.mjs
//
// Description:
//   Runs the playground scenarios natively and in Pyodide and fails if the traces differ
//   (ARCH.md section 10, "same scenarios in Node + Pyodide, traces equal to native").
//
//   1. Builds the bundle with tools/build_bundle.py, as the page gets it.
//   2. Runs drive.py natively (python3) for every scenario in scenarios/.
//   3. Boots Pyodide 0.27.7 once, unpacks the bundle to /nift, writes the map, and runs the same
//      drive.py on each scenario in the one interpreter, as the page's reset does.
//   4. Compares message by message. Strings, booleans and the message order must be equal.
//      Numbers may differ by TOL (the maths libraries differ in the last bits, ARCH.md section 7).
//   5. Runs the browser worker's own code (web/src/worker/core.ts) on the first scenario with
//      step messages, and checks that its path, goal_ack and event messages and its final state
//      equal the native ones. Then it lets the pacing loop run at 1x and at max and reports the
//      achieved real-time factor.
//
//   cd nift_playground/tests/pyodide && npm ci && node run_compare.mjs [--only NAME] [--json OUT]
//   Exit code 0 when everything matches.

import { execFileSync } from 'node:child_process'
import { readFileSync, readdirSync, writeFileSync, mkdtempSync } from 'node:fs'
import { tmpdir } from 'node:os'
import path from 'node:path'
import { HERE, MAP_FILE, PLAYGROUND, PYTHON, buildBundle, fileUrl, loadCore, loadPyodideNode, nodeEnv } from './node_env.mjs'

const TOL = 1e-6
const args = process.argv.slice(2)
const only = args.includes('--only') ? args[args.indexOf('--only') + 1] : null
const jsonOut = args.includes('--json') ? args[args.indexOf('--json') + 1] : null
const skipWorker = args.includes('--no-worker')

const scenarioDir = path.join(HERE, 'scenarios')
const scenarios = readdirSync(scenarioDir)
  .filter((f) => f.endsWith('.json') && (!only || f.startsWith(only)))
  .sort()
  .map((f) => ({ name: f.replace(/\.json$/, ''), file: path.join(scenarioDir, f), data: JSON.parse(readFileSync(path.join(scenarioDir, f), 'utf8')) }))

const results = { scenarios: {}, worker: null, ok: true }
const fail = (msg) => {
  results.ok = false
  console.log(`FAIL ${msg}`)
}

// ---------------------------------------------------------------- comparison

/** Walk two JSON values. Returns {maxDiff, firstMismatch}. */
function compareValues(a, b, where, acc) {
  if (acc.firstMismatch) return
  if (typeof a === 'number' && typeof b === 'number') {
    const d = Math.abs(a - b)
    if (Number.isNaN(a) !== Number.isNaN(b) || d > TOL) acc.firstMismatch = `${where}: ${a} vs ${b}`
    else if (d > acc.maxDiff) acc.maxDiff = d
    return
  }
  if (Array.isArray(a) && Array.isArray(b)) {
    if (a.length !== b.length) return void (acc.firstMismatch = `${where}: length ${a.length} vs ${b.length}`)
    a.forEach((x, i) => compareValues(x, b[i], `${where}[${i}]`, acc))
    return
  }
  if (a && b && typeof a === 'object' && typeof b === 'object') {
    const ka = Object.keys(a).sort()
    const kb = Object.keys(b).sort()
    if (ka.join() !== kb.join()) return void (acc.firstMismatch = `${where}: keys ${ka} vs ${kb}`)
    for (const k of ka) compareValues(a[k], b[k], `${where}.${k}`, acc)
    return
  }
  if (a !== b) acc.firstMismatch = `${where}: ${JSON.stringify(a)?.slice(0, 200)} vs ${JSON.stringify(b)?.slice(0, 200)}`
}

function compareTraces(native, pyo) {
  const acc = { maxDiff: 0, firstMismatch: null }
  const n = Math.min(native.length, pyo.length)
  for (let i = 0; i < n && !acc.firstMismatch; i++) {
    const [ta, ma] = native[i]
    const [tb, mb] = pyo[i]
    if (ta !== tb || ma.type !== mb.type) {
      acc.firstMismatch = `message ${i}: tick ${ta} ${ma.type} vs tick ${tb} ${mb.type}`
      break
    }
    compareValues(ma, mb, `message ${i} (tick ${ta}, ${ma.type})`, acc)
  }
  if (!acc.firstMismatch && native.length !== pyo.length) acc.firstMismatch = `message count ${native.length} vs ${pyo.length}`
  return acc
}

// ---------------------------------------------------------------- native

const bundleBase = buildBundle()
const zipPath = path.join(bundleBase, 'py/nift_bundle.zip')
const work = mkdtempSync(path.join(tmpdir(), 'nift-compare-'))
const native = {}
for (const sc of scenarios) {
  const out = path.join(work, `${sc.name}.native.json`)
  execFileSync(PYTHON, [path.join(HERE, 'drive.py'), sc.file, '--out', out], { stdio: ['ignore', 'ignore', 'inherit'] })
  native[sc.name] = JSON.parse(readFileSync(out, 'utf8'))
  console.log(`native  ${sc.name}: ${JSON.stringify(native[sc.name].summary)}`)
}

// ---------------------------------------------------------------- Pyodide, drive.py

const t0 = performance.now()
const py = await loadPyodideNode()
const tBoot = performance.now()
await py.loadPackage(['numpy', 'pyyaml'])
const tPkgs = performance.now()
py.unpackArchive(new Uint8Array(readFileSync(zipPath)), 'zip', { extractDir: '/nift' })
py.FS.mkdirTree('/nift/maps')
py.FS.writeFile('/nift/maps/lanelet2_odr2024.osm', new Uint8Array(readFileSync(MAP_FILE)))
py.FS.mkdirTree('/nift/nift_playground/tests/pyodide')
py.FS.writeFile('/nift/nift_playground/tests/pyodide/drive.py', new Uint8Array(readFileSync(path.join(HERE, 'drive.py'))))
const tBundle = performance.now()
py.runPython(`import sys; sys.path.insert(0, '/nift/nift_playground/tests/pyodide'); import drive, json`)
const loadedPkgs = Object.keys(py.loadedPackages).sort()
results.pyodide = {
  version: py.version,
  boot_s: (tBoot - t0) / 1e3,
  packages_s: (tPkgs - tBoot) / 1e3,
  bundle_and_map_s: (tBundle - tPkgs) / 1e3,
  loaded_packages: loadedPkgs,
  bundle_bytes: readFileSync(zipPath).length,
}
for (const p of loadedPkgs) if (!['numpy', 'pyyaml'].includes(p)) fail(`Pyodide loaded a package outside ARCH section 9: ${p}`)

for (const sc of scenarios) {
  py.globals.set('_sc_json', JSON.stringify(sc.data))
  let res
  try {
    res = JSON.parse(py.runPython(`json.dumps(drive.run(json.loads(_sc_json), '/nift'))`))
  } catch (e) {
    fail(`${sc.name}: Pyodide run raised\n${String(e.message).slice(-2000)}`)
    continue
  }
  const cmp = compareTraces(native[sc.name].messages, res.messages)
  const heapMb = py._module.HEAP8.length / 1e6
  results.scenarios[sc.name] = { native: native[sc.name].summary, pyodide: res.summary, maxDiff: cmp.maxDiff,
                                 mismatch: cmp.firstMismatch, heap_mb: heapMb }
  console.log(`pyodide ${sc.name}: ${JSON.stringify(res.summary)}`)
  if (cmp.firstMismatch) fail(`${sc.name}: ${cmp.firstMismatch}`)
  else console.log(`ok      ${sc.name}: ${res.messages.length} messages equal, max numeric diff ${cmp.maxDiff.toExponential(2)}, heap ${heapMb.toFixed(1)} MB`)
}

// ---------------------------------------------------------------- the worker's own code

if (!skipWorker && scenarios.length) {
  const sc = scenarios[0]
  try {
    results.worker = await runWorker(sc, native[sc.name])
  } catch (e) {
    fail(`worker: ${e.stack ?? e}`)
  }
}

if (jsonOut) writeFileSync(jsonOut, JSON.stringify(results, null, 1))
console.log(results.ok ? 'ALL OK' : 'FAILED')
process.exit(results.ok ? 0 : 1)

// ---------------------------------------------------------------- helpers

async function runWorker(sc, nat) {
  const { WorkerCore } = await loadCore()
  const posted = []
  let waiter = null
  const core = new WorkerCore(nodeEnv((m) => {
    posted.push(m)
    if (waiter && waiter.pred(m)) {
      const w = waiter
      waiter = null
      w.resolve(m)
    }
  }))
  const until = (pred, ms = 120000) =>
    new Promise((resolve, reject) => {
      const hit = posted.find(pred)
      if (hit) return resolve(hit)
      waiter = { pred, resolve }
      setTimeout(() => reject(new Error(`timeout waiting in worker test (${ms} ms)`)), ms).unref()
    })
  const send = (m) => core.onMessage(m)

  const w0 = performance.now()
  // The worker starts paced at 1x as soon as it is ready. Pause it first, so the step test is exact.
  send({ type: 'pause' })
  send({ type: 'init', baseUrl: fileUrl(bundleBase + '/'), mapUrl: fileUrl(MAP_FILE), launch: sc.data.launch })
  const ready = await until((m) => m.type === 'ready' || (m.type === 'error' && m.fatal))
  if (ready.type === 'error') throw new Error(`worker init: ${ready.message}\n${ready.traceback}`)
  const bootS = (performance.now() - w0) / 1e3
  const stages = posted.filter((m) => m.type === 'progress').map((m) => m.stage)

  // Replay the scenario with step messages, the way a page in pause-and-step would.
  let tick = 0
  for (const [i, ev] of sc.data.events.entries()) {
    if (ev.tick > tick) send({ type: 'step', n: ev.tick - tick })
    tick = ev.tick
    send({ ...ev.msg, id: `ev${i}` })
  }
  send({ type: 'step', n: sc.data.ticks - tick })
  tick = sc.data.ticks
  const keep = (m) => ['path', 'goal_ack', 'event'].includes(m.type)
  const nativeSel = nat.messages.map(([, m]) => m).filter(keep)
  const workerSel = posted.filter(keep)
  const acc = { maxDiff: 0, firstMismatch: null }
  compareValues(nativeSel, workerSel, 'worker path/goal_ack/event', acc)
  const lastState = posted.filter((m) => m.type === 'state').pop()
  const nativeState = nat.messages.map(([, m]) => m).filter((m) => m.type === 'state').pop()
  const strip = (s) => { const { rtf, factor, paused, ...rest } = s ?? {}; return rest }
  compareValues(strip(nativeState), strip(lastState), 'worker final state', acc)
  if (acc.firstMismatch) fail(`worker: ${acc.firstMismatch}`)
  else console.log(`ok      worker (core.ts, step mode): ${workerSel.length} path/goal_ack/event messages and the final state equal native`)
  if (!lastState?.paused) fail('worker: state after step should say paused')

  // Pacing: reset, then 1x for 3 s of wall time, then max for 3 s.
  send({ type: 'reset', id: 'r' })
  await until((m) => m.type === 'reply' && m.id === 'r')
  const pace = async (factor, ms) => {
    send({ type: 'speed', factor })
    send({ type: 'resume_sim' })
    const n0 = posted.length
    const tk0 = core.stats.ticks
    const a = performance.now()
    await new Promise((r) => setTimeout(r, ms))
    send({ type: 'pause' })
    const wall = (performance.now() - a) / 1e3
    const states = posted.slice(n0).filter((m) => m.type === 'state')
    const ticks = core.stats.ticks - tk0
    return { factor, wall_s: wall, ticks, achieved: (ticks * 0.05) / wall, reported_rtf: states.at(-2)?.rtf ?? null,
             states_per_s: states.length / wall }
  }
  const real = await pace(1, 3000)
  const max = await pace('max', 3000)
  console.log(`pacing  1x: ${real.achieved.toFixed(2)}x achieved (rtf ${real.reported_rtf}), ${real.states_per_s.toFixed(1)} state/s`)
  console.log(`pacing max: ${max.achieved.toFixed(1)}x achieved (rtf ${max.reported_rtf}), ${max.states_per_s.toFixed(1)} state/s`)
  if (Math.abs(real.achieved - 1) > 0.15) fail(`worker: 1x pacing achieved ${real.achieved.toFixed(2)}x`)
  return { boot_s: bootS, stages, bootMs: core.stats.bootMs, real, max, heap_mb: core.py._module.HEAP8.length / 1e6 }
}
