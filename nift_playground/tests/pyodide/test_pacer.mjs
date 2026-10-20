// File: test_pacer.mjs
//
// Description:
//   Unit tests of the worker's pacing (web/src/worker/core.ts, Pacer) and of its message handling
//   with a fake Python session, without Pyodide. node --test test_pacer.mjs

import test from 'node:test'
import assert from 'node:assert/strict'
import { loadCore } from './node_env.mjs'

const { Pacer, WorkerCore, BUDGET_MS, MAX_CATCHUP_MS } = await loadCore()

test('1x owes one tick per 50 ms of wall time', () => {
  const p = new Pacer()
  p.costMs = 0.5
  p.rebase(0)
  assert.equal(p.ticksFor(49), 0)
  assert.equal(p.ticksFor(50), 1)
  assert.equal(p.ticksFor(150), 2)
  assert.ok(p.delayMs() > 0 && p.delayMs() <= 50)
})

test('10x and 0.5x', () => {
  const p = new Pacer()
  p.costMs = 0.1
  p.setFactor(10)
  p.rebase(0)
  assert.equal(p.ticksFor(100), 20)
  p.setFactor(0.5)
  p.rebase(0)
  assert.equal(p.ticksFor(100), 1)
})

test('factor is clamped to 0.1..100 and rejects junk', () => {
  const p = new Pacer()
  assert.equal(p.setFactor(1000), 100)
  assert.equal(p.setFactor(0.01), 0.1)
  assert.equal(p.setFactor('max'), 'max')
  assert.throws(() => p.setFactor(-1))
  assert.throws(() => p.setFactor('fast'))
})

test('a turn never runs more than the budget allows, and no backlog builds up', () => {
  const p = new Pacer()
  p.costMs = 2 // 6 ticks fit in 12 ms
  p.setFactor(100)
  p.rebase(0)
  const fit = Math.floor(BUDGET_MS / 2)
  assert.equal(p.ticksFor(100), fit)
  assert.ok(p.owed <= fit)
})

test('a long stall (background tab) is not caught up', () => {
  const p = new Pacer()
  p.costMs = 0.01
  p.rebase(0)
  const n = p.ticksFor(60_000)
  assert.equal(n, Math.floor(MAX_CATCHUP_MS / 50))
})

test('max runs a budget of ticks every turn; paused runs none', () => {
  const p = new Pacer()
  p.costMs = 0.4
  p.setFactor('max')
  p.rebase(0)
  assert.equal(p.ticksFor(1), Math.floor(BUDGET_MS / 0.4))
  assert.equal(p.delayMs(), 0)
  p.paused = true
  assert.equal(p.ticksFor(1000), 0)
  assert.equal(p.delayMs(), Infinity)
})

test('rtf is sim seconds per wall second over the last second', () => {
  const p = new Pacer()
  for (let t = 0; t <= 1000; t += 100) p.record(t, 20, 1)
  assert.ok(Math.abs(p.rtf(1000) - 10) < 1e-9)
})

// A fake session in place of Python, to test the worker's protocol handling.
function fakeCore() {
  const posted = []
  const handled = []
  let tick = 0
  let now = 0
  const later = []
  const env = {
    loadPyodide: async () => ({
      version: '0.27.7',
      loadPackage: async () => {},
      unpackArchive: () => {},
      FS: { mkdirTree() {}, writeFile() {} },
      runPython() {},
      globals: {
        get(name) {
          return {
            worker_open: (j) => JSON.stringify([{ type: 'ready', launch: JSON.parse(j).launch ?? {} }, { type: 'state', t: 0, tick: 0 }]),
            worker_handle: (j) => {
              handled.push(JSON.parse(j))
              return JSON.stringify([{ type: 'reply', id: JSON.parse(j).id, ok: true, message: '' }])
            },
            worker_advance: (n) => {
              tick += Number(n)
              return JSON.stringify([{ type: 'state', t: tick * 0.05, tick }])
            },
            worker_dt: () => 0.05,
          }[name]
        },
      },
    }),
    fetchBytes: async () => new Uint8Array(1),
    post: (m) => posted.push(m),
    now: () => now,
    later: (fn) => later.push(fn),
    soon: (fn) => later.push(fn),
  }
  const core = new WorkerCore(env)
  return { core, posted, handled, tick: () => tick, setNow: (t) => (now = t), runLater: () => later.splice(0).forEach((f) => f()) }
}

test('init posts the progress stages in order, then ready; queued commands run after ready', async () => {
  const f = fakeCore()
  f.core.onMessage({ type: 'pause' })
  f.core.onMessage({ type: 'init', baseUrl: '/b/', mapUrl: '/m.osm', launch: { spawn: '04' } })
  f.core.onMessage({ type: 'goal', x: 1, y: 2, id: 'g' })
  await new Promise((r) => setTimeout(r, 10))
  const stages = f.posted.filter((m) => m.type === 'progress').map((m) => m.stage)
  assert.deepEqual(stages, ['pyodide', 'packages', 'bundle', 'map', 'nodes'])
  const ready = f.posted.find((m) => m.type === 'ready')
  assert.equal(ready.pyodide, '0.27.7')
  assert.deepEqual(ready.launch, { spawn: '04' })
  assert.equal(f.handled[0].type, 'goal')
  assert.ok(f.posted.indexOf(ready) < f.posted.findIndex((m) => m.type === 'reply' && m.id === 'g'))
})

test('step runs exactly n ticks, pauses, and posts the state with paused and factor', async () => {
  const f = fakeCore()
  f.core.onMessage({ type: 'init', baseUrl: '/b/', mapUrl: '/m.osm' })
  await new Promise((r) => setTimeout(r, 10))
  f.core.onMessage({ type: 'step', n: 7, id: 's' })
  assert.equal(f.tick(), 7)
  const st = f.posted.filter((m) => m.type === 'state').pop()
  assert.equal(st.tick, 7)
  assert.equal(st.paused, true)
  assert.equal(st.factor, 1)
  assert.ok(f.posted.some((m) => m.type === 'reply' && m.id === 's' && m.ok))
  f.core.onMessage({ type: 'step', n: 0, id: 'bad' })
  assert.ok(f.posted.some((m) => m.type === 'reply' && m.id === 'bad' && !m.ok))
  // pacing messages never reach Python
  assert.equal(f.handled.length, 0)
})

test('the loop advances with wall time at 1x and stops on pause', async () => {
  const f = fakeCore()
  f.core.onMessage({ type: 'init', baseUrl: '/b/', mapUrl: '/m.osm' })
  await new Promise((r) => setTimeout(r, 10))
  for (let t = 0; t <= 1000; t += 10) {
    f.setNow(t)
    f.runLater()
  }
  assert.ok(Math.abs(f.tick() - 20) <= 1, `ticks after 1 s at 1x: ${f.tick()}`)
  f.core.onMessage({ type: 'pause' })
  const at = f.tick()
  for (let t = 1000; t <= 2000; t += 10) {
    f.setNow(t)
    f.runLater()
  }
  assert.equal(f.tick(), at)
})
