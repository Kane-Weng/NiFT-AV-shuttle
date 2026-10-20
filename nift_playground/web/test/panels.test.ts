// The panels' pure parts: the scenario codec (ARCH.md 8.5), the ready.presets reader and the
// obstacle geometry.

import { describe, expect, it } from 'vitest'
import { deflateSync, inflateSync } from 'node:zlib'
import { decodeScenario, encodeScenario, parseScenarioInput, scenarioCodeFrom, urlWithScenario, validateScenario } from '../src/scenario/codec'
import { faultPresets, launchOptions, routePresets } from '../src/panels/presets'
import { boxPoly, centroid, nextObstacleId, pointInPolygon, translate } from '../src/panels/geom'
import type { ReadyMsg, Scenario } from '../src/protocol'

const scenario: Scenario = {
  v: 1,
  launch: { localization: 'ekf', perception: true, params: { trajectory_controller: { base_speed: 2.5 } } },
  events: [
    { tick: 0, msg: { type: 'goal', x: 91.98, y: -62.95, yaw: -1.612 } },
    { tick: 3, msg: { type: 'cmd', name: 'start' } },
    { tick: 40, msg: { type: 'obstacle_add', oid: 'o1', box: { x: 1, y: 2, yaw: 0, length: 4.5, width: 1.9 } } },
    { tick: 200, msg: { type: 'gnss_fault', preset: 'dropout', params: { duration_s: 1.5, antenna: 'both' } } },
  ],
  note: 'unit test, ü',
}

describe('scenario codec', () => {
  it('round-trips through base64url deflate', async () => {
    const code = await encodeScenario(scenario)
    expect(code).toMatch(/^[A-Za-z0-9_-]+$/)
    expect(await decodeScenario(code)).toEqual(scenario)
  })

  it('is what Python reads with zlib and urlsafe base64', async () => {
    const code = await encodeScenario(scenario)
    const b64 = code.replace(/-/g, '+').replace(/_/g, '/')
    const json = inflateSync(Buffer.from(b64, 'base64')).toString('utf8')
    expect(JSON.parse(json)).toEqual(scenario)
    // and the other way: a code made with zlib decodes in the page
    const fromPy = deflateSync(Buffer.from(JSON.stringify(scenario))).toString('base64url')
    expect(await decodeScenario(fromPy)).toEqual(scenario)
  })

  it('accepts plain base64url JSON', async () => {
    const plain = Buffer.from(JSON.stringify(scenario)).toString('base64url')
    expect(await decodeScenario(plain)).toEqual(scenario)
  })

  it('puts the code in the hash and reads it from hash or query', async () => {
    const code = await encodeScenario(scenario)
    const url = urlWithScenario('http://localhost:4173/?sim=mock&scenario=old#x', code)
    expect(url).toBe(`http://localhost:4173/?sim=mock#scenario=${code}`)
    expect(scenarioCodeFrom(url)).toBe(code)
    expect(scenarioCodeFrom(`http://h/?sim=mock&scenario=${code}`)).toBe(code)
    expect(scenarioCodeFrom('http://h/?sim=mock')).toBeNull()
  })

  it('reads a pasted link, code or JSON', async () => {
    const code = await encodeScenario(scenario)
    expect(await parseScenarioInput(`http://h/#scenario=${code}`)).toEqual(scenario)
    expect(await parseScenarioInput(code)).toEqual(scenario)
    expect(await parseScenarioInput(`#scenario=${code}`)).toEqual(scenario)
    expect(await parseScenarioInput(JSON.stringify(scenario))).toEqual(scenario)
  })

  it('refuses what is not a v1 scenario', async () => {
    expect(() => validateScenario({ v: 2, launch: {}, events: [] })).toThrow(/version/)
    expect(() => validateScenario({ v: 1, launch: {}, events: [{ tick: 'x' }] })).toThrow(/event/)
    await expect(decodeScenario('bm90IGpzb24')).rejects.toThrow()
    await expect(parseScenarioInput('http://h/?sim=mock')).rejects.toThrow(/no scenario/)
  })
})

const ready = (presets: unknown, launch: Record<string, unknown> = {}): ReadyMsg => ({ type: 'ready', pyodide: null, python: null, launch, presets: presets as ReadyMsg['presets'] })

describe('ready presets', () => {
  it('reads one list with a kind per item (launch_set.presets plus live)', () => {
    const r = ready({
      gnss_faults: [
        { name: 'dropout_3s_straight', kind: 'scheduled', route: '01', faults: [{ kind: 'dropout' }], target: { stop: true } },
        { name: 'dropout', kind: 'sim', params: { duration_s: 1.5 } },
        { name: 'outage', kind: 'link', params: { duration_s: 1.5 } },
      ],
      routes: [{ id: '01', spawn: [1, 2, 3], goal: [4, 5, 6] }, { id: 'bad' }],
    })
    const f = faultPresets(r)
    expect(f.scheduled.map((p) => p.name)).toEqual(['dropout_3s_straight'])
    expect(f.live.map((p) => `${p.kind}:${p.name}`)).toEqual(['sim:dropout', 'link:outage'])
    expect(routePresets(r).map((x) => x.id)).toEqual(['01'])
  })

  it('reads {scenarios, live} (sim_backend.gnss_fault_presets)', () => {
    const f = faultPresets(ready({ gnss_faults: { scenarios: [{ name: 'control_straight', route: '01', faults: [] }], live: [{ name: 'repeat', kind: 'link', params: {} }] }, routes: [] }))
    expect(f.scheduled).toHaveLength(1)
    expect(f.scheduled[0].kind).toBe('scheduled')
    expect(f.live[0].name).toBe('repeat')
  })

  it('survives a missing ready and finds the launch options', () => {
    expect(faultPresets(null)).toEqual({ live: [], scheduled: [] })
    expect(routePresets(null)).toEqual([])
    expect(launchOptions(ready({}, { options: { localization: 'ekf' } })).localization).toBe('ekf')
    expect(launchOptions(ready({}, { localization: 'shadow' })).localization).toBe('shadow')
  })
})

describe('obstacle geometry', () => {
  it('builds a box around its centre with its heading', () => {
    const b = boxPoly({ x: 10, y: -5, yaw: Math.PI / 2, length: 4, width: 2 })
    const c = centroid(b)
    expect(c[0]).toBeCloseTo(10, 12)
    expect(c[1]).toBeCloseTo(-5, 12)
    // length along +y after a quarter turn
    const ys = b.map((p) => p[1])
    expect(Math.max(...ys) - Math.min(...ys)).toBeCloseTo(4, 12)
    expect(pointInPolygon([10, -3.5], b)).toBe(true)
    expect(pointInPolygon([11.5, -5], b)).toBe(false)
  })

  it('translates and names new obstacles', () => {
    expect(translate([[0, 0]], 1, 2)).toEqual([[1, 2]])
    expect(nextObstacleId([])).toBe('o1')
    expect(nextObstacleId([{ oid: 'o1' }, { oid: 'o3' }])).toBe('o2')
  })
})
