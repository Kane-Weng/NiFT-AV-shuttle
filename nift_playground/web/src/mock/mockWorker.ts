/*
 * File: mockWorker.ts
 *
 * Description:
 *   A stand-in for the Pyodide sim worker. It speaks the same protocol (src/protocol.ts) and
 *   drives a fake shuttle along a lanelet route with a kinematic bicycle and pure pursuit, so the
 *   page can be built and tested without Pyodide. None of this logic is the stack's. The real
 *   planner, controller, GNSS chain and safety logic run only in the Pyodide worker.
 *
 *   What the mock fakes, so the panels have something to show: the controller's modes and service
 *   replies (worded like trajectory_controller's), manual teleop, the ECU E-stop latch and power
 *   cycle, GNSS faults as missing fixes (SAFE_STOP 0.85 s after the last fix, as the reference
 *   runs show), a "perceived" set when the perception option is on, set_param staged for the next
 *   reset (base_speed is the mock's cruise speed), and scenarios (record and replay by tick).
 *   Its presets are mock ones: two routes picked from the map and the live fault names. Replies say
 *   "mock" wherever the real worker would do something the mock does not.
 *
 *   Open the page with ?sim=mock to use it (the default until the Pyodide worker lands).
 */

/// <reference lib="webworker" />

import type {
  ControlMode,
  DiagItem,
  FaultPreset,
  LiveFaultPreset,
  NavStatus,
  Obstacle,
  PageToWorker,
  ReadyMsg,
  RoutePreset,
  Scenario,
  WorkerToPage,
  XY,
} from '../protocol'
import { parseOsm } from '../map/osm'
import { buildLaneletMap, type LaneletMap } from '../map/lanelets'
import { MapProjector } from '../geo/utm'
import { MAP_ORIGIN_LAT, MAP_ORIGIN_LON, VEHICLE } from '../geo/site'
import { nearestLanelet, planPath } from './route'

declare const self: DedicatedWorkerGlobalScope

const DT = 0.05 // sim step [s]
const ACCEL = 0.8
const DECEL = 1.5
const ESTOP_DECEL = 5.0
const LOOKAHEAD = 4.0
const ARRIVE_TOL = 1.0
const MAX_STEER = (VEHICLE.maxSteeringDeg * Math.PI) / 180
const STALE_S = 0.5 // mock: fix age that counts as stale
const GNSS_STOP_S = 0.85 // reference runs: SAFE_STOP this long after the last fix
const PERCEPTION_RANGE = 30
const RECORDED = new Set(['goal', 'cmd', 'teleop', 'initialpose', 'obstacle_add', 'obstacle_move', 'obstacle_remove', 'set_obstacles', 'gnss_fault'])

// Live fault names of playground/sim_backend.py (a copy for the mock's ready message)
const LIVE_PRESETS: LiveFaultPreset[] = [
  { name: 'dropout', kind: 'sim', params: { duration_s: 1.5, antenna: 'both' }, doc: 'no fixes from the antenna(s)' },
  { name: 'jump', kind: 'sim', params: { east_m: 0.5, north_m: 0, antenna: 'both', duration_s: 0 }, doc: 'fixes offset by (east_m, north_m)' },
  { name: 'noise', kind: 'sim', params: { sigma_m: 0.5, duration_s: 5, antenna: 'both' }, doc: 'fixes scattered' },
  { name: 'stuck', kind: 'sim', params: { duration_s: 2 }, doc: 'the feed repeats one epoch' },
  { name: 'outage', kind: 'link', params: { duration_s: 1.5 }, doc: 'Octane link down: both antennas silent' },
  { name: 'outage_front', kind: 'link', params: { duration_s: 1.5 }, doc: 'front antenna silent on the link' },
]
// Presets that stop the fixes (in the mock; the rest only mark GPS health bad)
const NO_FIX = new Set(['dropout', 'outage', 'stuck', 'freeze', 'burst'])

type Launch = {
  localization: string
  gnss_feed: string
  gnss_faults: string | unknown[]
  gnss_dead_zones: string
  gnss_seed: number
  gnss_checks: string
  perception: boolean
  filter_type: string
  spawn: string | { x: number; y: number; yaw: number } | null
  params: Record<string, Record<string, unknown>>
}
const DEFAULT_LAUNCH: Launch = {
  localization: 'direct',
  gnss_feed: 'ideal',
  gnss_faults: '',
  gnss_dead_zones: '',
  gnss_seed: 1,
  gnss_checks: 'true',
  perception: false,
  filter_type: '',
  spawn: null,
  params: {},
}

const post = (m: WorkerToPage) => self.postMessage(m)

let map: LaneletMap | null = null
let routes: RoutePreset[] = []
let launch: Launch = structuredClone(DEFAULT_LAUNCH)
let t = 0
let pose = { x: 0, y: 0, yaw: 0 }
let poseEst = { x: 0, y: 0, yaw: 0 }
let defaultSpawn = { x: 0, y: 0, yaw: 0 }
let speed = 0
let steer = 0
let mode: ControlMode = 'IDLE'
let path: XY[] = []
let pathIdx = 0
let goal: XY | null = null
let obstacles: Obstacle[] = []
let perceivedKey = ''
let perceived: XY[][] | null = null
let paused = false
let factor: number | 'max' = 1
let tick = 0
let stateHz = 10
let lastDiag = -1
let timer: ReturnType<typeof setInterval> | null = null
let manual = { v: 0, steer: 0 }
let ecuLatched = false
let ecuCommanded = false
let lastFix = 0
let gpsOk = true
let faults: { preset: string; from: number; to: number }[] = []
let recorded: { tick: number; msg: PageToWorker }[] = []
let replay: { tick: number; msg: PageToWorker }[] = []
let replaying = false

function cruise(): number {
  const v = launch.params.trajectory_controller?.base_speed
  return typeof v === 'number' ? v : 3.0
}

function log(level: 'debug' | 'info' | 'warn' | 'error', text: string, node = 'mock') {
  post({ type: 'log', t, level, text, node })
}

const STATUS: Record<ControlMode, NavStatus> = {
  IDLE: 'IDLE',
  AUTONOMOUS: 'NAVIGATING',
  MANUAL: 'MANUAL',
  ESTOP: 'ESTOP',
  SAFE_STOP: 'SAFE_STOP',
}

function setMode(m: ControlMode) {
  if (m === mode) return
  const was = mode
  mode = m
  post({ type: 'event', kind: 'mode', t, data: { status: STATUS[m], mode: m, from: STATUS[was] } })
}

function fixAge(): number {
  return t - lastFix
}

function locStatus(): 'OK' | 'DEGRADED' | 'LOST' | null {
  if (launch.localization === 'direct') return null
  const a = fixAge()
  return a > GNSS_STOP_S ? 'LOST' : a > 0.2 ? 'DEGRADED' : 'OK'
}

/** Signed distance from the shuttle to the path (left of the path positive), or null. */
function crossTrack(): number | null {
  if (path.length < 2) return null
  let best: number | null = null
  for (let i = Math.max(0, pathIdx - 5); i < Math.min(path.length - 1, pathIdx + 40); i++) {
    const [ax, ay] = path[i]
    const [bx, by] = path[i + 1]
    const dx = bx - ax
    const dy = by - ay
    const len2 = dx * dx + dy * dy || 1e-9
    const u = Math.max(0, Math.min(1, ((pose.x - ax) * dx + (pose.y - ay) * dy) / len2))
    const ex = pose.x - (ax + u * dx)
    const ey = pose.y - (ay + u * dy)
    const d = Math.hypot(ex, ey) * Math.sign(dx * ey - dy * ex || 1)
    if (best === null || Math.abs(d) < Math.abs(best)) best = d
  }
  return best
}

function sendState() {
  const end = path.length ? path[path.length - 1] : null
  post({
    type: 'state',
    t,
    tick,
    paused,
    factor,
    rtf: paused ? 0 : typeof factor === 'number' ? factor : 100,
    pose: { ...pose },
    poseEst: { ...poseEst },
    speed,
    steer,
    status: STATUS[mode],
    mode,
    targetSpeed: mode === 'AUTONOMOUS' ? cruise() : mode === 'MANUAL' ? manual.v : 0,
    cte: mode === 'AUTONOMOUS' ? crossTrack() : null,
    distEnd: end && mode === 'AUTONOMOUS' ? Math.hypot(end[0] - pose.x, end[1] - pose.y) : null,
    locStatus: locStatus(),
    locMessage: launch.localization === 'direct' ? null : `mock filter (${launch.localization}), fix age ${fixAge().toFixed(2)} s`,
    gear: speed < -0.01 ? 'R' : speed > 0.01 || mode === 'AUTONOMOUS' || mode === 'MANUAL' ? 'D' : 'N',
    ecu: ecuLatched ? 'ESTOP' : ecuCommanded ? 'ACTIVE' : 'NO_COMMAND',
    pedal: !ecuLatched && (mode === 'AUTONOMOUS' || mode === 'MANUAL') && Math.abs(speed) < Math.abs(mode === 'MANUAL' ? manual.v : cruise()) ? 0.3 : 0,
    brake: ecuLatched || mode === 'ESTOP' || mode === 'SAFE_STOP' ? 1 : 0,
    gnss: { ok: gpsOk, stale: fixAge() > STALE_S, fixAge: fixAge() },
    goal,
  })
}

function diag(): DiagItem[] {
  const item = (name: string, level: 0 | 1 | 2, message: string, values: Record<string, string> = {}): DiagItem => ({ name, hardwareId: 'mock', level, message, values })
  const age = fixAge()
  return [
    item('kinematic_sim: Sim', 0, paused ? 'paused' : `x${factor}`, { 'Sim time': t.toFixed(2), Tick: String(tick), Note: 'mock worker, not the stack' }),
    item('kinematic_sim: Virtual ECU', ecuLatched ? 2 : 0, ecuLatched ? 'E-stop latched — call /carla/dbw_power_cycle to release' : 'OK', { Mode: ecuLatched ? 'ESTOP' : 'ACTIVE' }),
    item('beacon_node: Octane feed', age > STALE_S ? 2 : gpsOk ? 0 : 1, age > STALE_S ? 'no fresh fix' : gpsOk ? 'OK' : 'GPS health bad', { 'Fix age': `${age.toFixed(2)} s` }),
    item('waypoint_node: Planner', !goal || path.length > 0 ? 0 : 1, path.length ? `${path.length} points` : 'no path', { Goal: goal ? `${goal[0].toFixed(1)}, ${goal[1].toFixed(1)}` : '-' }),
    item('trajectory_controller: Controller', mode === 'SAFE_STOP' ? 1 : mode === 'ESTOP' ? 2 : 0, mode === 'SAFE_STOP' ? 'SAFE STOP — call /resume to recover' : mode, {
      'Control Mode': mode,
      'Nominal Speed': `${cruise().toFixed(2)} m/s`,
      Obstacles: launch.perception ? `${perceived?.length ?? 0} perceived` : 'filter none (not watched)',
    }),
    ...(launch.localization === 'direct' ? [] : [item('localization_node: Filter', locStatus() === 'OK' ? 0 : locStatus() === 'DEGRADED' ? 1 : 2, locStatus() ?? '', { Mode: launch.localization })]),
  ]
}

/** Is a perceived obstacle within 6 m ahead and 1.5 m of the shuttle's line? */
function obstacleAhead(): boolean {
  const c = Math.cos(pose.yaw)
  const s = Math.sin(pose.yaw)
  for (const poly of perceived ?? []) {
    for (const [px, py] of poly) {
      const dx = px - pose.x
      const dy = py - pose.y
      const ahead = dx * c + dy * s
      const side = -dx * s + dy * c
      if (ahead > 0 && ahead < 6 && Math.abs(side) < 1.5) return true
    }
  }
  return false
}

/** Mock perception: obstacles whose centre is within range and in front of the shuttle. */
function perceive() {
  if (!launch.perception) {
    if (perceived !== null) {
      perceived = null
      perceivedKey = ''
      postObstacles()
    }
    return
  }
  const seen = obstacles.filter((o) => {
    const cx = o.poly.reduce((a, p) => a + p[0], 0) / o.poly.length
    const cy = o.poly.reduce((a, p) => a + p[1], 0) / o.poly.length
    const dx = cx - pose.x
    const dy = cy - pose.y
    return Math.hypot(dx, dy) < PERCEPTION_RANGE && dx * Math.cos(pose.yaw) + dy * Math.sin(pose.yaw) > 0
  })
  const key = seen.map((o) => o.oid + JSON.stringify(o.poly[0])).join('|')
  if (key !== perceivedKey || perceived === null) {
    perceivedKey = key
    perceived = seen.map((o) => o.poly)
    postObstacles()
  }
}

function postObstacles() {
  post({ type: 'obstacles', polygons: obstacles, perceived })
}

function gnssStep() {
  const active = faults.filter((f) => t >= f.from && t < f.to)
  if (!active.some((f) => NO_FIX.has(f.preset))) {
    lastFix = t
    poseEst = { ...pose }
  }
  const ok = active.length === 0
  if (ok !== gpsOk) {
    gpsOk = ok
    post({ type: 'event', kind: 'gps_health', t, data: { ok, faults: active.map((f) => f.preset) } })
  }
  faults = faults.filter((f) => f.to > t - 5)
  if ((mode === 'AUTONOMOUS' || mode === 'MANUAL') && fixAge() > GNSS_STOP_S) {
    setMode('SAFE_STOP')
    log('error', `No fresh GNSS (${fixAge().toFixed(2)} s): SAFE_STOP`, 'trajectory_controller')
  }
}

function step() {
  while (replay.length && replay[0].tick <= tick) apply(replay.shift()!.msg)
  t += DT
  tick++
  gnssStep()
  if (tick % 2 === 0) perceive()
  let target = 0
  if (mode === 'AUTONOMOUS' && path.length > 1) {
    // Advance the nearest path index, then pure pursuit to the lookahead point
    while (pathIdx < path.length - 1) {
      const d0 = Math.hypot(path[pathIdx][0] - pose.x, path[pathIdx][1] - pose.y)
      const d1 = Math.hypot(path[pathIdx + 1][0] - pose.x, path[pathIdx + 1][1] - pose.y)
      if (d1 <= d0) pathIdx++
      else break
    }
    let la = pathIdx
    while (la < path.length - 1 && Math.hypot(path[la][0] - pose.x, path[la][1] - pose.y) < LOOKAHEAD) la++
    const [gx, gy] = path[la]
    const alpha = Math.atan2(gy - pose.y, gx - pose.x) - pose.yaw
    const ld = Math.max(1e-3, Math.hypot(gx - pose.x, gy - pose.y))
    const want = Math.atan2(2 * VEHICLE.wheelBase * Math.sin(alpha), ld)
    steer = Math.max(-MAX_STEER, Math.min(MAX_STEER, want))
    const end = path[path.length - 1]
    const toEnd = Math.hypot(end[0] - pose.x, end[1] - pose.y)
    target = Math.min(cruise(), Math.sqrt(2 * DECEL * Math.max(0, toEnd - ARRIVE_TOL * 0.5)))
    if (toEnd < ARRIVE_TOL && pathIdx >= path.length - 3) {
      setMode('IDLE')
      log('info', 'Goal reached', 'trajectory_controller')
      post({ type: 'event', kind: 'arrived', t, data: { x: end[0], y: end[1] } })
      path = []
      post({ type: 'path', points: [], t })
    } else if (obstacleAhead()) {
      setMode('SAFE_STOP')
      log('warn', 'Obstacle ahead, SAFE_STOP. Call /resume to return to IDLE.', 'trajectory_controller')
    }
  } else if (mode === 'MANUAL') {
    target = manual.v
    steer = Math.max(-MAX_STEER, Math.min(MAX_STEER, manual.steer))
  }
  if (ecuLatched) target = 0
  const decel = ecuLatched || mode === 'ESTOP' || mode === 'SAFE_STOP' ? ESTOP_DECEL : DECEL
  if (speed < target) speed = Math.min(target, speed + ACCEL * DT)
  else speed = Math.max(target, speed - decel * DT)
  pose.x += speed * Math.cos(pose.yaw) * DT
  pose.y += speed * Math.sin(pose.yaw) * DT
  pose.yaw += (speed / VEHICLE.wheelBase) * Math.tan(steer) * DT
  pose.yaw = Math.atan2(Math.sin(pose.yaw), Math.cos(pose.yaw))
  if (Math.floor(t) !== lastDiag) {
    lastDiag = Math.floor(t)
    post({ type: 'diag', items: diag() })
  }
}

function startLoop() {
  if (timer) clearInterval(timer)
  const wallDt = 1 / stateHz
  let carry = 0
  timer = setInterval(() => {
    if (!paused) {
      carry += wallDt * (factor === 'max' ? 100 : factor)
      let n = 0
      while (carry >= DT && n < 2000) {
        step()
        carry -= DT
        n++
      }
    }
    sendState()
  }, wallDt * 1000)
}

function reply(id: string | undefined, ok: boolean, message: string) {
  if (id !== undefined) post({ type: 'reply', id, ok, message })
}

function readyMsg(): ReadyMsg {
  const scheduled: FaultPreset[] = routes.length
    ? [
        { name: 'mock_dropout_3s', kind: 'scheduled', route: routes[0].id, faults: [{ kind: 'dropout', at_m: 20, duration_s: 3.0 }], target: { stop: true } },
        { name: 'mock_control', kind: 'scheduled', route: routes[0].id, faults: [], target: { stop: false } },
      ]
    : []
  return {
    type: 'ready',
    pyodide: null,
    python: null,
    launch: { sim: 'mock', options: structuredClone(launch), nodes: [] },
    presets: { gnss_faults: [...LIVE_PRESETS, ...scheduled], gnss_dead_zones: [], routes },
  }
}

function spawnOf(l: Launch) {
  if (typeof l.spawn === 'string') {
    const r = routes.find((q) => q.id === l.spawn)
    if (r) return { x: r.spawn[0], y: r.spawn[1], yaw: r.spawn[2] }
  } else if (l.spawn) return { ...l.spawn }
  return { ...defaultSpawn }
}

/** Rebuilds the mock stack with these launch options (merged over the current ones). */
function resetSim(partial?: Record<string, unknown>): string | null {
  const next = { ...launch, ...(structuredClone(partial ?? {}) as Partial<Launch>) }
  for (const k of Object.keys(partial ?? {})) if (!(k in DEFAULT_LAUNCH)) return `unknown launch option ${k}`
  if (!['direct', 'ekf', 'shadow'].includes(next.localization)) return `localization ${next.localization}: expected direct, ekf or shadow`
  if (typeof next.perception === 'string') next.perception = next.perception === 'true'
  if (next.filter_type && next.filter_type !== 'none' && !next.perception) return `filter_type ${next.filter_type} needs perception`
  if (typeof next.spawn === 'string' && !routes.some((r) => r.id === next.spawn)) return `spawn: unknown route ${next.spawn}`
  launch = next
  pose = spawnOf(launch)
  poseEst = { ...pose }
  speed = 0
  steer = 0
  mode = 'IDLE'
  path = []
  goal = null
  obstacles = []
  perceived = launch.perception ? [] : null
  perceivedKey = ''
  t = 0
  tick = 0
  lastFix = 0
  lastDiag = -1
  gpsOk = true
  faults = []
  manual = { v: 0, steer: 0 }
  ecuLatched = false
  recorded = []
  replay = []
  post({ type: 'progress', stage: 'nodes', pct: 50 })
  post({ type: 'path', points: [], t })
  postObstacles()
  if (launch.gnss_faults) log('warn', `gnss_faults ${JSON.stringify(launch.gnss_faults)}: scheduled faults are not in the mock`, 'mock')
  post({ type: 'progress', stage: 'ready', pct: 100 })
  post(readyMsg())
  log('info', `Mock stack reset (localization ${launch.localization}, perception ${launch.perception}). This is not the stack.`)
  return null
}

/** Two mock routes picked from the map, so the route picker has something to load. */
function mockRoutes(m: LaneletMap): RoutePreset[] {
  const out: RoutePreset[] = []
  const tries: [XY, XY][] = [
    [
      [100, 160],
      [92, -60],
    ],
    [
      [0, 0],
      [-40, 80],
    ],
  ]
  for (const [a, b] of tries) {
    const s = nearestLanelet(m, a)
    if (!s) continue
    const p = planPath(m, s.point, s.heading, b)
    if (!p || p.length < 2) continue
    const e = p[p.length - 1]
    const q = p[p.length - 2]
    out.push({ id: `M${out.length + 1}`, name: `mock_${out.length + 1}`, description: 'mock route (not test_routes.yaml)', spawn: [s.point[0], s.point[1], s.heading], goal: [e[0], e[1], Math.atan2(e[1] - q[1], e[0] - q[0])], timeout_s: 120 })
  }
  return out
}

async function init(mapUrl: string) {
  post({ type: 'progress', stage: 'map', pct: 10 })
  const text = await (await fetch(mapUrl)).text()
  post({ type: 'progress', stage: 'map', pct: 50 })
  map = buildLaneletMap(parseOsm(text), new MapProjector(MAP_ORIGIN_LAT, MAP_ORIGIN_LON))
  post({ type: 'progress', stage: 'nodes', pct: 90 })
  const near = nearestLanelet(map, [0, 0])
  if (near) defaultSpawn = { x: near.point[0], y: near.point[1], yaw: near.heading }
  routes = mockRoutes(map)
  pose = { ...defaultSpawn }
  poseEst = { ...pose }
  log('info', `Mock sim ready: ${map.lanelets.length} lanelets. This is not the stack.`)
  post({ type: 'progress', stage: 'ready', pct: 100 })
  post(readyMsg())
  startLoop()
}

function boxPoly(b: { x: number; y: number; yaw: number; length: number; width: number }): XY[] {
  const c = Math.cos(b.yaw)
  const s = Math.sin(b.yaw)
  return [
    [1, 1],
    [-1, 1],
    [-1, -1],
    [1, -1],
  ].map(([u, v]) => {
    const lx = (u * b.length) / 2
    const ly = (v * b.width) / 2
    return [b.x + c * lx - s * ly, b.y + s * lx + c * ly] as XY
  })
}

/** Applies a sim-changing message (the ones a scenario records). */
function apply(m: PageToWorker) {
  if (RECORDED.has(m.type)) {
    const { id: _id, ...rest } = m as PageToWorker & { id?: string } // the reply id is not part of the run
    recorded.push({ tick, msg: structuredClone(rest as PageToWorker) })
  }
  const id = replaying ? undefined : m.id
  switch (m.type) {
    case 'goal': {
      if (!map) return reply(id, false, 'not ready')
      const p = planPath(map, [pose.x, pose.y], pose.yaw, [m.x, m.y])
      if (!p) {
        post({ type: 'goal_ack', x: m.x, y: m.y, ok: false, reason: 'no route' })
        log('warn', `No route to (${m.x.toFixed(1)}, ${m.y.toFixed(1)})`, 'waypoint_node')
        return reply(id, false, 'no route')
      }
      goal = p[p.length - 1]
      path = p
      pathIdx = 0
      post({ type: 'path', points: path, t })
      post({ type: 'goal_ack', x: goal[0], y: goal[1], ok: true })
      log('info', `Route planned: ${path.length} points`, 'waypoint_node')
      log('info', `New path loaded: ${path.length} waypoints`, 'trajectory_controller')
      return reply(id, true, 'goal set')
    }
    case 'cmd':
      return command(m.name, id)
    case 'teleop':
      if (mode === 'MANUAL') manual = { v: m.v, steer: m.steer }
      return reply(id, mode === 'MANUAL', mode === 'MANUAL' ? 'ok' : `ignored in ${mode}`)
    case 'initialpose':
      pose = { x: m.x, y: m.y, yaw: m.yaw }
      poseEst = { ...pose }
      speed = 0
      log('info', `Initial pose (${m.x.toFixed(1)}, ${m.y.toFixed(1)}, ${((m.yaw * 180) / Math.PI).toFixed(0)} deg)`, 'kinematic_sim')
      return reply(id, true, 'teleported')
    case 'set_obstacles':
      obstacles = m.obstacles
      perceive()
      postObstacles()
      return reply(id, true, `${obstacles.length} obstacles`)
    case 'obstacle_add': {
      const poly: XY[] | undefined = m.poly ?? (m.box && boxPoly(m.box))
      if (!poly || poly.length < 3) return reply(id, false, 'obstacle_add needs poly (3+ points) or box')
      obstacles = [...obstacles.filter((o) => o.oid !== m.oid), { oid: m.oid, poly }]
      perceive()
      postObstacles()
      return reply(id, true, `added ${m.oid}`)
    }
    case 'obstacle_move': {
      if (!obstacles.some((o) => o.oid === m.oid)) return reply(id, false, `no obstacle ${m.oid}`)
      obstacles = obstacles.map((o) => {
        if (o.oid !== m.oid) return o
        const cx = o.poly.reduce((a, p) => a + p[0], 0) / o.poly.length
        const cy = o.poly.reduce((a, p) => a + p[1], 0) / o.poly.length
        const r = m.yaw ?? 0
        const c = Math.cos(r)
        const s = Math.sin(r)
        return { oid: o.oid, poly: o.poly.map(([x, y]) => [m.x + c * (x - cx) - s * (y - cy), m.y + s * (x - cx) + c * (y - cy)] as XY) }
      })
      perceive()
      postObstacles()
      return reply(id, true, `moved ${m.oid}`)
    }
    case 'obstacle_remove':
      if (!obstacles.some((o) => o.oid === m.oid)) return reply(id, false, `no obstacle ${m.oid}`)
      obstacles = obstacles.filter((o) => o.oid !== m.oid)
      perceive()
      postObstacles()
      return reply(id, true, `removed ${m.oid}`)
    case 'gnss_fault': {
      const p = LIVE_PRESETS.find((q) => q.name === m.preset)
      if (!p) return reply(id, false, `gnss_fault ${m.preset}: not a live preset in the mock`)
      const params = { ...p.params, ...(m.params ?? {}) }
      const dur = Number(params.duration_s ?? 0) || DT
      faults.push({ preset: m.preset, from: t + DT, to: t + DT + dur })
      post({ type: 'event', kind: 'gnss_fault', t, data: { preset: m.preset, ...params } })
      log('warn', `GNSS fault ${m.preset} from t=${(t + DT).toFixed(2)} s for ${dur} s (mock: ${NO_FIX.has(m.preset) ? 'no fixes' : 'health bad only'})`, 'kinematic_sim')
      return reply(id, true, `mock fault ${m.preset} for ${dur} s from the next tick`)
    }
  }
}

function command(name: string, id: string | undefined) {
  if (name === 'start') {
    let why = ''
    if (mode === 'ESTOP') why = 'Cannot start: Vehicle is E-STOPPED. Restart the node to recover.'
    else if (mode === 'SAFE_STOP') why = 'Cannot start: Vehicle is in SAFE STOP. Call /resume first.'
    else if (path.length < 2) why = 'Cannot start: no valid path loaded.'
    else if (fixAge() > STALE_S) why = `Cannot start: no fresh GNSS (${fixAge().toFixed(2)} s).`
    if (why) {
      log('warn', why, 'trajectory_controller')
      return reply(id, false, why)
    }
    ecuCommanded = true
    setMode('AUTONOMOUS')
    log('info', 'Navigation started.', 'trajectory_controller')
    return reply(id, true, 'Navigation started successfully.')
  }
  if (name === 'stop' || name === 'estop') {
    setMode('SAFE_STOP')
    log('warn', 'SAFE STOP requested (/trigger_estop)', 'trajectory_controller')
    return reply(id, true, 'SAFE STOP engaged. Call /resume to return to IDLE.')
  }
  if (name === 'resume') {
    if (mode !== 'SAFE_STOP') return reply(id, false, `Not in SAFE_STOP (currently ${mode}); nothing to resume.`)
    setMode('IDLE')
    log('info', 'Resumed from SAFE STOP. Returned to IDLE.', 'trajectory_controller')
    return reply(id, true, 'Resumed. Vehicle returned to IDLE. Call /start_navigation when ready.')
  }
  if (name === 'manual') {
    if (mode === 'ESTOP') return reply(id, false, 'Cannot switch mode: Vehicle is E-STOPPED. Restart the node to recover.')
    if (mode === 'SAFE_STOP') return reply(id, false, 'Cannot switch mode: Vehicle is in SAFE STOP. Call /resume first.')
    if (mode === 'MANUAL') {
      setMode('IDLE')
      manual = { v: 0, steer: 0 }
      log('info', 'Switched to IDLE mode (Hit button to start navigation).', 'trajectory_controller')
      return reply(id, true, 'Switched to IDLE mode (Hit button to start navigation).')
    }
    ecuCommanded = true
    setMode('MANUAL')
    log('info', 'Switched to MANUAL mode.', 'trajectory_controller')
    return reply(id, true, 'Switched to MANUAL mode.')
  }
  if (name === 'dbw_estop') {
    ecuLatched = true
    log('error', 'E-stop latched — call /carla/dbw_power_cycle to release', 'kinematic_sim')
    return reply(id, true, 'mock: 0x100 E-stop frame sent, the ECU latched')
  }
  if (name === 'power_cycle') {
    const was = ecuLatched
    ecuLatched = false
    log('info', `ECU power cycled${was ? ', E-stop latch cleared' : ''}`, 'kinematic_sim')
    return reply(id, true, was ? 'mock: power cycled, E-stop latch cleared' : 'mock: power cycled')
  }
  return reply(id, false, `${name} is not in the mock`)
}

function handle(m: PageToWorker) {
  switch (m.type) {
    case 'init':
      if (m.stateHz) stateHz = m.stateHz
      init(m.mapUrl).catch((e) => post({ type: 'error', message: `Map load failed: ${e}`, traceback: '', fatal: true }))
      break
    case 'reset': {
      const err = resetSim(m.launch)
      if (err) {
        post({ type: 'error', message: `reset: ${err}`, traceback: '', fatal: false })
        return reply(m.id, false, err)
      }
      sendState()
      return reply(m.id, true, 'mock stack reset')
    }
    case 'pause':
      paused = true
      sendState()
      return reply(m.id, true, 'paused')
    case 'resume_sim':
      paused = false
      return reply(m.id, true, 'running')
    case 'step':
      if (paused) for (let i = 0; i < (m.n ?? 1); i++) step()
      sendState()
      return reply(m.id, paused, paused ? `stepped ${m.n ?? 1}` : 'step needs pause')
    case 'speed':
      factor = m.factor === 'max' ? 'max' : Math.max(0.1, Math.min(100, m.factor))
      sendState()
      return reply(m.id, true, `x${factor}`)
    case 'set_param': {
      if (typeof m.node !== 'string' || typeof m.name !== 'string') return reply(m.id, false, 'set_param needs node and name')
      launch.params = { ...launch.params, [m.node]: { ...(launch.params[m.node] ?? {}), [m.name]: m.value } }
      const note = m.node === 'trajectory_controller' && m.name === 'base_speed' ? '' : ' (the mock ignores it)'
      return reply(m.id, true, `${m.node}.${m.name} = ${JSON.stringify(m.value)} staged; it takes effect on the next reset${note}`)
    }
    case 'scenario_save': {
      const scenario: Scenario = { v: 1, launch: structuredClone(launch) as unknown as Record<string, unknown>, events: structuredClone(recorded), note: `mock, saved at tick ${tick}` }
      post({ type: 'scenario', scenario })
      return reply(m.id, true, `${recorded.length} events`)
    }
    case 'scenario_load': {
      const s = m.scenario
      if (!s || s.v !== 1 || !Array.isArray(s.events)) return reply(m.id, false, 'not a v1 scenario')
      const err = resetSim({ ...structuredClone(DEFAULT_LAUNCH), ...s.launch })
      if (err) return reply(m.id, false, err)
      replay = [...s.events].sort((a, b) => a.tick - b.tick)
      replaying = true
      while (replay.length && replay[0].tick <= 0) apply(replay.shift()!.msg)
      replaying = false
      sendState()
      return reply(m.id, true, `replaying ${s.events.length} events`)
    }
    default:
      if (RECORDED.has(m.type)) return apply(m)
      return reply((m as { id?: string }).id, false, `${(m as { type: string }).type} is not in the mock`)
  }
}

self.onmessage = (ev: MessageEvent<PageToWorker>) => {
  replaying = false
  handle(ev.data)
}
