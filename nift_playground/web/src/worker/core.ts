/*
 * File: core.ts
 *
 * Description:
 *   The Pyodide sim worker without the browser around it. It boots Pyodide, loads numpy and
 *   pyyaml, unpacks the Python bundle to /nift, writes the map, builds the playground session
 *   (playground/session.py) and then speaks the worker protocol of ARCH.md section 8.
 *
 *   Python never sees the wall clock. This file owns the pacing (ARCH.md section 7). Each loop
 *   turn works out how many 50 ms ticks are owed (wall time x speed / dt), runs at most a time
 *   budget's worth of them with session.advance(n), posts what Python returns and yields, so the
 *   worker always answers the page within about one budget. pause, resume_sim, step and speed are
 *   handled here and never reach Python. Every other page message goes to session.handle().
 *
 *   The file imports nothing on purpose. The browser entry (pyodide.worker.ts) and the Node test
 *   harness (tests/pyodide/) both pass in an environment (WorkerEnv) and run this same code. Node
 *   loads it with its built-in TypeScript type stripping, so keep to erasable syntax: no enums, no
 *   namespaces, no parameter properties.
 */

/** The parts of the Pyodide API this worker uses. */
export interface PyProxyLike {
  (...args: unknown[]): unknown
  destroy?: () => void
}

export interface PyodideLike {
  version: string
  loadPackage(names: string[], opts?: { messageCallback?: (m: string) => void }): Promise<unknown>
  unpackArchive(buffer: ArrayBuffer | Uint8Array, format: string, opts?: { extractDir?: string }): void
  runPython(code: string): unknown
  globals: { get(name: string): unknown }
  FS: {
    mkdirTree(path: string): void
    writeFile(path: string, data: Uint8Array | string): void
  }
  setStdout?(opts: { batched: (s: string) => void }): void
  setStderr?(opts: { batched: (s: string) => void }): void
}

/** What the host gives the worker. The browser entry and the Node harness each make one. */
export interface WorkerEnv {
  /** Load Pyodide 0.27.7 (the browser from its index URL, Node from node_modules). */
  loadPyodide(): Promise<PyodideLike>
  /** Fetch a URL (resolved against the page's baseUrl by the caller) as bytes. */
  fetchBytes(url: string): Promise<Uint8Array>
  /** Post one message to the page. */
  post(msg: Msg): void
  /** Monotonic wall clock in ms (performance.now). */
  now(): number
  /** Run fn after ms (setTimeout). */
  later(fn: () => void, ms: number): void
  /** Run fn as soon as the event loop is free, after pending messages (MessageChannel or setImmediate). */
  soon(fn: () => void): void
  /** Optional: where Python's print() goes. */
  log?(text: string): void
}

// Messages are plain JSON objects with a type field (ARCH.md section 8). The page side types live in
// src/protocol.ts. The worker passes Python's messages through, so it only needs the type here.
export type Msg = { type: string; [k: string]: unknown }
export type Factor = number | 'max'

export const PYODIDE_VERSION = '0.27.7'
/** ARCH.md section 9: the only packages the nodes need. Nothing else is loaded. */
export const PACKAGES = ['numpy', 'pyyaml']
export const BUNDLE_ROOT = '/nift'
export const MAP_PATH = '/nift/maps/lanelet2_odr2024.osm'
export const BUNDLE_ZIP = 'py/nift_bundle.zip'
export const BUNDLE_INFO = 'py/nift_bundle.json'

/** Pacing constants (ARCH.md section 7). */
export const BUDGET_MS = 12 // Python time per loop turn, so the worker stays responsive
export const MAX_CATCHUP_MS = 250 // a longer stall (a background tab) is dropped, not caught up
export const FACTOR_MIN = 0.1
export const FACTOR_MAX = 100

// The Python side of the worker. It builds the session the way the runner's native tests do and
// turns every Python exception into an 'error' message, so the JavaScript never sees a PythonError
// from handle() or advance().
const GLUE = `
import json, sys, traceback
if '${BUNDLE_ROOT}/nift_playground/py' not in sys.path:
    sys.path.insert(0, '${BUNDLE_ROOT}/nift_playground/py')
from playground import paths as _paths
_paths.setup('${BUNDLE_ROOT}')
from playground.session import Session as _Session

_session = None

def _err(e, fatal):
    return json.dumps([{'type': 'error', 'message': f'{type(e).__name__}: {e}',
                        'traceback': traceback.format_exc(), 'fatal': fatal}])

def worker_open(init_json):
    global _session
    try:
        _session = _Session(repo_root='${BUNDLE_ROOT}')
        return _session.handle(init_json)
    except Exception as e:
        return _err(e, True)

def worker_handle(msg_json):
    try:
        return _session.handle(msg_json)
    except Exception as e:
        return _err(e, False)

def worker_state(_=None):
    # the state at the current tick, after what is queued (no ticks run, not logged)
    try:
        return json.dumps(_session.collect() + [_session.state()])
    except Exception as e:
        return _err(e, False)

def worker_advance(n):
    try:
        return _session.advance(int(n))
    except Exception as e:
        return _err(e, False)
`

type PyFn = (arg?: unknown) => unknown

/** Turns wall time into ticks and measures the achieved real-time factor. Pure, so it is unit tested. */
export class Pacer {
  factor: Factor = 1
  paused = false
  dt = 0.05
  /** Ticks owed but not run yet (fractional). */
  owed = 0
  /** Moving average of the wall cost of one tick in ms. */
  costMs = 0.5
  private lastTurn: number | null = null
  private window: Array<[number, number]> = [] // [wall ms, ticks run]

  setFactor(f: unknown): Factor {
    if (f === 'max') this.factor = 'max'
    else {
      const x = Number(f)
      if (!Number.isFinite(x) || x <= 0) throw new Error(`speed factor must be a positive number or 'max', got ${String(f)}`)
      this.factor = Math.min(FACTOR_MAX, Math.max(FACTOR_MIN, x))
    }
    return this.factor
  }

  /** Forget the wall-clock reference (after a pause, a reset or a long step). */
  rebase(now: number) {
    this.lastTurn = now
    this.owed = 0
  }

  /** How many ticks to run this turn. */
  ticksFor(now: number): number {
    if (this.paused) return 0
    const last = this.lastTurn ?? now
    const elapsed = Math.min(Math.max(0, now - last), MAX_CATCHUP_MS)
    this.lastTurn = now
    const fit = Math.max(1, Math.floor(BUDGET_MS / Math.max(this.costMs, 1e-3)))
    if (this.factor === 'max') return fit
    this.owed += (elapsed * this.factor) / 1000 / this.dt
    let n = Math.floor(this.owed)
    if (n > fit) n = fit
    this.owed -= n
    // When Python cannot keep up, do not build a backlog. The page sees a lower rtf instead.
    if (this.owed > fit) this.owed = fit
    return n
  }

  /** ms until the next tick is owed (0 for 'max' or when one is already owed). */
  delayMs(): number {
    if (this.paused) return Infinity
    if (this.factor === 'max' || this.owed >= 1) return 0
    return ((1 - this.owed) * this.dt * 1000) / this.factor
  }

  record(now: number, ticks: number, wallMs: number) {
    if (ticks > 0) {
      const c = wallMs / ticks
      this.costMs = this.costMs * 0.8 + c * 0.2
    }
    this.window.push([now, ticks])
    while (this.window.length > 2 && now - this.window[0][0] > 1000) this.window.shift()
  }

  /** Achieved sim seconds per wall second over about the last second. 0 while paused. */
  rtf(now: number): number {
    if (this.paused || this.window.length < 2) return 0
    const span = now - this.window[0][0]
    if (span <= 0) return 0
    let ticks = 0
    for (let i = 1; i < this.window.length; i++) ticks += this.window[i][1]
    return (ticks * this.dt) / (span / 1000)
  }

  clearWindow() {
    this.window = []
  }
}

export interface CoreStats {
  bootMs: Record<string, number>
  ticks: number
  advanceMs: number
  bundleBytes: number
  mapBytes: number
}

export class WorkerCore {
  readonly pacer = new Pacer()
  readonly stats: CoreStats = { bootMs: {}, ticks: 0, advanceMs: 0, bundleBytes: 0, mapBytes: 0 }
  py: PyodideLike | null = null
  private env: WorkerEnv
  private ready = false
  private booting = false
  private stateHz = 10
  private queue: Msg[] = []
  private loopArmed = false
  private loopToken = 0
  private heldState: Msg | null = null
  private lastStatePost = -Infinity
  private fnOpen: PyFn | null = null
  private fnHandle: PyFn | null = null
  private fnAdvance: PyFn | null = null
  private fnState: PyFn | null = null

  constructor(env: WorkerEnv) {
    this.env = env
  }

  get isReady() {
    return this.ready
  }

  /** Entry point for every page message. */
  onMessage(msg: Msg) {
    try {
      this.dispatch(msg)
    } catch (e) {
      this.postError(e, false, msg)
    }
  }

  private dispatch(msg: Msg) {
    switch (msg.type) {
      case 'init':
        if (this.ready || this.booting) return this.reply(msg, false, 'already initialised')
        this.booting = true
        this.boot(msg).catch((e) => {
          this.booting = false
          this.postError(e, true, msg)
        })
        return
      case 'pause':
        this.setPaused(true)
        // advance() posts a state every few ticks only. Ask the session for one at the tick the sim
        // stopped on, so the page shows where it really is (and a step after it counts from there).
        this.stateNow()
        this.flushState(true)
        return this.reply(msg, true, 'paused')
      case 'resume_sim':
        this.setPaused(false)
        return this.reply(msg, true, 'running')
      case 'speed': {
        const f = this.pacer.setFactor(msg.factor)
        this.pacer.rebase(this.env.now())
        this.pacer.clearWindow()
        this.patchAndPostHeld()
        return this.reply(msg, true, `speed ${f}`)
      }
    }
    if (!this.ready) {
      // Commands sent while booting run, in order, once the session is ready.
      this.queue.push(msg)
      return
    }
    if (msg.type === 'step') {
      const n = msg.n === undefined ? 1 : Math.floor(Number(msg.n))
      if (!Number.isFinite(n) || n < 1) return this.reply(msg, false, `step n must be >= 1, got ${String(msg.n)}`)
      this.setPaused(true)
      this.advance(n)
      this.stateNow()
      this.flushState(true)
      return this.reply(msg, true, `stepped ${n}`)
    }
    this.handle(msg)
  }

  // ------------------------------------------------------------------ boot

  private async boot(init: Msg) {
    const env = this.env
    const stage = (s: string, pct: number) => env.post({ type: 'progress', stage: s, pct })
    const mark = (k: string, t0: number) => (this.stats.bootMs[k] = env.now() - t0)
    const base = String(init.baseUrl ?? '')
    if (typeof init.stateHz === 'number' && init.stateHz > 0) this.stateHz = init.stateHz

    let t0 = env.now()
    stage('pyodide', 0)
    const py = await env.loadPyodide()
    if (py.version !== PYODIDE_VERSION) env.log?.(`warning: Pyodide ${py.version}, expected ${PYODIDE_VERSION}`)
    py.setStdout?.({ batched: (s) => env.log?.(s) })
    py.setStderr?.({ batched: (s) => env.log?.(s) })
    this.py = py
    mark('pyodide', t0)

    t0 = env.now()
    stage('packages', 25)
    await py.loadPackage(PACKAGES)
    mark('packages', t0)

    t0 = env.now()
    stage('bundle', 50)
    // The page may name the zip with its hash as a cache key (SimClient.bundleUrl).
    const zipUrl = typeof init.bundleUrl === 'string' && init.bundleUrl ? init.bundleUrl : joinUrl(base, BUNDLE_ZIP)
    const zip = await env.fetchBytes(zipUrl)
    this.stats.bundleBytes = zip.byteLength
    py.unpackArchive(zip, 'zip', { extractDir: BUNDLE_ROOT })
    mark('bundle', t0)

    t0 = env.now()
    stage('map', 65)
    const map = await env.fetchBytes(String(init.mapUrl))
    this.stats.mapBytes = map.byteLength
    py.FS.mkdirTree(MAP_PATH.slice(0, MAP_PATH.lastIndexOf('/')))
    py.FS.writeFile(MAP_PATH, map)
    mark('map', t0)

    t0 = env.now()
    stage('nodes', 75)
    py.runPython(GLUE)
    this.fnOpen = py.globals.get('worker_open') as PyFn
    this.fnHandle = py.globals.get('worker_handle') as PyFn
    this.fnAdvance = py.globals.get('worker_advance') as PyFn
    this.fnState = (py.globals.get('worker_state') as PyFn | undefined) ?? null
    // The session gets the init message as the page sent it. It answers with ready (or an error).
    const open: Msg = { ...init, mapPath: MAP_PATH }
    delete open.baseUrl
    delete open.mapUrl
    delete open.bundleUrl
    const out = this.call(this.fnOpen, JSON.stringify(open))
    const readyMsg = out.find((m) => m.type === 'ready')
    if (readyMsg && typeof readyMsg.dt === 'number' && readyMsg.dt > 0) this.pacer.dt = readyMsg.dt
    mark('nodes', t0)

    const fatal = out.find((m) => m.type === 'error' && m.fatal)
    this.booting = false
    if (fatal) {
      this.postAll(out)
      return
    }
    this.ready = true
    this.postAll(out.map((m) => (m.type === 'ready' ? { ...m, pyodide: py.version, bootMs: this.stats.bootMs } : m)))
    for (const m of this.queue.splice(0)) this.onMessage(m)
    this.pacer.rebase(env.now())
    this.arm(0)
  }

  // ------------------------------------------------------------------ Python calls

  private call(fn: PyFn | null, arg: unknown): Msg[] {
    if (!fn) throw new Error('session not ready')
    const text = fn(arg)
    const out = JSON.parse(String(text))
    return Array.isArray(out) ? out : [out]
  }

  private handle(msg: Msg) {
    const out = this.call(this.fnHandle, JSON.stringify(msg))
    const isReset = msg.type === 'reset' || msg.type === 'scenario_load'
    if (isReset) {
      this.pacer.rebase(this.env.now())
      this.pacer.clearWindow()
      this.heldState = null
    }
    this.postAll(out)
    this.flushState(true)
    if (out.some((m) => m.type === 'error')) this.setPaused(true)
  }

  /**
   * The session's advance() gives a state every few ticks only (at stateHz of sim time). After a
   * pause or a step the page must see the tick the sim stopped on, so ask the session for a state
   * now (worker_state in the glue). No ticks run and nothing is logged.
   */
  private stateNow() {
    if (this.ready && this.fnState) this.postAll(this.call(this.fnState, undefined))
  }

  /** Run n ticks now. Returns the wall time used. */
  private advance(n: number): number {
    const t0 = this.env.now()
    const out = this.call(this.fnAdvance, n)
    const wall = this.env.now() - t0
    this.stats.ticks += n
    this.stats.advanceMs += wall
    this.pacer.record(this.env.now(), n, wall)
    this.postAll(out)
    if (out.some((m) => m.type === 'error')) this.setPaused(true)
    return wall
  }

  // ------------------------------------------------------------------ pacing loop

  private setPaused(p: boolean) {
    if (this.pacer.paused === p) return
    this.pacer.paused = p
    this.pacer.rebase(this.env.now())
    this.pacer.clearWindow()
    if (!p) this.arm(0)
  }

  private arm(delay: number) {
    if (!this.ready || this.pacer.paused) return
    if (this.loopArmed) return
    this.loopArmed = true
    const token = ++this.loopToken
    const run = () => {
      if (token !== this.loopToken) return
      this.loopArmed = false
      this.turn()
    }
    if (delay <= 0) this.env.soon(run)
    else this.env.later(run, delay)
  }

  private turn() {
    if (!this.ready || this.pacer.paused) return
    try {
      const n = this.pacer.ticksFor(this.env.now())
      if (n > 0) this.advance(n)
      this.flushState(false)
    } catch (e) {
      this.setPaused(true)
      this.postError(e, false)
      return
    }
    const d = this.pacer.delayMs()
    if (Number.isFinite(d)) this.arm(d)
  }

  // ------------------------------------------------------------------ output

  private postAll(msgs: Msg[]) {
    for (const m of msgs) {
      if (m.type === 'state') this.heldState = m
      else this.env.post(m)
    }
  }

  /** Post the newest state at most stateHz times a wall second, or now when forced. */
  private flushState(force: boolean) {
    if (!this.heldState) return
    const now = this.env.now()
    if (!force && now - this.lastStatePost < 1000 / this.stateHz) return
    this.patchAndPostHeld()
  }

  private patchAndPostHeld() {
    if (!this.heldState) return
    const now = this.env.now()
    this.env.post({
      ...this.heldState,
      paused: this.pacer.paused,
      factor: this.pacer.factor,
      rtf: round3(this.pacer.rtf(now)),
    })
    this.heldState = null
    this.lastStatePost = now
  }

  private reply(msg: Msg, ok: boolean, message: string) {
    if (typeof msg.id === 'string') this.env.post({ type: 'reply', id: msg.id, ok, message })
  }

  private postError(e: unknown, fatal: boolean, msg?: Msg) {
    const err = e instanceof Error ? e : new Error(String(e))
    this.env.post({ type: 'error', message: err.message.split('\n').filter(Boolean).pop() ?? err.message,
                    traceback: err.stack ?? err.message, fatal })
    if (msg) this.reply(msg, false, err.message)
  }
}

function round3(x: number) {
  return Math.round(x * 1000) / 1000
}

/** baseUrl + relative path, keeping exactly one slash. */
export function joinUrl(base: string, rel: string): string {
  if (!base) return rel
  return base.endsWith('/') ? base + rel : `${base}/${rel}`
}
