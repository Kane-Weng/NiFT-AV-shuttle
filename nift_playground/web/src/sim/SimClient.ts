/*
 * File: SimClient.ts
 *
 * Description:
 *   The page side of the worker protocol. It owns the worker, keeps the latest state, path,
 *   obstacles and log, and tells listeners when they change. Panels read from it and send
 *   through it, so none of them touch the worker directly.
 *
 *   Which worker runs comes from the URL. The default (or ?sim=pyodide) is the real stack: the
 *   Pyodide worker in src/worker/ runs our nodes unchanged. ?sim=mock runs the mock worker in
 *   src/mock/, which is quick to start and handy while working on the page.
 *
 *   For the Pyodide worker the client first reads py/nift_bundle.json (revalidated on every load)
 *   and asks for the bundle zip with its hash in the query, so a new bundle is never served from
 *   an old cache entry and an unchanged one comes from the cache.
 */

import type { DiagItem, ErrorMsg, LogMsg, Obstacle, PageToWorker, ReadyMsg, ReplyMsg, StateMsg, WorkerToPage, XY } from '../protocol'

export type SimKind = 'mock' | 'pyodide'

export interface SimSnapshot {
  ready: boolean
  readyInfo: ReadyMsg | null
  error: ErrorMsg | null
  progress: { stage: string; pct: number } | null
  state: StateMsg | null
  path: XY[]
  goal: XY | null
  obstacles: Obstacle[]
  perceived: XY[][] | null
  diag: DiagItem[]
  log: LogMsg[]
}

const LOG_MAX = 500

function sameXY(a: XY | null, b: XY | null): boolean {
  return a === b || (!!a && !!b && a[0] === b[0] && a[1] === b[1])
}

/** The sim the page URL asks for: ?sim=mock, else the real stack in Pyodide. */
export function simKindFromUrl(href: string): SimKind {
  return new URL(href).searchParams.get('sim') === 'mock' ? 'mock' : 'pyodide'
}

function createWorker(kind: SimKind): Worker {
  if (kind === 'pyodide') return new Worker(new URL('../worker/pyodide.worker.ts', import.meta.url), { type: 'module' })
  return new Worker(new URL('../mock/mockWorker.ts', import.meta.url), { type: 'module' })
}

/** The bundle zip URL with its hash as a cache key, from tools/build_bundle.py's JSON next to it. */
export async function bundleUrl(baseUrl: string): Promise<string | undefined> {
  try {
    const res = await fetch(new URL('py/nift_bundle.json', baseUrl), { cache: 'no-cache' })
    if (!res.ok) return undefined
    const info = (await res.json()) as { zip?: string; sha256?: string }
    if (!info.zip || !info.sha256) return undefined
    return new URL(`py/${info.zip}?v=${info.sha256.slice(0, 16)}`, baseUrl).href
  } catch {
    return undefined // the worker then asks for py/nift_bundle.zip without a cache key
  }
}

export class SimClient {
  readonly kind: SimKind
  private worker: Worker
  private listeners = new Set<(s: SimSnapshot) => void>()
  private msgListeners = new Set<(m: WorkerToPage) => void>()
  snapshot: SimSnapshot = {
    ready: false,
    readyInfo: null,
    error: null,
    progress: null,
    state: null,
    path: [],
    goal: null,
    obstacles: [],
    perceived: null,
    diag: [],
    log: [],
  }

  /** baseUrl may be relative to the page (Vite's './'). The worker gets absolute URLs. */
  constructor(kind: SimKind, mapUrl: string, baseUrl: string) {
    this.kind = kind
    const base = new URL(baseUrl, window.location.href).href
    const map = new URL(mapUrl, window.location.href).href
    this.worker = createWorker(kind)
    this.worker.onmessage = (ev: MessageEvent<WorkerToPage>) => this.receive(ev.data)
    this.worker.onerror = (ev) => this.receive({ type: 'error', message: `worker: ${ev.message || 'failed to start'}`, traceback: '', fatal: true })
    this.snapshot = { ...this.snapshot, progress: { stage: kind === 'pyodide' ? 'starting' : 'mock', pct: 0 } }
    if (kind === 'pyodide') void bundleUrl(base).then((bundle) => this.send({ type: 'init', mapUrl: map, baseUrl: base, ...(bundle ? { bundleUrl: bundle } : {}) }))
    else this.send({ type: 'init', mapUrl: map, baseUrl: base })
  }

  private nextId = 0
  private lastStateGoal: XY | null = null
  private pending = new Map<string, (r: ReplyMsg) => void>()

  /** Posts a message. A goal shows on the map at once; the worker's goal_ack then corrects it. */
  send(m: PageToWorker) {
    if (m.type === 'goal') this.update({ goal: [m.x, m.y] })
    if (m.type === 'reset') this.update({ goal: null, path: [], ready: false })
    this.worker.postMessage(m)
  }

  /** Posts a message with an id and resolves with the worker's reply. */
  request(m: PageToWorker): Promise<ReplyMsg> {
    const id = `p${++this.nextId}`
    return new Promise((resolve) => {
      this.pending.set(id, resolve)
      this.send({ ...m, id })
    })
  }

  /** Called with the whole snapshot on every change. Returns an unsubscribe function. */
  subscribe(fn: (s: SimSnapshot) => void): () => void {
    this.listeners.add(fn)
    fn(this.snapshot)
    return () => this.listeners.delete(fn)
  }

  /** Called with every raw worker message. Returns an unsubscribe function. */
  onMessage(fn: (m: WorkerToPage) => void): () => void {
    this.msgListeners.add(fn)
    return () => this.msgListeners.delete(fn)
  }

  terminate() {
    this.worker.terminate()
    this.listeners.clear()
    this.msgListeners.clear()
  }

  private update(patch: Partial<SimSnapshot>) {
    this.snapshot = { ...this.snapshot, ...patch }
    for (const fn of this.listeners) fn(this.snapshot)
  }

  private receive(m: WorkerToPage) {
    for (const fn of this.msgListeners) fn(m)
    switch (m.type) {
      case 'ready':
        this.update({ ready: true, readyInfo: m, error: null })
        break
      case 'reply': {
        const r = this.pending.get(m.id)
        this.pending.delete(m.id)
        r?.(m)
        break
      }
      case 'error':
        this.update({ error: m })
        break
      case 'progress':
        this.update({ progress: { stage: m.stage, pct: m.pct } })
        break
      case 'state': {
        // Before the first odom (just after a boot or a reset) the stack has no pose yet. Keep the
        // last one, or wait for one, so the panels and the scene always get a pose.
        const pose = m.pose ?? this.snapshot.state?.pose
        if (!pose) break
        // /carla/dbw_state names the ECU mode in lower case (carla_dbw.EcuMode: no_command, active,
        // timeout, estop). The panels use upper case, as the mock does.
        m = { ...m, pose, speed: m.speed ?? 0, steer: m.steer ?? 0, ecu: String(m.ecu ?? '-').toUpperCase() }
        // The state's goal is what the stack holds. Take it when it changes, so a state that
        // crosses a new goal on its way does not wipe the page's guess.
        if (!sameXY(m.goal ?? null, this.lastStateGoal)) {
          this.lastStateGoal = m.goal ?? null
          this.update({ state: m, goal: m.goal ?? null })
        } else this.update({ state: m })
        break
      }
      case 'path':
        this.update({ path: m.points })
        break
      case 'obstacles':
        this.update({ obstacles: m.polygons, perceived: m.perceived })
        break
      case 'goal_ack':
        this.update({ goal: m.ok ? [m.x, m.y] : this.snapshot.goal })
        break
      case 'diag':
        this.update({ diag: m.items })
        break
      case 'log':
        this.update({ log: [...this.snapshot.log, m].slice(-LOG_MAX) })
        break
    }
  }
}
