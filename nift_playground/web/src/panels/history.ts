/*
 * File: history.ts
 *
 * Description:
 *   What the panels keep in JavaScript, beside the SimClient snapshot: the last minute of speed,
 *   target speed and cross-track error for the plots, the events (mode changes, arrival, GNSS
 *   faults), command results and errors. Python sends single values. The history lives here so the
 *   worker never has to keep or resend it.
 *
 *   One store per SimClient, fed by its raw message stream from page load on, so a panel that
 *   mounts later (a collapsed section opened) still sees everything.
 */

import { useSyncExternalStore } from 'react'
import type { SimClient } from '../sim/SimClient'
import type { PageToWorker, ReplyMsg, WorkerToPage } from '../protocol'

export const PLOT_WINDOW_S = 60
const EVENTS_MAX = 300

export interface Sample {
  t: number
  speed: number
  target: number | null
  cte: number | null
}

export type FeedKind = 'mode' | 'arrived' | 'gnss_fault' | 'gps_health' | 'goal' | 'command' | 'error'

export interface FeedItem {
  seq: number
  t: number | null // sim time, null when the page had no state yet
  kind: FeedKind
  ok: boolean
  text: string
  detail?: string
}

export interface History {
  samples: Sample[]
  feed: FeedItem[]
  errors: number
  lastCommand: FeedItem | null
  version: number
}

export class HistoryStore {
  private h: History = { samples: [], feed: [], errors: 0, lastCommand: null, version: 0 }
  private listeners = new Set<() => void>()
  private seq = 0
  private lastT: number | null = null

  constructor(private client: SimClient) {
    client.onMessage((m) => this.receive(m))
  }

  get(): History {
    return this.h
  }

  subscribe = (fn: () => void) => {
    this.listeners.add(fn)
    return () => this.listeners.delete(fn)
  }

  /** Sends a message with an id and records the worker's reply as a command result. */
  async command(msg: PageToWorker, label: string): Promise<ReplyMsg> {
    const r = await this.client.request(msg)
    const item = this.push('command', r.ok, `${label}: ${r.message || (r.ok ? 'ok' : 'failed')}`)
    this.h = { ...this.h, lastCommand: item }
    this.emit()
    return r
  }

  /** Adds a line of the page's own (a bad link, a failed save) to the feed. */
  note(kind: FeedKind, ok: boolean, text: string) {
    this.push(kind, ok, text)
    this.emit()
  }

  clearFeed() {
    this.h = { ...this.h, feed: [], errors: 0 }
    this.emit()
  }

  private push(kind: FeedKind, ok: boolean, text: string, detail?: string): FeedItem {
    const item: FeedItem = { seq: ++this.seq, t: this.lastT, kind, ok, text, detail }
    const feed = [...this.h.feed, item].slice(-EVENTS_MAX)
    this.h = { ...this.h, feed, errors: this.h.errors + (ok ? 0 : 1) }
    return item
  }

  private emit() {
    this.h = { ...this.h, version: this.h.version + 1 }
    for (const fn of this.listeners) fn()
  }

  private receive(m: WorkerToPage) {
    switch (m.type) {
      case 'state': {
        // A reset starts sim time again: drop the old curves
        const samples = this.lastT !== null && m.t < this.lastT ? [] : this.h.samples
        this.lastT = m.t
        if (samples.length && samples[samples.length - 1].t === m.t) return
        const next = [...samples, { t: m.t, speed: m.speed, target: m.targetSpeed, cte: m.cte }]
        let cut = 0
        while (cut < next.length && next[cut].t < m.t - PLOT_WINDOW_S) cut++
        this.h = { ...this.h, samples: cut ? next.slice(cut) : next }
        this.emit()
        return
      }
      case 'event': {
        this.lastT = m.t
        const d = (m.data ?? {}) as Record<string, unknown>
        if (m.kind === 'mode') this.push('mode', true, `Mode ${String(d.status ?? d.mode ?? JSON.stringify(m.data))}`)
        else if (m.kind === 'arrived') this.push('arrived', true, 'Arrived at the goal')
        // GNSS faults are events, not page errors: the Errors tab is for exceptions and refusals
        else if (m.kind === 'gnss_fault') this.push('gnss_fault', true, `GNSS fault: ${describe(m.data)}`)
        else if (m.kind === 'gps_health') this.push('gps_health', true, `GPS health: ${describe(m.data)}`)
        break
      }
      case 'goal_ack':
        this.push('goal', m.ok, m.ok ? `Goal (${m.x.toFixed(1)}, ${m.y.toFixed(1)}) planned` : `No path to (${m.x.toFixed(1)}, ${m.y.toFixed(1)})`, m.reason)
        break
      case 'error':
        this.push('error', false, m.fatal ? `Fatal: ${m.message}` : m.message, m.traceback || undefined)
        break
      default:
        return
    }
    this.emit()
  }
}

function describe(data: unknown): string {
  if (data === null || data === undefined) return ''
  if (typeof data === 'string') return data
  if (typeof data !== 'object') return String(data)
  return Object.entries(data as Record<string, unknown>)
    .map(([k, v]) => `${k} ${typeof v === 'object' ? JSON.stringify(v) : String(v)}`)
    .join(', ')
}

const stores = new WeakMap<SimClient, HistoryStore>()

/** The client's store, created on first use. Call it once at page start so nothing is missed. */
export function historyOf(client: SimClient): HistoryStore {
  let s = stores.get(client)
  if (!s) {
    s = new HistoryStore(client)
    stores.set(client, s)
  }
  return s
}

export function useHistory(client: SimClient): History {
  const s = historyOf(client)
  return useSyncExternalStore(s.subscribe, () => s.get())
}
