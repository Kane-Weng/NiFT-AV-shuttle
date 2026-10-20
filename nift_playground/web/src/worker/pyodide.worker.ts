/*
 * File: pyodide.worker.ts
 *
 * Description:
 *   The browser entry of the Pyodide sim worker. It wires core.ts to the Web Worker globals:
 *   postMessage, fetch, performance.now, setTimeout and a MessageChannel to yield. Everything else
 *   (boot, protocol, pacing) is in core.ts, which the Node test harness runs too.
 *
 *   Pyodide comes from the jsDelivr CDN by default. Set VITE_PYODIDE_URL at build time to a folder
 *   that holds pyodide.mjs, the lock file and the numpy and pyyaml wheels to host it yourself.
 *
 *   The page creates it with
 *     new Worker(new URL('./worker/pyodide.worker.ts', import.meta.url), { type: 'module' })
 *   and sends init { baseUrl, mapUrl, bundleUrl? } with absolute URLs, or URLs relative to the page.
 */

/// <reference lib="webworker" />

import { PYODIDE_VERSION, WorkerCore, type Msg, type PyodideLike, type WorkerEnv } from './core'

declare const self: DedicatedWorkerGlobalScope

const INDEX_URL: string =
  (import.meta.env.VITE_PYODIDE_URL as string | undefined) ?? `https://cdn.jsdelivr.net/pyodide/v${PYODIDE_VERSION}/full/`

// A MessageChannel yields to pending page messages without setTimeout's 4 ms clamp.
const channel = new MessageChannel()
const soonQueue: Array<() => void> = []
channel.port1.onmessage = () => soonQueue.shift()?.()

// Page URLs are relative to the page, not to this worker. The page's init carries baseUrl, and
// relative URLs are resolved against it.
let pageBase = self.location.href

const env: WorkerEnv = {
  async loadPyodide() {
    const mod = await import(/* @vite-ignore */ `${INDEX_URL}pyodide.mjs`)
    return (await mod.loadPyodide({ indexURL: INDEX_URL })) as PyodideLike
  },
  async fetchBytes(url) {
    const res = await fetch(new URL(url, pageBase))
    if (!res.ok) throw new Error(`fetch ${url}: HTTP ${res.status}`)
    return new Uint8Array(await res.arrayBuffer())
  },
  post: (m) => self.postMessage(m),
  now: () => performance.now(),
  later: (fn, ms) => void setTimeout(fn, ms),
  soon: (fn) => {
    soonQueue.push(fn)
    channel.port2.postMessage(null)
  },
  log: (text) => console.log(`[py] ${text}`),
}

const core = new WorkerCore(env)

self.onmessage = (ev: MessageEvent<Msg>) => {
  const m = ev.data
  if (m && m.type === 'init') {
    // A relative baseUrl (Vite's BASE_URL is './' by default) means the page's folder, which this
    // worker cannot see from its own URL. The map sits at <base>maps/<file>, so its URL gives it.
    const mapUrl = new URL(String(m.mapUrl), self.location.href)
    pageBase = new URL(String(m.baseUrl ?? './'), new URL('../', mapUrl)).href
    m.baseUrl = pageBase
    m.mapUrl = mapUrl.href
    if (typeof m.bundleUrl === 'string') m.bundleUrl = new URL(m.bundleUrl, pageBase).href
  }
  core.onMessage(m)
}
