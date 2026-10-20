/*
 * File: LogDock.tsx
 *
 * Description:
 *   The dock under the map, like rqt_console: the nodes' log with sim time, level and node name
 *   (filter by level, node and text), the events (mode changes, arrival, GNSS faults, goal results,
 *   command replies) and the errors (Python exceptions, refused commands). It follows the newest
 *   line unless you scroll up. It folds to its tab bar.
 */

import { useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react'
import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { LogLevel, LogMsg } from '../protocol'
import { historyOf, useHistory, type FeedItem } from './history'
import { input } from './Panel'

type Tab = 'log' | 'events' | 'errors'
const LEVELS: LogLevel[] = ['debug', 'info', 'warn', 'error', 'fatal']
const LEVEL_CLS: Record<LogLevel, string> = {
  debug: 'text-gray-400 dark:text-gray-500',
  info: 'text-gray-700 dark:text-gray-300',
  warn: 'text-amber-600 dark:text-amber-400',
  error: 'text-red-600 dark:text-red-400',
  fatal: 'text-red-700 dark:text-red-300 font-bold',
}

function useFollow(dep: unknown) {
  const ref = useRef<HTMLDivElement>(null)
  const atEnd = useRef(true)
  useLayoutEffect(() => {
    const el = ref.current
    if (el && atEnd.current) el.scrollTop = el.scrollHeight
  }, [dep])
  const onScroll = () => {
    const el = ref.current
    if (el) atEnd.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24
  }
  return { ref, onScroll }
}

export default function LogDock() {
  const client = useSimClient()
  const { log } = useSimSnapshot()
  const hist = useHistory(client)
  const [tab, setTab] = useState<Tab>('log')
  const [open, setOpen] = useState(true)
  const [minLevel, setMinLevel] = useState<LogLevel>('info')
  const [node, setNode] = useState('')
  const [text, setText] = useState('')
  const [clearedAt, setClearedAt] = useState<LogMsg | null>(null)

  const nodes = useMemo(() => [...new Set(log.map((l) => l.node))].sort(), [log])
  const shown = useMemo(() => {
    const from = clearedAt ? log.indexOf(clearedAt) + 1 : 0
    const min = LEVELS.indexOf(minLevel)
    const q = text.toLowerCase()
    return log.slice(from).filter((l) => LEVELS.indexOf(l.level) >= min && (!node || l.node === node) && (!q || l.text.toLowerCase().includes(q)))
  }, [log, clearedAt, minLevel, node, text])
  const events = hist.feed.filter((f) => f.kind !== 'error' && (f.ok || f.kind !== 'command'))
  const errors = hist.feed.filter((f) => !f.ok)
  const warnCount = shown.filter((l) => l.level === 'warn').length
  const errCount = shown.filter((l) => l.level === 'error' || l.level === 'fatal').length

  // A new error opens the dock on its tab once, so it is not missed
  const lastErr = useRef(0)
  useEffect(() => {
    if (hist.errors > lastErr.current && errors.length && errors[errors.length - 1].kind === 'error') {
      setOpen(true)
      setTab('errors')
    }
    lastErr.current = hist.errors
  }, [hist.errors, errors])

  const tabBtn = (t: Tab, label: string, extra?: React.ReactNode) => (
    <button
      data-testid={`dock-tab-${t}`}
      onClick={() => {
        if (tab === t) setOpen(!open)
        else {
          setTab(t)
          setOpen(true)
        }
      }}
      className={`px-2.5 py-1 text-[11px] font-medium border-b-2 ${tab === t && open ? 'border-sky-500 text-gray-900 dark:text-gray-100' : 'border-transparent text-gray-500 hover:text-gray-800 dark:hover:text-gray-200'}`}
    >
      {label} {extra}
    </button>
  )

  return (
    <div className="shrink-0 border-t border-gray-200 dark:border-gray-800 bg-white dark:bg-gray-900" data-testid="log-dock">
      <div className="flex items-center gap-1 px-2">
        {tabBtn(
          'log',
          'Log',
          <span className="font-mono text-gray-400">
            {shown.length}
            {warnCount ? <span className="text-amber-500"> · {warnCount}w</span> : null}
            {errCount ? <span className="text-red-500"> · {errCount}e</span> : null}
          </span>,
        )}
        {tabBtn('events', 'Events', <span className="font-mono text-gray-400">{events.length}</span>)}
        {tabBtn('errors', 'Errors', <span className={`font-mono ${errors.length ? 'text-red-500' : 'text-gray-400'}`}>{errors.length}</span>)}
        {open && tab === 'log' && (
          <div className="ml-auto flex items-center gap-1.5 py-1">
            <select data-testid="log-level" className={input} value={minLevel} onChange={(e) => setMinLevel(e.target.value as LogLevel)}>
              {LEVELS.map((l) => (
                <option key={l} value={l}>
                  {l}+
                </option>
              ))}
            </select>
            <select data-testid="log-node" className={`${input} max-w-[11rem]`} value={node} onChange={(e) => setNode(e.target.value)}>
              <option value="">all nodes</option>
              {nodes.map((n) => (
                <option key={n} value={n}>
                  {n}
                </option>
              ))}
            </select>
            <input className={`${input} w-32`} placeholder="filter text" value={text} onChange={(e) => setText(e.target.value)} />
            <button className="text-[11px] text-gray-500 hover:text-gray-900 dark:hover:text-white px-1" onClick={() => setClearedAt(log[log.length - 1] ?? null)}>
              Clear
            </button>
          </div>
        )}
        {open && tab !== 'log' && (
          <button className="ml-auto text-[11px] text-gray-500 hover:text-gray-900 dark:hover:text-white px-1" onClick={() => historyOf(client).clearFeed()}>
            Clear
          </button>
        )}
        <button className="text-gray-400 hover:text-gray-800 dark:hover:text-gray-200 px-1 text-xs" title={open ? 'Fold' : 'Open'} onClick={() => setOpen(!open)}>
          {open ? '▾' : '▴'}
        </button>
      </div>
      {open && tab === 'log' && <LogList lines={shown} />}
      {open && tab === 'events' && <FeedList items={events} empty="No events yet." />}
      {open && tab === 'errors' && <FeedList items={errors} empty="No errors." />}
    </div>
  )
}

function LogList({ lines }: { lines: LogMsg[] }) {
  const { ref, onScroll } = useFollow(lines)
  return (
    <div ref={ref} onScroll={onScroll} data-testid="log-list" className="h-36 overflow-y-auto px-2 pb-1 font-mono text-[11px] leading-4">
      {lines.length === 0 && <div className="text-gray-500 py-1">No log lines.</div>}
      {lines.map((l, i) => (
        <div key={i} className={`flex gap-2 ${LEVEL_CLS[l.level]}`}>
          <span className="text-gray-400 dark:text-gray-600 tabular-nums w-14 text-right shrink-0">{l.t.toFixed(2)}</span>
          <span className="w-10 shrink-0 uppercase">{l.level}</span>
          <span className="w-44 shrink-0 truncate text-sky-700 dark:text-sky-400" title={l.node}>
            [{l.node}]
          </span>
          <span className="whitespace-pre-wrap break-words min-w-0">{l.text}</span>
        </div>
      ))}
    </div>
  )
}

const KIND_CLS: Record<FeedItem['kind'], string> = {
  mode: 'text-sky-700 dark:text-sky-400',
  arrived: 'text-emerald-700 dark:text-emerald-400',
  gnss_fault: 'text-amber-700 dark:text-amber-400',
  gps_health: 'text-amber-700 dark:text-amber-400',
  goal: 'text-fuchsia-700 dark:text-fuchsia-400',
  command: 'text-gray-600 dark:text-gray-400',
  error: 'text-red-600 dark:text-red-400',
}

function FeedList({ items, empty }: { items: FeedItem[]; empty: string }) {
  const { ref, onScroll } = useFollow(items)
  return (
    <div ref={ref} onScroll={onScroll} className="h-36 overflow-y-auto px-2 pb-1 font-mono text-[11px] leading-4" data-testid="feed-list">
      {items.length === 0 && <div className="text-gray-500 py-1">{empty}</div>}
      {items.map((f) => (
        <div key={f.seq} className="flex gap-2">
          <span className="text-gray-400 dark:text-gray-600 tabular-nums w-14 text-right shrink-0">{f.t === null ? '' : f.t.toFixed(2)}</span>
          <span className={`w-20 shrink-0 ${KIND_CLS[f.kind]}`}>{f.kind}</span>
          <span className={`min-w-0 ${f.ok ? 'text-gray-700 dark:text-gray-300' : 'text-red-600 dark:text-red-400'}`}>
            {f.text}
            {f.detail && (
              <details className="text-gray-500">
                <summary className="cursor-pointer">details</summary>
                <pre className="whitespace-pre-wrap">{f.detail}</pre>
              </details>
            )}
          </span>
        </div>
      ))}
    </div>
  )
}
