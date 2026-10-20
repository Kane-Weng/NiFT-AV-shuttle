/*
 * File: Panel.tsx
 *
 * Description:
 *   The frame every sidebar panel uses, plus small shared pieces: a title bar that can fold the
 *   panel (remembered per panel in this browser), sub-sections that fold, buttons and badges.
 */

import { useState, type ReactNode } from 'react'

function loadOpen(key: string, def: boolean): boolean {
  try {
    const v = localStorage.getItem(`playground.open.${key}`)
    return v === null ? def : v === '1'
  } catch {
    return def
  }
}

function saveOpen(key: string, v: boolean) {
  try {
    localStorage.setItem(`playground.open.${key}`, v ? '1' : '0')
  } catch {
    // storage blocked: the choice lasts for this page only
  }
}

function useOpen(key: string | undefined, def: boolean): [boolean, () => void] {
  const [open, setOpen] = useState(() => (key ? loadOpen(key, def) : def))
  return [
    open,
    () => {
      setOpen(!open)
      if (key) saveOpen(key, !open)
    },
  ]
}

interface PanelProps {
  title: string
  /** Fold key; without one the panel cannot fold. */
  fold?: string
  defaultOpen?: boolean
  right?: ReactNode
  testid?: string
  children?: ReactNode
}

export function Panel({ title, fold, defaultOpen = true, right, testid, children }: PanelProps) {
  const [open, toggle] = useOpen(fold, defaultOpen)
  const isOpen = fold ? open : true
  return (
    <section data-testid={testid} className="rounded-md border border-gray-200 dark:border-gray-800 bg-white dark:bg-gray-900">
      <h2 className="flex items-center justify-between gap-2 px-3 py-2 text-[11px] font-semibold uppercase tracking-wider text-gray-500 dark:text-gray-400">
        {fold ? (
          <button onClick={toggle} className="flex items-center gap-1.5 uppercase tracking-wider hover:text-gray-800 dark:hover:text-gray-200">
            <Caret open={isOpen} />
            {title}
          </button>
        ) : (
          <span>{title}</span>
        )}
        <span className="normal-case tracking-normal font-normal">{right}</span>
      </h2>
      {isOpen && <div className="px-3 pb-3">{children}</div>}
    </section>
  )
}

export function Caret({ open }: { open: boolean }) {
  return <span className={`inline-block w-2 text-[9px] transition-transform ${open ? 'rotate-90' : ''}`}>▶</span>
}

/** A folding block inside a panel. */
export function Section({ title, fold, defaultOpen = false, right, children, testid }: PanelProps) {
  const [open, toggle] = useOpen(fold, defaultOpen)
  return (
    <div className="mt-2 border-t border-gray-100 dark:border-gray-800 pt-2" data-testid={testid}>
      <div className="flex items-center justify-between">
        <button onClick={toggle} className="flex items-center gap-1.5 text-[11px] font-medium text-gray-600 dark:text-gray-300 hover:text-gray-900 dark:hover:text-white">
          <Caret open={open} />
          {title}
        </button>
        {right}
      </div>
      {open && <div className="mt-1.5">{children}</div>}
    </div>
  )
}

export const btn =
  'rounded px-2 py-1 text-xs border border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-gray-800 disabled:opacity-40 disabled:hover:bg-transparent'
export const btnActive = 'bg-sky-600 border-sky-500 text-white hover:bg-sky-500 dark:hover:bg-sky-500'
export const input =
  'rounded border border-gray-300 dark:border-gray-700 bg-white dark:bg-gray-950 px-1.5 py-0.5 text-xs font-mono text-gray-900 dark:text-gray-100'

export type Tone = 'ok' | 'warn' | 'error' | 'info' | 'muted' | 'manual'

const TONE: Record<Tone, string> = {
  ok: 'bg-emerald-100 text-emerald-800 border-emerald-300 dark:bg-emerald-900/40 dark:text-emerald-300 dark:border-emerald-700',
  warn: 'bg-amber-100 text-amber-800 border-amber-300 dark:bg-amber-900/40 dark:text-amber-300 dark:border-amber-700',
  error: 'bg-red-100 text-red-800 border-red-300 dark:bg-red-900/40 dark:text-red-300 dark:border-red-700',
  info: 'bg-sky-100 text-sky-800 border-sky-300 dark:bg-sky-900/40 dark:text-sky-300 dark:border-sky-700',
  manual: 'bg-orange-100 text-orange-800 border-orange-300 dark:bg-orange-900/40 dark:text-orange-300 dark:border-orange-700',
  muted: 'bg-gray-100 text-gray-600 border-gray-300 dark:bg-gray-800 dark:text-gray-400 dark:border-gray-700',
}

export function Badge({ tone, children, testid, title }: { tone: Tone; children: ReactNode; testid?: string; title?: string }) {
  return (
    <span data-testid={testid} title={title} className={`inline-flex items-center rounded border px-1.5 py-px text-[11px] font-mono font-semibold ${TONE[tone]}`}>
      {children}
    </span>
  )
}

export function Row({ k, children, testid }: { k: string; children: ReactNode; testid?: string }) {
  return (
    <div className="flex justify-between gap-2 text-xs leading-5">
      <span className="text-gray-500 whitespace-nowrap">{k}</span>
      <span data-testid={testid} className="font-mono tabular-nums text-right truncate">
        {children}
      </span>
    </div>
  )
}
