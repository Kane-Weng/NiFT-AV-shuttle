/*
 * File: SettingsDialog.tsx
 *
 * Description:
 *   The settings window under the gear in the header. It floats over the top right of the page and
 *   has five tabs:
 *     - View: the theme, lane centerlines, the 3D world and the ROS log, each with a small picture
 *     - Scenarios: test routes with a picture of each, and the GNSS fault scenarios (ScenariosPane.tsx)
 *     - Launch: launch options, applied with a reset
 *     - Parameters: staged for the next reset
 *     - Share: save the run as a link, or load one
 *   The controls used while watching a run (sim clock, obstacles, live GNSS faults) stay in the sidebar.
 *
 *   The window stays mounted while closed, so a half-filled launch form or the staged parameters
 *   survive closing it. Escape or a press outside closes it. The last tab is remembered in this browser.
 */

import { useEffect, useRef, useState, type ReactNode } from 'react'
import { setLog, setUi, useUi } from './ui'
import { LaunchOptions, Params, ScenarioBox } from './PlaygroundControls'
import ScenariosPane from './ScenariosPane'
import type { ThemeName } from '../scene/theme'
import type { LaneletMap } from '../map/lanelets'

const TABS = [
  { id: 'view', label: 'View' },
  { id: 'scenarios', label: 'Scenarios' },
  { id: 'launch', label: 'Launch' },
  { id: 'params', label: 'Parameters' },
  { id: 'share', label: 'Share' },
] as const
type TabId = (typeof TABS)[number]['id']

function loadTab(): TabId {
  try {
    const v = localStorage.getItem('playground.settingsTab')
    if (TABS.some((t) => t.id === v)) return v as TabId
  } catch {
    // storage blocked: start on View
  }
  return 'view'
}

interface Props {
  open: boolean
  onClose: () => void
  theme: ThemeName
  setTheme: (t: ThemeName) => void
  map: LaneletMap | null
}

export default function SettingsDialog({ open, onClose, theme, setTheme, map }: Props) {
  const ref = useRef<HTMLDivElement>(null)
  const [tab, setTab] = useState<TabId>(loadTab)

  const pick = (t: TabId) => {
    setTab(t)
    try {
      localStorage.setItem('playground.settingsTab', t)
    } catch {
      // storage blocked: the tab lasts for this page only
    }
  }

  useEffect(() => {
    if (!open) return
    const onKey = (e: KeyboardEvent) => e.key === 'Escape' && onClose()
    const onDown = (e: PointerEvent) => {
      const t = e.target as Element | null
      if (ref.current?.contains(t) || t?.closest?.('[data-dialog-toggle]')) return
      onClose()
    }
    window.addEventListener('keydown', onKey)
    document.addEventListener('pointerdown', onDown, true)
    return () => {
      window.removeEventListener('keydown', onKey)
      document.removeEventListener('pointerdown', onDown, true)
    }
  }, [open, onClose])

  return (
    <div
      ref={ref}
      role="dialog"
      aria-label="Settings"
      data-testid="settings-dialog"
      className={`${open ? 'flex' : 'hidden'} fixed z-50 top-12 right-3 w-[27rem] max-w-[calc(100vw-1.5rem)] max-h-[calc(100vh-4rem)] flex-col rounded-lg border border-gray-200 dark:border-gray-700 bg-white dark:bg-gray-900 shadow-xl`}
    >
      <div className="flex items-center gap-1 px-2 pt-2 border-b border-gray-200 dark:border-gray-800" role="tablist">
        {TABS.map((t) => (
          <button
            key={t.id}
            role="tab"
            aria-selected={tab === t.id}
            data-testid={`settings-tab-${t.id}`}
            onClick={() => pick(t.id)}
            className={`px-3 py-1.5 -mb-px text-xs border-b-2 transition-colors ${
              tab === t.id ? 'border-sky-500 text-sky-700 dark:text-sky-300 font-medium' : 'border-transparent text-gray-500 hover:text-gray-800 dark:hover:text-gray-200'
            }`}
          >
            {t.label}
          </button>
        ))}
        <button data-testid="settings-close" onClick={onClose} aria-label="Close settings" className="ml-auto mb-1 px-2 py-0.5 rounded text-gray-400 hover:text-gray-800 dark:hover:text-gray-100 hover:bg-gray-100 dark:hover:bg-gray-800">
          ✕
        </button>
      </div>
      <div className="p-3 overflow-y-auto">
        {/* Every tab stays mounted so its state survives switching tabs */}
        <div hidden={tab !== 'view'}>
          <ViewTab theme={theme} setTheme={setTheme} />
        </div>
        <div hidden={tab !== 'scenarios'}>
          <ScenariosPane map={map} />
        </div>
        <div hidden={tab !== 'launch'}>
          <LaunchOptions />
        </div>
        <div hidden={tab !== 'params'}>
          <Params />
        </div>
        <div hidden={tab !== 'share'}>
          <ScenarioBox />
        </div>
      </div>
    </div>
  )
}

// ------------------------------------------------------------------ view tab

function ViewTab({ theme, setTheme }: { theme: ThemeName; setTheme: (t: ThemeName) => void }) {
  const { centerlines, world, worldLoad, log } = useUi()
  const worldNote =
    worldLoad.state === 'loading'
      ? `Loading ${worldLoad.pct}%`
      : worldLoad.state === 'error'
        ? `Did not load: ${worldLoad.error}`
        : worldLoad.state === 'ready'
          ? 'Buildings, trees and terrain from the public digital twin (MIT).'
          : 'Buildings, trees and terrain from the public digital twin (MIT). About 2 MB, loads on first use.'

  return (
    <div data-testid="view-section" className="space-y-3">
      <div className="flex items-center justify-between">
        <span className="text-xs font-semibold text-gray-700 dark:text-gray-200">Theme</span>
        <div className="flex rounded-md border border-gray-300 dark:border-gray-700 overflow-hidden text-xs" role="radiogroup" aria-label="Theme">
          {(['dark', 'light'] as const).map((t) => (
            <button
              key={t}
              role="radio"
              aria-checked={theme === t}
              data-testid={`theme-${t}`}
              onClick={() => setTheme(t)}
              className={`px-3 py-1 capitalize ${theme === t ? 'bg-sky-600 text-white' : 'text-gray-600 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-800'}`}
            >
              {t}
            </button>
          ))}
        </div>
      </div>
      <div className="grid grid-cols-2 gap-2">
        <OptionCard testid="toggle-centerlines" title="Lane centerlines" on={centerlines} onToggle={() => setUi({ centerlines: !centerlines })} picture={<CenterlinePicture />}>
          The middle of each lane, the line the planner routes along.
        </OptionCard>
        <OptionCard
          testid="toggle-world"
          title="3D world"
          on={world}
          onToggle={() => setUi({ world: !world })}
          picture={<WorldPicture />}
          busy={worldLoad.state === 'loading' ? worldLoad.pct : undefined}
          error={worldLoad.state === 'error'}
        >
          {worldNote}
        </OptionCard>
        <OptionCard testid="toggle-log" title="ROS log" on={log} onToggle={() => setLog(!log)} picture={<LogPicture />}>
          The nodes' log, events and errors under the map, like rqt_console.
        </OptionCard>
      </div>
    </div>
  )
}

/** A view option as a card: a picture of what it shows, its name, a switch and a note. */
function OptionCard(props: { testid: string; title: string; on: boolean; onToggle: () => void; picture: ReactNode; children: ReactNode; busy?: number; error?: boolean }) {
  const { on } = props
  return (
    <button
      role="switch"
      aria-checked={on}
      data-testid={props.testid}
      onClick={props.onToggle}
      className={`text-left rounded-lg border p-1.5 transition-colors ${
        on ? 'border-sky-500 bg-sky-50 dark:bg-sky-950/40' : 'border-gray-200 dark:border-gray-700 hover:border-gray-400 dark:hover:border-gray-500'
      }`}
    >
      <div className={`rounded overflow-hidden transition-opacity ${on ? '' : 'opacity-60'}`}>{props.picture}</div>
      <div className="flex items-center justify-between gap-2 mt-1.5 px-0.5">
        <span className="text-xs font-medium text-gray-800 dark:text-gray-100">{props.title}</span>
        <span className={`relative inline-block w-7 h-4 rounded-full transition-colors shrink-0 ${on ? 'bg-sky-600' : 'bg-gray-300 dark:bg-gray-600'}`}>
          <span className={`absolute top-0.5 w-3 h-3 rounded-full bg-white shadow transition-all ${on ? 'left-3.5' : 'left-0.5'}`} />
        </span>
      </div>
      {props.busy !== undefined && (
        <div className="mx-0.5 mt-1 h-1 rounded bg-gray-200 dark:bg-gray-700 overflow-hidden">
          <div className="h-full bg-sky-500 transition-all duration-300" style={{ width: `${Math.max(3, props.busy)}%` }} />
        </div>
      )}
      <p className={`px-0.5 mt-1 text-[11px] leading-4 ${props.error ? 'text-red-600 dark:text-red-400 break-words' : 'text-gray-500'}`}>{props.children}</p>
    </button>
  )
}

// The pictures use the map's own colours (scene/theme.ts): white lane edges, a grey divider and
// the blue centerlines on the dark RViz background, in both themes, as a preview of the map.

/** A two-lane road on the map with each lane's centerline drawn in blue. */
function CenterlinePicture() {
  const road = 'M-4 50 C40 50 70 18 124 18'
  return (
    <svg viewBox="0 0 120 64" className="block w-full" aria-hidden="true">
      <rect width="120" height="64" fill="#303030" />
      <g fill="none" strokeLinecap="round">
        <path d={road} transform="translate(0 -13)" stroke="#e6e6e6" strokeWidth="1.5" />
        <path d={road} transform="translate(0 13)" stroke="#e6e6e6" strokeWidth="1.5" />
        <path d={road} stroke="#b5b5b5" strokeWidth="1" strokeDasharray="4 3" />
        <path d={road} transform="translate(0 -6.5)" stroke="#3fa7e0" strokeWidth="2" />
        <path d={road} transform="translate(0 6.5)" stroke="#3fa7e0" strokeWidth="2" />
      </g>
      {/* Direction ticks on the centerlines */}
      <g fill="#3fa7e0">
        <path d="M-3 -2.5l6 2.5-6 2.5z" transform="translate(32.8 49.6) rotate(-19)" />
        <path d="M-3 -2.5l6 2.5-6 2.5z" transform="translate(87.4 16.5) rotate(165)" />
      </g>
    </svg>
  )
}

/** A few lines of the log dock: time, level, node and message. */
function LogPicture() {
  const rows = [
    { lvl: '#9ca3af', w: 44 },
    { lvl: '#9ca3af', w: 56 },
    { lvl: '#f59e0b', w: 38 },
    { lvl: '#9ca3af', w: 50 },
    { lvl: '#ef4444', w: 30 },
  ]
  return (
    <svg viewBox="0 0 120 64" className="block w-full" aria-hidden="true">
      <rect width="120" height="64" fill="#111827" />
      <rect width="120" height="11" fill="#1f2937" />
      <rect x="5" y="4" width="14" height="3" rx="1" fill="#38bdf8" />
      <rect x="24" y="4" width="14" height="3" rx="1" fill="#6b7280" />
      <rect x="43" y="4" width="14" height="3" rx="1" fill="#6b7280" />
      {rows.map((r, i) => (
        <g key={i} transform={`translate(0 ${17 + i * 9.5})`}>
          <rect x="5" width="9" height="3" rx="1" fill="#6b7280" />
          <rect x="17" width="9" height="3" rx="1" fill={r.lvl} />
          <rect x="29" width="20" height="3" rx="1" fill="#38bdf8" fillOpacity="0.7" />
          <rect x="53" width={r.w} height="3" rx="1" fill="#d1d5db" fillOpacity="0.8" />
        </g>
      ))}
    </svg>
  )
}

/** The same road on ground with buildings and trees, seen at a slant. */
function WorldPicture() {
  return (
    <svg viewBox="0 0 120 64" className="block w-full" aria-hidden="true">
      <defs>
        <linearGradient id="sky" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#3b4a5c" />
          <stop offset="1" stopColor="#56677a" />
        </linearGradient>
      </defs>
      <rect width="120" height="64" fill="url(#sky)" />
      {/* Ground */}
      <path d="M0 26 L120 22 L120 64 L0 64z" fill="#5b6b4a" />
      {/* Road in perspective, with its lanes */}
      <path d="M44 64 L58 24 L70 24 L96 64z" fill="#55585c" />
      <g fill="none" strokeLinecap="round">
        <path d="M44 64 L58 24" stroke="#e6e6e6" strokeWidth="1.2" />
        <path d="M96 64 L70 24" stroke="#e6e6e6" strokeWidth="1.2" />
        <path d="M70 64 L64 24" stroke="#b5b5b5" strokeWidth="0.9" strokeDasharray="3 2.5" />
      </g>
      {/* Buildings: a front face and a lighter roof */}
      <path d="M6 44 V26 L22 23 V41z" fill="#8a8f98" />
      <path d="M6 26 L14 22 L30 19 L22 23z" fill="#b3b8c0" />
      <path d="M22 41 V23 L30 19 V36z" fill="#6c717a" />
      <path d="M98 40 V24 L112 22 V38z" fill="#8a8f98" />
      <path d="M98 24 L104 20 L118 18 L112 22z" fill="#b3b8c0" />
      {/* Trees */}
      <g fill="#2f5d34">
        <circle cx="36" cy="38" r="5" />
        <circle cx="30" cy="50" r="6" />
        <circle cx="104" cy="50" r="6" />
        <circle cx="86" cy="32" r="3.5" />
      </g>
      <g fill="#3f7a45">
        <circle cx="35" cy="36.5" r="2.5" />
        <circle cx="29" cy="48" r="3" />
        <circle cx="103" cy="48" r="3" />
      </g>
    </svg>
  )
}
