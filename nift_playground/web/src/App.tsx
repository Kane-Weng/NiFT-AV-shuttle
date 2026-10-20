/*
 * File: App.tsx
 *
 * Description:
 *   The page shell of the web playground: a header, the map view with the log dock under it, and a
 *   sidebar with the operator panel, telemetry and playground controls. The map loads in the page first, so it
 *   shows before the sim worker is ready.
 *
 *   URL options: ?sim=pyodide (the default, the real stack) or ?sim=mock, ?theme=dark|light, ?cam=ortho|persp, and a scenario as
 *   #scenario=<code> (or ?scenario=<code>), loaded once the sim is ready.
 */

import { useEffect, useState } from 'react'
import MapView from './scene/MapView'
import OperatorPanel from './panels/OperatorPanel'
import TelemetryPanel from './panels/TelemetryPanel'
import PlaygroundControls from './panels/PlaygroundControls'
import LogDock from './panels/LogDock'
import StatusPill from './panels/StatusPill'
import { historyOf } from './panels/history'
import { SimContext, useSimSnapshot } from './sim/useSim'
import type { SimClient, SimKind } from './sim/SimClient'
import { loadLaneletMap, mapUrl } from './map/loadMap'
import type { LaneletMap } from './map/lanelets'
import type { ThemeName } from './scene/theme'

const params = new URLSearchParams(window.location.search)

function initialTheme(): ThemeName {
  const p = params.get('theme')
  if (p === 'light' || p === 'dark') return p
  try {
    const saved = localStorage.getItem('playground.theme')
    if (saved === 'light' || saved === 'dark') return saved
  } catch {
    // storage blocked: fall back to the RViz look
  }
  return 'dark'
}

function SimStatus({ kind }: { kind: SimKind }) {
  const { ready, progress } = useSimSnapshot()
  const dot = ready ? 'bg-green-500' : 'bg-yellow-400 animate-pulse'
  return (
    <div className="flex items-center gap-2 text-xs text-gray-500 dark:text-gray-400" data-testid="sim-status">
      <div className={`w-2 h-2 rounded-full ${dot}`} />
      {ready ? `Sim ready (${kind})` : progress ? `${progress.stage} ${progress.pct.toFixed(0)}%` : 'Starting sim…'}
    </div>
  )
}

const STAGE_TEXT: Record<string, string> = {
  starting: 'Starting the sim worker',
  mock: 'Starting the mock sim',
  pyodide: 'Loading Python (Pyodide)',
  packages: 'Loading numpy and pyyaml',
  bundle: 'Loading our ROS nodes',
  map: 'Loading the Lanelet2 map',
  nodes: 'Starting the nodes',
}

/** Over the map until the first ready: the boot stages of the worker, or why it failed. */
function BootOverlay({ kind }: { kind: SimKind }) {
  const { readyInfo, progress, error } = useSimSnapshot()
  if (readyInfo) return null
  const failed = error?.fatal ? error : null
  const pct = Math.max(2, Math.min(100, progress?.pct ?? 0))
  return (
    <div className="absolute inset-0 flex items-center justify-center pointer-events-none" data-testid="boot-overlay">
      <div className="pointer-events-auto w-80 max-w-[calc(100%-2rem)] rounded-lg border border-gray-200 dark:border-gray-700 bg-white/95 dark:bg-gray-900/95 shadow-lg p-4 text-sm">
        {failed ? (
          <>
            <div className="font-semibold text-red-600 dark:text-red-400 mb-1">The sim did not start</div>
            <div className="text-xs font-mono break-words text-gray-700 dark:text-gray-300" data-testid="boot-error">{failed.message}</div>
            <div className="text-[11px] text-gray-500 mt-2">The traceback is under Errors in the log dock. Reload to try again.</div>
          </>
        ) : (
          <>
            <div className="flex justify-between mb-2">
              <span className="font-semibold">{STAGE_TEXT[progress?.stage ?? 'starting'] ?? progress?.stage}</span>
              <span className="font-mono text-gray-500 tabular-nums">{pct.toFixed(0)}%</span>
            </div>
            <div className="h-1.5 rounded bg-gray-200 dark:bg-gray-700 overflow-hidden">
              <div className="h-full bg-indigo-500 transition-all duration-300" style={{ width: `${pct}%` }} />
            </div>
            <div className="text-[11px] text-gray-500 mt-2">
              {kind === 'pyodide'
                ? 'The real stack runs in your browser: our nodes, unchanged, in Python. The first visit downloads about 10 MB (Python, numpy, our nodes, the map).'
                : 'Mock sim (?sim=mock). Remove it from the URL for the real stack.'}
            </div>
          </>
        )}
      </div>
    </div>
  )
}

export default function App({ client }: { client: SimClient }) {
  const [theme, setTheme] = useState<ThemeName>(initialTheme)
  const [map, setMap] = useState<LaneletMap | null>(null)
  const [mapError, setMapError] = useState<string | null>(null)
  const kind: SimKind = client.kind
  historyOf(client) // start keeping plots, events and errors from the first message

  useEffect(() => {
    document.documentElement.classList.toggle('dark', theme === 'dark')
    try {
      localStorage.setItem('playground.theme', theme)
    } catch {
      // storage blocked: the choice lasts for this page only
    }
  }, [theme])

  useEffect(() => {
    loadLaneletMap(mapUrl())
      .then(setMap)
      .catch((e) => setMapError(String(e)))
  }, [])

  return (
    <SimContext.Provider value={client}>
      <div className="flex flex-col h-screen bg-gray-100 dark:bg-gray-950 text-gray-900 dark:text-gray-100">
        <header className="flex items-center justify-between px-4 py-2 bg-white dark:bg-gray-900 border-b border-gray-200 dark:border-gray-800 shrink-0">
          <div className="flex items-center gap-3">
            <span className="font-bold tracking-widest text-sm">NIFT</span>
            <span className="text-gray-500 text-xs">Web Playground</span>
          </div>
          <div className="flex items-center gap-4">
            <StatusPill />
            <SimStatus kind={kind} />
            <button
              data-testid="theme-toggle"
              onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}
              className="text-xs px-2 py-1 rounded border border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-gray-800"
            >
              {theme === 'dark' ? 'Light' : 'Dark'}
            </button>
          </div>
        </header>

        <div className="flex flex-1 min-h-0 flex-col md:flex-row">
          <main className="flex-1 min-w-0 min-h-[55vh] md:min-h-0 flex flex-col">
            <div className="flex-1 min-h-0 relative">
              <MapView map={map} theme={theme} />
              <BootOverlay kind={kind} />
              {mapError && (
                <div className="absolute bottom-10 left-3 right-3 rounded bg-red-600 text-white text-xs p-2">Map failed to load: {mapError}</div>
              )}
            </div>
            <LogDock />
          </main>
          <aside className="md:w-[22rem] flex flex-col gap-2 p-2 bg-gray-50 dark:bg-gray-950 md:border-l border-t md:border-t-0 border-gray-200 dark:border-gray-800 overflow-y-auto shrink-0">
            <OperatorPanel />
            <TelemetryPanel />
            <PlaygroundControls />
          </aside>
        </div>
      </div>
    </SimContext.Provider>
  )
}
