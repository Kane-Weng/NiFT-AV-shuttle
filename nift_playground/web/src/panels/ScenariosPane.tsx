/*
 * File: ScenariosPane.tsx
 *
 * Description:
 *   The Scenarios tab of the settings window: ready-made runs that reset the sim and drive.
 *     - Test routes of test_routes.yaml as cards, each with a top-down picture of the route. Load
 *       resets at the route's spawn and sends its goal; "start" also starts navigation.
 *     - GNSS fault scenarios of gnss_faults.yaml: reset on the scenario's route, goal, start, with
 *       the faults scheduled along the way.
 *
 *   The pictures are drawn from the page's own map. The line is the mock's shortest lanelet route
 *   (mock/route.ts), a preview only: the stack's planner makes the real path after Load.
 */

import { useMemo, useState } from 'react'
import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { RoutePreset } from '../protocol'
import type { LaneletMap, XY } from '../map/lanelets'
import { planPath } from '../mock/route'
import { faultPresets, routePresets } from './presets'
import { resetThenRoute } from './PlaygroundControls'
import { Pane, btn, btnActive } from './Panel'
import {
  FaultBurstIcon,
  FaultClockIcon,
  FaultDelayIcon,
  FaultDropoutIcon,
  FaultJumpIcon,
  FaultNoiseIcon,
  FaultNoneIcon,
  FaultStuckIcon,
  FaultSwapIcon,
} from './icons'

/** An icon and a tint per fault kind of gnss_faults.yaml; a scenario shows its first fault's. */
const FAULT_LOOK: Record<string, { icon: () => JSX.Element; tint: string; what: string }> = {
  dropout: { icon: FaultDropoutIcon, tint: 'text-red-500 bg-red-500/10', what: 'no fixes' },
  burst: { icon: FaultBurstIcon, tint: 'text-red-500 bg-red-500/10', what: 'short losses' },
  stuck: { icon: FaultStuckIcon, tint: 'text-amber-500 bg-amber-500/10', what: 'the feed repeats one epoch' },
  freeze: { icon: FaultStuckIcon, tint: 'text-amber-500 bg-amber-500/10', what: 'the fix freezes' },
  clock: { icon: FaultClockIcon, tint: 'text-violet-500 bg-violet-500/10', what: 'stamps off' },
  delay: { icon: FaultDelayIcon, tint: 'text-violet-500 bg-violet-500/10', what: 'fixes arrive late' },
  jump: { icon: FaultJumpIcon, tint: 'text-sky-500 bg-sky-500/10', what: 'the fix jumps' },
  noise: { icon: FaultNoiseIcon, tint: 'text-sky-500 bg-sky-500/10', what: 'scattered fixes' },
  swap: { icon: FaultSwapIcon, tint: 'text-sky-500 bg-sky-500/10', what: 'antennas swapped' },
}
const NO_FAULT = { icon: FaultNoneIcon, tint: 'text-emerald-500 bg-emerald-500/10', what: 'no faults' }
const lookOf = (faults: Record<string, unknown>[]) => FAULT_LOOK[String(faults[0]?.kind ?? '')] ?? NO_FAULT

/** "03_left_turn" -> "left turn" */
const shortName = (r: RoutePreset) => (r.name ?? r.description ?? '').replace(/^\d+_/, '').replace(/_/g, ' ')

export default function ScenariosPane({ map }: { map: LaneletMap | null }) {
  const client = useSimClient()
  const { ready, readyInfo } = useSimSnapshot()
  const routes = useMemo(() => routePresets(readyInfo), [readyInfo])
  const { scheduled } = useMemo(() => faultPresets(readyInfo), [readyInfo])
  const [routeId, setRouteId] = useState('')
  const [autoStart, setAutoStart] = useState(false)
  const route = routes.find((r) => r.id === routeId) ?? routes[0]

  const runFault = (name: string, routeOf?: string | null) => {
    const r = routes.find((q) => q.id === routeOf) ?? null
    resetThenRoute(client, { gnss_faults: name, ...(r ? { spawn: r.id } : {}) }, r, true, `Fault scenario ${name}`)
  }

  return (
    <div data-testid="scenarios-section" className="space-y-4">
      <Pane title="Test routes" right={<span className="text-[11px] text-gray-500">{routes.length} routes</span>}>
        {routes.length === 0 ? (
          <p className="text-xs text-gray-500">The worker sent no test routes.</p>
        ) : (
          <>
            <div className="grid grid-cols-3 gap-1.5">
              {routes.map((r) => (
                <button
                  key={r.id}
                  data-testid={`route-${r.id}`}
                  title={r.description ?? undefined}
                  onClick={() => setRouteId(r.id)}
                  className={`text-left rounded-md border p-1 transition-colors ${
                    r.id === route?.id ? 'border-sky-500 bg-sky-50 dark:bg-sky-950/40' : 'border-gray-200 dark:border-gray-700 hover:border-gray-400 dark:hover:border-gray-500'
                  }`}
                >
                  <RoutePicture map={map} route={r} />
                  <div className="mt-1 px-0.5 text-[11px] leading-4 truncate">
                    <span className="font-mono font-semibold">{r.id}</span> <span className="text-gray-600 dark:text-gray-300">{shortName(r)}</span>
                  </div>
                </button>
              ))}
            </div>
            {route && (
              <div className="mt-2 flex items-center gap-2">
                <p className="flex-1 min-w-0 text-[11px] text-gray-500 leading-4 line-clamp-2" title={route.description ?? undefined}>
                  {route.description}
                </p>
                <label className="flex items-center gap-1 text-[11px] text-gray-500 shrink-0" title="Start navigation once the goal is planned">
                  <input type="checkbox" checked={autoStart} onChange={(e) => setAutoStart(e.target.checked)} /> start
                </label>
                <button
                  data-testid="route-load"
                  className={`${btn} ${btnActive} shrink-0`}
                  disabled={!ready}
                  onClick={() => resetThenRoute(client, { spawn: route.id, gnss_faults: '' }, route, autoStart, `Route ${route.id}`)}
                >
                  Load {route.id}
                </button>
              </div>
            )}
          </>
        )}
      </Pane>

      <Pane title="GNSS fault scenarios" right={<span className="text-[11px] text-gray-500">{scheduled.length} scenarios</span>}>
        <p className="text-[11px] text-gray-500 mb-1.5 leading-4">Each one resets on its route, drives it and injects a GNSS fault on the way. Watch how the stack reacts (often a Safe Stop).</p>
        {scheduled.length === 0 ? (
          <p className="text-xs text-gray-500">The worker sent no fault scenarios.</p>
        ) : (
          <ul className="max-h-56 overflow-y-auto divide-y divide-gray-100 dark:divide-gray-800 rounded border border-gray-200 dark:border-gray-700" data-testid="fault-scenarios">
            {scheduled.map((s) => {
              const look = lookOf(s.faults)
              return (
                <li key={s.name} className="flex items-center gap-2 px-2 py-1">
                  <span className={`shrink-0 rounded p-1 ${look.tint}`} title={look.what}>
                    <look.icon />
                  </span>
                  <div className="flex-1 min-w-0">
                    <div className="font-mono text-xs truncate">{s.name}</div>
                    <div className="text-[11px] text-gray-500 truncate" title={faultText(s.faults)}>
                      {faultText(s.faults)}
                    </div>
                  </div>
                  {s.route && (
                    <span className="text-[11px] font-mono text-gray-500 shrink-0" title={routes.find((r) => r.id === s.route)?.description ?? undefined}>
                      route {s.route}
                    </span>
                  )}
                  <button data-testid={`fault-run-${s.name}`} className={`${btn} shrink-0`} disabled={!ready} onClick={() => runFault(s.name, s.route)}>
                    Run
                  </button>
                </li>
                )
            })}
          </ul>
        )}
      </Pane>
    </div>
  )
}

function faultText(faults: Record<string, unknown>[]): string {
  if (!faults.length) return 'no faults (control run)'
  return faults.map((f) => Object.entries(f).map(([k, v]) => `${k} ${String(v)}`).join(', ')).join('; ')
}

// ------------------------------------------------------------------ route picture

const W = 120
const H = 80

/** The route's area of the map, top-down: lane edges, the route, the spawn (orange) and the goal (magenta). */
function RoutePicture({ map, route }: { map: LaneletMap | null; route: RoutePreset }) {
  const pic = useMemo(() => (map ? drawRoute(map, route) : null), [map, route])
  return (
    <svg viewBox={`0 0 ${W} ${H}`} className="block w-full rounded" aria-hidden="true">
      <rect width={W} height={H} fill="#303030" />
      {pic && (
        <>
          <path d={pic.lanes} fill="none" stroke="#6b6b6b" strokeWidth="0.6" />
          <path d={pic.route} fill="none" stroke="#19ff00" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
          <circle cx={pic.spawn[0]} cy={pic.spawn[1]} r="3" fill="#ff8c1a" stroke="#303030" strokeWidth="1" />
          <circle cx={pic.goal[0]} cy={pic.goal[1]} r="3.2" fill="#e040fb" stroke="#303030" strokeWidth="1" />
        </>
      )}
    </svg>
  )
}

/**
 * Fits the route into the picture with some map around it (at least 80 m across, 3:2) and turns
 * map metres into picture units, y up. Only lane edges that cross the picture are drawn.
 */
function drawRoute(map: LaneletMap, r: RoutePreset) {
  const start: XY = [r.spawn[0], r.spawn[1]]
  const goal: XY = [r.goal[0], r.goal[1]]
  const path = planPath(map, start, r.spawn[2], goal) ?? [start, goal]
  const xs = path.map((p) => p[0])
  const ys = path.map((p) => p[1])
  const cx = (Math.min(...xs) + Math.max(...xs)) / 2
  const cy = (Math.min(...ys) + Math.max(...ys)) / 2
  const pad = 15
  const w = Math.max(80, Math.max(...xs) - Math.min(...xs) + 2 * pad, (Math.max(...ys) - Math.min(...ys) + 2 * pad) * (W / H))
  const h = (w * H) / W
  const k = W / w
  const px = (p: XY): XY => [(p[0] - cx) * k + W / 2, H / 2 - (p[1] - cy) * k]
  const line = (pts: XY[]) => pts.map((p, i) => `${i ? 'L' : 'M'}${px(p)[0].toFixed(1)} ${px(p)[1].toFixed(1)}`).join('')
  const inView = (pts: XY[]) => pts.some((p) => Math.abs(p[0] - cx) < w / 2 + 5 && Math.abs(p[1] - cy) < h / 2 + 5)
  const lanes = map.borders
    .filter((b) => b.kind === 'edge' && inView(b.points))
    .map((b) => line(b.points))
    .join('')
  return { lanes, route: line(path), spawn: px(start), goal: px(goal) }
}
