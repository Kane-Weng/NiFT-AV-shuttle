/*
 * File: PlaygroundControls.tsx
 *
 * Description:
 *   The playground's own controls, the things a real shuttle does not have (ARCH.md 7, 8.1, 8.4,
 *   8.5):
 *     - sim clock: pause, run, step n ticks, speed factor (0.1 to 100 or max), reset
 *     - obstacles: the Obstacle tool (click the map to place), drag to move, delete, and what the
 *       stack perceived beside what was placed (only with the perception option)
 *     - GNSS faults: the live presets (now) and the scheduled scenarios of gnss_faults.yaml (reset)
 *     - routes and launch options (localization mode, GNSS feed, perception, filter), on reset
 *     - parameters: set_param on a short whitelist, staged for the next reset
 *     - scenario: save, share as a link (#scenario=...), load from a link or JSON
 */

import { useEffect, useMemo, useRef, useState } from 'react'
import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { SimClient } from '../sim/SimClient'
import type { LiveFaultPreset, PageToWorker, RoutePreset, Scenario, WorkerToPage, XY } from '../protocol'
import { historyOf } from './history'
import { OBSTACLE_SIZES, setUi, toggleTool, useUi, type ObstacleSize } from './ui'
import { centroid, nextObstacleId, pointInPolygon } from './geom'
import { LAUNCH_CHOICES, PARAMS, faultPresets, launchOptions, routePresets } from './presets'
import { Badge, Panel, Section, btn, btnActive, input } from './Panel'
import { encodeScenario, parseScenarioInput, scenarioCodeFrom, decodeScenario, urlWithScenario } from '../scenario/codec'

const SPEEDS: (number | 'max')[] = [0.5, 1, 2, 5, 10, 'max']

/** Runs a function on the next message of a type (once). Returns a cancel function. */
function onNext<T extends WorkerToPage['type']>(client: SimClient, type: T, fn: (m: Extract<WorkerToPage, { type: T }>) => void): () => void {
  const off = client.onMessage((m) => {
    if (m.type === type) {
      off()
      fn(m as Extract<WorkerToPage, { type: T }>)
    }
  })
  return off
}

/**
 * Resets with launch options, then sends the route's goal once the stack is ready, and starts
 * when the goal is planned (if asked). Like sim_route_run does on ROS.
 */
function resetThenRoute(client: SimClient, launch: Record<string, unknown>, route: RoutePreset | null, autoStart: boolean, label: string) {
  const h = historyOf(client)
  if (route) {
    onNext(client, 'ready', () => {
      if (autoStart) onNext(client, 'goal_ack', (a) => a.ok && void h.command({ type: 'cmd', name: 'start' }, 'Start navigation'))
      client.send({ type: 'goal', x: route.goal[0], y: route.goal[1], yaw: route.goal[2] })
    })
  }
  void h.command({ type: 'reset', launch }, label)
}

export default function PlaygroundControls() {
  const client = useSimClient()
  const { ready, state } = useSimSnapshot()
  useScenarioFromUrl(client)
  const [n, setN] = useState(10)
  const paused = state?.paused ?? false
  const factor = state?.factor ?? 1
  const h = historyOf(client)

  return (
    <Panel
      title="Playground"
      testid="playground-panel"
      right={
        state ? (
          <span className="font-mono" data-testid="sim-clock">
            tick {state.tick} · rtf {state.rtf.toFixed(1)}
          </span>
        ) : null
      }
    >
      <div className="flex flex-wrap items-center gap-1.5 mb-1.5">
        <button data-testid="sim-pause" className={`${btn} w-14 ${paused ? btnActive : ''}`} disabled={!ready} onClick={() => client.send({ type: paused ? 'resume_sim' : 'pause' })}>
          {paused ? 'Run' : 'Pause'}
        </button>
        <button data-testid="sim-step" className={btn} disabled={!ready || !paused} onClick={() => client.send({ type: 'step', n })} title="Run n ticks of 50 ms while paused">
          Step
        </button>
        <input data-testid="sim-step-n" type="number" min={1} max={2000} className={`${input} w-14`} value={n} onChange={(e) => setN(Math.max(1, Math.min(2000, Math.round(+e.target.value) || 1)))} />
        <span className="text-[11px] text-gray-500">ticks</span>
        <button data-testid="sim-reset" className={`${btn} ml-auto`} disabled={!ready} onClick={() => void h.command({ type: 'reset' }, 'Reset')} title="Rebuild the stack with the current launch options and staged parameters">
          Reset
        </button>
      </div>
      <div className="flex gap-1" data-testid="sim-speeds">
        {SPEEDS.map((s) => (
          <button key={s} data-testid={`speed-${s}`} disabled={!ready} onClick={() => client.send({ type: 'speed', factor: s })} className={`${btn} flex-1 px-0 ${factor === s ? btnActive : ''}`}>
            {s === 'max' ? 'max' : `x${s}`}
          </button>
        ))}
      </div>

      <Obstacles />
      <GnssFaults />
      <RoutesAndLaunch />
      <Params />
      <ScenarioBox />
    </Panel>
  )
}

// ------------------------------------------------------------------ obstacles

function Obstacles() {
  const client = useSimClient()
  const { ready, obstacles, perceived, state } = useSimSnapshot()
  const { tool, selected, obstacleSize } = useUi()
  const h = historyOf(client)

  const seen = (poly: XY[]) => !!perceived?.some((p) => p.length && (pointInPolygon(centroid(p), poly) || Math.hypot(centroid(p)[0] - centroid(poly)[0], centroid(p)[1] - centroid(poly)[1]) < 1.5))
  const addAhead = () => {
    if (!state) return
    const { x, y, yaw } = state.pose
    const sz = OBSTACLE_SIZES[obstacleSize]
    const id = nextObstacleId(obstacles)
    void h.command({ type: 'obstacle_add', oid: id, box: { x: x + 12 * Math.cos(yaw), y: y + 12 * Math.sin(yaw), yaw, length: sz.length, width: sz.width } }, `Add ${id}`)
    setUi({ selected: id })
  }

  return (
    <Section
      title={`Obstacles (${obstacles.length})`}
      fold="pg.obstacles"
      defaultOpen
      testid="obstacles-section"
      right={
        <span className="text-[11px]">
          {perceived === null ? <Badge tone="muted">perception off</Badge> : <Badge tone="info">{`perceived ${perceived.length}`}</Badge>}
        </span>
      }
    >
      <div className="flex flex-wrap items-center gap-1.5">
        <button data-testid="tool-obstacle-panel" className={`${btn} ${tool === 'obstacle' ? btnActive : ''}`} onClick={() => toggleTool('obstacle')} title="Click the map to place an obstacle (O)">
          {tool === 'obstacle' ? 'Click the map…' : 'Place (O)'}
        </button>
        <select className={`${input} w-24`} value={obstacleSize} onChange={(e) => setUi({ obstacleSize: e.target.value as ObstacleSize })}>
          {Object.entries(OBSTACLE_SIZES).map(([k, v]) => (
            <option key={k} value={k}>
              {v.label}
            </option>
          ))}
        </select>
        <button className={btn} disabled={!ready || !state} onClick={addAhead} title="Place one 12 m ahead of the shuttle">
          Ahead
        </button>
        <button className={btn} disabled={!ready || !obstacles.length} onClick={() => void h.command({ type: 'set_obstacles', obstacles: [] }, 'Clear obstacles')}>
          Clear
        </button>
      </div>
      <ul className="mt-1.5 space-y-px" data-testid="obstacle-list">
        {obstacles.map((o) => {
          const c = centroid(o.poly)
          return (
            <li key={o.oid} className={`flex items-center gap-2 rounded px-1 text-xs ${selected === o.oid ? 'bg-sky-100 dark:bg-sky-900/40' : ''}`}>
              <button className="font-mono font-semibold w-8 text-left" onClick={() => setUi({ selected: o.oid })}>
                {o.oid}
              </button>
              <span className="font-mono text-gray-500">
                {c[0].toFixed(1)}, {c[1].toFixed(1)}
              </span>
              {perceived !== null && (seen(o.poly) ? <Badge tone="ok">seen</Badge> : <Badge tone="muted">unseen</Badge>)}
              <button data-testid={`obstacle-del-${o.oid}`} className="ml-auto text-gray-400 hover:text-red-500 px-1" title="Delete" onClick={() => void h.command({ type: 'obstacle_remove', oid: o.oid }, `Remove ${o.oid}`)}>
                ✕
              </button>
            </li>
          )
        })}
      </ul>
      <p className="mt-1 text-[11px] text-gray-500 leading-4">
        Drag an obstacle on the map to move it. Delete removes the selected one.{' '}
        {perceived === null
          ? 'Perception is off (sim_kinematic): the stack does not see obstacles and the safety filter is none. Turn perception on under Launch.'
          : 'Dashed outlines are what /obstacles held (sensor range and field of view).'}
      </p>
    </Section>
  )
}

// ------------------------------------------------------------------ GNSS faults

function GnssFaults() {
  const client = useSimClient()
  const { ready, readyInfo } = useSimSnapshot()
  const { live, scheduled } = useMemo(() => faultPresets(readyInfo), [readyInfo])
  const routes = useMemo(() => routePresets(readyInfo), [readyInfo])
  const [liveName, setLiveName] = useState('')
  const [params, setParams] = useState<Record<string, string>>({})
  const [schedName, setSchedName] = useState('')
  const preset: LiveFaultPreset | undefined = live.find((p) => p.name === liveName) ?? live[0]
  const sched = scheduled.find((s) => s.name === schedName) ?? scheduled[0]
  const h = historyOf(client)

  useEffect(() => {
    if (preset) setParams(Object.fromEntries(Object.entries(preset.params ?? {}).map(([k, v]) => [k, String(v)])))
  }, [preset?.name]) // eslint-disable-line react-hooks/exhaustive-deps

  const apply = () => {
    if (!preset) return
    const out: Record<string, number | string> = {}
    for (const [k, v] of Object.entries(params)) {
      const def = preset.params[k]
      out[k] = typeof def === 'number' ? Number(v) : v
    }
    void h.command({ type: 'gnss_fault', preset: preset.name, params: out }, `GNSS fault ${preset.name}`)
  }

  const runScheduled = () => {
    if (!sched) return
    const route = routes.find((r) => r.id === sched.route) ?? null
    resetThenRoute(client, { gnss_faults: sched.name, ...(route ? { spawn: route.id } : {}) }, route, true, `Fault scenario ${sched.name}`)
  }

  return (
    <Section title="GNSS faults" fold="pg.gnss" testid="gnss-section" right={<span className="text-[11px] text-gray-500">{live.length + scheduled.length} presets</span>}>
      {live.length === 0 && scheduled.length === 0 && <p className="text-xs text-gray-500">The worker sent no fault presets.</p>}
      {live.length > 0 && (
        <div>
          <div className="text-[11px] text-gray-500 mb-1">Now (from the next tick)</div>
          <div className="flex gap-1.5 mb-1">
            <select data-testid="fault-live" className={`${input} flex-1`} value={preset?.name ?? ''} onChange={(e) => setLiveName(e.target.value)}>
              {(['sim', 'link'] as const).map((k) => (
                <optgroup key={k} label={k === 'sim' ? 'sim GNSS (GnssFault)' : 'Octane link'}>
                  {live
                    .filter((p) => p.kind === k)
                    .map((p) => (
                      <option key={p.name} value={p.name}>
                        {p.name}
                      </option>
                    ))}
                </optgroup>
              ))}
            </select>
            <button data-testid="fault-apply" className={btn} disabled={!ready || !preset} onClick={apply}>
              Apply
            </button>
          </div>
          {preset?.doc && <div className="text-[11px] text-gray-500 mb-1">{preset.doc}</div>}
          <div className="grid grid-cols-2 gap-x-2 gap-y-1">
            {Object.entries(params).map(([k, v]) => (
              <label key={k} className="flex items-center justify-between gap-1 text-[11px]">
                <span className="text-gray-500">{k}</span>
                {k === 'antenna' ? (
                  <select className={`${input} w-20`} value={v} onChange={(e) => setParams({ ...params, [k]: e.target.value })}>
                    {['both', 'front', 'rear'].map((a) => (
                      <option key={a}>{a}</option>
                    ))}
                  </select>
                ) : (
                  <input className={`${input} w-20`} value={v} onChange={(e) => setParams({ ...params, [k]: e.target.value })} />
                )}
              </label>
            ))}
          </div>
        </div>
      )}
      {scheduled.length > 0 && (
        <div className="mt-2">
          <div className="text-[11px] text-gray-500 mb-1">Scenario of gnss_faults.yaml (reset on its route, goal, start)</div>
          <div className="flex gap-1.5">
            <select data-testid="fault-scenario" className={`${input} flex-1 min-w-0`} value={sched?.name ?? ''} onChange={(e) => setSchedName(e.target.value)}>
              {scheduled.map((s) => (
                <option key={s.name} value={s.name}>
                  {s.name}
                </option>
              ))}
            </select>
            <button className={btn} disabled={!ready || !sched} onClick={runScheduled}>
              Run
            </button>
          </div>
          {sched && (
            <div className="mt-1 text-[11px] text-gray-500 font-mono break-words">
              route {sched.route ?? '-'} · {sched.faults.length ? sched.faults.map((f) => Object.entries(f).map(([k, v]) => `${k}=${String(v)}`).join(' ')).join('; ') : 'no faults (control)'}
              {sched.target ? ` · target ${JSON.stringify(sched.target)}` : ''}
            </div>
          )}
        </div>
      )}
    </Section>
  )
}

// ------------------------------------------------------------------ routes and launch options

function RoutesAndLaunch() {
  const client = useSimClient()
  const { ready, readyInfo } = useSimSnapshot()
  const routes = useMemo(() => routePresets(readyInfo), [readyInfo])
  const current = useMemo(() => launchOptions(readyInfo), [readyInfo])
  const [routeId, setRouteId] = useState('')
  const [autoStart, setAutoStart] = useState(false)
  const [form, setForm] = useState<Record<string, string>>({})
  const route = routes.find((r) => r.id === routeId) ?? routes[0]

  useEffect(() => {
    setForm({
      localization: String(current.localization ?? 'direct'),
      gnss_feed: String(current.gnss_feed ?? 'ideal'),
      gnss_checks: String(current.gnss_checks ?? 'true'),
      perception: String(current.perception ?? 'false'),
      filter_type: String(current.filter_type ?? ''),
      gnss_seed: String(current.gnss_seed ?? 1),
    })
  }, [current])

  const changed = Object.entries(form).some(([k, v]) => String(current[k] ?? defaultOf(k)) !== v)
  const apply = () => {
    const perception = form.perception === 'true'
    resetThenRoute(
      client,
      {
        localization: form.localization,
        gnss_feed: form.gnss_feed,
        gnss_checks: form.gnss_checks,
        perception,
        filter_type: perception || form.filter_type === 'none' ? form.filter_type : '',
        gnss_seed: Math.round(Number(form.gnss_seed) || 0),
      },
      null,
      false,
      'Launch options',
    )
  }

  return (
    <Section title="Routes and launch" fold="pg.launch" testid="launch-section" right={<span className="text-[11px] font-mono text-gray-500">{String(current.localization ?? '')}</span>}>
      {routes.length > 0 ? (
        <div className="flex flex-wrap items-center gap-1.5 mb-2">
          <select data-testid="route-select" className={`${input} flex-1 min-w-0`} value={route?.id ?? ''} onChange={(e) => setRouteId(e.target.value)}>
            {routes.map((r) => (
              <option key={r.id} value={r.id}>
                {r.id} {r.description ?? r.name ?? ''}
              </option>
            ))}
          </select>
          <button data-testid="route-load" className={btn} disabled={!ready || !route} onClick={() => route && resetThenRoute(client, { spawn: route.id, gnss_faults: '' }, route, autoStart, `Route ${route.id}`)}>
            Load
          </button>
          <label className="flex items-center gap-1 text-[11px] text-gray-500">
            <input type="checkbox" checked={autoStart} onChange={(e) => setAutoStart(e.target.checked)} /> start
          </label>
        </div>
      ) : (
        <p className="text-xs text-gray-500 mb-2">The worker sent no test routes.</p>
      )}
      <div className="grid grid-cols-2 gap-x-2 gap-y-1">
        {(Object.keys(LAUNCH_CHOICES) as (keyof typeof LAUNCH_CHOICES)[]).map((k) => (
          <label key={k} className="flex items-center justify-between gap-1 text-[11px]">
            <span className="text-gray-500">{k}</span>
            <select data-testid={`launch-${k}`} className={`${input} w-24`} value={form[k] ?? ''} onChange={(e) => setForm({ ...form, [k]: e.target.value })}>
              {LAUNCH_CHOICES[k].map((v) => (
                <option key={v} value={v}>
                  {v || '(default)'}
                </option>
              ))}
            </select>
          </label>
        ))}
        <label className="flex items-center justify-between gap-1 text-[11px]">
          <span className="text-gray-500">gnss_seed</span>
          <input className={`${input} w-24`} value={form.gnss_seed ?? ''} onChange={(e) => setForm({ ...form, gnss_seed: e.target.value })} />
        </label>
      </div>
      <div className="flex items-center gap-2 mt-1.5">
        <button data-testid="launch-apply" className={`${btn} ${changed ? btnActive : ''}`} disabled={!ready} onClick={apply}>
          Apply and reset
        </button>
        <span className="text-[11px] text-gray-500">{changed ? 'changed, not applied' : 'as running'}</span>
      </div>
    </Section>
  )
}

function defaultOf(k: string): string {
  return { localization: 'direct', gnss_feed: 'ideal', gnss_checks: 'true', perception: 'false', filter_type: '', gnss_seed: '1' }[k] ?? ''
}

// ------------------------------------------------------------------ parameters

function Params() {
  const client = useSimClient()
  const { ready, readyInfo } = useSimSnapshot()
  const applied = (launchOptions(readyInfo).params ?? {}) as Record<string, Record<string, unknown>>
  const [values, setValues] = useState<Record<string, string>>({})
  const [staged, setStaged] = useState<Record<string, number>>({})
  const h = historyOf(client)

  // A reset applies what was staged
  useEffect(() => setStaged({}), [readyInfo])

  const key = (p: { node: string; name: string }) => `${p.node}.${p.name}`
  const stage = async (p: (typeof PARAMS)[number]) => {
    const v = Number(values[key(p)])
    if (!Number.isFinite(v)) return
    const r = await h.command({ type: 'set_param', node: p.node, name: p.name, value: v }, `${p.name} = ${v}`)
    if (r.ok) setStaged((s) => ({ ...s, [key(p)]: v }))
  }
  const nStaged = Object.keys(staged).length

  return (
    <Section title="Parameters" fold="pg.params" testid="params-section" right={nStaged ? <Badge tone="warn">{`${nStaged} staged`}</Badge> : null}>
      <p className="text-[11px] text-gray-500 mb-1 leading-4">The nodes read parameters at start-up, so a change takes effect on the next reset (trajectory_controller, planning.yaml values as default).</p>
      <table className="w-full text-[11px]">
        <tbody>
          {PARAMS.map((p) => {
            const k = key(p)
            const now = applied[p.node]?.[p.name]
            return (
              <tr key={k} title={p.doc}>
                <td className="text-gray-600 dark:text-gray-300 pr-1 truncate max-w-[9rem]">{p.name}</td>
                <td className="font-mono text-gray-500 text-right pr-1">{String(now ?? p.def)}</td>
                <td className="w-16">
                  <input
                    data-testid={`param-${p.name}`}
                    type="number"
                    step={p.step}
                    min={p.min}
                    max={p.max}
                    placeholder={String(now ?? p.def)}
                    className={`${input} w-16`}
                    value={values[k] ?? ''}
                    onChange={(e) => setValues({ ...values, [k]: e.target.value })}
                    onKeyDown={(e) => e.key === 'Enter' && void stage(p)}
                  />
                </td>
                <td className="text-gray-500 px-1">{p.unit}</td>
                <td>
                  <button data-testid={`param-set-${p.name}`} className={`${btn} px-1.5 py-0`} disabled={!ready || values[k] === undefined || values[k] === ''} onClick={() => void stage(p)}>
                    {staged[k] !== undefined ? 'staged' : 'set'}
                  </button>
                </td>
              </tr>
            )
          })}
        </tbody>
      </table>
      <button className={`${btn} mt-1.5 ${nStaged ? btnActive : ''}`} disabled={!ready} onClick={() => void h.command({ type: 'reset' }, 'Reset')}>
        Reset to apply
      </button>
    </Section>
  )
}

// ------------------------------------------------------------------ scenario

/** Saves the running scenario: scenario_save, then the scenario message. */
export function saveScenario(client: SimClient): Promise<Scenario> {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      off()
      reject(new Error('no scenario from the worker in 5 s'))
    }, 5000)
    const off = onNext(client, 'scenario', (m) => {
      clearTimeout(timer)
      resolve(m.scenario)
    })
    client.request({ type: 'scenario_save' }).then((r) => {
      if (!r.ok) {
        clearTimeout(timer)
        off()
        reject(new Error(r.message))
      }
    })
  })
}

function ScenarioBox() {
  const client = useSimClient()
  const { ready } = useSimSnapshot()
  const [link, setLink] = useState('')
  const [paste, setPaste] = useState('')
  const [msg, setMsg] = useState<{ ok: boolean; text: string } | null>(null)
  const [saved, setSaved] = useState<Scenario | null>(null)

  const share = async () => {
    try {
      const s = await saveScenario(client)
      const code = await encodeScenario(s)
      const url = urlWithScenario(location.href, code)
      history.replaceState(null, '', url)
      setLink(url)
      setSaved(s)
      let copied = false
      try {
        await navigator.clipboard.writeText(url)
        copied = true
      } catch {
        // clipboard blocked: the link is in the box to copy by hand
      }
      setMsg({ ok: true, text: `${s.events.length} events, ${url.length} characters${copied ? ', link copied' : ''}` })
    } catch (e) {
      setMsg({ ok: false, text: String((e as Error).message ?? e) })
    }
  }

  const load = async () => {
    try {
      const s = await parseScenarioInput(paste)
      const r = await historyOf(client).command({ type: 'scenario_load', scenario: s }, 'Load scenario')
      setMsg({ ok: r.ok, text: r.ok ? `loaded ${s.events.length} events` : r.message })
    } catch (e) {
      setMsg({ ok: false, text: String((e as Error).message ?? e) })
    }
  }

  const download = () => {
    if (!saved) return
    const a = document.createElement('a')
    a.href = URL.createObjectURL(new Blob([JSON.stringify(saved, null, 1)], { type: 'application/json' }))
    a.download = `playground_scenario_t${Math.round((saved.events.at(-1)?.tick ?? 0) / 20)}s.json`
    a.click()
    URL.revokeObjectURL(a.href)
  }

  return (
    <Section title="Scenario" fold="pg.scenario" testid="scenario-section">
      <p className="text-[11px] text-gray-500 mb-1 leading-4">A scenario is the launch options plus every command with its tick. Loading one resets and replays it exactly.</p>
      <div className="flex gap-1.5">
        <button data-testid="scenario-share" className={btn} disabled={!ready} onClick={() => void share()}>
          Save and share link
        </button>
        <button className={btn} disabled={!saved} onClick={download}>
          JSON
        </button>
      </div>
      {link && <input data-testid="scenario-link" readOnly className={`${input} w-full mt-1`} value={link} onFocus={(e) => e.target.select()} />}
      <div className="flex gap-1.5 mt-1.5">
        <input data-testid="scenario-paste" className={`${input} flex-1 min-w-0`} placeholder="paste a link, code or JSON" value={paste} onChange={(e) => setPaste(e.target.value)} />
        <button data-testid="scenario-load" className={btn} disabled={!ready || !paste.trim()} onClick={() => void load()}>
          Load
        </button>
      </div>
      {msg && <div className={`mt-1 text-[11px] ${msg.ok ? 'text-emerald-700 dark:text-emerald-400' : 'text-red-600 dark:text-red-400'}`}>{msg.text}</div>}
    </Section>
  )
}

/** On the first ready, loads a scenario from the page URL (#scenario= or ?scenario=), once. */
function useScenarioFromUrl(client: SimClient) {
  const { ready } = useSimSnapshot()
  const done = useRef(false)
  useEffect(() => {
    if (!ready || done.current) return
    done.current = true
    const code = scenarioCodeFrom(location.href)
    if (!code) return
    decodeScenario(code)
      .then((s) => historyOf(client).command({ type: 'scenario_load', scenario: s } as PageToWorker, `Scenario from the link (${s.events.length} events)`))
      .catch((e) => historyOf(client).note('error', false, `The scenario in the link cannot be read: ${e.message}`))
  }, [ready, client])
}
