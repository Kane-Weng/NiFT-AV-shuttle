/*
 * File: PlaygroundControls.tsx
 *
 * Description:
 *   The playground's own controls, the things a real shuttle does not have (ARCH.md 7, 8.1, 8.4,
 *   8.5). The page's bottom bar is one row, like RViz's time panel: run or pause, step n ticks,
 *   the speed factor, the sim time and real-time factor, and reset. Obstacles are placed with the map's Obstacle tool, and GNSS
 *   faults come with the fault scenarios.
 *   The settings window (SettingsDialog.tsx) shows the set-up panes exported here:
 *     - launch options (localization mode, GNSS feed, perception, filter), on reset
 *     - parameters: set_param on a short whitelist, staged for the next reset
 *     - share: save the run as a link (#scenario=...), load one from a link or JSON
 *   Its Scenarios tab (ScenariosPane.tsx) has the test routes and the GNSS fault scenarios.
 */

import { useEffect, useMemo, useRef, useState } from 'react'
import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { SimClient } from '../sim/SimClient'
import type { PageToWorker, RoutePreset, Scenario, WorkerToPage } from '../protocol'
import { historyOf } from './history'
import { LAUNCH_CHOICES, PARAMS, launchOptions } from './presets'
import { Badge, Pane, btn, btnActive, input } from './Panel'
import { PauseIcon, PlayIcon, ResetIcon, StepIcon } from './icons'
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
export function resetThenRoute(client: SimClient, launch: Record<string, unknown>, route: RoutePreset | null, autoStart: boolean, label: string) {
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

  const seg = (on: boolean) =>
    `px-2 py-1 text-xs [&+&]:border-l border-gray-300 dark:border-gray-700 disabled:opacity-40 ${on ? 'bg-sky-600 text-white' : 'hover:bg-gray-100 dark:hover:bg-gray-800'}`
  const group = 'flex rounded-md border border-gray-300 dark:border-gray-700 overflow-hidden'

  return (
    <div
      data-testid="playground-panel"
      className="flex flex-wrap items-center gap-x-3 gap-y-1.5 px-3 py-1.5 bg-white dark:bg-gray-900 border-t border-gray-200 dark:border-gray-800 shrink-0"
    >
      <span className="text-[11px] font-semibold uppercase tracking-wider text-gray-500 dark:text-gray-400">Sim</span>
      <div className={group} data-testid="sim-bar">
        <button
          data-testid="sim-pause"
          className={`${seg(paused)} flex items-center gap-1 w-[4.5rem]`}
          disabled={!ready}
          onClick={() => client.send({ type: paused ? 'resume_sim' : 'pause' })}
          title={paused ? 'Run the sim clock' : 'Pause the sim clock'}
        >
          {paused ? <PlayIcon /> : <PauseIcon />}
          {paused ? 'Run' : 'Pause'}
        </button>
        <button data-testid="sim-step" className={`${seg(false)} flex items-center gap-1`} disabled={!ready || !paused} onClick={() => client.send({ type: 'step', n })} title="Run n ticks of 50 ms while paused">
          <StepIcon />
          Step
        </button>
        <input
          data-testid="sim-step-n"
          type="number"
          min={1}
          max={2000}
          title="ticks per step"
          className="w-12 border-l border-gray-300 dark:border-gray-700 bg-white dark:bg-gray-950 px-1 text-xs font-mono"
          value={n}
          onChange={(e) => setN(Math.max(1, Math.min(2000, Math.round(+e.target.value) || 1)))}
        />
      </div>
      <div className="flex items-center gap-1.5">
        <span className="text-[11px] text-gray-500">speed</span>
        <div className={group} data-testid="sim-speeds" role="radiogroup" aria-label="Sim speed">
          {SPEEDS.map((s) => (
            <button key={s} role="radio" aria-checked={factor === s} data-testid={`speed-${s}`} disabled={!ready} onClick={() => client.send({ type: 'speed', factor: s })} className={`${seg(factor === s)} w-10 px-0`}>
              {s === 'max' ? 'max' : `x${s}`}
            </button>
          ))}
        </div>
      </div>
      {state && (
        <div className="flex items-center gap-3 font-mono text-[11px] tabular-nums text-gray-500 dark:text-gray-400" data-testid="sim-clock">
          <span>
            sim time <span className="text-gray-800 dark:text-gray-200">{state.t.toFixed(1)} s</span>
          </span>
          <span className="hidden sm:inline">tick {state.tick}</span>
          <span title="real-time factor: sim seconds per wall second">rtf {state.rtf.toFixed(1)}</span>
        </div>
      )}
      <button
        data-testid="sim-reset"
        className="ml-auto flex items-center gap-1 rounded-md px-2.5 py-1 text-xs font-medium bg-red-600 hover:bg-red-500 text-white disabled:opacity-40"
        disabled={!ready}
        onClick={() => void h.command({ type: 'reset' }, 'Reset')}
        title="Rebuild the stack with the current launch options and staged parameters"
      >
        <ResetIcon />
        Reset
      </button>
    </div>
  )
}

// ------------------------------------------------------------------ launch options

export function LaunchOptions() {
  const client = useSimClient()
  const { ready, readyInfo } = useSimSnapshot()
  const current = useMemo(() => launchOptions(readyInfo), [readyInfo])
  const [form, setForm] = useState<Record<string, string>>({})

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
    <Pane title="Launch options" testid="launch-section" right={<span className="text-[11px] font-mono text-gray-500">{String(current.localization ?? '')}</span>}>
      <p className="text-[11px] text-gray-500 mb-2 leading-4">How the stack is launched. Applying rebuilds it with a reset.</p>
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
    </Pane>
  )
}

function defaultOf(k: string): string {
  return { localization: 'direct', gnss_feed: 'ideal', gnss_checks: 'true', perception: 'false', filter_type: '', gnss_seed: '1' }[k] ?? ''
}

// ------------------------------------------------------------------ parameters

export function Params() {
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
    <Pane title="Parameters" testid="params-section" right={nStaged ? <Badge tone="warn">{`${nStaged} staged`}</Badge> : null}>
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
    </Pane>
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

export function ScenarioBox() {
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
    <Pane title="Save and share this run" testid="scenario-section">
      <p className="text-[11px] text-gray-500 mb-1 leading-4">A saved run is the launch options plus every command with its tick. Loading one resets and replays it exactly.</p>
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
    </Pane>
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
