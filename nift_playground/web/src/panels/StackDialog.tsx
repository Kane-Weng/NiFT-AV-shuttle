/*
 * File: StackDialog.tsx
 *
 * Description:
 *   The stack window, under the nodes icon in the header. It draws what nift_bringup starts for
 *   profile:=sim_kinematic as the bringup launcher does (nift_bringup/launcher.py): one stop per
 *   component, coloured lines for what each needs (REQUIRES), and an "or" junction where any one
 *   provider is enough. Each stop shows its node's health from /diagnostics.
 *
 *   The model (components, requires, which ones switch) comes from the worker (launch_set.stack()),
 *   the layout and colours are the launcher's. A click on a stop selects it: the card under the map
 *   says what the node does and what driving loses without it, with an On | Off switch. The switch
 *   follows the launcher's rules (launcher_model.py):
 *     - On to Off also switches off everything that needed it
 *     - Off to On also switches on what it needs (the first of an "any one of")
 *     - can is the sim itself and always on
 *   Changes are a draft until Apply, which resets the sim with the `components`, `localization`
 *   and `perception` launch options. Full stack puts every node back on.
 *
 *   Below the map: the selected node, and every diagnostic, worst first, like rqt_runtime_monitor.
 */

import { useEffect, useMemo, useRef, useState } from 'react'
import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { DiagItem, StackModel } from '../protocol'
import { historyOf } from './history'
import { launchOptions } from './presets'
import { Badge, Caret, btn, type Tone } from './Panel'

// ------------------------------------------------------------------ layout (launcher.py)

/** Grid units, x to the right, y up (row 0 = inputs), as launcher.py POSITIONS. */
const POS: Record<string, [number, number]> = {
  perception: [-2, 0],
  can: [0, 0],
  gnss: [2, 0],
  longitudinal: [0, 2],
  localization: [2, 2],
  filter: [3, 2],
  planning: [3, 3],
  lateral: [2, 4],
}
const SUBTITLE: Record<string, string> = {
  can: 'kinematic_sim',
  gnss: 'beacon_node',
  localization: 'shuttle_tf_node',
  filter: 'localization_node',
  perception: 'playground_perception',
  planning: 'waypoint_node',
  longitudinal: 'ADRC speed',
  lateral: 'Pure Pursuit',
}
const LABEL_SIDE: Record<string, 'left' | 'right' | 'above' | 'below'> = {
  longitudinal: 'left',
  localization: 'left',
  filter: 'right',
  planning: 'right',
  lateral: 'above',
}
const COLOR: Record<string, string> = {
  can: '#e5534b',
  longitudinal: '#e5534b',
  gnss: '#3d8fd6',
  localization: '#3d8fd6',
  planning: '#3d8fd6',
  filter: '#7cc4fa',
  lateral: '#9c6ade',
  perception: '#45b36b',
}
/** Where an "any one of" meets before going on (launcher.py JUNCTIONS), by "node|alternatives". */
const JUNCTION: Record<string, [number, number]> = {
  'planning|localization,filter': [2, 3],
  'lateral|localization,filter': [2, 3],
  'lateral|planning,static_path': [3, 4],
}
const BENDS: Record<string, [number, number][]> = { 'gnss>filter': [[3, 1]] }
/** The launch names whose diagnostics belong to each stop. */
const NODES: Record<string, string[]> = {
  can: ['kinematic_sim'],
  gnss: ['beacon_node'],
  localization: ['shuttle_dynamic_broadcaster', 'shuttle_tf_node'],
  filter: ['localization_node'],
  perception: ['playground_perception'],
  planning: ['waypoint_node'],
  longitudinal: ['trajectory_controller'],
  lateral: [],
}

const U = 96 // px per grid unit
const X0 = -3.1
const Y0 = 4.75
const W = 7.8 * U // room right of filter and planning for their labels
const H = 6.05 * U // room under the inputs' labels for the legend
const px = ([x, y]: [number, number]): [number, number] => [(x - X0) * U, (Y0 - y) * U]

const LEVEL: { name: string; tone: Tone; color: string }[] = [
  { name: 'OK', tone: 'ok', color: '#3fb950' },
  { name: 'WARN', tone: 'warn', color: '#f0a030' },
  { name: 'ERROR', tone: 'error', color: '#f85149' },
  { name: 'STALE', tone: 'muted', color: '#6e7681' },
]
const rank = (l: number) => (l === 3 ? 1.5 : l) // STALE between WARN and ERROR, like rqt_runtime_monitor

// ------------------------------------------------------------------ rules (launcher_model.py)

const LOCAL_MODE: Record<string, [boolean, boolean]> = { direct: [true, false], ekf: [false, true], shadow: [true, true], off: [false, false] }

function unmet(model: StackModel, on: Set<string>, c: string): string[][] {
  return (model.requires[c] ?? []).filter((alts) => !alts.some((a) => on.has(a)))
}

/** Switches `c` off, and then everything left without one of its needs. */
function switchOff(model: StackModel, on: Set<string>, c: string): Set<string> {
  const next = new Set(on)
  next.delete(c)
  for (let changed = true; changed; ) {
    changed = false
    for (const d of [...next]) {
      if (unmet(model, next, d).length) {
        next.delete(d)
        changed = true
      }
    }
  }
  return next
}

/** Switches `c` on with what it needs, the first alternative of each "any one of". */
function switchOn(model: StackModel, on: Set<string>, c: string): Set<string> {
  const next = new Set(on)
  const add = (x: string) => {
    if (next.has(x) || !(x in POS)) return
    next.add(x)
    for (const alts of unmet(model, next, x)) add(alts.find((a) => a in POS) ?? alts[0])
  }
  add(c)
  return next
}

/** The components on now: the launch's plan.on, plus the playground's perception option. */
function runningOf(readyInfo: ReturnType<typeof useSimSnapshot>['readyInfo']): Set<string> {
  const launch = (readyInfo?.launch ?? {}) as { components?: string[] }
  const on = new Set((launch.components ?? []).filter((c) => c in POS))
  on.add('can')
  if (launchOptions(readyInfo).perception === true) on.add('perception')
  return on
}

const FULL = ['can', 'gnss', 'localization', 'planning', 'longitudinal', 'lateral']

/** What driving loses without each node, for the card and the operator panel's hint. */
const WITHOUT: Record<string, string> = {
  gnss: 'no position fixes, so nothing downstream can work',
  localization: 'no map -> base_link from GNSS (unless the filter gives it)',
  filter: 'no filtered pose (fine while localization gives map -> base_link)',
  planning: 'a goal gets no path',
  longitudinal: 'no speed control, so the shuttle cannot move at all, not even in manual',
  lateral: 'no steering along the path, so Start navigation does not drive (manual still works)',
  perception: 'obstacles are not seen, so the shuttle drives into them',
}

/**
 * Why the shuttle may not move with this stack, or null when nothing that driving needs is off.
 * The operator panel shows it, so a stack switched off in the stack window is not a mystery.
 */
export function stackHint(readyInfo: ReturnType<typeof useSimSnapshot>['readyInfo']): { off: string[]; text: string; manualOk: boolean } | null {
  if (!readyInfo?.presets?.stack) return null
  const on = runningOf(readyInfo)
  const off = FULL.filter((c) => !on.has(c))
  if (!off.length) return null
  const noTf = !on.has('localization') && !on.has('filter')
  const first = ['gnss', 'longitudinal', 'planning', 'lateral'].find((c) => !on.has(c))
  const text = noTf && first !== 'gnss' && first !== 'longitudinal' ? 'no map -> base_link, so there is no path and Start navigation does not drive (manual still works)' : WITHOUT[first ?? 'localization']
  return { off: off.filter((c) => !(c === 'localization' && on.has('filter'))), text, manualOk: on.has('longitudinal') && on.has('gnss') }
}

// ------------------------------------------------------------------ the window

interface Props {
  open: boolean
  onClose: () => void
}

export default function StackDialog({ open, onClose }: Props) {
  const ref = useRef<HTMLDivElement>(null)
  const client = useSimClient()
  const { ready, readyInfo, diag } = useSimSnapshot()
  const model = readyInfo?.presets?.stack ?? null
  const running = useMemo(() => runningOf(readyInfo), [readyInfo])
  const [draft, setDraft] = useState<Set<string>>(running)
  const [selected, setSelected] = useState<string | null>(null)
  const [msg, setMsg] = useState<{ ok: boolean; text: string } | null>(null)

  // A reset (ours or any other) shows what now runs
  useEffect(() => setDraft(running), [running])

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

  const health = useMemo(() => {
    const out: Record<string, DiagItem[]> = {}
    for (const [c, names] of Object.entries(NODES)) out[c] = diag.filter((d) => names.some((n) => d.name.startsWith(`${n}:`) || d.name === n))
    return out
  }, [diag])

  const changed = [...new Set([...draft, ...running])].some((c) => draft.has(c) !== running.has(c))

  const select = (c: string) => setSelected(c)
  const setOn = (c: string, v: boolean) => {
    if (!model || c === 'can' || draft.has(c) === v) return
    setDraft(v ? switchOn(model, draft, c) : switchOff(model, draft, c))
  }

  const apply = async (on: Set<string>) => {
    if (!model) return
    const mode = Object.entries(LOCAL_MODE).find(([, v]) => v[0] === on.has('localization') && v[1] === on.has('filter'))![0]
    const components = Object.fromEntries(model.switchable.map((c) => [c, on.has(c)]))
    const perception = on.has('perception')
    const filterType = String(launchOptions(readyInfo).filter_type ?? '')
    const launch = { localization: mode, components, perception, filter_type: perception || filterType === 'none' ? filterType : '' }
    const off = FULL.filter((c) => !on.has(c))
    const r = await historyOf(client).command({ type: 'reset', launch }, off.length ? `Stack without ${off.join(', ')}` : 'Full stack')
    setMsg({ ok: r.ok, text: r.ok ? 'Applied: the sim was reset with this stack.' : r.message })
  }

  return (
    <div
      ref={ref}
      role="dialog"
      aria-label="Stack"
      data-testid="stack-dialog"
      className={`${open ? 'flex' : 'hidden'} fixed z-50 top-12 right-3 w-[44rem] max-w-[calc(100vw-1.5rem)] max-h-[calc(100vh-4rem)] flex-col rounded-lg border border-gray-200 dark:border-gray-700 bg-white dark:bg-gray-900 shadow-xl`}
    >
      <div className="flex items-center gap-2 px-3 py-2 border-b border-gray-200 dark:border-gray-800">
        <div>
          <div className="text-sm font-semibold">Stack</div>
          <div className="text-[11px] text-gray-500">What nift_bringup starts for profile sim_kinematic. Pick a node, switch it on or off, then Apply.</div>
        </div>
        <button data-testid="stack-close" onClick={onClose} aria-label="Close" className="ml-auto self-start px-2 py-0.5 rounded text-gray-400 hover:text-gray-800 dark:hover:text-gray-100 hover:bg-gray-100 dark:hover:bg-gray-800">
          ✕
        </button>
      </div>

      <div className="overflow-y-auto p-3">
        {!model ? (
          <p className="text-xs text-gray-500">The stack model comes with the sim. Waiting for it to start.</p>
        ) : (
          <>
            <StackMap model={model} on={draft} running={running} health={health} selected={selected} onClick={select} />
            <div className="flex flex-wrap items-center gap-2 mt-2">
              <button data-testid="stack-apply" className={`${btn} ${changed ? 'bg-sky-600 border-sky-500 text-white hover:bg-sky-500' : ''}`} disabled={!ready || !changed} onClick={() => void apply(draft)}>
                Apply and reset
              </button>
              <button data-testid="stack-revert" className={btn} disabled={!changed} onClick={() => setDraft(running)}>
                Undo changes
              </button>
              <button data-testid="stack-full" className={btn} disabled={!ready || FULL.every((c) => running.has(c))} onClick={() => void apply(new Set([...FULL, ...(running.has('perception') ? ['perception'] : [])]))} title="Every node of sim_kinematic on (perception stays as it is in Launch)">
                Full stack
              </button>
              <span className="text-[11px] text-gray-500">{changed ? 'Changed, not applied' : msg ? '' : 'As running'}</span>
              {msg && <span className={`text-[11px] ${msg.ok ? 'text-emerald-700 dark:text-emerald-400' : 'text-red-600 dark:text-red-400'}`}>{msg.text}</span>}
            </div>
            {selected ? (
              <NodeInfo model={model} c={selected} on={draft} running={running} health={health[selected] ?? []} onSet={(v) => setOn(selected, v)} />
            ) : (
              <p className="mt-2 text-[11px] text-gray-500">Click a node on the map to see what it does and to switch it on or off.</p>
            )}
          </>
        )}
        <Diagnostics />
      </div>
    </div>
  )
}

// ------------------------------------------------------------------ the map

function StackMap(props: {
  model: StackModel
  on: Set<string>
  running: Set<string>
  health: Record<string, DiagItem[]>
  selected: string | null
  onClick: (c: string) => void
}) {
  const { model, on, running, health, selected } = props
  const lines: { key: string; pts: [number, number][]; color: string; lit: boolean }[] = []
  const seen = new Set<string>()
  for (const [dst, needs] of Object.entries(model.requires)) {
    if (!(dst in POS)) continue
    for (const alts of needs) {
      const j = JUNCTION[`${dst}|${alts.join(',')}`]
      for (const src of alts) {
        if (!(src in POS)) continue
        const end = j ?? POS[dst]
        const key = `${src}>${j ? j.join(',') : dst}`
        if (seen.has(key)) continue
        seen.add(key)
        lines.push({ key, pts: [POS[src], ...(BENDS[`${src}>${dst}`] ?? []), end], color: COLOR[src], lit: on.has(src) })
      }
      if (j) {
        const key = `${j.join(',')}>${dst}`
        if (!seen.has(key)) {
          seen.add(key)
          lines.push({ key, pts: [j, POS[dst]], color: COLOR[dst], lit: alts.some((a) => on.has(a)) && on.has(dst) })
        }
      }
    }
  }
  const junctions = [...new Set(Object.values(JUNCTION).map((j) => j.join(',')))].map((k) => k.split(',').map(Number) as [number, number])

  return (
    <svg viewBox={`0 0 ${W} ${H}`} className="block w-full rounded-md bg-[#16181b]" data-testid="stack-map">
      {lines.map((l) => (
        <polyline
          key={l.key}
          points={l.pts.map((p) => px(p).join(',')).join(' ')}
          fill="none"
          stroke={l.lit ? l.color : '#3a3e44'}
          strokeWidth={7}
          strokeLinecap="round"
          strokeLinejoin="round"
          opacity={l.lit ? 0.9 : 1}
        />
      ))}
      {/* What perception gives the controller: not a requirement, dashed */}
      <polyline points={[POS.perception, POS.longitudinal].map((p) => px(p).join(',')).join(' ')} fill="none" stroke={on.has('perception') ? COLOR.perception : '#3a3e44'} strokeWidth={3} strokeDasharray="6 6" />
      <text x={(px(POS.perception)[0] + px(POS.longitudinal)[0]) / 2 - 8} y={(px(POS.perception)[1] + px(POS.longitudinal)[1]) / 2 - 8} fill="#8a8f96" fontSize="12" textAnchor="end">
        obstacles
      </text>
      {junctions
        .filter((j) => lines.some((l) => l.pts.some((p) => p[0] === j[0] && p[1] === j[1])))
        .map((j) => (
          <g key={j.join(',')}>
            <circle cx={px(j)[0]} cy={px(j)[1]} r={6} fill="#16181b" stroke="#8a8f96" strokeWidth={2} />
            <text x={px(j)[0] - 10} y={px(j)[1] + 4} fill="#8a8f96" fontSize="11" textAnchor="end">
              or
            </text>
          </g>
        ))}
      {Object.keys(POS).map((c) => {
        const [x, y] = px(POS[c])
        const isOn = on.has(c)
        const pending = isOn !== running.has(c)
        const blocked = !isOn && unmet(model, on, c).length > 0
        const worst = (health[c] ?? []).reduce((w, d) => (w === -1 || rank(d.level) > rank(w) ? d.level : w), -1)
        const side = LABEL_SIDE[c] ?? 'below'
        const [tx, ty, anchor] =
          side === 'left' ? [x - 26, y + 4, 'end'] : side === 'right' ? [x + 26, y + 4, 'start'] : side === 'above' ? [x, y - 34, 'middle'] : [x, y + 36, 'middle']
        return (
          <g key={c} data-testid={`stack-node-${c}`} data-on={isOn ? '1' : '0'} onClick={() => props.onClick(c)} className={c === 'can' ? 'cursor-default' : 'cursor-pointer'}>
            {selected === c && <circle cx={x} cy={y} r={27} fill="none" stroke="#e8e8e8" strokeWidth={1.5} strokeDasharray="3 3" />}
            <circle cx={x} cy={y} r={18} fill={isOn ? COLOR[c] : '#2a2d31'} stroke={isOn ? '#16181b' : blocked ? '#4a4f55' : COLOR[c]} strokeWidth={isOn ? 4 : 2.5} strokeDasharray={!isOn && !blocked ? '5 4' : undefined} />
            {c === 'can' && <path d={`M${x - 5} ${y - 1}v-3a5 5 0 0 1 10 0v3M${x - 7} ${y - 1}h14v9h-14z`} fill="none" stroke="#16181b" strokeWidth={2} />}
            {pending && <circle cx={x - 15} cy={y - 15} r={5} fill="#f0a030" stroke="#16181b" strokeWidth={2} />}
            {isOn && running.has(c) && worst >= 0 && <circle cx={x + 15} cy={y - 15} r={5.5} fill={LEVEL[worst].color} stroke="#16181b" strokeWidth={2} />}
            <text x={tx} y={ty} fill={isOn ? '#e8e8e8' : '#8a8f96'} fontSize="14" fontWeight={600} textAnchor={anchor}>
              {c}
            </text>
            <text x={tx} y={ty + 15} fill="#8a8f96" fontSize="11" textAnchor={anchor}>
              {SUBTITLE[c]}
            </text>
          </g>
        )
      })}
      <g transform={`translate(12 ${H - 14})`} fontSize="11" fill="#8a8f96">
        <circle cx={5} cy={-4} r={5} fill="#3d8fd6" />
        <text x={15} y={0}>on</text>
        <circle cx={45} cy={-4} r={5} fill="#2a2d31" stroke="#3d8fd6" strokeWidth={1.5} strokeDasharray="3 2" />
        <text x={55} y={0}>off, can start</text>
        <circle cx={160} cy={-4} r={5} fill="#2a2d31" stroke="#4a4f55" strokeWidth={1.5} />
        <text x={170} y={0}>needs another node</text>
        <circle cx={292} cy={-4} r={4} fill="#f0a030" />
        <text x={301} y={0}>not applied</text>
        <circle cx={380} cy={-4} r={4} fill="#3fb950" />
        <text x={389} y={0}>health (/diagnostics)</text>
      </g>
    </svg>
  )
}

function NodeInfo(props: { model: StackModel; c: string; on: Set<string>; running: Set<string>; health: DiagItem[]; onSet: (v: boolean) => void }) {
  const { model, c, on, running, health } = props
  const needs = model.requires[c] ?? []
  const isOn = on.has(c)
  // What the switch takes with it, so a click on Off or On is not a surprise
  const also = isOn ? [...on].filter((x) => x !== c && !switchOff(model, on, c).has(x)) : [...switchOn(model, on, c)].filter((x) => x !== c && !on.has(x))
  const seg = (active: boolean) => `px-3 py-0.5 text-xs [&+&]:border-l border-gray-300 dark:border-gray-700 disabled:opacity-40 ${active ? 'bg-sky-600 text-white' : 'hover:bg-gray-100 dark:hover:bg-gray-800'}`
  return (
    <div className="mt-2 rounded-md border border-gray-200 dark:border-gray-700 p-2 text-xs" data-testid="stack-node-info">
      <div className="flex items-center gap-2">
        <span className="inline-block w-2.5 h-2.5 rounded-full" style={{ background: COLOR[c] }} />
        <span className="font-semibold">{c}</span>
        <span className="text-gray-500 truncate">{model.components[c]}</span>
        {isOn !== running.has(c) && <Badge tone="warn">not applied</Badge>}
        <div className="ml-auto flex rounded-md border border-gray-300 dark:border-gray-700 overflow-hidden shrink-0" role="radiogroup" aria-label={`${c} on or off`}>
          <button data-testid="stack-node-on" role="radio" aria-checked={isOn} className={seg(isOn)} disabled={c === 'can'} onClick={() => props.onSet(true)}>
            On
          </button>
          <button data-testid="stack-node-off" role="radio" aria-checked={!isOn} className={seg(!isOn)} disabled={c === 'can'} onClick={() => props.onSet(false)}>
            Off
          </button>
        </div>
      </div>
      <div className="mt-1 text-[11px] text-gray-500 leading-4">
        {c === 'can' ? (
          'The sim is the vehicle here, so it is always on.'
        ) : (
          <>
            {needs.length ? `Needs ${needs.map((alts) => alts.filter((a) => a in POS).join(' or ')).join(', ')}. ` : 'Needs nothing else. '}
            {WITHOUT[c] && `Without it: ${WITHOUT[c]}.`}
            {also.length > 0 && (
              <span className="block text-amber-600 dark:text-amber-400">
                {isOn ? 'Off also switches off ' : 'On also switches on '}
                {also.join(', ')}.
              </span>
            )}
          </>
        )}
      </div>
      {health.map((d) => (
        <div key={d.name} className="flex items-center gap-1.5 mt-1">
          <Badge tone={LEVEL[d.level]?.tone ?? 'muted'}>{LEVEL[d.level]?.name ?? d.level}</Badge>
          <span className="truncate">{d.name}</span>
          <span className="ml-auto truncate text-gray-500 max-w-[50%]" title={d.message}>
            {d.message}
          </span>
        </div>
      ))}
    </div>
  )
}

// ------------------------------------------------------------------ diagnostics

function Diagnostics() {
  const { diag } = useSimSnapshot()
  const [open, setOpen] = useState<Set<string>>(new Set())
  const counts = [0, 0, 0, 0]
  for (const d of diag) counts[d.level]++
  // Worst first, then by name, like rqt_runtime_monitor
  const items = [...diag].sort((a, b) => rank(b.level) - rank(a.level) || a.name.localeCompare(b.name))
  const toggle = (name: string) => {
    const n = new Set(open)
    if (n.has(name)) n.delete(name)
    else n.add(name)
    setOpen(n)
  }
  return (
    <div className="mt-3" data-testid="diag-section">
      <div className="flex items-center justify-between mb-1">
        <span className="text-xs font-semibold text-gray-700 dark:text-gray-200">Diagnostics ({diag.length})</span>
        <span className="flex gap-1">{counts.map((c, i) => (c ? <Badge key={i} tone={LEVEL[i].tone}>{`${c} ${LEVEL[i].name}`}</Badge> : null))}</span>
      </div>
      {items.length === 0 && <div className="text-xs text-gray-500">No /diagnostics yet.</div>}
      <ul className="max-h-72 overflow-y-auto space-y-px" data-testid="diag-list">
        {items.map((d) => (
          <DiagRow key={d.name} d={d} open={open.has(d.name)} onToggle={() => toggle(d.name)} />
        ))}
      </ul>
    </div>
  )
}

function DiagRow({ d, open, onToggle }: { d: DiagItem; open: boolean; onToggle: () => void }) {
  const values = Object.entries(d.values ?? {})
  return (
    <li className="text-xs">
      <button onClick={onToggle} className="w-full flex items-center gap-1.5 text-left rounded px-1 py-0.5 hover:bg-gray-100 dark:hover:bg-gray-800">
        <Caret open={open} />
        <Badge tone={LEVEL[d.level]?.tone ?? 'muted'}>{LEVEL[d.level]?.name ?? d.level}</Badge>
        <span className="truncate font-medium">{d.name}</span>
        <span className="ml-auto truncate text-gray-500 max-w-[45%]" title={d.message}>
          {d.message}
        </span>
      </button>
      {open && (
        <div className="ml-5 mb-1 border-l border-gray-200 dark:border-gray-800 pl-2">
          {d.hardwareId && <div className="text-[10px] text-gray-500">hardware id {d.hardwareId}</div>}
          <div className="text-[11px] text-gray-600 dark:text-gray-300">{d.message}</div>
          {values.map(([k, v]) => (
            <div key={k} className="flex justify-between gap-2 text-[11px] leading-4">
              <span className="text-gray-500">{k}</span>
              <span className="font-mono text-right break-all">{v}</span>
            </div>
          ))}
        </div>
      )}
    </li>
  )
}
