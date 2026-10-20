/*
 * File: TelemetryPanel.tsx
 *
 * Description:
 *   Telemetry from the state message (ARCH.md 8.2): speed, wheel angle, gear, pedal and brake from
 *   /carla/dbw_state, the controller's target speed, cross-track error and distance to the end
 *   from /nav_telemetry, the localization status and the error between the stack's pose and the
 *   truth. Below: the speed and cross-track plots (kept in JavaScript, history.ts) and the
 *   diagnostics, latest per status name, grouped by level.
 */

import { useState } from 'react'
import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { DiagItem } from '../protocol'
import { useHistory } from './history'
import Plot from './Plot'
import { locTone } from './OperatorPanel'
import { Badge, Caret, Panel, Row, Section, type Tone } from './Panel'

const fmt = (v: number | null | undefined, d = 2, unit = '') => (v === null || v === undefined || !Number.isFinite(v) ? '-' : `${v.toFixed(d)}${unit}`)

function Bar({ label, value, tone }: { label: string; value: number; tone: string }) {
  const pct = Math.max(0, Math.min(1, value)) * 100
  return (
    <div className="flex items-center gap-2 text-xs leading-5">
      <span className="text-gray-500 w-10">{label}</span>
      <div className="flex-1 h-2 rounded bg-gray-200 dark:bg-gray-800 overflow-hidden">
        <div className={`h-full ${tone}`} style={{ width: `${pct}%` }} />
      </div>
      <span className="font-mono tabular-nums w-10 text-right">{(value * 100).toFixed(0)}%</span>
    </div>
  )
}

export default function TelemetryPanel() {
  const client = useSimClient()
  const { state } = useSimSnapshot()
  const { samples } = useHistory(client)
  const s = state
  const poseErr = s && s.poseEst ? Math.hypot(s.pose.x - s.poseEst.x, s.pose.y - s.poseEst.y) : null

  return (
    <Panel title="Telemetry" fold="telemetry" testid="telemetry-panel" right={s ? <span className="font-mono">t {s.t.toFixed(1)} s</span> : null}>
      <div data-testid="telemetry">
        <div className="grid grid-cols-3 gap-1.5 mb-2">
          <Big label="speed" value={fmt(s?.speed, 2)} unit="m/s" testid="tm-speed" />
          <Big label="target" value={fmt(s?.targetSpeed, 2)} unit="m/s" />
          <Big label="steer" value={s ? ((s.steer * 180) / Math.PI).toFixed(1) : '-'} unit="deg" testid="tm-steer" />
        </div>
        <div className="grid grid-cols-2 gap-x-4">
          <Row k="gear" testid="tm-gear">{s?.gear ?? '-'}</Row>
          <Row k="CTE">{fmt(s?.cte, 2, ' m')}</Row>
          <Row k="to end">{fmt(s?.distEnd, 1, ' m')}</Row>
          <Row k="yaw">{s ? `${((s.pose.yaw * 180) / Math.PI).toFixed(0)}°` : '-'}</Row>
          <Row k="x, y">{s ? `${s.pose.x.toFixed(1)}, ${s.pose.y.toFixed(1)}` : '-'}</Row>
          <Row k="pose err">{fmt(poseErr, 2, ' m')}</Row>
        </div>
        <div className="mt-1">
          <Bar label="pedal" value={s?.pedal ?? 0} tone="bg-emerald-500" />
          <Bar label="brake" value={s?.brake ?? 0} tone="bg-red-500" />
        </div>
        <div className="mt-1.5 flex flex-wrap items-center gap-1.5 text-xs">
          <span className="text-gray-500">localization</span>
          <Badge tone={locTone(s)}>{s ? (s.locStatus ?? 'direct, no filter') : '-'}</Badge>
          <span className="text-gray-500 truncate flex-1 min-w-0" title={s?.locMessage ?? ''}>
            {s?.locMessage ?? ''}
          </span>
        </div>
        <div className="mt-1 flex flex-wrap items-center gap-1.5 text-xs">
          <span className="text-gray-500">GNSS fix age</span>
          <span className="font-mono">{fmt(s?.gnss.fixAge, 2, ' s')}</span>
          {s?.gnss.stale && <Badge tone="error">stale</Badge>}
          {s && !s.gnss.ok && <Badge tone="warn">health bad</Badge>}
        </div>
      </div>

      <Plot
        title="Speed"
        unit="m/s"
        testid="plot-speed"
        samples={samples}
        minSpan={4}
        series={[
          { label: 'speed', get: (x) => x.speed, stroke: 'stroke-sky-600 dark:stroke-sky-400' },
          { label: 'target', get: (x) => x.target, stroke: 'stroke-gray-400 dark:stroke-gray-500', dashed: true },
        ]}
      />
      <Plot title="Cross-track error" unit="m" testid="plot-cte" samples={samples} minSpan={0.5} symmetric series={[{ label: 'CTE', get: (x) => x.cte, stroke: 'stroke-violet-600 dark:stroke-violet-400' }]} />

      <Diagnostics />
    </Panel>
  )
}

function Big({ label, value, unit, testid }: { label: string; value: string; unit: string; testid?: string }) {
  return (
    <div className="rounded bg-gray-50 dark:bg-gray-950 border border-gray-200 dark:border-gray-800 px-2 py-1">
      <div className="text-[10px] uppercase tracking-wide text-gray-500">{label}</div>
      <div className="font-mono tabular-nums text-base leading-5" data-testid={testid}>
        {value}
        <span className="text-[10px] text-gray-500 ml-0.5">{unit}</span>
      </div>
    </div>
  )
}

const LEVEL: { name: string; tone: Tone }[] = [
  { name: 'OK', tone: 'ok' },
  { name: 'WARN', tone: 'warn' },
  { name: 'ERROR', tone: 'error' },
  { name: 'STALE', tone: 'muted' },
]

function Diagnostics() {
  const { diag } = useSimSnapshot()
  const [open, setOpen] = useState<Set<string>>(new Set())
  const counts = [0, 0, 0, 0]
  for (const d of diag) counts[d.level]++
  // Worst first, then by name, like rqt_runtime_monitor
  const items = [...diag].sort((a, b) => (b.level === 3 ? 1.5 : b.level) - (a.level === 3 ? 1.5 : a.level) || a.name.localeCompare(b.name))
  const toggle = (name: string) => {
    const n = new Set(open)
    if (n.has(name)) n.delete(name)
    else n.add(name)
    setOpen(n)
  }
  return (
    <Section
      title={`Diagnostics (${diag.length})`}
      fold="tm.diag"
      testid="diag-section"
      right={
        <span className="flex gap-1">
          {counts.map((c, i) => (c ? <Badge key={i} tone={LEVEL[i].tone}>{`${c} ${LEVEL[i].name}`}</Badge> : null))}
        </span>
      }
    >
      {items.length === 0 && <div className="text-xs text-gray-500">No /diagnostics yet.</div>}
      <ul className="max-h-72 overflow-y-auto space-y-px" data-testid="diag-list">
        {items.map((d) => (
          <DiagRow key={d.name} d={d} open={open.has(d.name)} onToggle={() => toggle(d.name)} />
        ))}
      </ul>
    </Section>
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
