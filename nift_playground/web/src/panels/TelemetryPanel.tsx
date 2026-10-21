/*
 * File: TelemetryPanel.tsx
 *
 * Description:
 *   Telemetry from the state message (ARCH.md 8.2): speed, wheel angle, gear, pedal and brake from
 *   /carla/dbw_state, the controller's target speed and distance to the end from /nav_telemetry,
 *   and the error between the stack's pose and the truth. Below: the speed and cross-track plots
 *   (kept in JavaScript, history.ts). The diagnostics are in the stack window (StackDialog.tsx).
 */

import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { Gear } from '../protocol'
import { useHistory } from './history'
import Plot from './Plot'
import { Panel, Row } from './Panel'

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
          <Row k="to end">{fmt(s?.distEnd, 1, ' m')}</Row>
          <Row k="yaw">{s ? `${((s.pose.yaw * 180) / Math.PI).toFixed(0)}°` : '-'}</Row>
          <Row k="x, y">{s ? `${s.pose.x.toFixed(1)}, ${s.pose.y.toFixed(1)}` : '-'}</Row>
          <Row k="pose err">{fmt(poseErr, 2, ' m')}</Row>
        </div>
        <div className="mt-1">
          <GearRow gear={s?.gear ?? null} />
          <Bar label="pedal" value={s?.pedal ?? 0} tone="bg-emerald-500" />
          <Bar label="brake" value={s?.brake ?? 0} tone="bg-red-500" />
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
    </Panel>
  )
}

const GEARS: { gear: Gear; label: string }[] = [
  { gear: 'D', label: 'Forward' },
  { gear: 'R', label: 'Reverse' },
  { gear: 'N', label: 'Neutral' },
]

/** The gear as three lamps, the one engaged lit. */
function GearRow({ gear }: { gear: Gear | null }) {
  return (
    <div className="flex items-center gap-2 text-xs leading-5 mb-1" data-testid="tm-gear" data-gear={gear ?? ''}>
      <span className="text-gray-500 w-10">gear</span>
      <div className="flex-1 grid grid-cols-3 rounded border border-gray-200 dark:border-gray-800 overflow-hidden text-[11px]">
        {GEARS.map((g) => (
          <span
            key={g.gear}
            className={`text-center py-px [&+&]:border-l border-gray-200 dark:border-gray-800 transition-colors ${
              gear === g.gear ? (g.gear === 'R' ? 'bg-amber-500 text-gray-900 font-semibold' : 'bg-sky-600 text-white font-semibold') : 'text-gray-400 dark:text-gray-600'
            }`}
          >
            {g.label}
          </span>
        ))}
      </div>
      <span className="w-10" />
    </div>
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
