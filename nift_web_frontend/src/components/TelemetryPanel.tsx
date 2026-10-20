/*
 * File: TelemetryPanel.tsx
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Right-sidebar panel that displays live vehicle telemetry streamed from
 *   the ROS2 stack. Contains three sections:
 *
 *   1. Metric boxes — Speed (m/s) from /gnss/vel and Heading (degrees) from
 *      /gnss/heading_deg, displayed as compact readout cards.
 *
 *   2. Nav Telemetry table — The /nav_telemetry string published by the
 *      trajectory controller is parsed into key-value rows (split on '\n' then
 *      ':').  The STATUS row is color-coded: orange = MANUAL, red = BLOCKED/
 *      ESTOP, green = TRACKING/IDLE/REACHED, yellow = anything else.
 *
 *   3. Speed history chart — A scrolling line chart of the last 80 speed
 *      samples, rendered with the Recharts library. Useful for spotting
 *      sudden stops or speed oscillations during a run.
 *
 *   Speed history is stored in a useRef (not useState) because we don't want
 *   React to trigger a full re-render just to update the buffer — the chart
 *   re-draws naturally on each render caused by the incoming topic messages.
 *
 * Usage:
 *   <TelemetryPanel />
 *   (No props — reads directly from ROS topics via useRosTopic)
 */

import { useRef, useState } from 'react'
import {
  LineChart,
  Line,
  YAxis,
  Tooltip,
  ResponsiveContainer,
} from 'recharts'
import { useRosTopic } from '../hooks/useRosTopic'
import type { TwistStamped, StringMsg } from '../types/ros'

const HISTORY_LEN = 80

interface SpeedPoint {
  i: number
  v: number
}

export default function TelemetryPanel() {
  const vel = useRosTopic<TwistStamped>('/gnss/vel', 'geometry_msgs/TwistStamped')
  const status = useRosTopic<StringMsg>('/nav_telemetry', 'std_msgs/String')
  const [showHistory, setShowHistory] = useState(false)

  const historyRef = useRef<SpeedPoint[]>([])
  const counterRef = useRef(0)

  const speed = vel ? Math.abs(vel.twist.linear.x) : 0

  counterRef.current++
  historyRef.current = [
    ...historyRef.current.slice(-(HISTORY_LEN - 1)),
    { i: counterRef.current, v: speed },
  ]

  return (
    <div className="flex flex-col gap-2">
      <h2 className="text-xs font-semibold uppercase tracking-widest text-gray-400">
        Telemetry
      </h2>

      <TelemetryTable raw={status?.data ?? null} />

      <div className="bg-gray-800 rounded">
        <button
          onClick={() => setShowHistory(v => !v)}
          className="w-full flex items-center justify-between px-2 py-1.5 text-xs text-gray-500 hover:text-gray-300 transition-colors"
        >
          <span>Speed history</span>
          <span>{showHistory ? '▲' : '▼'}</span>
        </button>
        {showHistory && (
          <div className="px-2 pb-2">
            <ResponsiveContainer width="100%" height={72}>
              <LineChart data={historyRef.current}>
                <YAxis domain={[0, 6]} hide />
                <Tooltip
                  contentStyle={{ background: '#1f2937', border: 'none', fontSize: 11 }}
                  formatter={(v: number) => [`${v.toFixed(2)} m/s`]}
                  labelFormatter={() => ''}
                />
                <Line
                  type="monotone"
                  dataKey="v"
                  stroke="#3b82f6"
                  dot={false}
                  strokeWidth={1.5}
                  isAnimationActive={false}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        )}
      </div>
    </div>
  )
}

function statusColor(val: string): string {
  const v = val.toUpperCase()
  if (v.includes('MANUAL')) return 'text-orange-400'
  if (v.includes('BLOCK') || v.includes('ESTOP') || v.includes('STOP')) return 'text-red-400'
  if (v.includes('REACH') || v.includes('TRACK') || v.includes('IDLE')) return 'text-green-400'
  return 'text-yellow-400'
}

function TelemetryTable({ raw }: { raw: string | null }) {
  if (!raw) {
    return (
      <div className="bg-gray-800 rounded p-2">
        <div className="text-xs text-gray-500 mb-1">Nav Telemetry</div>
        <div className="text-xs text-gray-600 font-mono">—</div>
      </div>
    )
  }

  const rows = raw
    .split('\n')
    .map(line => {
      const idx = line.indexOf(':')
      if (idx === -1) return null
      return { key: line.slice(0, idx).trim(), val: line.slice(idx + 1).trim() }
    })
    .filter((r): r is { key: string; val: string } => r !== null && r.key.length > 0)

  return (
    <div className="bg-gray-800 rounded p-2">
      <div className="text-xs text-gray-500 mb-1">Nav Telemetry</div>
      <table className="w-full text-xs">
        <tbody>
          {rows.map(({ key, val }) => (
            <tr key={key}>
              <td className="text-gray-500 pr-2 py-0.5 whitespace-nowrap">{key}</td>
              <td className={`font-mono py-0.5 ${key === 'STATUS' ? statusColor(val) : 'text-white'}`}>
                {val}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}
