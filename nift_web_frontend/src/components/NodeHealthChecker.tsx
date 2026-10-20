/*
 * File: NodeHealthChecker.tsx
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Collapsible sidebar widget that shows the health of all ROS2 nodes
 *   that publish to /diagnostics (diagnostic_msgs/DiagnosticArray).
 *
 *   Summary bar (always visible):
 *     Color reflects the worst level across all tracked nodes:
 *       Green  — all nodes OK
 *       Yellow — at least one WARN
 *       Red    — at least one ERROR or STALE (pulses to draw attention)
 *     Clicking the bar toggles the detail list open/closed.
 *
 *   Detail list (expandable):
 *     One row per node. OK nodes are shown in muted gray (name only).
 *     WARN nodes are shown in yellow with the diagnostic message below.
 *     ERROR/STALE nodes are shown in red with the diagnostic message below.
 *
 *   State is kept in a ref-backed Map so that stale nodes from an earlier
 *   message are not discarded — the map is only updated, never reset.
 *   A throttle of 500 ms is applied on the rosbridge side to avoid
 *   flooding the UI if many nodes publish diagnostics simultaneously.
 *
 * Usage:
 *   <NodeHealthChecker />
 *   (No props — reads /diagnostics directly via useRosTopic)
 */

import { useState, useEffect, useRef } from 'react'
import { useRosTopic } from '../hooks/useRosTopic'
import type { DiagnosticArray, DiagnosticStatus } from '../types/ros'

const LEVEL_OK = 0
const LEVEL_WARN = 1
const LEVEL_ERROR = 2

interface NodeState {
  level: number
  message: string
}

export default function NodeHealthChecker() {
  const diagnostics = useRosTopic<DiagnosticArray>(
    '/diagnostics',
    'diagnostic_msgs/DiagnosticArray',
  )
  const [expanded, setExpanded] = useState(false)
  const nodesRef = useRef<Map<string, NodeState>>(new Map())
  const [nodes, setNodes] = useState<Map<string, NodeState>>(new Map())

  useEffect(() => {
    if (!diagnostics) return
    const updated = new Map(nodesRef.current)
    for (const s of diagnostics.status as DiagnosticStatus[]) {
      updated.set(s.name, { level: s.level, message: s.message })
    }
    nodesRef.current = updated
    setNodes(new Map(updated))
  }, [diagnostics])

  const worstLevel = Array.from(nodes.values()).reduce(
    (worst, n) => Math.max(worst, n.level),
    LEVEL_OK,
  )

  const summaryStyle =
    worstLevel >= LEVEL_ERROR
      ? { bar: 'bg-red-900/60 hover:bg-red-900/80 border-red-700', dot: 'bg-red-400', text: 'text-red-300', label: 'Node Error' }
      : worstLevel >= LEVEL_WARN
      ? { bar: 'bg-yellow-900/60 hover:bg-yellow-900/80 border-yellow-700', dot: 'bg-yellow-400', text: 'text-yellow-300', label: 'Node Warning' }
      : { bar: 'bg-green-900/40 hover:bg-green-900/60 border-green-800', dot: 'bg-green-400', text: 'text-green-300', label: nodes.size === 0 ? 'No Data' : 'Nodes OK' }

  return (
    <div className="rounded border border-gray-800 overflow-hidden">
      <button
        onClick={() => setExpanded(v => !v)}
        className={`w-full flex items-center justify-between px-2 py-1.5 border-b ${summaryStyle.bar} transition-colors`}
      >
        <div className="flex items-center gap-2">
          <span className={`w-2 h-2 rounded-full ${summaryStyle.dot} ${worstLevel === LEVEL_OK && nodes.size > 0 ? '' : worstLevel >= LEVEL_ERROR ? 'animate-pulse' : ''}`} />
          <span className={`text-xs font-medium ${summaryStyle.text}`}>
            {summaryStyle.label}
          </span>
        </div>
        <span className="text-gray-500 text-xs">{expanded ? '▲' : '▼'}</span>
      </button>

      {expanded && (
        <div className="bg-gray-900 max-h-48 overflow-y-auto">
          {nodes.size === 0 ? (
            <div className="text-xs text-gray-600 px-2 py-2">No diagnostics received yet.</div>
          ) : (
            Array.from(nodes.entries()).map(([name, state]) => (
              <NodeRow key={name} name={name} state={state} />
            ))
          )}
        </div>
      )}
    </div>
  )
}

function NodeRow({ name, state }: { name: string; state: NodeState }) {
  const dotColor =
    state.level >= LEVEL_ERROR
      ? 'bg-red-400'
      : state.level >= LEVEL_WARN
      ? 'bg-yellow-400'
      : 'bg-green-400'

  const textColor =
    state.level >= LEVEL_ERROR
      ? 'text-red-300'
      : state.level >= LEVEL_WARN
      ? 'text-yellow-300'
      : 'text-gray-400'

  return (
    <div className="flex flex-col px-2 py-1 border-b border-gray-800 last:border-0">
      <div className="flex items-center gap-1.5">
        <span className={`w-1.5 h-1.5 rounded-full shrink-0 ${dotColor}`} />
        <span className={`text-xs font-mono truncate ${textColor}`}>{name}</span>
      </div>
      {state.level > LEVEL_OK && state.message && (
        <div className="text-xs text-gray-500 pl-3 truncate">{state.message}</div>
      )}
    </div>
  )
}
