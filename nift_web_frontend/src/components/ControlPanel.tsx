/*
 * File: ControlPanel.tsx
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Right-sidebar panel for sending commands to the ROS2 stack. The panel
 *   has two distinct interfaces that swap based on the current drive mode,
 *   detected by watching the STATUS field in /nav_telemetry.
 *
 *   Mode toggle pill (header row):
 *     Shows "AUTO" (green) or "MANUAL" (orange). Clicking calls the
 *     /toggle_manual_mode Trigger service, which switches the trajectory
 *     controller between autonomous and manual modes.
 *
 *   Autonomous mode interface:
 *     1. Set Goal — activates goal-placement mode in MapView. When active the
 *        button pulses green and the map cursor turns into a crosshair.
 *        Clicking the map plants the goal and deactivates the mode.
 *        The actual publish happens in MapView → /goal_pose.
 *     2. Start Navigation — calls /start_navigation (Trigger service).
 *        Tells the trajectory controller to begin tracking the planned path.
 *     3. E-STOP — calls /trigger_estop (Trigger service). Immediately halts
 *        the vehicle. Button briefly shows "E-STOP SENT" as visual feedback.
 *
 *   Manual mode interface:
 *     D-pad buttons (↑ ↓ ← →) plus WASD / arrow-key keyboard bindings.
 *     Directions are held via pointer events (onPointerDown / onPointerLeave).
 *     A 100 ms interval publishes geometry_msgs/Twist to /teleop_cmd_vel
 *     while any direction is active (10 Hz, matching the RViz panel).
 *     Spacebar zeroes velocity immediately. Steering is negated when reversing
 *     so "left key = turn left" stays intuitive in both directions.
 *     E-STOP button remains visible at the bottom of this panel.
 *
 * Usage:
 *   <ControlPanel goalMode={goalMode} onGoalModeChange={setGoalMode} />
 */

import { useCallback, useEffect, useRef, useState } from 'react'
import { useRosPublisher } from '../hooks/useRosPublisher'
import { useRosService } from '../hooks/useRosService'
import { useRosTopic } from '../hooks/useRosTopic'
import type { StringMsg } from '../types/ros'

// Steering slider limit [deg]: the shuttle's full lock
const MAX_STEER_DEG = 15   // owner: nift_common/vehicle.yaml max_steering_deg (TS copy, tested)

interface Props {
  goalMode: boolean
  onGoalModeChange: (v: boolean) => void
}

export default function ControlPanel({ goalMode, onGoalModeChange }: Props) {
  const telemetry = useRosTopic<StringMsg>('/nav_telemetry', 'std_msgs/String')
  const isManual   = telemetry?.data.includes('STATUS:    MANUAL')    ?? false
  const isSafeStop = telemetry?.data.includes('STATUS:    SAFE_STOP') ?? false

  const callToggleManual = useRosService('/toggle_manual_mode', 'std_srvs/Trigger')
  const callStartNav     = useRosService('/start_navigation',   'std_srvs/Trigger')
  const callEstop        = useRosService('/trigger_estop',      'std_srvs/Trigger')
  const callResume       = useRosService('/resume',             'std_srvs/Trigger')

  const [navFired,    setNavFired]    = useState(false)
  const [estopFired,  setEstopFired]  = useState(false)
  const [resumeFired, setResumeFired] = useState(false)

  function handleStartNav() {
    callStartNav()
    setNavFired(true)
    setTimeout(() => setNavFired(false), 2000)
  }

  function handleEstop() {
    callEstop()
    setEstopFired(true)
    setTimeout(() => setEstopFired(false), 2000)
  }

  function handleResume() {
    callResume()
    setResumeFired(true)
    setTimeout(() => setResumeFired(false), 2000)
  }

  return (
    <div className="flex flex-col gap-2">
      {/* Header row with mode toggle — disabled during SAFE_STOP */}
      <div className="flex items-center justify-between">
        <h2 className="text-xs font-semibold uppercase tracking-widest text-gray-400">
          Control
        </h2>
        <button
          onClick={() => !isSafeStop && callToggleManual()}
          disabled={isSafeStop}
          className={`text-xs px-2 py-0.5 rounded font-medium transition-colors ${
            isSafeStop
              ? 'bg-gray-700 text-gray-500 cursor-not-allowed'
              : isManual
                ? 'bg-orange-700 hover:bg-orange-600 text-white'
                : 'bg-green-800 hover:bg-green-700 text-green-200'
          }`}
        >
          {isSafeStop ? 'SAFE STOP' : isManual ? 'MANUAL' : 'AUTO'}
        </button>
      </div>

      {isSafeStop ? (
        <SafeStopPanel
          onResume={handleResume}
          resumeFired={resumeFired}
          onEstop={handleEstop}
          estopFired={estopFired}
        />
      ) : isManual ? (
        <ManualPanel onEstop={handleEstop} estopFired={estopFired} />
      ) : (
        <AutoPanel
          goalMode={goalMode}
          onGoalModeChange={onGoalModeChange}
          onStartNav={handleStartNav}
          navFired={navFired}
          onEstop={handleEstop}
          estopFired={estopFired}
        />
      )}
    </div>
  )
}

/* ── Safe Stop mode ── */
function SafeStopPanel({
  onResume,
  resumeFired,
  onEstop,
  estopFired,
}: {
  onResume: () => void
  resumeFired: boolean
  onEstop: () => void
  estopFired: boolean
}) {
  return (
    <>
      <div className="rounded bg-amber-900/40 border border-amber-600 px-3 py-2 text-xs text-amber-300 leading-relaxed">
        Vehicle is held in <span className="font-bold">Safe Stop</span>. Confirm
        the path is clear before resuming.
      </div>

      <button
        onClick={onResume}
        className={`w-full py-2 rounded text-sm font-bold tracking-wider transition-colors ${
          resumeFired
            ? 'bg-amber-400 text-gray-900'
            : 'bg-amber-500 hover:bg-amber-400 active:bg-amber-600 text-gray-900'
        }`}
      >
        {resumeFired ? 'Resuming…' : 'Resume'}
      </button>

      <button
        onClick={onEstop}
        className={`w-full py-3 rounded font-bold text-sm tracking-wider transition-colors ${
          estopFired
            ? 'bg-red-400 text-white'
            : 'bg-red-600 hover:bg-red-500 active:bg-red-700 text-white'
        }`}
      >
        {estopFired ? 'E-STOP SENT' : 'E-STOP'}
      </button>
    </>
  )
}

/* ── Autonomous mode ── */
function AutoPanel({
  goalMode,
  onGoalModeChange,
  onStartNav,
  navFired,
  onEstop,
  estopFired,
}: {
  goalMode: boolean
  onGoalModeChange: (v: boolean) => void
  onStartNav: () => void
  navFired: boolean
  onEstop: () => void
  estopFired: boolean
}) {
  return (
    <>
      <button
        onClick={() => onGoalModeChange(!goalMode)}
        className={`w-full py-2 rounded text-sm font-medium transition-colors ${
          goalMode
            ? 'bg-green-600 text-white ring-2 ring-green-400 animate-pulse'
            : 'bg-gray-700 hover:bg-gray-600 text-gray-100'
        }`}
      >
        {goalMode ? 'Click map to place goal…' : 'Set Goal'}
      </button>

      <button
        onClick={onStartNav}
        className={`w-full py-2 rounded text-sm font-medium transition-colors ${
          navFired
            ? 'bg-blue-400 text-white'
            : 'bg-blue-700 hover:bg-blue-600 text-white'
        }`}
      >
        {navFired ? 'Navigation Started' : 'Start Navigation'}
      </button>

      <button
        onClick={onEstop}
        className={`w-full py-3 rounded font-bold text-sm tracking-wider transition-colors ${
          estopFired
            ? 'bg-red-400 text-white'
            : 'bg-red-600 hover:bg-red-500 active:bg-red-700 text-white'
        }`}
      >
        {estopFired ? 'E-STOP SENT' : 'E-STOP'}
      </button>
    </>
  )
}

/* ── Manual / teleop mode ── */
type DirFlags = { fwd: boolean; rev: boolean; left: boolean; right: boolean }

function ManualPanel({ onEstop, estopFired }: { onEstop: () => void; estopFired: boolean }) {
  const publishTwist = useRosPublisher('/teleop_cmd_vel', 'geometry_msgs/Twist')
  const keysRef = useRef<DirFlags>({ fwd: false, rev: false, left: false, right: false })

  const [linearSpeed, setLinearSpeed] = useState(1.0)
  const [angularDeg, setAngularDeg]   = useState(10.0)

  const buildTwist = useCallback((flags: DirFlags) => {
    const linear    = flags.fwd ? linearSpeed : flags.rev ? -linearSpeed : 0
    const angularRad = angularDeg * (Math.PI / 180.0)
    const angularRaw = flags.left ? angularRad : flags.right ? -angularRad : 0
    // Negate steering when reversing so left key = turn left in both directions
    const angular = linear < 0 ? -angularRaw : angularRaw
    return { linear: { x: linear, y: 0, z: 0 }, angular: { x: 0, y: 0, z: angular } }
  }, [linearSpeed, angularDeg])

  // 10 Hz publish loop
  useEffect(() => {
    const id = setInterval(() => {
      const flags = keysRef.current
      publishTwist(buildTwist(flags))
    }, 100)
    return () => clearInterval(id)
  }, [publishTwist, buildTwist])

  // Keyboard WASD / arrows
  useEffect(() => {
    function onKeyDown(e: KeyboardEvent) {
      if (e.repeat) return
      if (e.key === 'w' || e.key === 'ArrowUp')    keysRef.current.fwd   = true
      if (e.key === 's' || e.key === 'ArrowDown')  keysRef.current.rev   = true
      if (e.key === 'a' || e.key === 'ArrowLeft')  keysRef.current.left  = true
      if (e.key === 'd' || e.key === 'ArrowRight') keysRef.current.right = true
      if (e.key === ' ') {
        keysRef.current = { fwd: false, rev: false, left: false, right: false }
        publishTwist({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } })
      }
    }
    function onKeyUp(e: KeyboardEvent) {
      if (e.key === 'w' || e.key === 'ArrowUp')    keysRef.current.fwd   = false
      if (e.key === 's' || e.key === 'ArrowDown')  keysRef.current.rev   = false
      if (e.key === 'a' || e.key === 'ArrowLeft')  keysRef.current.left  = false
      if (e.key === 'd' || e.key === 'ArrowRight') keysRef.current.right = false
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      // Zero out on unmount
      publishTwist({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } })
    }
  }, [publishTwist])

  return (
    <div className="flex flex-col gap-4 bg-gray-800 p-3 rounded-md">
      
      {/* Speed Controls */}
      <div className="flex flex-col gap-3">
        <div className="flex flex-col gap-1">
          <label className="text-xs text-gray-300 flex justify-between">
            <span>Linear Speed</span>
            <span className="font-mono">{linearSpeed.toFixed(2)} m/s</span>
          </label>
          <input
            type="range" min="0" max="3" step="0.1"
            value={linearSpeed} onChange={(e) => setLinearSpeed(parseFloat(e.target.value))}
            className="accent-blue-500"
          />
        </div>

        <div className="flex flex-col gap-1">
          <label className="text-xs text-gray-300 flex justify-between">
            <span>Angle</span>
            <span className="font-mono">{angularDeg.toFixed(1)}&deg;</span>
          </label>
          <input
            type="range" min="0" max={MAX_STEER_DEG} step="0.5"
            value={angularDeg} onChange={(e) => setAngularDeg(parseFloat(e.target.value))}
            className="accent-blue-500"
          />
        </div>
      </div>

      <div className="text-xs text-gray-400 text-center bg-gray-900 py-2 rounded border border-gray-700">
        <span className="font-bold text-gray-200">W/A/S/D</span> or <span className="font-bold text-gray-200">Arrows</span> to drive<br/>
        <span className="font-bold text-gray-200">Space</span> to quick stop
      </div>

      <button
        onClick={onEstop}
        className={`w-full py-3 rounded font-bold text-sm tracking-wider transition-colors ${
          estopFired
            ? 'bg-red-400 text-white'
            : 'bg-red-600 hover:bg-red-500 active:bg-red-700 text-white'
        }`}
      >
        {estopFired ? 'E-STOP SENT' : 'E-STOP'}
      </button>
    </div>
  )
}