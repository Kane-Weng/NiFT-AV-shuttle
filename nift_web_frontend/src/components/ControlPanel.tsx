/*
 * File: ControlPanel.tsx
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Right-sidebar panel for sending commands to the ROS2 stack. The panel
 *   has distinct interfaces that swap based on the current drive mode,
 *   detected by reading the STATUS line of /nav_telemetry (utils/control.ts).
 *
 *   Mode toggle pill (header row):
 *     Shows "AUTO" (green) or "MANUAL" (orange). Clicking calls the
 *     /toggle_manual_mode Trigger service, which switches the trajectory
 *     controller between autonomous and manual modes.
 *
 *   Every button that calls a Trigger service shows the controller's reply
 *   (useTrigger). A refusal shows the controller's reason under the button,
 *   e.g. "Cannot start: no valid path loaded.", and so does a failed call.
 *
 *   Autonomous mode interface:
 *     1. Set Goal — activates goal-placement mode in MapView. When active the
 *        button pulses green and the map cursor turns into a crosshair.
 *        Clicking the map plants the goal and deactivates the mode.
 *        The actual publish happens in MapView → /goal_pose.
 *     2. Start Navigation — calls /start_navigation (Trigger service).
 *        Tells the trajectory controller to begin tracking the planned path.
 *        Shows "Navigation Started" only when the controller accepts.
 *     3. Stop — calls /safe_stop (Trigger service). The soft stop: the
 *        controller brakes into SAFE_STOP, and Resume continues from there.
 *        Enabled while NAVIGATING, shown disabled in IDLE (utils/control.ts).
 *     4. E-STOP — calls /trigger_estop (Trigger service). The hardware E-stop,
 *        which latches until trajectory_controller restarts and the shuttle is
 *        power-cycled. The button says so, so nobody uses it as a pause.
 *        It shows "E-STOP SENT", or "E-STOP FAILED" when the call fails (no
 *        connection, no service, no reply).
 *
 *   Manual mode interface:
 *     WASD / arrow-key keyboard bindings.
 *     A 100 ms interval publishes geometry_msgs/Twist to /teleop_cmd_vel
 *     while any direction is active (10 Hz, matching the RViz panel). When
 *     the last key is released it publishes one zero Twist and then nothing,
 *     like the RViz panel when it loses focus. The controller keeps the last
 *     command, so that zero is the stop. Losing window focus counts as
 *     releasing every key, since the page no longer sees key releases.
 *     Spacebar zeroes velocity immediately. Steering is negated when reversing
 *     so "left key = turn left" stays intuitive in both directions.
 *     Stop and E-STOP buttons remain visible at the bottom of this panel.
 *
 *   Safe stop and E-stop interfaces:
 *     SAFE_STOP shows Resume (/resume) and E-STOP. ESTOP is the controller's
 *     latched stop. The controller refuses to start or switch mode until it
 *     restarts, so that view shows only a notice and E-STOP. Stop is hidden
 *     in both.
 *
 * Usage:
 *   <ControlPanel goalMode={goalMode} onGoalModeChange={setGoalMode} />
 */

import { useCallback, useEffect, useRef, useState } from 'react'
import type { ReactNode } from 'react'
import { useRosPublisher } from '../hooks/useRosPublisher'
import { useRosTopic } from '../hooks/useRosTopic'
import { useTrigger } from '../hooks/useTrigger'
import type { TriggerState } from '../hooks/useTrigger'
import { controlView, ESTOP_LATCH_NOTE, navStatus, NO_KEYS, stopButton, teleopStep, ZERO_TWIST } from '../utils/control'
import type { DirFlags, StopButton as StopButtonMode, Twist } from '../utils/control'
import type { StringMsg } from '../types/ros'

// Steering slider limit [deg]: the shuttle's full lock
const MAX_STEER_DEG = 15   // owner: nift_common/vehicle.yaml max_steering_deg (TS copy, tested)

interface Props {
  goalMode: boolean
  onGoalModeChange: (v: boolean) => void
}

export default function ControlPanel({ goalMode, onGoalModeChange }: Props) {
  const telemetry = useRosTopic<StringMsg>('/nav_telemetry', 'std_msgs/String')
  const status = navStatus(telemetry?.data)
  const view = controlView(status)
  const stopMode = stopButton(status)
  const stopped = view === 'safe_stop' || view === 'estop'

  const [toggleManual, toggleState] = useTrigger('/toggle_manual_mode')
  const [startNav,     startState]  = useTrigger('/start_navigation')
  const [safeStop,     stopState]   = useTrigger('/safe_stop')
  const [estop,        estopState]  = useTrigger('/trigger_estop')
  const [resume,       resumeState] = useTrigger('/resume')

  // Owned here, not in ManualPanel, so ManualPanel's final zero on unmount still has a publisher
  const publishTwist = useRosPublisher('/teleop_cmd_vel', 'geometry_msgs/Twist')

  return (
    <div className="flex flex-col gap-2">
      {/* Header row with mode toggle — disabled during SAFE_STOP and ESTOP */}
      <div className="flex items-center justify-between">
        <h2 className="text-xs font-semibold uppercase tracking-widest text-gray-400">
          Control
        </h2>
        <button
          onClick={() => !stopped && toggleManual()}
          disabled={stopped}
          className={`text-xs px-2 py-0.5 rounded font-medium transition-colors ${
            view === 'estop'
              ? 'bg-red-900 text-red-300 cursor-not-allowed'
              : stopped
                ? 'bg-gray-700 text-gray-500 cursor-not-allowed'
                : view === 'manual'
                  ? 'bg-orange-700 hover:bg-orange-600 text-white'
                  : 'bg-green-800 hover:bg-green-700 text-green-200'
          }`}
        >
          {view === 'estop' ? 'E-STOP' : view === 'safe_stop' ? 'SAFE STOP' : view === 'manual' ? 'MANUAL' : 'AUTO'}
        </button>
      </div>
      <ReplyLine state={toggleState} />
      {/* A successful Resume or Stop changes the view, so their replies also show in the next one */}
      {view !== 'safe_stop' && <ReplyLine state={resumeState} />}

      {view === 'estop' ? (
        <EstopPanel onEstop={estop} estopState={estopState} />
      ) : view === 'safe_stop' ? (
        <SafeStopPanel
          stopState={stopState}
          onResume={resume}
          resumeState={resumeState}
          onEstop={estop}
          estopState={estopState}
        />
      ) : view === 'manual' ? (
        <ManualPanel
          publishTwist={publishTwist}
          stop={<StopButton mode={stopMode} onStop={safeStop} stopState={stopState} />}
          onEstop={estop}
          estopState={estopState}
        />
      ) : (
        <AutoPanel
          goalMode={goalMode}
          onGoalModeChange={onGoalModeChange}
          onStartNav={startNav}
          startState={startState}
          stop={<StopButton mode={stopMode} onStop={safeStop} stopState={stopState} />}
          onEstop={estop}
          estopState={estopState}
        />
      )}
    </div>
  )
}

/* ── Service reply under a button ── */
function ReplyLine({ state }: { state: TriggerState | null }) {
  if (!state || state.phase === 'pending' || !state.text) return null
  const color =
    state.phase === 'ok' ? 'text-green-400' : state.phase === 'refused' ? 'text-amber-300' : 'text-red-400'
  return (
    <div role="status" className={`text-xs leading-snug ${color}`}>
      {state.text}
    </div>
  )
}

/* ── Stop button: the soft stop (/safe_stop), resumable ── */
function StopButton({
  mode,
  onStop,
  stopState,
}: {
  mode: StopButtonMode
  onStop: () => void
  stopState: TriggerState | null
}) {
  if (mode === 'hidden') return null
  const phase = stopState?.phase
  const enabled = mode === 'enabled'
  return (
    <>
      <button
        onClick={onStop}
        disabled={!enabled}
        title={enabled
          ? 'Soft stop. The shuttle brakes into SAFE STOP. Resume continues.'
          : 'Only while navigating or in manual mode'}
        className={`w-full py-2 rounded text-sm font-bold tracking-wider transition-colors ${
          !enabled
            ? 'bg-gray-700 text-gray-500 cursor-not-allowed'
            : phase === 'pending' || phase === 'ok'
              ? 'bg-amber-400 text-gray-900'
              : 'bg-amber-500 hover:bg-amber-400 active:bg-amber-600 text-gray-900'
        }`}
      >
        {phase === 'pending' ? 'Stopping…' : phase === 'ok' ? 'Stopped' : phase ? 'Not Stopped' : 'Stop'}
      </button>
      <ReplyLine state={stopState} />
    </>
  )
}

/* ── E-STOP button, the same in every view. It latches, and says so ── */
function EstopButton({ onEstop, estopState }: { onEstop: () => void; estopState: TriggerState | null }) {
  const phase = estopState?.phase
  const sent = phase === 'pending' || phase === 'ok'
  return (
    <>
      <button
        onClick={onEstop}
        title={`Hardware E-stop. ${ESTOP_LATCH_NOTE} Use Stop to pause.`}
        className={`w-full py-2 rounded font-bold tracking-wider transition-colors ${
          sent
            ? 'bg-red-400 text-white'
            : 'bg-red-600 hover:bg-red-500 active:bg-red-700 text-white'
        }`}
      >
        <span className="block text-sm">
          {phase === 'error' ? 'E-STOP FAILED' : phase === 'refused' ? 'E-STOP REFUSED' : sent ? 'E-STOP SENT' : 'E-STOP'}
        </span>
        <span className="block text-[10px] font-medium tracking-normal opacity-90">{ESTOP_LATCH_NOTE}</span>
      </button>
      <ReplyLine state={estopState} />
    </>
  )
}

/* ── E-stop (latched) mode ── */
function EstopPanel({ onEstop, estopState }: { onEstop: () => void; estopState: TriggerState | null }) {
  return (
    <>
      <div className="rounded bg-red-950/60 border border-red-600 px-3 py-2 text-xs text-red-300 leading-relaxed">
        Vehicle is <span className="font-bold">E-STOPPED</span>. The controller
        refuses to start or switch mode until trajectory_controller restarts.
      </div>

      <EstopButton onEstop={onEstop} estopState={estopState} />
    </>
  )
}

/* ── Safe Stop mode ── */
function SafeStopPanel({
  stopState,
  onResume,
  resumeState,
  onEstop,
  estopState,
}: {
  stopState: TriggerState | null
  onResume: () => void
  resumeState: TriggerState | null
  onEstop: () => void
  estopState: TriggerState | null
}) {
  const phase = resumeState?.phase
  return (
    <>
      <div className="rounded bg-amber-900/40 border border-amber-600 px-3 py-2 text-xs text-amber-300 leading-relaxed">
        Vehicle is held in <span className="font-bold">Safe Stop</span>. Confirm
        the path is clear before resuming.
      </div>
      <ReplyLine state={stopState} />

      <button
        onClick={onResume}
        className={`w-full py-2 rounded text-sm font-bold tracking-wider transition-colors ${
          phase === 'pending' || phase === 'ok'
            ? 'bg-amber-400 text-gray-900'
            : 'bg-amber-500 hover:bg-amber-400 active:bg-amber-600 text-gray-900'
        }`}
      >
        {phase === 'pending' ? 'Resuming…' : phase === 'ok' ? 'Resumed' : phase ? 'Not Resumed' : 'Resume'}
      </button>
      <ReplyLine state={resumeState} />

      <EstopButton onEstop={onEstop} estopState={estopState} />
    </>
  )
}

/* ── Autonomous mode ── */
function AutoPanel({
  goalMode,
  onGoalModeChange,
  onStartNav,
  startState,
  stop,
  onEstop,
  estopState,
}: {
  goalMode: boolean
  onGoalModeChange: (v: boolean) => void
  onStartNav: () => void
  startState: TriggerState | null
  stop: ReactNode
  onEstop: () => void
  estopState: TriggerState | null
}) {
  const phase = startState?.phase
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
          phase === 'ok'
            ? 'bg-blue-400 text-white'
            : 'bg-blue-700 hover:bg-blue-600 text-white'
        }`}
      >
        {phase === 'pending'
          ? 'Starting…'
          : phase === 'ok'
            ? 'Navigation Started'
            : phase
              ? 'Not Started'
              : 'Start Navigation'}
      </button>
      <ReplyLine state={startState} />

      {stop}

      <EstopButton onEstop={onEstop} estopState={estopState} />
    </>
  )
}

/* ── Manual / teleop mode ── */
function ManualPanel({
  publishTwist,
  stop,
  onEstop,
  estopState,
}: {
  publishTwist: (msg: Twist) => void
  stop: ReactNode
  onEstop: () => void
  estopState: TriggerState | null
}) {
  const keysRef = useRef<DirFlags>({ ...NO_KEYS })
  const drivingRef = useRef(false)   // the last Twist sent was a drive command, not the stop

  const [linearSpeed, setLinearSpeed] = useState(1.0)
  const [angularDeg, setAngularDeg]   = useState(10.0)
  const speedRef = useRef({ linearSpeed, angularDeg })
  speedRef.current = { linearSpeed, angularDeg }

  // Publish for the held keys, the stop once after the last key is released, else nothing
  const step = useCallback(() => {
    const { linearSpeed, angularDeg } = speedRef.current
    const { publish, driving } = teleopStep(drivingRef.current, keysRef.current, linearSpeed, angularDeg)
    drivingRef.current = driving
    if (publish) publishTwist(publish)
  }, [publishTwist])

  // 10 Hz publish loop (publishes only while a key is held)
  useEffect(() => {
    const id = setInterval(step, 100)
    return () => clearInterval(id)
  }, [step])

  // Keyboard WASD / arrows
  useEffect(() => {
    function onKeyDown(e: KeyboardEvent) {
      if (e.repeat) return
      if (e.key === 'w' || e.key === 'ArrowUp')    keysRef.current.fwd   = true
      if (e.key === 's' || e.key === 'ArrowDown')  keysRef.current.rev   = true
      if (e.key === 'a' || e.key === 'ArrowLeft')  keysRef.current.left  = true
      if (e.key === 'd' || e.key === 'ArrowRight') keysRef.current.right = true
      if (e.key === ' ') {
        keysRef.current = { ...NO_KEYS }
        drivingRef.current = false
        publishTwist(ZERO_TWIST)   // quick stop: always sent, even when not driving
        return
      }
      step()
    }
    function onKeyUp(e: KeyboardEvent) {
      if (e.key === 'w' || e.key === 'ArrowUp')    keysRef.current.fwd   = false
      if (e.key === 's' || e.key === 'ArrowDown')  keysRef.current.rev   = false
      if (e.key === 'a' || e.key === 'ArrowLeft')  keysRef.current.left  = false
      if (e.key === 'd' || e.key === 'ArrowRight') keysRef.current.right = false
      step()   // the stop goes out at once when this was the last key
    }
    // The page sees no key release after it loses focus: release every key
    function releaseAll() {
      keysRef.current = { ...NO_KEYS }
      step()
    }
    function onVisibility() {
      if (document.visibilityState === 'hidden') releaseAll()
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', releaseAll)
    window.addEventListener('pagehide', releaseAll)
    document.addEventListener('visibilitychange', onVisibility)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      window.removeEventListener('blur', releaseAll)
      window.removeEventListener('pagehide', releaseAll)
      document.removeEventListener('visibilitychange', onVisibility)
      // Stop on unmount, if still driving
      if (drivingRef.current) {
        drivingRef.current = false
        publishTwist(ZERO_TWIST)
      }
    }
  }, [publishTwist, step])

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

      {stop}

      <EstopButton onEstop={onEstop} estopState={estopState} />
    </div>
  )
}
