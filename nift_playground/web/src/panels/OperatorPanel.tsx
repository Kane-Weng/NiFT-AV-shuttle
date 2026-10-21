/*
 * File: OperatorPanel.tsx
 *
 * Description:
 *   The operator panel, kept to the buttons. It behaves like our RViz NavPanel and the nift_web
 *   ControlPanel, against the same services (ARCH.md 8.1):
 *     - the status from /nav_telemetry STATUS in the title bar
 *     - Start Navigation (only in IDLE, like the RViz panel), Stop (/safe_stop, a SAFE_STOP) and
 *       Resume (only from SAFE_STOP)
 *     - Manual mode (/toggle_manual_mode) and keyboard teleop: click the teleop pad, then
 *       W/A/S/D or the arrows, Space stops. Twist at 10 Hz on /teleop_cmd_vel, linear.x = v and
 *       angular.z = wheel angle, A = wheels left in either gear (as the RViz panel). The first
 *       drive key asks for MANUAL if the controller is not in it yet.
 *   When the stack window left out a node that driving needs, a hint says what no longer works and
 *   opens the stack window. 2D Goal and Set pose are on the map's tool bar. A refused command says why under the buttons
 *   for a few seconds; every reply is in the log's Events tab.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useSimClient, useSimSnapshot } from '../sim/useSim'
import type { CmdName, NavStatus } from '../protocol'
import { historyOf } from './history'
import { Badge, Panel, Section, input, type Tone } from './Panel'
import { stackHint } from './StackDialog'
import { setUi } from './ui'
import { VEHICLE } from '../geo/site'

export const STATUS_TONE: Record<NavStatus, Tone> = {
  IDLE: 'muted',
  NAVIGATING: 'ok',
  MANUAL: 'manual',
  SAFE_STOP: 'warn',
  ESTOP: 'error',
}

const NO_TELEMETRY = 'No /nav_telemetry yet. The controller starts in IDLE and publishes nothing until it leaves IDLE once.'

const CMD_LABEL: Record<CmdName, string> = {
  start: 'Start navigation',
  stop: 'Stop',
  estop: 'E-STOP',
  resume: 'Resume',
  manual: 'Manual',
  dbw_estop: 'DBW E-stop',
  power_cycle: 'Power cycle',
}

export default function OperatorPanel() {
  const client = useSimClient()
  const { ready, state, readyInfo } = useSimSnapshot()
  const hint = useMemo(() => stackHint(readyInfo), [readyInfo])
  const [busy, setBusy] = useState<CmdName | null>(null)
  const [refused, setRefused] = useState<string | null>(null)
  const status = state?.status ?? null

  const cmd = useCallback(
    async (name: CmdName) => {
      setBusy(name)
      try {
        const r = await historyOf(client).command({ type: 'cmd', name }, CMD_LABEL[name])
        setRefused(r.ok ? null : `${CMD_LABEL[name]}: ${r.message}`)
      } finally {
        setBusy(null)
      }
    },
    [client],
  )
  useEffect(() => {
    if (!refused) return
    const id = setTimeout(() => setRefused(null), 6000)
    return () => clearTimeout(id)
  }, [refused])

  const stopped = status === 'SAFE_STOP' || status === 'ESTOP'
  // No telemetry yet is the controller's start-up IDLE (the RViz panel's Start is enabled then too)
  const idle = status === 'IDLE' || (state !== null && status === null)
  const big = 'w-full rounded py-2 text-sm font-semibold disabled:opacity-40 transition-colors'

  return (
    <Panel title="Operator" testid="operator-panel" right={<Badge tone={status ? STATUS_TONE[status] : 'muted'} testid="status" title={state && !status ? NO_TELEMETRY : undefined}>{status ?? (state ? 'IDLE' : '-')}</Badge>}>
      {hint && (
        <div data-testid="stack-hint" className="mb-2 rounded border border-sky-400 bg-sky-50 dark:bg-sky-950/40 dark:border-sky-800 px-2 py-1.5 text-xs text-sky-900 dark:text-sky-200">
          <b>{hint.off.join(', ')}</b> {hint.off.length > 1 ? 'are' : 'is'} off in the stack: {hint.text}.{' '}
          <button data-testid="stack-hint-open" data-dialog-toggle className="underline font-medium hover:text-sky-600 dark:hover:text-sky-300" onClick={() => setUi({ win: 'stack' })}>
            Open the stack
          </button>
        </div>
      )}
      {status === 'SAFE_STOP' && (
        <div data-testid="safe-stop-banner" className="mb-2 rounded border border-amber-400 bg-amber-50 dark:bg-amber-900/30 dark:border-amber-700 px-2 py-1.5 text-xs text-amber-800 dark:text-amber-300">
          The vehicle is held in <b>Safe Stop</b>. Check the path is clear, then Resume (back to IDLE) and Start again.
        </div>
      )}
      {status === 'ESTOP' && (
        <div className="mb-2 rounded border border-red-400 bg-red-50 dark:bg-red-900/30 dark:border-red-700 px-2 py-1.5 text-xs text-red-800 dark:text-red-300">
          The controller is <b>E-stopped</b>. Only a node restart recovers it: Reset the playground.
        </div>
      )}
      <div data-tour="nav-buttons">
        <button
          data-testid="cmd-start"
          disabled={!ready || !idle || busy !== null}
          onClick={() => cmd('start')}
          className={`${big} mb-1.5 bg-indigo-600 hover:bg-indigo-500 text-white`}
          title="/start_navigation. Needs IDLE and a path"
        >
          {status === 'NAVIGATING' ? 'Navigation running' : status === 'MANUAL' ? 'Navigation disabled in manual' : 'Start navigation'}
        </button>
        <div className="grid grid-cols-2 gap-1.5 mb-1.5">
          <button data-testid="cmd-stop" disabled={!ready} onClick={() => cmd('stop')} className={`${big} bg-slate-500 hover:bg-slate-400 text-white`} title="/safe_stop: a SAFE_STOP that Resume clears">
            Stop
          </button>
          <button
            data-testid="cmd-resume"
            disabled={!ready || status !== 'SAFE_STOP'}
            onClick={() => cmd('resume')}
            className={`${big} ${status === 'SAFE_STOP' ? 'bg-amber-500 hover:bg-amber-400 text-gray-900' : 'bg-gray-300 dark:bg-gray-700 text-gray-700 dark:text-gray-300'}`}
            title="/resume: SAFE_STOP back to IDLE"
          >
            Resume
          </button>
        </div>
      </div>
      {refused && (
        <div className="text-[11px] leading-4 text-red-600 dark:text-red-400 mb-1" data-testid="last-reply">
          {refused}
        </div>
      )}

      <Section title="Manual drive" fold="op.manual" defaultOpen testid="manual-section"
        right={
          <button
            data-testid="cmd-manual"
            disabled={!ready || stopped || busy !== null}
            onClick={() => cmd('manual')}
            className={`rounded px-2 py-0.5 text-[11px] font-semibold disabled:opacity-40 ${status === 'MANUAL' ? 'bg-orange-500 text-white hover:bg-orange-400' : 'bg-emerald-700 text-white hover:bg-emerald-600'}`}
            title="/toggle_manual_mode"
          >
            {status === 'MANUAL' ? 'Disable manual' : 'Enable manual'}
          </button>
        }
      >
        <Teleop status={status} onManual={() => cmd('manual')} />
      </Section>

    </Panel>
  )
}

// ------------------------------------------------------------------ teleop

type Keys = { w: boolean; s: boolean; a: boolean; d: boolean }
const KEYMAP: Record<string, keyof Keys> = { w: 'w', W: 'w', ArrowUp: 'w', s: 's', S: 's', ArrowDown: 's', a: 'a', A: 'a', ArrowLeft: 'a', d: 'd', D: 'd', ArrowRight: 'd' }
const MAX_STEER_DEG = VEHICLE.maxSteeringDeg

function Teleop({ status, onManual }: { status: NavStatus | null; onManual: () => void }) {
  const client = useSimClient()
  const [active, setActive] = useState(false)
  const [speed, setSpeed] = useState(1.0)
  const [steerDeg, setSteerDeg] = useState(MAX_STEER_DEG)
  const [cmd, setCmd] = useState({ v: 0, steer: 0 })
  const keys = useRef<Keys>({ w: false, s: false, a: false, d: false })
  const asked = useRef(false)
  const statusRef = useRef(status)
  statusRef.current = status

  const compute = useCallback(() => {
    const k = keys.current
    const steer = (steerDeg * Math.PI) / 180
    setCmd({ v: (k.w ? speed : 0) - (k.s ? speed : 0), steer: (k.a ? steer : 0) - (k.d ? steer : 0) })
  }, [speed, steerDeg])

  // 10 Hz while the pad has focus and the controller is in MANUAL (it ignores teleop otherwise)
  const cmdRef = useRef(cmd)
  cmdRef.current = cmd
  useEffect(() => {
    if (!active) return
    const id = setInterval(() => {
      if (statusRef.current === 'MANUAL') client.send({ type: 'teleop', v: cmdRef.current.v, steer: cmdRef.current.steer })
    }, 100)
    return () => clearInterval(id)
  }, [active, client])

  const stop = () => {
    keys.current = { w: false, s: false, a: false, d: false }
    setCmd({ v: 0, steer: 0 })
    if (statusRef.current === 'MANUAL') client.send({ type: 'teleop', v: 0, steer: 0 })
  }

  const onKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === ' ') {
      e.preventDefault()
      e.stopPropagation()
      stop()
      return
    }
    const k = KEYMAP[e.key]
    if (!k) return
    e.preventDefault()
    e.stopPropagation()
    if (e.repeat) return
    keys.current[k] = true
    if (statusRef.current !== 'MANUAL' && !asked.current) {
      asked.current = true // like the RViz panel: the first drive key asks for MANUAL, once per focus
      onManual()
    }
    compute()
  }
  const onKeyUp = (e: React.KeyboardEvent) => {
    const k = KEYMAP[e.key]
    if (!k) return
    e.preventDefault()
    e.stopPropagation()
    keys.current[k] = false
    compute()
  }

  return (
    <div>
      <div className="flex items-center gap-2 text-xs mb-1.5">
        <label className="flex items-center gap-1">
          <span className="text-gray-500">speed</span>
          <input type="number" className={`${input} w-14`} min={0.1} max={3} step={0.1} value={speed} onChange={(e) => setSpeed(Math.max(0.1, Math.min(3, +e.target.value || 0.1)))} />
          <span className="text-gray-500">m/s</span>
        </label>
        <label className="flex items-center gap-1">
          <span className="text-gray-500">steer</span>
          <input type="number" className={`${input} w-12`} min={3} max={MAX_STEER_DEG} step={1} value={steerDeg} onChange={(e) => setSteerDeg(Math.max(3, Math.min(MAX_STEER_DEG, +e.target.value || 3)))} />
          <span className="text-gray-500">deg</span>
        </label>
      </div>
      <div
        tabIndex={0}
        data-testid="teleop-pad"
        onFocus={() => {
          setActive(true)
          asked.current = false
        }}
        onBlur={() => {
          stop()
          setActive(false)
        }}
        onKeyDown={onKeyDown}
        onKeyUp={onKeyUp}
        className={`rounded border-2 px-2 py-2 text-center font-mono text-[11px] outline-none select-none cursor-pointer ${
          active ? 'border-emerald-500 bg-emerald-50 text-emerald-800 dark:bg-emerald-950/50 dark:text-emerald-300' : 'border-gray-300 dark:border-gray-700 text-gray-500'
        }`}
      >
        {active ? (
          <>
            TELEOP ACTIVE · WASD / arrows · Space stops
            <br />
            v {cmd.v.toFixed(1)} m/s · δ {((cmd.steer * 180) / Math.PI).toFixed(1)}°{status !== 'MANUAL' ? ' · not in MANUAL' : ''}
          </>
        ) : (
          <>
            Click here for keyboard teleop
            <br />
            WASD / arrows · Space stops
          </>
        )}
      </div>
    </div>
  )
}
