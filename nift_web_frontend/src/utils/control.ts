/*
 * File: control.ts
 *
 * Description:
 *   Pure helpers for ControlPanel. No React and no roslib here, so
 *   test/control.test.ts runs them in Node.
 *
 *   navStatus / controlView
 *     trajectory_controller publishes /nav_telemetry as "Key:   value" lines.
 *     The STATUS line holds its mode as it names it: IDLE, NAVIGATING, MANUAL,
 *     SAFE_STOP or ESTOP. controlView picks the panel view for that mode.
 *     ESTOP is the controller's latched stop that only a restart clears.
 *     Any other status gets the AUTO view.
 *
 *   stopButton
 *     Whether the Stop button (/safe_stop) is shown and enabled. /safe_stop
 *     puts a driving shuttle into SAFE_STOP, which /resume leaves. It acts
 *     while NAVIGATING or MANUAL, so the button is enabled there. In IDLE it
 *     would change nothing, so it is shown but disabled. In SAFE_STOP the
 *     shuttle already holds (Resume is the button there), and in ESTOP the
 *     controller refuses it, so it is hidden in both.
 *
 *   triggerFeedback
 *     Turns the outcome of a std_srvs/Trigger call into the text the panel
 *     shows. A reply with success false is a refusal and carries the
 *     controller's reason, e.g. "Cannot start: no valid path loaded."
 *
 *   teleopStep
 *     Decides what manual driving publishes on /teleop_cmd_vel. A Twist
 *     while a drive key is held, then one zero Twist when the last key is
 *     released, then nothing. The controller keeps the last /teleop_cmd_vel
 *     it got, so that one zero is what stops the shuttle.
 *
 * Usage:
 *   import { navStatus, controlView, stopButton, triggerFeedback, teleopStep } from '../utils/control'
 */

import type { TriggerResponse } from '../types/ros'
import type { ServiceOutcome } from '../hooks/useRosService'

/* ── Mode from /nav_telemetry ── */

export type ControlView = 'auto' | 'manual' | 'safe_stop' | 'estop'

/** The value of the STATUS line in a /nav_telemetry message, or null if there is none */
export function navStatus(telemetry: string | null | undefined): string | null {
  if (!telemetry) return null
  for (const line of telemetry.split('\n')) {
    const idx = line.indexOf(':')
    if (idx !== -1 && line.slice(0, idx).trim() === 'STATUS') return line.slice(idx + 1).trim()
  }
  return null
}

/** The ControlPanel view for a controller status */
export function controlView(status: string | null): ControlView {
  switch (status) {
    case 'MANUAL':    return 'manual'
    case 'SAFE_STOP': return 'safe_stop'
    case 'ESTOP':     return 'estop'
    default:          return 'auto'
  }
}

/* ── Soft stop (/safe_stop) and E-stop (/trigger_estop) ── */

export type StopButton = 'enabled' | 'disabled' | 'hidden'

/** How the Stop button shows for a controller status */
export function stopButton(status: string | null): StopButton {
  switch (controlView(status)) {
    case 'manual':    return 'enabled'
    case 'safe_stop': return 'hidden'
    case 'estop':     return 'hidden'
    default:          return status === 'NAVIGATING' ? 'enabled' : 'disabled'
  }
}

/** Said on and under the E-STOP button, so nobody takes it for a pause */
export const ESTOP_LATCH_NOTE = 'Latches. Needs a controller restart and a shuttle power cycle.'

/* ── Service replies ── */

export type FeedbackKind = 'ok' | 'refused' | 'error'

export interface Feedback {
  kind: FeedbackKind
  text: string
}

/** What to show for the outcome of a Trigger call */
export function triggerFeedback(outcome: ServiceOutcome<TriggerResponse>): Feedback {
  if (!outcome.ok) return { kind: 'error', text: `Call failed: ${outcome.error}` }
  const message = outcome.response.message ?? ''
  if (outcome.response.success === true) return { kind: 'ok', text: message || 'Done.' }
  return { kind: 'refused', text: message || 'Refused by the controller.' }
}

/* ── Manual driving ── */

export interface DirFlags {
  fwd: boolean
  rev: boolean
  left: boolean
  right: boolean
}

export interface Twist {
  linear: { x: number; y: number; z: number }
  angular: { x: number; y: number; z: number }
}

export const NO_KEYS: DirFlags = { fwd: false, rev: false, left: false, right: false }

export const ZERO_TWIST: Twist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }

/** True while any drive key is held */
export function anyKey(flags: DirFlags): boolean {
  return flags.fwd || flags.rev || flags.left || flags.right
}

/** The Twist for the held keys. angular.z is a steering angle in radians, as the controller reads it */
export function buildTwist(flags: DirFlags, linearSpeed: number, angularDeg: number): Twist {
  const linear    = flags.fwd ? linearSpeed : flags.rev ? -linearSpeed : 0
  const angularRad = angularDeg * (Math.PI / 180.0)
  const angularRaw = flags.left ? angularRad : flags.right ? -angularRad : 0
  // Negate steering when reversing so left key = turn left in both directions
  const angular = linear < 0 ? -angularRaw : angularRaw
  return { linear: { x: linear, y: 0, z: 0 }, angular: { x: 0, y: 0, z: angular } }
}

/**
 * One step of manual driving. driving says whether the last thing sent was a
 * drive command. Returns the Twist to publish now (or null) and the new driving.
 */
export function teleopStep(
  driving: boolean,
  flags: DirFlags,
  linearSpeed: number,
  angularDeg: number,
): { publish: Twist | null; driving: boolean } {
  if (anyKey(flags)) return { publish: buildTwist(flags, linearSpeed, angularDeg), driving: true }
  if (driving) return { publish: ZERO_TWIST, driving: false }
  return { publish: null, driving: false }
}
