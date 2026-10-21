/*
 * File: interp.ts
 *
 * Description:
 *   Smooth playback of the shuttle's pose between states. The page gets a state about 10 times a
 *   second (the mock on a wall clock timer, the Pyodide worker once per batch of ticks), while the
 *   screen draws 60 frames a second. Drawn straight from the states, the box stands still for
 *   five frames out of six and then steps 0.3 m (1.5 m at x5).
 *
 *   So the drawn pose is played back a little behind the newest state: a render clock runs in sim
 *   time at the observed rate (sim seconds per wall second) and stays about one arrival gap behind
 *   the newest state, and the pose is interpolated between the two states around it. Position is
 *   interpolated straight, heading on the circle. The clock is steered gently towards its target
 *   lag, so jitter in the arrivals does not show and a change of the speed factor settles in a
 *   fraction of a second.
 *
 *   It does not interpolate while the sim is paused or stepping (each state is shown as it comes),
 *   after a reset (sim time going back) or across a teleport (Set pose, a jump faster than any
 *   drive). Then it shows the newest state at once.
 *
 *   Plain TypeScript with no three.js, so it is tested on its own (test/interp.test.ts).
 */

import { wrapAngle } from './follower'

export interface Pose {
  x: number
  y: number
  yaw: number
}

interface Sample {
  t: number
  pose: Pose
  est: Pose | null
}

/** A jump longer than this [m] and faster than TELEPORT_SPEED is a teleport, never driven. */
const TELEPORT_M = 2
const TELEPORT_SPEED = 25 // m/s
/** A heading change bigger than this [rad] at more than TELEPORT_YAW_RATE is a teleport too. */
const TELEPORT_YAW = 0.35
const TELEPORT_YAW_RATE = 2 // rad/s
/** Lag behind the newest state, in arrival gaps: a little over one, so a late state does not stall the box. */
const LAG_GAPS = 1.3
/** [ms] Time constant of the rate and gap estimates, and of the clock's steering. */
const EST_MS = 500
const STEER_MS = 250
/** States that arrive within this [ms] of each other are one arrival (a worker batch). */
const SAME_ARRIVAL_MS = 4

export function lerpPose(a: Pose, b: Pose, k: number): Pose {
  return { x: a.x + (b.x - a.x) * k, y: a.y + (b.y - a.y) * k, yaw: wrapAngle(a.yaw + wrapAngle(b.yaw - a.yaw) * k) }
}

export class PoseInterpolator {
  private buf: Sample[] = []
  private renderT = 0
  private rate = 0 // sim s per wall s
  private gap = 0 // sim s between arrivals
  private lastArrival: { wall: number; t: number } | null = null
  private lastSampleWall = 0

  /** Forgets everything: the next state is shown at once. */
  reset() {
    this.buf = []
    this.lastArrival = null
    this.rate = 0
    this.gap = 0
  }

  /** A new state: sim time `t` [s], the true pose and the stack's estimate, received at `wall` [ms]. */
  push(t: number, pose: Pose, est: Pose | null, paused: boolean, wall: number) {
    const s: Sample = { t, pose: { ...pose }, est: est && { ...est } }
    const last = this.buf[this.buf.length - 1]
    if (last && t === last.t) {
      // The same tick again (a paused sim, a Set pose in it): take the newest values, at once if it moved
      const moved = Math.hypot(pose.x - last.pose.x, pose.y - last.pose.y) > TELEPORT_M || Math.abs(wrapAngle(pose.yaw - last.pose.yaw)) > TELEPORT_YAW
      if (paused || moved) this.buf = [s]
      else this.buf[this.buf.length - 1] = s
      if (this.buf.length === 1) this.renderT = t
      return
    }
    const snap = !last || paused || t < last.t || this.teleport(last, s)
    this.estimate(t, wall, paused || !last || t < last.t)
    if (snap) {
      this.buf = [s]
      this.renderT = t
      return
    }
    this.buf.push(s)
    // Keep what the clock can still reach
    while (this.buf.length > 2 && this.buf[1].t < this.renderT) this.buf.shift()
    if (this.buf.length > 256) this.buf.splice(0, this.buf.length - 256)
  }

  /** The pose and estimate to draw at wall time `wall` [ms], or null before the first state. */
  sample(wall: number): { pose: Pose; est: Pose | null } | null {
    const dtMs = this.lastSampleWall ? Math.max(0, wall - this.lastSampleWall) : 0
    this.lastSampleWall = wall
    const buf = this.buf
    if (buf.length === 0) return null
    const newest = buf[buf.length - 1]
    if (buf.length === 1) {
      this.renderT = newest.t
      return { pose: newest.pose, est: newest.est }
    }
    this.renderT += (dtMs / 1000) * this.rate
    const target = newest.t - LAG_GAPS * this.gap
    const err = target - this.renderT
    // Far off (a long hidden tab, a big change of speed factor): jump to the target lag
    if (Math.abs(err) > Math.max(4 * this.gap, 0.05)) this.renderT = target
    else this.renderT += err * (1 - Math.exp(-dtMs / STEER_MS))
    this.renderT = Math.min(newest.t, Math.max(buf[0].t, this.renderT))
    let i = 1
    while (i < buf.length - 1 && buf[i].t < this.renderT) i++
    const a = buf[i - 1]
    const b = buf[i]
    const k = b.t > a.t ? Math.min(1, Math.max(0, (this.renderT - a.t) / (b.t - a.t))) : 1
    const est = a.est && b.est ? lerpPose(a.est, b.est, k) : k < 1 ? a.est : b.est
    return { pose: lerpPose(a.pose, b.pose, k), est }
  }

  private teleport(a: Sample, b: Sample): boolean {
    const dt = Math.max(1e-6, b.t - a.t)
    const d = Math.hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y)
    const dyaw = Math.abs(wrapAngle(b.pose.yaw - a.pose.yaw))
    return (d > TELEPORT_M && d / dt > TELEPORT_SPEED) || (dyaw > TELEPORT_YAW && dyaw / dt > TELEPORT_YAW_RATE)
  }

  /** Updates the sim rate and the arrival gap from a state's arrival; not while paused or after a reset. */
  private estimate(t: number, wall: number, still: boolean) {
    const prev = this.lastArrival
    if (prev && wall - prev.wall < SAME_ARRIVAL_MS) return // the same batch: the arrival is the first state's
    this.lastArrival = { wall, t }
    if (!prev || still) return
    const dWall = wall - prev.wall
    const dSim = t - prev.t
    if (dWall <= 0 || dSim <= 0) return
    const a = this.rate === 0 ? 1 : 1 - Math.exp(-dWall / EST_MS)
    this.rate += (dSim / (dWall / 1000) - this.rate) * a
    this.gap += (dSim - this.gap) * a
  }
}
