/*
 * File: follower.ts
 *
 * Description:
 *   The smoothing behind the map's Follow. The camera does not copy the shuttle's pose. It glides
 *   after it with a critically damped follower per axis (x, y and heading), stepped once per drawn
 *   frame with that frame's time. This is the same idea as the chase views of our RViz setup.
 *
 *   Why it matters. States come at 10 Hz, so a camera that copies them moves in steps. With a GNSS
 *   fault the stack steers on a noisy pose, so the shuttle itself weaves a little, and a view that
 *   turns with every small heading change turns the whole map with it. 20 m behind the shuttle, a
 *   0.3 degree flicker swings the camera about 10 cm sideways.
 *
 *   Heading is smoothed on the circle (always the short way round). Below a walking pace the target
 *   heading is held, so a shuttle that stands still never turns the view on a tiny change. A big
 *   change (Set pose at the same spot) still goes through. Reversing needs nothing special, since
 *   the shuttle's own heading does not flip when it backs up.
 *
 *   Plain TypeScript with no three.js, so it is tested on its own (test/follower.test.ts).
 */

/** An angle wrapped to (-pi, pi]. */
export function wrapAngle(a: number): number {
  const w = a - 2 * Math.PI * Math.round(a / (2 * Math.PI))
  return w <= -Math.PI ? w + 2 * Math.PI : w
}

/**
 * One axis of a critically damped follower ("smooth damp", Game Programming Gems 4, 1.10). It is
 * stable for any frame time and settles on a still target without overshoot. A target moving at a
 * steady rate r is followed r * smoothTime behind.
 */
export class SmoothDamp {
  constructor(
    public value = 0,
    /** The value's own speed [unit/s]. */
    public rate = 0,
  ) {}

  /** Moves towards `target` over `dt` [s]. `smoothTime` [s] is about the time to catch up; 0 jumps. */
  step(target: number, smoothTime: number, dt: number) {
    if (smoothTime <= 0) {
      this.value = target
      this.rate = 0
      return
    }
    const omega = 2 / smoothTime
    const x = omega * dt
    const decay = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x) // close to exp(-x)
    const change = this.value - target
    const temp = (this.rate + omega * change) * dt
    this.value = target + (change + temp) * decay
    this.rate = (this.rate - omega * temp) * decay
  }
}

export interface FollowPose {
  x: number
  y: number
  yaw: number
}

export class FollowSmoother {
  /** [s] How softly the view follows the position (0 follows exactly). */
  positionTime = 0.25
  /** [s] How softly it turns with the heading (0 turns exactly). */
  yawTime = 0.6
  /** [s] A frame gap longer than this (a hidden tab) jumps to the shuttle instead of gliding. */
  maxGap = 1.0
  /** [m] A jump this far (a reset, a far Set pose) is taken at once. */
  snapDistance = 10
  /** [m/s] Below this the target heading is held, so a standing shuttle never turns the view. */
  holdSpeed = 0.3
  /** [rad] A heading change this big still goes through while held (Set pose on the spot). */
  holdYaw = (10 * Math.PI) / 180

  private x = new SmoothDamp()
  private y = new SmoothDamp()
  private yaw = new SmoothDamp()
  private heldYaw = 0
  private primed = false

  /** Forgets the state: the next update() starts on its target. */
  reset() {
    this.primed = false
  }

  /** Advances by `dt` [s] towards the shuttle's latest pose and returns the smoothed pose. */
  update(target: FollowPose, dt: number): FollowPose {
    const jump = Math.hypot(target.x - this.x.value, target.y - this.y.value)
    if (!this.primed || !(dt >= 0) || dt > this.maxGap || !(jump <= this.snapDistance)) {
      this.snap(target)
      return this.pose()
    }
    this.x.step(target.x, this.positionTime, dt)
    this.y.step(target.y, this.positionTime, dt)
    // The smoothed velocity is a calm speed estimate: the 10 Hz steps and the noise are filtered out
    const moving = Math.hypot(this.x.rate, this.y.rate) >= this.holdSpeed
    if (moving || Math.abs(wrapAngle(target.yaw - this.heldYaw)) >= this.holdYaw) this.heldYaw = target.yaw
    this.yaw.step(this.yaw.value + wrapAngle(this.heldYaw - this.yaw.value), this.yawTime, dt)
    this.yaw.value = wrapAngle(this.yaw.value)
    return this.pose()
  }

  pose(): FollowPose {
    return { x: this.x.value, y: this.y.value, yaw: this.yaw.value }
  }

  private snap(t: FollowPose) {
    this.x = new SmoothDamp(t.x)
    this.y = new SmoothDamp(t.y)
    this.yaw = new SmoothDamp(wrapAngle(t.yaw))
    this.heldYaw = t.yaw
    this.primed = true
  }
}
