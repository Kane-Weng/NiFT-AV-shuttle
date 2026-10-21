// The shuttle's smooth playback between states (src/scene/interp.ts): even steps every frame, the
// heading the short way round, and no interpolation while paused, after a reset or across a teleport.

import { describe, expect, it } from 'vitest'
import { PoseInterpolator, lerpPose } from '../src/scene/interp'

/** Drives east at `v` m/s; states every `every` ms of wall time, `factor` sim s per wall s. Returns the drawn x per frame. */
function drive(ip: PoseInterpolator, v: number, factor: number, every = 100, seconds = 3, jitterMs = 0) {
  const xs: number[] = []
  let nextState = 0
  let k = 0
  for (let wall = 0; wall < seconds * 1000; wall += 1000 / 60) {
    if (wall >= nextState) {
      const t = (nextState / 1000) * factor
      ip.push(t, { x: v * t, y: 0, yaw: 0 }, null, false, wall)
      k++
      nextState = k * every + (k % 2 ? jitterMs : -jitterMs)
    }
    xs.push(ip.sample(wall)!.pose.x)
  }
  return xs
}

const steps = (xs: number[]) => xs.slice(1).map((x, i) => x - xs[i])

describe('lerpPose', () => {
  it('turns the short way round', () => {
    const p = lerpPose({ x: 0, y: 0, yaw: 3.1 }, { x: 2, y: 0, yaw: -3.1 }, 0.5)
    expect(p.x).toBe(1)
    expect(Math.abs(Math.abs(p.yaw) - Math.PI)).toBeLessThan(1e-9)
  })
})

describe('PoseInterpolator', () => {
  for (const factor of [1, 5]) {
    it(`moves the box every frame by an even step at x${factor}`, () => {
      const xs = drive(new PoseInterpolator(), 3, factor, 100, 3, 8)
      const d = steps(xs).slice(60) // after a second of settling
      const mean = d.reduce((a, b) => a + b, 0) / d.length
      expect(mean).toBeCloseTo((3 * factor) / 60, 2)
      for (const s of d) expect(Math.abs(s - mean)).toBeLessThan(0.35 * mean)
    })
  }

  it('stays about one state behind the newest', () => {
    const ip = new PoseInterpolator()
    const xs = drive(ip, 3, 1)
    // The newest state is at most 0.1 s old; the box is 1 to 2.5 states (0.3 to 0.75 m) behind it
    const behind = 3 * 2.95 - xs[xs.length - 1]
    expect(behind).toBeGreaterThan(0.2)
    expect(behind).toBeLessThan(0.8)
  })

  it('shows each state at once while paused', () => {
    const ip = new PoseInterpolator()
    drive(ip, 3, 1, 100, 1)
    ip.push(1.0, { x: 3, y: 0, yaw: 0 }, null, true, 1100)
    expect(ip.sample(1101)!.pose.x).toBe(3)
    ip.push(1.02, { x: 3.06, y: 0, yaw: 0 }, null, true, 1200)
    expect(ip.sample(1201)!.pose.x).toBeCloseTo(3.06)
  })

  it('jumps on a teleport and on a reset', () => {
    const ip = new PoseInterpolator()
    drive(ip, 3, 1, 100, 1)
    ip.push(1.0, { x: 50, y: 20, yaw: 1 }, null, false, 1000)
    expect(ip.sample(1001)!.pose).toEqual({ x: 50, y: 20, yaw: 1 })
    ip.push(0, { x: 0, y: 0, yaw: 0 }, null, false, 1100)
    expect(ip.sample(1101)!.pose).toEqual({ x: 0, y: 0, yaw: 0 })
  })

  it('draws the estimate with the pose, and hides it when it goes', () => {
    const ip = new PoseInterpolator()
    ip.push(0, { x: 0, y: 0, yaw: 0 }, { x: 0.5, y: 0, yaw: 0 }, false, 0)
    expect(ip.sample(1)!.est).toEqual({ x: 0.5, y: 0, yaw: 0 })
    ip.push(0.1, { x: 0.3, y: 0, yaw: 0 }, null, true, 100)
    expect(ip.sample(101)!.est).toBeNull()
  })
})
