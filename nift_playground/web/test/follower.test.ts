// The map Follow's smoothing (src/scene/follower.ts): settles without overshoot, the same at any
// frame rate, the short way round the circle, calm under heading noise, still when the shuttle stands.

import { describe, expect, it } from 'vitest'
import { FollowSmoother, SmoothDamp, wrapAngle } from '../src/scene/follower'

const DEG = Math.PI / 180

/** A seeded normal sample (Box-Muller on a small LCG), so the noise test is the same every run. */
function gauss(seed: number) {
  let s = seed
  const u = () => {
    s = (s * 1664525 + 1013904223) % 4294967296
    return (s + 1) / 4294967297
  }
  return () => Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u())
}

describe('wrapAngle', () => {
  it('wraps to (-pi, pi]', () => {
    expect(wrapAngle(3 * Math.PI)).toBeCloseTo(Math.PI)
    expect(wrapAngle(-Math.PI)).toBeCloseTo(Math.PI)
    expect(wrapAngle(0.5 - 4 * Math.PI)).toBeCloseTo(0.5)
  })
})

describe('SmoothDamp', () => {
  it('settles on a step without overshoot at any frame time', () => {
    for (const dt of [1 / 144, 1 / 60, 1 / 10, 0.5]) {
      const d = new SmoothDamp(0)
      let max = 0
      for (let t = 0; t < 3; t += dt) {
        d.step(1, 0.25, dt)
        max = Math.max(max, d.value)
      }
      expect(max).toBeLessThanOrEqual(1 + 1e-9)
      expect(d.value).toBeCloseTo(1, 3)
    }
  })

  it('moves the same at 30 and 144 frames a second', () => {
    const a = new SmoothDamp(0)
    const b = new SmoothDamp(0)
    for (let i = 0; i < 30; i++) a.step(5, 0.6, 1 / 30)
    for (let i = 0; i < 144; i++) b.step(5, 0.6, 1 / 144)
    expect(Math.abs(a.value - b.value)).toBeLessThan(0.05)
  })
})

describe('FollowSmoother', () => {
  const run = (f: FollowSmoother, pose: (t: number) => { x: number; y: number; yaw: number }, s: number, dt = 1 / 60) => {
    const out: { x: number; y: number; yaw: number }[] = []
    for (let t = 0; t < s; t += dt) out.push(f.update(pose(t), dt))
    return out
  }

  it('starts on the shuttle, then turns the short way round', () => {
    const f = new FollowSmoother()
    expect(f.update({ x: 1, y: 2, yaw: 179 * DEG }, 0.016)).toEqual({ x: 1, y: 2, yaw: 179 * DEG })
    // -179 deg is 2 deg away, not 358: driving on, the view turns through 180, never round the other way
    const out = run(f, (t) => ({ x: 1 + 5 * t, y: 2, yaw: -179 * DEG }), 3)
    for (const p of out) expect(Math.abs(wrapAngle(p.yaw - 180 * DEG))).toBeLessThanOrEqual(1.01 * DEG)
    expect(wrapAngle(out[out.length - 1].yaw + 179 * DEG)).toBeCloseTo(0, 3)
  })

  it('does not turn a standing shuttle on small heading changes, but takes a big one', () => {
    const f = new FollowSmoother()
    const noise = gauss(7)
    const out = run(f, () => ({ x: 0, y: 0, yaw: 2 * DEG * noise() }), 3)
    // It starts on the first heading and stays there
    for (const p of out) expect(Math.abs(p.yaw - out[0].yaw)).toBeLessThan(0.01 * DEG)
    const turned = run(f, () => ({ x: 0, y: 0, yaw: 90 * DEG }), 3)
    expect(turned[turned.length - 1].yaw).toBeCloseTo(90 * DEG, 2)
  })

  it('turns far less than a noisy heading while driving', () => {
    const f = new FollowSmoother()
    const noise = gauss(3)
    // 5 m/s east, states at 10 Hz, the heading weaving with a 1 deg sigma
    let held = { x: 0, y: 0, yaw: 0 }
    let next = 0
    const out = run(f, (t) => {
      if (t >= next) {
        held = { x: 5 * t, y: 0, yaw: DEG * noise() }
        next += 0.1
      }
      return held
    }, 10)
    // The raw heading steps about a sigma (1 deg) per state; the view turns far less per frame
    let smooth = 0
    for (let i = 61; i < out.length; i++) smooth = Math.max(smooth, Math.abs(wrapAngle(out[i].yaw - out[i - 1].yaw)))
    expect(smooth).toBeLessThan(DEG / 5)
  })

  it('jumps on a reset far away and after a long frame gap', () => {
    const f = new FollowSmoother()
    f.update({ x: 0, y: 0, yaw: 0 }, 0.016)
    expect(f.update({ x: 100, y: 0, yaw: 1 }, 0.016)).toEqual({ x: 100, y: 0, yaw: 1 })
    expect(f.update({ x: 105, y: 0, yaw: 2 }, 5)).toEqual({ x: 105, y: 0, yaw: 2 })
  })
})
