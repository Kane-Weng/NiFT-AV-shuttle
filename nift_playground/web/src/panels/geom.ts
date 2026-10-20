/*
 * File: geom.ts
 *
 * Description:
 *   Plane geometry for the obstacle tools: the centroid ARCH.md 8.1 moves (the mean of the
 *   vertices, which is the area centroid for the boxes the page places), point in polygon for
 *   picking, and the box footprint.
 */

import type { XY } from '../protocol'

export function centroid(poly: XY[]): XY {
  let x = 0
  let y = 0
  for (const p of poly) {
    x += p[0]
    y += p[1]
  }
  return [x / poly.length, y / poly.length]
}

export function pointInPolygon(p: XY, poly: XY[]): boolean {
  let inside = false
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i]
    const [xj, yj] = poly[j]
    if (yi > p[1] !== yj > p[1] && p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi) + xi) inside = !inside
  }
  return inside
}

export function translate(poly: XY[], dx: number, dy: number): XY[] {
  return poly.map(([x, y]) => [x + dx, y + dy] as XY)
}

export function boxPoly(b: { x: number; y: number; yaw: number; length: number; width: number }): XY[] {
  const c = Math.cos(b.yaw)
  const s = Math.sin(b.yaw)
  return [
    [1, 1],
    [-1, 1],
    [-1, -1],
    [1, -1],
  ].map(([u, v]) => {
    const lx = (u * b.length) / 2
    const ly = (v * b.width) / 2
    return [b.x + c * lx - s * ly, b.y + s * lx + c * ly] as XY
  })
}

/** A new obstacle id that is not in use: o1, o2, ... */
export function nextObstacleId(used: { oid: string }[]): string {
  const ids = new Set(used.map((o) => o.oid))
  let i = 1
  while (ids.has(`o${i}`)) i++
  return `o${i}`
}
