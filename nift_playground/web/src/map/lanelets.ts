/*
 * File: lanelets.ts
 *
 * Description:
 *   Builds the drawable lanelet map from parsed .osm data: every point projected into the map
 *   frame, lanelets with oriented bounds and a centreline, and one border entry per way with
 *   its line style.
 *
 *   Line style comes from the way's tags first (type, subtype as in the Lanelet2 tagging spec).
 *   The odr2024 map has no way tags, so a way without them is styled by how the lanelets use it:
 *     - used by one lanelet: road edge, solid
 *     - shared by lanelets that drive the same way: lane divider, dashed
 *     - shared by lanelets that drive opposite ways: centre line, solid and amber
 */

import type { OsmData, Tags } from './osm'
import { MapProjector } from '../geo/utm'

export type XY = [number, number]

export type BorderKind = 'edge' | 'divider' | 'center' | 'virtual' | 'stop_line'

export interface Border {
  wayId: number
  points: XY[]
  kind: BorderKind
  dashed: boolean
}

export interface Lanelet {
  id: number
  left: XY[]          // left bound, oriented in the driving direction
  right: XY[]         // right bound, oriented in the driving direction
  centerline: XY[]
  tags: Tags
  successors: number[]
}

export interface LaneletMap {
  lanelets: Lanelet[]
  byId: Map<number, Lanelet>
  borders: Border[]
  bounds: { minX: number; minY: number; maxX: number; maxY: number }
}

const dist = (a: XY, b: XY) => Math.hypot(a[0] - b[0], a[1] - b[1])

function polylineLength(p: XY[]): number {
  let s = 0
  for (let i = 1; i < p.length; i++) s += dist(p[i - 1], p[i])
  return s
}

/** Point at fraction t (0..1) of the arc length of a polyline. */
function pointAt(p: XY[], cum: number[], t: number): XY {
  const total = cum[cum.length - 1]
  if (total === 0) return p[0]
  const s = t * total
  let i = 1
  while (i < cum.length - 1 && cum[i] < s) i++
  const seg = cum[i] - cum[i - 1]
  const u = seg > 0 ? (s - cum[i - 1]) / seg : 0
  return [p[i - 1][0] + u * (p[i][0] - p[i - 1][0]), p[i - 1][1] + u * (p[i][1] - p[i - 1][1])]
}

function cumulative(p: XY[]): number[] {
  const c = [0]
  for (let i = 1; i < p.length; i++) c.push(c[i - 1] + dist(p[i - 1], p[i]))
  return c
}

/** Centreline as the mean of both bounds sampled at the same arc-length fractions. */
export function centerlineOf(left: XY[], right: XY[]): XY[] {
  const len = Math.max(polylineLength(left), polylineLength(right))
  const n = Math.max(2, Math.min(200, Math.ceil(len / 1.0) + 1, Math.max(left.length, right.length) * 2))
  const cl = cumulative(left)
  const cr = cumulative(right)
  const out: XY[] = []
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1)
    const a = pointAt(left, cl, t)
    const b = pointAt(right, cr, t)
    out.push([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2])
  }
  return out
}

function styleFromTags(tags: Tags): { kind: BorderKind; dashed: boolean } | null {
  const type = tags.type
  const subtype = tags.subtype
  if (!type && !subtype) return null
  if (type === 'virtual') return { kind: 'virtual', dashed: true }
  if (type === 'stop_line') return { kind: 'stop_line', dashed: false }
  if (type === 'road_border' || type === 'curbstone') return { kind: 'edge', dashed: false }
  const dashed = subtype === 'dashed' || subtype === 'dashed_solid' || subtype === 'solid_dashed'
  const kind: BorderKind = tags.color === 'yellow' || subtype === 'solid_solid' ? 'center' : 'divider'
  return { kind, dashed }
}

/** Builds the map from parsed .osm data, projected with the given map-frame projector. */
export function buildLaneletMap(osm: OsmData, projector: MapProjector): LaneletMap {
  const xy = new Map<number, XY>()
  let minX = Infinity
  let minY = Infinity
  let maxX = -Infinity
  let maxY = -Infinity
  for (const n of osm.nodes.values()) xy.set(n.id, projector.toMapXY(n.lat, n.lon))

  const wayPoints = new Map<number, XY[]>()
  for (const w of osm.ways.values()) {
    const pts: XY[] = []
    for (const id of w.nodes) {
      const p = xy.get(id)
      if (p) pts.push(p)
    }
    wayPoints.set(w.id, pts)
  }

  // How each way is used: lanelet id and whether the lanelet drives along the way's stored order
  const usage = new Map<number, { lanelet: number; forward: boolean }[]>()
  const use = (way: number, lanelet: number, forward: boolean) => {
    const list = usage.get(way) ?? []
    list.push({ lanelet, forward })
    usage.set(way, list)
  }

  const lanelets: Lanelet[] = []
  for (const r of osm.relations.values()) {
    if (r.tags.type !== 'lanelet') continue
    const lm = r.members.find((m) => m.role === 'left' && m.type === 'way')
    const rm = r.members.find((m) => m.role === 'right' && m.type === 'way')
    if (!lm || !rm) continue
    let right = wayPoints.get(rm.ref)
    let left = wayPoints.get(lm.ref)
    if (!left || !right || left.length < 2 || right.length < 2) continue
    // Orient both bounds the way Lanelet2's loader does (checked against it on odr2024): first
    // the left bound along the right one, then both reversed if the left bound lies on the right.
    let leftFwd = dist(left[0], right[0]) + dist(left[left.length - 1], right[right.length - 1]) <=
      dist(left[0], right[right.length - 1]) + dist(left[left.length - 1], right[0])
    let rightFwd = true
    if (!leftFwd) left = [...left].reverse()
    const r0 = right[0]
    const r1 = right[right.length - 1]
    const lm0 = left[Math.floor(left.length / 2)]
    const rm0 = right[Math.floor(right.length / 2)]
    const side = (r1[0] - r0[0]) * (lm0[1] - rm0[1]) - (r1[1] - r0[1]) * (lm0[0] - rm0[0])
    if (side < 0) {
      left = [...left].reverse()
      right = [...right].reverse()
      leftFwd = !leftFwd
      rightFwd = false
    }
    use(lm.ref, r.id, leftFwd)
    use(rm.ref, r.id, rightFwd)
    lanelets.push({ id: r.id, left, right, centerline: centerlineOf(left, right), tags: r.tags, successors: [] })
    for (const p of [...left, ...right]) {
      if (p[0] < minX) minX = p[0]
      if (p[1] < minY) minY = p[1]
      if (p[0] > maxX) maxX = p[0]
      if (p[1] > maxY) maxY = p[1]
    }
  }

  const borders: Border[] = []
  for (const [wayId, uses] of usage) {
    const points = wayPoints.get(wayId)!
    const tagged = styleFromTags(osm.ways.get(wayId)?.tags ?? {})
    if (tagged) {
      borders.push({ wayId, points, ...tagged })
    } else if (uses.length === 1) {
      borders.push({ wayId, points, kind: 'edge', dashed: false })
    } else if (uses.every((u) => u.forward === uses[0].forward)) {
      borders.push({ wayId, points, kind: 'divider', dashed: true })
    } else {
      borders.push({ wayId, points, kind: 'center', dashed: false })
    }
  }

  // Successors: B follows A when both bounds of B start where the bounds of A end (Lanelet2's rule)
  // A 1 m grid of start points, searched in the 3 x 3 cells around each end point
  const cell = (v: number) => Math.floor(v)
  const starts = new Map<string, Lanelet[]>()
  for (const l of lanelets) {
    const k = `${cell(l.right[0][0])},${cell(l.right[0][1])}`
    const list = starts.get(k) ?? []
    list.push(l)
    starts.set(k, list)
  }
  const JOIN_TOL = 0.2
  for (const a of lanelets) {
    const aEndL = a.left[a.left.length - 1]
    const aEndR = a.right[a.right.length - 1]
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        for (const b of starts.get(`${cell(aEndR[0]) + dx},${cell(aEndR[1]) + dy}`) ?? []) {
          if (b.id !== a.id && dist(b.left[0], aEndL) < JOIN_TOL && dist(b.right[0], aEndR) < JOIN_TOL) {
            a.successors.push(b.id)
          }
        }
      }
    }
  }

  return {
    lanelets,
    byId: new Map(lanelets.map((l) => [l.id, l])),
    borders,
    bounds: { minX, minY, maxX, maxY },
  }
}
