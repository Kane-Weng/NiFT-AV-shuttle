/*
 * File: route.ts
 *
 * Description:
 *   Shortest lanelet route for the mock worker only. The real planner is waypoint_node on the
 *   Lanelet2 stand-in. This is just enough to give the mock shuttle a believable path.
 */

import type { Lanelet, LaneletMap, XY } from '../map/lanelets'

interface Projection {
  lanelet: Lanelet
  index: number // centreline segment index
  dist: number
  heading: number
  point: XY // the projection onto the centreline
}

/** Nearest centreline segment of every lanelet to a point; with a yaw, lanelets facing away are skipped. */
export function nearestLanelet(map: LaneletMap, p: XY, yaw?: number): Projection | null {
  let best: Projection | null = null
  for (const l of map.lanelets) {
    const c = l.centerline
    for (let i = 0; i < c.length - 1; i++) {
      const [ax, ay] = c[i]
      const [bx, by] = c[i + 1]
      const dx = bx - ax
      const dy = by - ay
      const len2 = dx * dx + dy * dy || 1e-9
      const u = Math.max(0, Math.min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / len2))
      const d = Math.hypot(ax + u * dx - p[0], ay + u * dy - p[1])
      const heading = Math.atan2(dy, dx)
      if (yaw !== undefined && Math.cos(heading - yaw) < 0.3) continue
      if (!best || d < best.dist) best = { lanelet: l, index: i, dist: d, heading, point: [ax + u * dx, ay + u * dy] }
    }
  }
  return best
}

function laneletLength(l: Lanelet): number {
  let s = 0
  for (let i = 1; i < l.centerline.length; i++) {
    s += Math.hypot(l.centerline[i][0] - l.centerline[i - 1][0], l.centerline[i][1] - l.centerline[i - 1][1])
  }
  return s
}

/** Dijkstra over successor links from one lanelet to another. Returns lanelet ids or null. */
export function shortestRoute(map: LaneletMap, from: number, to: number): number[] | null {
  const dist = new Map<number, number>([[from, 0]])
  const prev = new Map<number, number>()
  const done = new Set<number>()
  const open = new Set<number>([from])
  while (open.size > 0) {
    let cur = -1
    let curD = Infinity
    for (const id of open) {
      const d = dist.get(id)!
      if (d < curD) {
        curD = d
        cur = id
      }
    }
    open.delete(cur)
    if (cur === to) break
    done.add(cur)
    const l = map.byId.get(cur)!
    for (const s of l.successors) {
      if (done.has(s)) continue
      const nd = curD + laneletLength(map.byId.get(s)!)
      if (nd < (dist.get(s) ?? Infinity)) {
        dist.set(s, nd)
        prev.set(s, cur)
        open.add(s)
      }
    }
  }
  if (from !== to && !prev.has(to)) return null
  const ids = [to]
  while (ids[0] !== from) ids.unshift(prev.get(ids[0])!)
  return ids
}

/** A path along the route's centrelines from the start point's projection to the goal's. */
export function planPath(map: LaneletMap, start: XY, startYaw: number, goal: XY): XY[] | null {
  const a = nearestLanelet(map, start, startYaw)
  const b = nearestLanelet(map, goal)
  if (!a || !b) return null
  let ids: number[] | null
  if (a.lanelet.id === b.lanelet.id && b.index >= a.index) {
    ids = [a.lanelet.id]
  } else {
    // Go round when the goal is behind on the same lanelet: leave through each successor
    ids = null
    if (a.lanelet.id === b.lanelet.id) {
      for (const s of a.lanelet.successors) {
        const r = shortestRoute(map, s, b.lanelet.id)
        if (r && (!ids || r.length + 1 < ids.length)) ids = [a.lanelet.id, ...r]
      }
    } else {
      ids = shortestRoute(map, a.lanelet.id, b.lanelet.id)
    }
  }
  if (!ids) return null
  const pts: XY[] = [start]
  ids.forEach((id, k) => {
    const c = map.byId.get(id)!.centerline
    const from = k === 0 ? a.index + 1 : 1
    const to = k === ids!.length - 1 ? b.index : c.length - 1
    for (let i = from; i <= to; i++) pts.push(c[i])
  })
  pts.push(b.point)
  return pts
}
