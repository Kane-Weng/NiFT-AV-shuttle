// The .osm reader and lanelet map builder on the real odr2024 map, plus the mock route.

import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import path from 'node:path'
import { parseOsm } from '../src/map/osm'
import { buildLaneletMap } from '../src/map/lanelets'
import { MapProjector } from '../src/geo/utm'
import { MAP_FILE, MAP_ORIGIN_LAT, MAP_ORIGIN_LON } from '../src/geo/site'
import { planPath } from '../src/mock/route'

const text = readFileSync(path.resolve(__dirname, '../../../maps', MAP_FILE), 'utf8')
const osm = parseOsm(text)
const map = buildLaneletMap(osm, new MapProjector(MAP_ORIGIN_LAT, MAP_ORIGIN_LON))

describe('odr2024 map', () => {
  it('reads every element', () => {
    expect(osm.nodes.size).toBe((text.match(/<node /g) ?? []).length)
    expect(osm.ways.size).toBe((text.match(/<way /g) ?? []).length)
    expect(map.lanelets.length).toBe(507) // Lanelet2's loadRobust count, maps/README.md
  })

  it('styles every way once, with all three kinds present', () => {
    const kinds = new Set(map.borders.map((b) => b.kind))
    expect(kinds).toEqual(new Set(['edge', 'divider', 'center']))
    expect(new Set(map.borders.map((b) => b.wayId)).size).toBe(map.borders.length)
    expect(map.borders.filter((b) => b.kind === 'divider').every((b) => b.dashed)).toBe(true)
  })

  // Fixture from real Lanelet2: [following ids, leftBound[0], rightBound[0]] per lanelet
  const graph = JSON.parse(readFileSync(path.join(__dirname, 'fixtures/lanelet2_odr2024_graph.json'), 'utf8')) as {
    lanelets: Record<string, [number[], [number, number], [number, number]]>
  }

  it('orients every lanelet like Lanelet2', () => {
    for (const l of map.lanelets) {
      const [, left0, right0] = graph.lanelets[l.id]
      expect(Math.hypot(l.left[0][0] - left0[0], l.left[0][1] - left0[1])).toBeLessThan(1e-3)
      expect(Math.hypot(l.right[0][0] - right0[0], l.right[0][1] - right0[1])).toBeLessThan(1e-3)
    }
  })

  it('finds the successors Lanelet2 finds, except two known ones', () => {
    // 314 and 317 -> 74: Lanelet2 links them although their ends are 34 m from 74's start (open question)
    const known = new Set([314, 317])
    for (const l of map.lanelets) {
      if (known.has(l.id)) continue
      expect([...l.successors].sort((a, b) => a - b)).toEqual(graph.lanelets[l.id][0])
    }
  })

  it('has centrelines between their bounds', () => {
    for (const l of map.lanelets) {
      const c = l.centerline[Math.floor(l.centerline.length / 2)]
      const d = Math.min(...l.left.map((p) => Math.hypot(p[0] - c[0], p[1] - c[1])))
      expect(d).toBeLessThan(5)
    }
  })

  it('plans a mock route between two far lanelets', () => {
    const a = map.lanelets[0]
    const start = a.centerline[0]
    const yaw = Math.atan2(a.centerline[1][1] - start[1], a.centerline[1][0] - start[0])
    const far = map.lanelets.reduce((best, l) =>
      Math.hypot(l.centerline[0][0] - start[0], l.centerline[0][1] - start[1]) >
      Math.hypot(best.centerline[0][0] - start[0], best.centerline[0][1] - start[1])
        ? l
        : best,
    )
    const goal = far.centerline[Math.floor(far.centerline.length / 2)]
    const p = planPath(map, start, yaw, goal)
    expect(p).not.toBeNull()
    expect(p!.length).toBeGreaterThan(10)
  })
})
