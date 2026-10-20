// The TS projection against real Lanelet2 (fixture made with UtmProjector in nift_pub_deps:humble).

import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import path from 'node:path'
import { MapProjector, utmZone } from '../src/geo/utm'
import { MAP_ORIGIN_LAT, MAP_ORIGIN_LON } from '../src/geo/site'

const fixture = JSON.parse(readFileSync(path.join(__dirname, 'fixtures/utm_lanelet2.json'), 'utf8')) as {
  origin: [number, number]
  points: [number, number, number, number][]
}

describe('MapProjector', () => {
  const proj = new MapProjector(MAP_ORIGIN_LAT, MAP_ORIGIN_LON)

  it('uses the origin of the fixture and zone 17', () => {
    expect(fixture.origin).toEqual([MAP_ORIGIN_LAT, MAP_ORIGIN_LON])
    expect(utmZone(MAP_ORIGIN_LON)).toBe(17)
  })

  it('matches Lanelet2 within 1 mm on map nodes and far points', () => {
    let worst = 0
    for (const [lat, lon, x, y] of fixture.points) {
      const [px, py] = proj.toMapXY(lat, lon)
      worst = Math.max(worst, Math.hypot(px - x, py - y))
    }
    expect(worst).toBeLessThan(1e-3)
  })

  it('puts the origin at zero', () => {
    const [x, y] = proj.toMapXY(MAP_ORIGIN_LAT, MAP_ORIGIN_LON)
    expect(Math.abs(x) + Math.abs(y)).toBe(0)
  })
})
