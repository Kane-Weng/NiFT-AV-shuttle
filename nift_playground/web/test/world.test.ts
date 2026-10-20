// The 3D world's files in public/world/ (built by tools/world3d): sizes, licence notice, no brand
// names or textures in the model, and the height grid read back against the lanelet map.

import { describe, expect, it } from 'vitest'
import { readFileSync, readdirSync, statSync } from 'node:fs'
import { gunzipSync } from 'node:zlib'
import path from 'node:path'
import { HeightField, gunzipMaybe } from '../src/scene/heightField'
import { parseOsm } from '../src/map/osm'
import { buildLaneletMap } from '../src/map/lanelets'
import { MapProjector } from '../src/geo/utm'
import { MAP_FILE, MAP_ORIGIN_LAT, MAP_ORIGIN_LON } from '../src/geo/site'

const dir = path.resolve(__dirname, '../public/world')
const manifest = JSON.parse(readFileSync(path.join(dir, 'world.json'), 'utf8'))
const BRAND = /m[\s_-]?city|umich|mdot|planet[\s_-]?m|block[\s_-]?m\b/i

function glbJson(file: string) {
  const b = readFileSync(file)
  expect(b.toString('ascii', 0, 4)).toBe('glTF')
  return JSON.parse(b.subarray(20, 20 + b.readUInt32LE(12)).toString('utf8'))
}

describe('world files', () => {
  it('stay under 5 MB each and about 2 MB in all', () => {
    let total = 0
    for (const f of readdirSync(dir)) {
      const n = statSync(path.join(dir, f)).size
      expect(n, f).toBeLessThan(5 * 1024 * 1024)
      total += n
    }
    expect(total).toBeLessThan(2.5e6)
  })

  it('ship the MIT notice of the twin', () => {
    const notice = readFileSync(path.join(dir, 'NOTICE'), 'utf8')
    expect(notice).toContain('MIT License')
    expect(notice).toContain('Copyright (c) 2024 Quantum Signal AI LLC')
    expect(notice).toContain('github.com/mcity/mcity-digital-twin')
  })

  it('hold no textures and no brand names', () => {
    const gltf = glbJson(path.join(dir, manifest.model))
    expect(gltf.images ?? []).toHaveLength(0)
    expect(gltf.textures ?? []).toHaveLength(0)
    expect(gltf.extensionsUsed).toContain('KHR_draco_mesh_compression')
    const names = [...(gltf.nodes ?? []), ...(gltf.meshes ?? []), ...(gltf.materials ?? []), ...(gltf.accessors ?? [])].map((p) => p.name ?? '')
    expect(names.filter((n) => BRAND.test(n))).toEqual([])
    const { copyright, ...asset } = gltf.asset
    expect(copyright).toContain('MIT')
    expect(JSON.stringify({ ...gltf, asset })).not.toMatch(BRAND)
    const trees = readFileSync(path.join(dir, manifest.trees), 'utf8')
    expect(trees).not.toMatch(BRAND)
  })

  it('draw every tree of the twin at a plausible size', () => {
    const t = JSON.parse(readFileSync(path.join(dir, manifest.trees), 'utf8'))
    let n = 0
    for (const fam of Object.values(t.families) as { rows: number[][] }[]) {
      for (const [, , , h, w] of fam.rows) {
        expect(h).toBeGreaterThan(0.1)
        expect(h).toBeLessThan(25)
        expect(w).toBeLessThan(20)
      }
      n += fam.rows.length
    }
    expect(n).toBe(2009)
  })
})

describe('height grid', () => {
  const raw = readFileSync(path.join(dir, manifest.height.file))
  const field = HeightField.fromDeltas(manifest.height, gunzipSync(raw).buffer.slice(0) as ArrayBuffer)

  it('inflates in the page like in Node', async () => {
    const buf = await gunzipMaybe(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength) as ArrayBuffer)
    const again = HeightField.fromDeltas(manifest.height, buf)
    expect(again.z).toEqual(field.z)
    // already inflated data (a server that sent Content-Encoding) passes through
    expect(await gunzipMaybe(buf)).toBe(buf)
  })

  it('interpolates between grid points and holds at the edge', () => {
    const { x0, y0, cell, nx } = manifest.height
    const z = (i: number, j: number) => field.z[j * nx + i]
    expect(field.at(x0 + 10 * cell, y0 + 20 * cell)).toBeCloseTo(z(10, 20), 5)
    expect(field.at(x0 + 10.5 * cell, y0 + 20 * cell)).toBeCloseTo((z(10, 20) + z(11, 20)) / 2, 5)
    expect(field.at(x0 - 50, y0 - 50)).toBeCloseTo(z(0, 0), 5)
  })

  it('covers the lanelet map, with a gentle ground under its lanes', () => {
    const text = readFileSync(path.resolve(__dirname, '../../../maps', MAP_FILE), 'utf8')
    const map = buildLaneletMap(parseOsm(text), new MapProjector(MAP_ORIGIN_LAT, MAP_ORIGIN_LON))
    const { x0, y0, cell, nx, ny } = manifest.height
    const b = map.bounds
    expect(b.minX).toBeGreaterThan(x0)
    expect(b.minY).toBeGreaterThan(y0)
    expect(b.maxX).toBeLessThan(x0 + (nx - 1) * cell)
    expect(b.maxY).toBeLessThan(y0 + (ny - 1) * cell)
    // The road rises about 13 m over the site, but never in steps: < 25 % between centreline points 1 m apart
    let worst = 0
    for (const l of map.lanelets) {
      const c = l.centerline
      for (let i = 1; i < c.length; i++) {
        const d = Math.hypot(c[i][0] - c[i - 1][0], c[i][1] - c[i - 1][1])
        if (d < 0.5) continue
        worst = Math.max(worst, Math.abs(field.at(c[i][0], c[i][1]) - field.at(c[i - 1][0], c[i - 1][1])) / d)
      }
    }
    expect(worst).toBeLessThan(0.25)
    // z0 is the median road height under the lanelet nodes, so the map sits around 0
    const zs = map.lanelets.flatMap((l) => l.centerline.map(([x, y]) => field.at(x, y))).sort((a, b) => a - b)
    expect(Math.abs(zs[Math.floor(zs.length / 2)])).toBeLessThan(1.0)
  })
})
