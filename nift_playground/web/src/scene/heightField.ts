/*
 * File: heightField.ts
 *
 * Description:
 *   The ground height of the 3D world, a grid baked by tools/world3d (1 m cells over the site, in
 *   the map frame, metres relative to the world's z0). The page lifts its lanes, path, goal,
 *   obstacles and the shuttle onto it when the world is shown. The stack itself stays 2D.
 *
 *   File format (world.json "height"): gzip of int16 little endian, row-major from (x0, y0), +x
 *   along a row. Each value is the height in centimetres minus the previous value in its row
 *   (the first one against 0), which gzips to about half.
 */

export interface HeightMeta {
  x0: number
  y0: number
  cell: number
  nx: number
  ny: number
  scale: number
}

/** Inflates gzip data. Servers that already sent it with Content-Encoding pass it through as is. */
export async function gunzipMaybe(buf: ArrayBuffer): Promise<ArrayBuffer> {
  const b = new Uint8Array(buf, 0, Math.min(2, buf.byteLength))
  if (b[0] !== 0x1f || b[1] !== 0x8b) return buf
  const stream = new Blob([buf]).stream().pipeThrough(new DecompressionStream('gzip'))
  return new Response(stream).arrayBuffer()
}

export type Vec3 = { x: number; y: number; z: number }

export class HeightField {
  private stats: { lo: number; hi: number; slope: number } | null = null

  constructor(
    readonly meta: HeightMeta,
    readonly z: Float32Array,
  ) {}

  /** Lowest and highest grid height, and the steepest slope of the bilinear ground (rise per metre). */
  shape(): { lo: number; hi: number; slope: number } {
    if (!this.stats) {
      const { nx, ny, cell } = this.meta
      const z = this.z
      let lo = Infinity
      let hi = -Infinity
      let slope = 0
      for (let j = 0; j < ny; j++) {
        for (let i = 0; i < nx; i++) {
          const k = j * nx + i
          lo = Math.min(lo, z[k])
          hi = Math.max(hi, z[k])
          if (i + 1 < nx && j + 1 < ny) {
            const gx = Math.max(Math.abs(z[k + 1] - z[k]), Math.abs(z[k + nx + 1] - z[k + nx]))
            const gy = Math.max(Math.abs(z[k + nx] - z[k]), Math.abs(z[k + nx + 1] - z[k + 1]))
            slope = Math.max(slope, Math.hypot(gx, gy) / cell)
          }
        }
      }
      this.stats = { lo, hi, slope }
    }
    return this.stats
  }

  /**
   * First point where a ray meets the ground raised by lift, or null when it misses (it points
   * up, starts below the ground or leaves the height band). Only the height grid counts, so
   * buildings, bridges and trees never catch a click.
   *
   * The walk can't step over the ground. Each step is the ray's height above the ground divided
   * by the fastest that height can shrink (the ray's own descent plus the steepest slope). So a
   * ray that only grazes a crest still stops there, within 0.1 mm of the ground.
   */
  rayHit(o: Vec3, d: Vec3, lift = 0): Vec3 | null {
    if (d.z >= 0) return null
    const { lo, hi, slope } = this.shape()
    const above = (t: number) => o.z + t * d.z - (this.at(o.x + t * d.x, o.y + t * d.y) + lift)
    const rate = -d.z + slope * Math.hypot(d.x, d.y)
    const tEnd = (lo + lift - 0.01 - o.z) / d.z
    let t = Math.max(0, (hi + lift + 0.01 - o.z) / d.z)
    if (above(t) < 0) return null // the eye is under the ground
    for (let n = 0; n < 200000 && t <= tEnd; n++) {
      const a = above(t)
      if (a <= 1e-4) return { x: o.x + t * d.x, y: o.y + t * d.y, z: o.z + t * d.z }
      t += a / rate
    }
    return null
  }

  /** Builds the field from the inflated file contents. */
  static fromDeltas(meta: HeightMeta, raw: ArrayBuffer): HeightField {
    const { nx, ny, scale } = meta
    if (raw.byteLength !== nx * ny * 2) throw new Error(`height grid: ${raw.byteLength} bytes, expected ${nx * ny * 2}`)
    const d = new DataView(raw)
    const z = new Float32Array(nx * ny)
    for (let j = 0; j < ny; j++) {
      let acc = 0
      for (let i = 0; i < nx; i++) {
        const k = j * nx + i
        acc += d.getInt16(2 * k, true)
        z[k] = acc * scale
      }
    }
    return new HeightField(meta, z)
  }

  /** Ground height at a map point, bilinear between grid points, held at the grid's edge. */
  at(x: number, y: number): number {
    const { x0, y0, cell, nx, ny } = this.meta
    const fx = Math.min(Math.max((x - x0) / cell, 0), nx - 1)
    const fy = Math.min(Math.max((y - y0) / cell, 0), ny - 1)
    const i = Math.min(Math.floor(fx), nx - 2)
    const j = Math.min(Math.floor(fy), ny - 2)
    const u = fx - i
    const v = fy - j
    const z = this.z
    const a = z[j * nx + i]
    const b = z[j * nx + i + 1]
    const c = z[(j + 1) * nx + i]
    const e = z[(j + 1) * nx + i + 1]
    return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + e * u) * v
  }
}
