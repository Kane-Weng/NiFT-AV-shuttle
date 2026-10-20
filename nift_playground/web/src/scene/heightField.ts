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

export class HeightField {
  constructor(
    readonly meta: HeightMeta,
    readonly z: Float32Array,
  ) {}

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
