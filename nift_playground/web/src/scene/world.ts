/*
 * File: world.ts
 *
 * Description:
 *   The optional 3D world under the map, loaded on demand (the page imports this module the
 *   first time someone turns the world on). It holds the twin's static world in flat colours
 *   (world.glb, Draco compressed, instanced), the ground height grid, and our own low-poly trees
 *   at the twin's plant positions. tools/world3d builds the files; public/world/NOTICE carries
 *   the twin's MIT notice.
 *
 *   Everything is returned in the map frame (x east, y north, z up), heights relative to the
 *   world's z0, so the page can lift its 2D drawing onto the ground with height.at(x, y).
 */

import * as THREE from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js'
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js'
import { HeightField, gunzipMaybe, type HeightMeta } from './heightField'

export interface WorldManifest {
  model: string
  trees: string
  height: HeightMeta & { file: string; z0_elevation_m: number }
  bytes: Record<string, number>
  stats: { triangles: number; drawCalls: number }
}

type TreeFamily = 'conifer' | 'broadleaf' | 'shrub' | 'grass'
interface TreesFile {
  families: Record<TreeFamily, { species: string[]; rows: [number, number, number, number, number, number][] }>
}

export interface LoadedWorld {
  /** The twin's static world and the trees, in the map frame. */
  root: THREE.Group
  /** The trees alone, so the page can hide them. */
  trees: THREE.Group
  height: HeightField
  manifest: WorldManifest
  /** Bytes fetched for the world (model, height grid, trees, decoder). */
  bytes: number
  dispose(): void
}

export type Progress = (fraction: number, label: string) => void

/** The Draco decoder from the three package, served next to the world files (see vite.config.ts). */
export function worldBaseUrl(): string {
  return new URL(`${import.meta.env.BASE_URL}world/`, window.location.href).href
}

async function fetchBytes(url: string, onBytes: (n: number) => void): Promise<ArrayBuffer> {
  const res = await fetch(url)
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`)
  if (!res.body) return res.arrayBuffer()
  const reader = res.body.getReader()
  const parts: Uint8Array[] = []
  let n = 0
  for (;;) {
    const { done, value } = await reader.read()
    if (done) break
    parts.push(value)
    n += value.byteLength
    onBytes(value.byteLength)
  }
  const out = new Uint8Array(n)
  let o = 0
  for (const p of parts) {
    out.set(p, o)
    o += p.byteLength
  }
  return out.buffer
}

// ------------------------------------------------------------------ trees (our own shapes)

// Colours per family [crown, trunk], with a small deterministic variation per tree
const TREE_COLOURS: Record<TreeFamily, [number, number]> = {
  conifer: [0x2f5d3a, 0x5b4632],
  broadleaf: [0x4c7a34, 0x6b5238],
  shrub: [0x5a7d3c, 0x5a7d3c],
  grass: [0x6d8f45, 0x6d8f45],
}

function painted(geo: THREE.BufferGeometry, color: number): THREE.BufferGeometry {
  const g = geo.index ? geo.toNonIndexed() : geo
  const c = new THREE.Color(color)
  const n = g.getAttribute('position').count
  const col = new Float32Array(n * 3)
  for (let i = 0; i < n; i++) col.set([c.r, c.g, c.b], 3 * i)
  g.setAttribute('color', new THREE.BufferAttribute(col, 3))
  g.deleteAttribute('uv')
  g.deleteAttribute('normal')
  return g
}

/** One unit-sized shape per family: 1 m wide, 1 m tall, base at z = 0, scaled per tree. */
function treeShape(family: TreeFamily): THREE.BufferGeometry {
  const [crown, trunk] = TREE_COLOURS[family]
  const up = (g: THREE.BufferGeometry) => g.rotateX(Math.PI / 2) // three's shapes stand on y
  let parts: THREE.BufferGeometry[]
  if (family === 'conifer') {
    parts = [
      painted(up(new THREE.CylinderGeometry(0.05, 0.07, 0.25, 6)).translate(0, 0, 0.125), trunk),
      painted(up(new THREE.ConeGeometry(0.5, 0.85, 8)).translate(0, 0, 0.15 + 0.425), crown),
    ]
  } else if (family === 'broadleaf') {
    parts = [
      painted(up(new THREE.CylinderGeometry(0.05, 0.07, 0.5, 6)).translate(0, 0, 0.25), trunk),
      painted(new THREE.IcosahedronGeometry(0.5, 1).scale(1, 1, 0.84).translate(0, 0, 0.58), crown),
    ]
  } else if (family === 'shrub') {
    parts = [painted(new THREE.IcosahedronGeometry(0.5, 1).translate(0, 0, 0.45), crown)]
  } else {
    parts = [painted(up(new THREE.CylinderGeometry(0.5, 0.5, 1, 10, 1)).translate(0, 0, 0.5), crown)]
  }
  const g = mergeGeometries(parts)
  g.computeVertexNormals()
  return g
}

function hash01(i: number): number {
  const x = Math.sin(i * 12.9898 + 78.233) * 43758.5453
  return x - Math.floor(x)
}

function buildTrees(data: TreesFile, height: HeightField): THREE.Group {
  const group = new THREE.Group()
  group.name = 'trees'
  const m = new THREE.Matrix4()
  const q = new THREE.Quaternion()
  const s = new THREE.Vector3()
  const p = new THREE.Vector3()
  const z = new THREE.Vector3(0, 0, 1)
  const col = new THREE.Color()
  for (const family of ['conifer', 'broadleaf', 'shrub', 'grass'] as TreeFamily[]) {
    const fam = data.families[family]
    if (!fam || fam.rows.length === 0) continue
    const mat = new THREE.MeshLambertMaterial({ vertexColors: true, flatShading: true })
    const mesh = new THREE.InstancedMesh(treeShape(family), mat, fam.rows.length)
    mesh.name = `trees_${family}`
    fam.rows.forEach(([x, y, yaw, h, w], i) => {
      // Sunk a little so a crown on a slope never floats
      p.set(x, y, height.at(x, y) - 0.05 * h)
      q.setFromAxisAngle(z, yaw)
      s.set(w, w, h)
      mesh.setMatrixAt(i, m.compose(p, q, s))
      const k = 0.85 + 0.3 * hash01(i + family.length * 7919)
      mesh.setColorAt(i, col.setRGB(k, k, k))
    })
    mesh.instanceMatrix.needsUpdate = true
    mesh.computeBoundingSphere()
    group.add(mesh)
  }
  return group
}

// ------------------------------------------------------------------ the world

/** Turns the file's PBR materials into cheap Lambert ones, pushed back so lines drawn on the ground win. */
function prepare(model: THREE.Object3D) {
  const cache = new Map<THREE.Material, THREE.Material>()
  model.traverse((o) => {
    const mesh = o as THREE.Mesh
    if (!mesh.isMesh) return
    mesh.castShadow = false
    mesh.receiveShadow = false
    const src = mesh.material as THREE.MeshStandardMaterial
    let dst = cache.get(src)
    if (!dst) {
      const lm = new THREE.MeshLambertMaterial({
        color: src.color,
        emissive: src.emissive,
        transparent: src.transparent,
        opacity: src.opacity,
        side: src.side,
        depthWrite: !src.transparent,
        polygonOffset: true,
        polygonOffsetFactor: 1,
        polygonOffsetUnits: 2,
      })
      lm.name = src.name
      cache.set(src, lm)
      src.dispose()
      dst = lm
    }
    mesh.material = dst
  })
}

export async function loadWorld(onProgress: Progress = () => {}): Promise<LoadedWorld> {
  const base = worldBaseUrl()
  const res = await fetch(`${base}world.json`)
  if (!res.ok) throw new Error(`world.json: HTTP ${res.status}`)
  const manifest = (await res.json()) as WorldManifest
  const total = (manifest.bytes[manifest.model] ?? 0) + (manifest.bytes[manifest.height.file] ?? 0) + (manifest.bytes[manifest.trees] ?? 0)
  let got = 0
  const tick = (n: number) => {
    got += n
    onProgress(Math.min(1, got / Math.max(1, total)), 'Loading world')
  }
  onProgress(0, 'Loading world')

  const draco = new DRACOLoader()
  draco.setDecoderPath(`${base}draco/`)
  draco.setDecoderConfig({ type: 'wasm' })
  const loader = new GLTFLoader()
  loader.setDRACOLoader(draco)

  try {
    const [glbBuf, heightBuf, treesBuf] = await Promise.all([
      fetchBytes(`${base}${manifest.model}`, tick),
      fetchBytes(`${base}${manifest.height.file}`, tick),
      fetchBytes(`${base}${manifest.trees}`, tick),
    ])
    onProgress(1, 'Decoding world')
    const height = HeightField.fromDeltas(manifest.height, await gunzipMaybe(heightBuf))
    const trees = JSON.parse(new TextDecoder().decode(await gunzipMaybe(treesBuf))) as TreesFile
    const gltf = await loader.parseAsync(glbBuf, base)

    const root = new THREE.Group()
    root.name = 'world'
    const model = gltf.scene
    model.rotation.x = Math.PI / 2 // glTF is y up, the map frame z up
    prepare(model)
    root.add(model)
    const treeGroup = buildTrees(trees, height)
    root.add(treeGroup)
    root.updateMatrixWorld(true)
    return {
      root,
      trees: treeGroup,
      height,
      manifest,
      bytes: glbBuf.byteLength + heightBuf.byteLength + treesBuf.byteLength,
      dispose() {
        root.traverse((o) => {
          const mesh = o as THREE.Mesh
          if (!mesh.isMesh) return
          mesh.geometry.dispose()
          ;(Array.isArray(mesh.material) ? mesh.material : [mesh.material]).forEach((m) => m.dispose())
        })
      },
    }
  } finally {
    draco.dispose()
  }
}
