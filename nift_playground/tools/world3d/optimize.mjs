// File: optimize.mjs
//
// Description:
//   The glTF step of the playground's 3D world (run by build_world.sh). It takes the raw .glb
//   from export_world.py, shares repeated meshes as GPU instances, joins what can be drawn
//   together, welds, Draco-compresses, strips the twin's brand name from every name and checks
//   the result: no textures, no brand names, under the size limit.
//
//   node optimize.mjs <world_raw.glb> <world.glb> [--stats <stats.json>]

import { readFileSync, writeFileSync } from 'node:fs'
import { NodeIO } from '@gltf-transform/core'
import { ALL_EXTENSIONS } from '@gltf-transform/extensions'
import { dedup, draco, flatten, instance, join, prune, simplify, weld } from '@gltf-transform/functions'
import { MeshoptSimplifier } from 'meshoptimizer'
import draco3d from 'draco3dgltf'

const [input, output] = process.argv.slice(2)
const statsAt = process.argv.indexOf('--stats')
const statsFile = statsAt > 0 ? process.argv[statsAt + 1] : null
const opt = (name, def) => {
  const i = process.argv.indexOf(`--${name}`)
  return i > 0 ? Number(process.argv[i + 1]) : def
}
// Draco quantization bits (defaults are what build_world.sh ships). Positions are quantized over each joined mesh's box (up to ~500 m).
const QPOS = opt('qpos', 14)
const QNORM = opt('qnorm', 8)
// meshoptimizer simplification error, relative to each joined mesh's size (0 skips it)
const SIMPLIFY = opt('simplify', 0.0001)
if (!input || !output) {
  console.error('usage: node optimize.mjs <world_raw.glb> <world.glb> [--stats <file>]')
  process.exit(2)
}

// The same marks export_world.py refuses, checked again on the final file
export const BRAND = /m[\s_-]?city|umich|mdot|planet[\s_-]?m|block[\s_-]?m\b/i
const scrub = (s) => (s ?? '').replace(/m[\s_-]?city[_-]?/gi, '').replace(/umich|mdot|planet[\s_-]?m/gi, '')
const MAX_BYTES = 5 * 1024 * 1024

const io = new NodeIO()
  .registerExtensions(ALL_EXTENSIONS)
  .registerDependencies({ 'draco3d.encoder': await draco3d.createEncoderModule(), 'draco3d.decoder': await draco3d.createDecoderModule() })

const doc = await io.read(input)
const root = doc.getRoot()
const count = () => ({
  nodes: root.listNodes().length,
  meshes: root.listMeshes().length,
  primitives: root.listMeshes().reduce((n, m) => n + m.listPrimitives().length, 0),
  materials: root.listMaterials().length,
})
const before = count()

await doc.transform(
  dedup(),
  instance({ min: 2 }),
  flatten(),
  join({ keepNamed: false }),
  weld(),
  ...(SIMPLIFY > 0 ? [simplify({ simplifier: MeshoptSimplifier, ratio: 0, error: SIMPLIFY, lockBorder: true })] : []),
  prune({ keepLeaves: false }),
)

for (const list of [root.listNodes(), root.listMeshes(), root.listMaterials(), root.listAccessors(), root.listScenes(), root.listBuffers()]) {
  for (const p of list) p.setName(scrub(p.getName()))
}
root.getAsset().generator = 'nift_playground tools/world3d (Blender, glTF-Transform)'
root.getAsset().copyright = 'Derived from the public digital twin by Quantum Signal AI LLC and The Regents of the University of Michigan, MIT License. See NOTICE.'
const after = count()

// Draw calls as three.js will issue them: one per primitive per node (instanced nodes are one each)
let drawCalls = 0
let instances = 0
for (const node of root.listNodes()) {
  const mesh = node.getMesh()
  if (!mesh) continue
  drawCalls += mesh.listPrimitives().length
  const inst = node.getExtension('EXT_mesh_gpu_instancing')
  instances += inst ? inst.getAttribute('TRANSLATION')?.getCount() ?? 1 : 1
}
let triangles = 0
for (const node of root.listNodes()) {
  const mesh = node.getMesh()
  if (!mesh) continue
  const inst = node.getExtension('EXT_mesh_gpu_instancing')
  const k = inst ? inst.getAttribute('TRANSLATION')?.getCount() ?? 1 : 1
  for (const p of mesh.listPrimitives()) triangles += k * ((p.getIndices()?.getCount() ?? p.getAttribute('POSITION').getCount()) / 3)
}

await doc.transform(draco({ method: 'edgebreaker', encodeSpeed: 3, decodeSpeed: 5, quantizePosition: QPOS, quantizeNormal: QNORM }))
await io.write(output, doc)

// ------------------------------------------------------------------ checks on the written file
const bytes = readFileSync(output)
const jsonLen = bytes.readUInt32LE(12)
const gltf = JSON.parse(bytes.subarray(20, 20 + jsonLen).toString('utf8'))
const problems = []
if (bytes.length > MAX_BYTES) problems.push(`${output} is ${bytes.length} bytes, over 5 MB`)
if ((gltf.images ?? []).length || (gltf.textures ?? []).length) problems.push('the file holds images or textures')
const text = JSON.stringify({ ...gltf, asset: { ...gltf.asset, copyright: '' } })
const hit = text.match(BRAND)
if (hit) problems.push(`brand name in the file: "${hit[0]}" near ${text.slice(Math.max(0, hit.index - 60), hit.index + 60)}`)
const stats = { input: before, output: after, drawCalls, instances, triangles, bytes: bytes.length, extensionsUsed: gltf.extensionsUsed ?? [] }
console.log(JSON.stringify(stats))
if (statsFile) writeFileSync(statsFile, JSON.stringify(stats, null, 1))
if (problems.length) {
  console.error('CHECK FAILED\n' + problems.join('\n'))
  process.exit(1)
}
