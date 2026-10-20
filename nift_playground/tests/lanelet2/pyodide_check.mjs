// File: pyodide_check.mjs
//
// Description:
//   Runs the Lanelet2 stand-in in Pyodide under Node: map load time, centerline time, and the same
//   strict equivalence check as test_equivalence.py against the saved real-Lanelet2 fixtures. Pyodide's
//   C library (musl) rounds sin, cos and friends a little differently from glibc, so this shows what
//   that does to the results in the browser. Exits with status 1 if any check fails, a nearest-lanelet
//   tie picked differently included.
//
//   node nift_playground/tests/lanelet2/pyodide_check.mjs
//   (needs Pyodide in nift_playground/tests/pyodide/node_modules: npm install there first)

import { createRequire } from 'node:module'
import { readFileSync, readdirSync, statSync } from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLAYGROUND = path.resolve(HERE, '../..')
const REPO = path.resolve(PLAYGROUND, '..')
const require = createRequire(path.join(PLAYGROUND, 'tests/pyodide/package.json'))
const { loadPyodide } = require('pyodide')

function copyTree(py, src, dst) {
  py.FS.mkdirTree(dst)
  for (const name of readdirSync(src)) {
    if (name === '__pycache__') continue
    const s = path.join(src, name)
    if (statSync(s).isDirectory()) copyTree(py, s, `${dst}/${name}`)
    else py.FS.writeFile(`${dst}/${name}`, readFileSync(s))
  }
}

const py = await loadPyodide()
copyTree(py, path.join(PLAYGROUND, 'py/fakelanelet2'), '/nift/nift_playground/py/fakelanelet2')
copyTree(py, HERE, '/nift/nift_playground/tests/lanelet2')
py.FS.mkdirTree('/nift/maps')
py.FS.writeFile('/nift/maps/lanelet2_odr2024.osm', readFileSync(path.join(REPO, 'maps/lanelet2_odr2024.osm')))

const out = py.runPython(`
import sys, time, json
sys.path[:0] = ['/nift/nift_playground/py/fakelanelet2', '/nift/nift_playground/tests/lanelet2']
import lanelet2, equivalence as eq, lanelet2_dump
from lanelet2.projection import UtmProjector
t0 = time.perf_counter()
m = lanelet2.io.load('/nift/maps/lanelet2_odr2024.osm', UtmProjector(lanelet2.io.Origin(42.3005, -83.6987)))
t1 = time.perf_counter()
n = sum(len(ll.centerline) for ll in m.laneletLayer)
t2 = time.perf_counter()
res = {'python': sys.version.split()[0], 'load_s': t1 - t0, 'centerlines_s': t2 - t1}
for name in sorted(eq.MAPS):
    fx = eq.load_fixture(name)
    q = dict(fx['queries']); q['map'] = eq.map_path(name)
    stand = lanelet2_dump.dump(lanelet2, q)
    stats, fails = eq.compare(fx['real'], stand, fx['queries'], eq._CostOracle(lanelet2, fx['queries']), strict=True)
    res[name] = {'stats': stats, 'n_failures': len(fails), 'failures': fails[:10], 'timing': stand['timing']}
res['node_cache_used'] = bool(m._node_cache_used)
json.dumps(res)
`)
const res = JSON.parse(out)
console.log(JSON.stringify(res, null, 1))
const failed = Object.entries(res).filter(([, v]) => v && typeof v === 'object' && v.n_failures > 0)
if (failed.length) {
  console.error(`FAIL: ${failed.map(([k, v]) => `${k} (${v.n_failures} failures)`).join(', ')}`)
  process.exitCode = 1
} else {
  console.log('OK: the stand-in matches the real Lanelet2 fixtures under Pyodide (strict)')
}
