// File: node_env.mjs
//
// Description:
//   The Node side of the Pyodide tests. It builds the Python bundle with tools/build_bundle.py,
//   loads Pyodide 0.27.7 from this folder's node_modules (npm install here first) and makes the
//   WorkerEnv that web/src/worker/core.ts expects, so the tests run the browser worker's own code.
//   Node 22.18 or newer loads core.ts with its built-in type stripping.
//
//   numpy and pyyaml come from the Pyodide CDN on the first run and are cached in
//   node_modules/pyodide after that (CI caches that folder).

import { createRequire } from 'node:module'
import { execFileSync } from 'node:child_process'
import { mkdtempSync, readFileSync } from 'node:fs'
import { tmpdir } from 'node:os'
import path from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

export const HERE = path.dirname(fileURLToPath(import.meta.url))
export const PLAYGROUND = path.resolve(HERE, '../..')
export const REPO = path.resolve(PLAYGROUND, '..')
export const MAP_FILE = path.join(REPO, 'maps/lanelet2_odr2024.osm')
export const PYTHON = process.env.PYTHON ?? 'python3'

const require = createRequire(import.meta.url)

export async function loadCore() {
  return import(pathToFileURL(path.join(PLAYGROUND, 'web/src/worker/core.ts')).href)
}

/** Build the bundle into a fresh temp folder. Returns the folder that holds py/nift_bundle.zip. */
export function buildBundle() {
  const base = mkdtempSync(path.join(tmpdir(), 'nift-bundle-'))
  execFileSync(PYTHON, [path.join(PLAYGROUND, 'tools/build_bundle.py'), '--out', path.join(base, 'py')], {
    stdio: ['ignore', 'ignore', 'inherit'],
  })
  return base
}

/** Pyodide from node_modules, quiet unless NIFT_VERBOSE is set. */
export async function loadPyodideNode() {
  const { loadPyodide } = require('pyodide')
  const verbose = !!process.env.NIFT_VERBOSE
  return loadPyodide({ stdout: verbose ? console.log : () => {}, stderr: verbose ? console.error : () => {} })
}

/**
 * The WorkerEnv for core.ts. URLs are file:// URLs or plain paths. Messages the worker posts go to
 * onPost. soon() uses setImmediate, which, like the browser's MessageChannel, lets queued work run.
 */
export function nodeEnv(onPost, { log = () => {} } = {}) {
  return {
    loadPyodide: loadPyodideNode,
    async fetchBytes(url) {
      const p = url.startsWith('file:') ? fileURLToPath(url) : url
      return new Uint8Array(readFileSync(p))
    },
    post: (m) => onPost(structuredClone(m)),
    now: () => performance.now(),
    later: (fn, ms) => void setTimeout(fn, ms),
    soon: (fn) => void setImmediate(fn),
    log,
  }
}

export function fileUrl(p) {
  return pathToFileURL(p).href
}
