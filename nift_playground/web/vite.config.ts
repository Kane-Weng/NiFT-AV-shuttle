/*
 * File: vite.config.ts
 *
 * Description:
 *   Vite build of the playground page. The Lanelet2 map is not copied into this folder. The
 *   plugin below serves the repo's maps/ file at <base>maps/<file> in dev and preview, and emits
 *   it into dist/maps/ on build, so git holds the map once.
 *
 *   PLAYGROUND_BASE sets the base path (for example /NiFT-AV-shuttle/ on GitHub Pages).
 */

import { defineConfig, type Plugin } from 'vite'
import react from '@vitejs/plugin-react'
import { readFileSync, createReadStream, statSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

const here = path.dirname(fileURLToPath(import.meta.url))
const MAPS_DIR = path.resolve(here, '../../maps')
const MAPS = ['lanelet2_odr2024.osm']

function repoMaps(): Plugin {
  const serve = (base: string) => (req: { url?: string }, res: import('node:http').ServerResponse, next: () => void) => {
    const url = (req.url ?? '').split('?')[0]
    const name = MAPS.find((m) => url === `${base}maps/${m}`)
    if (!name) return next()
    const file = path.join(MAPS_DIR, name)
    res.setHeader('Content-Type', 'application/xml')
    res.setHeader('Content-Length', statSync(file).size)
    createReadStream(file).pipe(res)
  }
  let base = '/'
  return {
    name: 'nift-repo-maps',
    configResolved(c) {
      base = c.base.startsWith('.') ? '/' : c.base // a relative base serves from the root in dev
    },
    configureServer(server) {
      server.middlewares.use(serve(base))
    },
    configurePreviewServer(server) {
      server.middlewares.use(serve(base))
    },
    generateBundle() {
      for (const m of MAPS) {
        this.emitFile({ type: 'asset', fileName: `maps/${m}`, source: readFileSync(path.join(MAPS_DIR, m)) })
      }
    },
  }
}

export default defineConfig({
  base: process.env.PLAYGROUND_BASE ?? './',
  plugins: [react(), repoMaps()],
  worker: { format: 'es' },
  build: { target: 'es2022', chunkSizeWarningLimit: 1500 },
  server: { fs: { allow: [here, MAPS_DIR] } },
})
