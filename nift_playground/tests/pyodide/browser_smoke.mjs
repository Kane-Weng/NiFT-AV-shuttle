// File: browser_smoke.mjs
//
// Description:
//   The Pyodide worker in a real browser. Builds the Python bundle into web/public/py, starts the
//   Vite dev server of web/, opens web/src/worker/smoke.html in headless Chrome (web's puppeteer)
//   and waits until the worker has booted Pyodide from the CDN, planned route 04 and arrived.
//   Prints boot times, the achieved speed and the tab's memory.
//
//   cd nift_playground/web && npm ci; then from tests/pyodide: node browser_smoke.mjs [--port 5199]
//   Needs network for the Pyodide CDN, as the page does.

import { execFileSync } from 'node:child_process'
import { createRequire } from 'node:module'
import path from 'node:path'
import { PLAYGROUND, PYTHON } from './node_env.mjs'

const WEB = path.join(PLAYGROUND, 'web')
const args = process.argv.slice(2)
const PORT = Number(args.includes('--port') ? args[args.indexOf('--port') + 1] : 5199)
const webRequire = createRequire(path.join(WEB, 'package.json'))

execFileSync(PYTHON, [path.join(PLAYGROUND, 'tools/build_bundle.py')], { stdio: 'inherit' })

const { createServer } = await import(webRequire.resolve('vite'))
const puppeteer = (await import(webRequire.resolve('puppeteer'))).default
const server = await createServer({ root: WEB, configFile: path.join(WEB, 'vite.config.ts'), server: { port: PORT, strictPort: true }, logLevel: 'warn' })
await server.listen()
let code = 1
const browser = await puppeteer.launch({ headless: true, args: ['--enable-precise-memory-info'] })
try {
  const page = await browser.newPage()
  page.on('console', (m) => process.env.NIFT_VERBOSE && console.log(`[page] ${m.text()}`))
  const t0 = Date.now()
  await page.goto(`http://localhost:${PORT}/src/worker/smoke.html`)
  await page.waitForFunction('window.__smoke && window.__smoke.done', { timeout: 300000, polling: 500 })
  const r = await page.evaluate(() => {
    const s = window.__smoke
    const ready = s.msgs.find((m) => m.type === 'ready')
    const states = s.msgs.filter((m) => m.type === 'state')
    const last = states.at(-1)
    const arrived = s.msgs.find((m) => m.type === 'event' && m.kind === 'arrived')
    const start = s.msgs.find((m) => m.type === 'reply' && m.id === 's')
    const drive = arrived && start ? { sim_s: arrived.t - 2.0, wall_s: (arrived._wall - start._wall) / 1e3 } : null
    return {
      error: s.error,
      readyS: ready ? (ready._wall - s.t0) / 1e3 : null,
      bootMs: ready?.bootMs,
      pyodide: ready?.pyodide,
      python: ready?.python,
      status: last?.status,
      pose: last?.pose,
      maxRtf: Math.max(...states.map((x) => x.rtf ?? 0)),
      drive,
      jsHeapMb: s.jsHeapMb ?? null,
    }
  })
  const metrics = await page.metrics()
  const cdp = await page.createCDPSession()
  let workerMb = null
  try {
    // The tab's whole heap (page and worker); the worker holds the WebAssembly memory.
    await cdp.send('Memory.forciblyPurgeJavaScriptMemory').catch(() => {})
    const proc = await browser.process()
    if (proc?.pid) {
      const out = execFileSync('ps', ['-o', 'rss=', '--ppid', String(proc.pid)], { encoding: 'utf8' })
      workerMb = out.trim().split('\n').map(Number).reduce((a, b) => a + b, 0) / 1024
    }
  } catch {}
  console.log(JSON.stringify({ ...r, wall_total_s: (Date.now() - t0) / 1e3, pageJsHeapMb: metrics.JSHeapUsedSize / 1e6,
                               chromeChildrenRssMb: workerMb }, null, 1))
  code = r.error || r.status !== 'IDLE' ? 1 : 0
  console.log(code === 0 ? 'BROWSER OK' : 'BROWSER FAILED')
} finally {
  await browser.close()
  await server.close()
}
process.exit(code)
