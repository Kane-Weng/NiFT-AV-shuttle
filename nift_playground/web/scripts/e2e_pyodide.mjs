// File: e2e_pyodide.mjs
//
// Description:
//   End-to-end test of the page on the real stack: our nodes, unchanged, in Pyodide in a Web Worker,
//   in headless Chrome. It builds the page (npm run build: the Python bundle, then Vite), serves
//   dist/ with vite preview, opens the page without ?sim (the real stack is the default) and drives
//   it through its own buttons and the map, as a user would:
//
//     1. boot: the progress overlay shows, the bundle is fetched with its hash as cache key, ready
//     2. route 04: Load the route, click its goal on the map, Start, run at max speed, and wait for
//        the arrival (NAVIGATING then IDLE). The shuttle must have followed the planned path.
//     3. GNSS dropout: on route 04 again, a 3 s dropout while navigating ends in SAFE_STOP; Resume
//        goes back to IDLE, and Start drives on to the goal.
//     4. Stop (SAFE_STOP) and Resume, E-STOP (latches, Resume disabled); pause and step n; set_param refusals; the GNSS
//        fault presets (one flat list with kind); Reset.
//
//   Screenshots (dark and light, while driving) go to --out. Pyodide comes from the jsDelivr CDN, so
//   the test needs network (set VITE_PYODIDE_URL at build time to host it yourself).
//
//   cd nift_playground/web && npm ci && npm run e2e -- [--out <dir>] [--port 4174] [--no-build]
//
// Puppeteer uses its own Chrome (npx puppeteer browsers install chrome) or PUPPETEER_EXECUTABLE_PATH.

import { execFileSync, spawn } from 'node:child_process'
import { mkdirSync } from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import puppeteer from 'puppeteer'
import { closeSettings, openSection as openSettingsSection, toggleTheme } from './settings.mjs'

const here = path.dirname(fileURLToPath(import.meta.url))
const root = path.resolve(here, '..')
const arg = (name, def) => {
  const i = process.argv.indexOf(`--${name}`)
  return i > 0 ? process.argv[i + 1] : def
}
const OUT = path.resolve(arg('out', path.join(os.tmpdir(), 'nift-playground-e2e')))
const PORT = Number(arg('port', 4174))
const BUILD = !process.argv.includes('--no-build')
const ROUTE = arg('route', '04')
mkdirSync(OUT, { recursive: true })

const sleep = (ms) => new Promise((r) => setTimeout(r, ms))
const fail = (msg) => {
  throw new Error(`E2E FAIL: ${msg}`)
}
const log = (s) => console.error(`[e2e] ${s}`)

if (BUILD) execFileSync('npm', ['run', 'build'], { cwd: root, stdio: ['ignore', 'ignore', 'inherit'] })

const viteBin = path.join(root, 'node_modules/vite/bin/vite.js')
const server = spawn(process.execPath, [viteBin, 'preview', '--port', String(PORT), '--strictPort'], { cwd: root, stdio: 'ignore' })
const url = `http://localhost:${PORT}/`
let browser
const errors = []
const result = {}
try {
  for (let i = 0; ; i++) {
    try {
      if ((await fetch(url)).ok) break
    } catch {
      // not up yet
    }
    if (i > 100) fail(`vite preview did not start on ${url}`)
    await sleep(200)
  }

  browser = await puppeteer.launch({ headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] })
  const page = await browser.newPage()
  await page.setViewport({ width: 1400, height: 860 })
  page.on('console', (m) => {
    if (m.type() === 'error') errors.push(m.text())
    if (process.env.NIFT_VERBOSE) log(`[page] ${m.text()}`)
  })
  page.on('pageerror', (e) => errors.push(String(e)))

  // Every worker message, as the page gets it. States are kept small: the fields the checks use.
  await page.evaluateOnNewDocument(() => {
    const W = window.Worker
    const rec = { msgs: [], states: [], sent: [], t0: performance.now() }
    window.__e2e = rec
    window.Worker = class extends W {
      constructor(u, o) {
        super(u, o)
        rec.workerUrl = String(u)
        this.addEventListener('message', (ev) => {
          const m = ev.data
          if (m.type === 'state') {
            rec.states.push({ t: m.t, tick: m.tick, x: m.pose?.x, y: m.pose?.y, status: m.status, speed: m.speed, paused: m.paused, stale: m.gnss?.stale })
            if (rec.states.length > 20000) rec.states.splice(0, 5000)
          } else if (m.type !== 'log' && m.type !== 'diag') rec.msgs.push({ ...m, _wall: performance.now() - rec.t0 })
        })
      }
      postMessage(m, ...rest) {
        if (m && m.type !== 'teleop') rec.sent.push(m)
        return super.postMessage(m, ...rest)
      }
    }
  })

  const snap = () => page.evaluate(() => window.__playground.client.snapshot)
  const st = () => page.evaluate(() => window.__playground.client.snapshot.state)
  const msgs = (type) => page.evaluate((t) => window.__e2e.msgs.filter((m) => m.type === t), type)
  const text = (sel) => page.$eval(sel, (e) => e.textContent ?? '')
  const until = async (fn, what, ms = 60000) => {
    const t0 = Date.now()
    while (Date.now() - t0 < ms) {
      if (await fn()) return
      await sleep(150)
    }
    const s = await st().catch(() => null)
    fail(`timed out waiting for ${what} (status ${s?.status}, t ${s?.t}, paused ${s?.paused})`)
  }
  const status = async () => (await st())?.status ?? null
  const click = (id) => page.click(`[data-testid="${id}"]`)
  const openSection = (section, probe) => openSettingsSection(page, section, probe)
  const nextReady = async (before, ms = 60000) =>
    page.waitForFunction((b) => window.__playground.client.snapshot.ready && window.__playground.client.snapshot.readyInfo !== b, { timeout: ms }, before)
  const run = async (factor) => {
    await click(`speed-${factor}`)
    if ((await st()).paused) await click('sim-pause')
  }
  const pause = async () => {
    if (!(await st()).paused) await click('sim-pause')
    await until(async () => (await st()).paused, 'pause', 5000)
  }

  // ------------------------------------------------------------------ 1. boot
  const t0 = Date.now()
  // follow=0: the camera stays put, so map clicks land where they were projected
  await page.goto(`${url}?theme=dark&follow=0&log=1`, { waitUntil: 'load' })
  await page.waitForSelector('[data-testid="boot-overlay"]', { timeout: 10000 })
  await page.waitForFunction(() => window.__playground?.mapReady === true, { timeout: 30000 })
  await page.screenshot({ path: path.join(OUT, 'e2e_boot.png') })
  await page.waitForFunction(() => window.__playground?.client?.snapshot?.ready || window.__playground?.client?.snapshot?.error?.fatal, { timeout: 300000, polling: 250 })
  const boot = await snap()
  if (!boot.ready) fail(`the sim did not start: ${boot.error?.message}\n${boot.error?.traceback}`)
  const kind = await page.evaluate(() => window.__playground.client.kind)
  if (kind !== 'pyodide') fail(`the default sim is ${kind}, not pyodide`)
  if (await page.$('[data-testid="boot-overlay"]')) fail('the boot overlay is still shown after ready')
  const stages = (await msgs('progress')).map((m) => m.stage)
  for (const s of ['pyodide', 'packages', 'bundle', 'map', 'nodes']) if (!stages.includes(s)) fail(`no progress stage ${s}: ${stages}`)
  // The worker's own fetches are not page requests: check what the page asked it to load
  const init = await page.evaluate(() => window.__e2e.sent.find((m) => m.type === 'init'))
  const workerUrl = await page.evaluate(() => window.__e2e.workerUrl)
  if (!/pyodide\.worker/.test(workerUrl)) fail(`the page started ${workerUrl}, not the Pyodide worker`)
  const zipReq = init?.bundleUrl
  const sha = await (await fetch(`${url}py/nift_bundle.json`)).json()
  if (!zipReq || !zipReq.endsWith(`py/${sha.zip}?v=${sha.sha256.slice(0, 16)}`)) fail(`the bundle URL has no hash: ${zipReq}`)
  if (!/^https?:/.test(init.baseUrl) || !/^https?:/.test(init.mapUrl)) fail(`init URLs are not absolute: ${JSON.stringify(init)}`)
  if (!(await fetch(zipReq)).ok) fail(`${zipReq} is not served`)
  const ready = boot.readyInfo
  result.boot = { readyS: (Date.now() - t0) / 1e3, pyodide: ready.pyodide, python: ready.python, stages: [...new Set(stages)].join(' '), bundle: zipReq.split('/').pop() }
  log(`ready in ${result.boot.readyS} s (Pyodide ${ready.pyodide}, Python ${ready.python})`)

  // presets: one flat list of GNSS faults, each with its kind (ARCH 12)
  const faults = ready.presets.gnss_faults
  if (!Array.isArray(faults)) fail('presets.gnss_faults is not a list')
  const kinds = new Set(faults.map((f) => f.kind))
  for (const f of faults) if (!['scheduled', 'sim', 'link'].includes(f.kind)) fail(`fault preset ${f.name} has kind ${f.kind}`)
  if (!faults.some((f) => f.kind === 'sim' && f.name === 'dropout')) fail('no sim preset dropout')
  if (!faults.some((f) => f.kind === 'link')) fail('no link presets')
  result.presets = `${faults.length} GNSS fault presets (${[...kinds].join(', ')}), ${ready.presets.routes.length} routes`

  const route = ready.presets.routes.find((r) => r.id === ROUTE)
  if (!route) fail(`no route ${ROUTE} in ready.presets.routes`)
  await page.evaluate(() => {
    for (const b of document.querySelectorAll('button')) if (b.textContent === 'Top-down') b.click()
  })

  /** Loads the route through the panel, clicks its goal on the map and waits for the path. */
  const loadRouteAndGoal = async () => {
    await openSection('scenarios-section', `route-${route.id}`)
    await click(`route-${route.id}`)
    const before = (await snap()).readyInfo
    await click('route-load')
    await nextReady(before)
    await closeSettings(page)
    // Route Load sends the route's goal itself. Click it on the map as well, the way a user sets one.
    await until(async () => (await snap()).path.length > 1, 'the path of the route goal')
    const [gx, gy] = route.goal
    await page.evaluate((x, y) => window.__playground.scene.focus(x, y, 60), gx, gy)
    await sleep(300)
    await click('tool-goal')
    const p = await page.evaluate((x, y) => window.__playground.scene.project(x, y), gx, gy)
    const acks = (await msgs('goal_ack')).length
    await page.mouse.click(p.x, p.y)
    await until(async () => (await msgs('goal_ack')).length > acks, 'goal_ack after the click')
    const ack = (await msgs('goal_ack')).at(-1)
    if (!ack.ok) fail(`the clicked goal was refused: ${ack.reason}`)
    if (Math.hypot(ack.x - gx, ack.y - gy) > 1.0) fail(`the clicked goal (${ack.x}, ${ack.y}) is not route ${route.id}'s goal (${gx}, ${gy})`)
    await until(async () => (await snap()).path.length > 1, 'the path')
    return { ack, path: (await snap()).path, start: (await st()).pose }
  }

  // ------------------------------------------------------------------ 2. route: goal, start, arrive
  await run(1)
  const r1 = await loadRouteAndGoal()
  const pathLen = r1.path.reduce((a, p, i) => (i ? a + Math.hypot(p[0] - r1.path[i - 1][0], p[1] - r1.path[i - 1][1]) : 0), 0)
  log(`route ${route.id}: path ${r1.path.length} points, ${pathLen.toFixed(1)} m`)
  const nStates = await page.evaluate(() => window.__e2e.states.length)
  await click('cmd-start')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING after Start', 15000)
  // A screenshot of the real stack driving, dark and light
  await until(async () => (await st()).speed > 1.5, 'the shuttle to speed up', 30000)
  await sleep(2500)
  await pause()
  const sp = await st()
  await page.evaluate((p) => window.__playground.scene.focus(p.x, p.y, 70), sp.pose)
  await sleep(500)
  await page.screenshot({ path: path.join(OUT, 'e2e_driving_dark.png') })
  await toggleTheme(page)
  await sleep(500)
  await page.screenshot({ path: path.join(OUT, 'e2e_driving_light.png') })
  await toggleTheme(page)

  // pause and step n: exactly n ticks, and nothing while paused
  const tick0 = (await st()).tick
  await page.click('[data-testid="sim-step-n"]', { clickCount: 3 })
  await page.type('[data-testid="sim-step-n"]', '17')
  await click('sim-step')
  await until(async () => (await st()).tick === tick0 + 17, `tick ${tick0 + 17} after step 17`, 10000)
  await sleep(600)
  if ((await st()).tick !== tick0 + 17) fail('the sim ran on while paused')
  result.step = `paused at ${tick0}, step 17 -> ${tick0 + 17}`

  await run('max')
  const tNav = Date.now()
  await until(async () => (await msgs('event')).some((m) => m.kind === 'arrived'), 'the arrival', 240000)
  await until(async () => (await status()) === 'IDLE', 'IDLE after arrival', 10000)
  const wallNav = (Date.now() - tNav) / 1e3
  await pause()
  const drive = await page.evaluate((n) => window.__e2e.states.slice(n), nStates)
  const nav = drive.filter((s) => s.status === 'NAVIGATING' && Number.isFinite(s.x))
  const end = (await st()).pose
  // Distance from each pose to the planned path, and how far along it the shuttle got
  const toPath = (x, y) => {
    let best = Infinity
    for (let i = 1; i < r1.path.length; i++) {
      const [ax, ay] = r1.path[i - 1]
      const [bx, by] = r1.path[i]
      const dx = bx - ax
      const dy = by - ay
      const L = dx * dx + dy * dy
      const u = L > 0 ? Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / L)) : 0
      best = Math.min(best, Math.hypot(x - ax - u * dx, y - ay - u * dy))
    }
    return best
  }
  const maxOff = Math.max(...nav.map((s) => toPath(s.x, s.y)))
  const travelled = nav.reduce((a, s, i) => (i ? a + Math.hypot(s.x - nav[i - 1].x, s.y - nav[i - 1].y) : 0), 0)
  const last = r1.path.at(-1)
  const toGoal = Math.hypot(end.x - last[0], end.y - last[1])
  const seq = drive.map((s) => s.status).filter((s, i, a) => s && s !== a[i - 1])
  if (nav.length < 20) fail(`only ${nav.length} NAVIGATING states`)
  if (maxOff > 1.5) fail(`the shuttle left the planned path by ${maxOff.toFixed(2)} m`)
  if (travelled < 0.8 * pathLen) fail(`the shuttle drove ${travelled.toFixed(1)} m of a ${pathLen.toFixed(1)} m path`)
  if (toGoal > 3.0) fail(`stopped ${toGoal.toFixed(2)} m from the end of the path`)
  if (seq.join(' ') !== 'NAVIGATING IDLE') fail(`status sequence ${seq.join(' ')}`)
  const arrived = (await msgs('event')).find((m) => m.kind === 'arrived')
  result.route = {
    route: route.id,
    path_m: +pathLen.toFixed(1),
    sim_s: +(arrived.t - nav[0].t).toFixed(2),
    wall_s: +wallNav.toFixed(1),
    max_off_path_m: +maxOff.toFixed(3),
    travelled_m: +travelled.toFixed(1),
    end_to_goal_m: +toGoal.toFixed(2),
    statuses: seq.join(' -> '),
  }
  log(`arrived: ${JSON.stringify(result.route)}`)

  // ------------------------------------------------------------------ 3. GNSS dropout -> SAFE_STOP -> Resume
  await run(1)
  await loadRouteAndGoal()
  await click('cmd-start')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING', 15000)
  await run(2)
  await until(async () => (await st()).speed > 1.5, 'speed before the fault', 30000)
  // Live faults have no buttons any more (the page offers the fault scenarios); the worker still takes them
  const tFault = (await st()).t
  await page.evaluate(() => window.__playground.client.send({ type: 'gnss_fault', preset: 'dropout', params: { duration_s: 3 } }))
  await until(async () => (await status()) === 'SAFE_STOP', 'SAFE_STOP after the dropout', 30000)
  const tStop = (await st()).t
  await page.waitForSelector('[data-testid="safe-stop-banner"]')
  if (!(await page.evaluate(() => window.__e2e.states.some((s) => s.stale)))) fail('the GNSS never went stale')
  await until(async () => (await st()).speed < 0.05, 'the shuttle to stop', 30000)
  await pause()
  await page.evaluate((p) => window.__playground.scene.focus(p.x, p.y, 60), (await st()).pose)
  await sleep(400)
  await page.screenshot({ path: path.join(OUT, 'e2e_safe_stop_dark.png') })
  await run(2)
  await until(async () => (await st()).gnss.stale === false, 'fresh GNSS after the dropout', 30000)
  await click('cmd-resume')
  await until(async () => (await status()) === 'IDLE', 'IDLE after Resume', 10000)
  await click('cmd-start')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING after the second Start', 15000)
  await run('max')
  const arrivals = (await msgs('event')).filter((m) => m.kind === 'arrived').length
  await until(async () => (await msgs('event')).filter((m) => m.kind === 'arrived').length > arrivals, 'the arrival after Resume', 240000)
  result.gnss_dropout = `dropout 3 s at t ${tFault.toFixed(1)} s -> SAFE_STOP at ${tStop.toFixed(1)} s -> Resume -> IDLE -> Start -> arrived`
  log(result.gnss_dropout)

  // ------------------------------------------------------------------ 4. Stop, E-STOP, refusals, reset
  await run(1)
  await loadRouteAndGoal()
  // Stop is /safe_stop: SAFE_STOP, and Resume goes back to IDLE
  await click('cmd-start')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING before Stop', 15000)
  await sleep(1500)
  await click('cmd-stop')
  await until(async () => (await status()) === 'SAFE_STOP', 'SAFE_STOP after Stop', 15000)
  await until(async () => (await st()).speed < 0.05, 'the stop after Stop', 30000)
  await click('cmd-resume')
  await until(async () => (await status()) === 'IDLE', 'IDLE after Resume', 10000)
  // E-STOP is /trigger_estop: it latches the controller and, through the 0x100 frame, the ECU. Only Reset recovers
  await click('cmd-start')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING before E-STOP', 15000)
  await sleep(1500)
  // The page has no E-STOP button any more; the service is still there
  const estopReply = (await page.evaluate(() => window.__playground.client.request({ type: 'cmd', name: 'estop' }))).message
  await until(async () => (await status()) === 'ESTOP', 'ESTOP after E-STOP', 15000)
  await until(async () => (await st()).ecu === 'ESTOP', 'the ECU latched by the E-stop frame', 15000)
  await until(async () => (await st()).speed < 0.05, 'the stop after E-STOP', 30000)
  if (!estopReply.includes('E-STOP engaged')) fail(`E-STOP reply: ${estopReply}`)
  if (!(await page.$eval('[data-testid="cmd-resume"]', (b) => b.disabled))) fail('Resume is enabled in ESTOP')
  result.estop_stop = 'Stop: SAFE_STOP, Resume -> IDLE. E-STOP: ESTOP and the ECU latched, Resume disabled'

  const refused = await page.evaluate(async () => {
    const c = window.__playground.client
    const ask = (m) => c.request(m)
    return [
      await ask({ type: 'set_param', node: 'trajectory_controller', name: 'base_speed', value: 99 }),
      await ask({ type: 'set_param', node: 'waypoint_node', name: 'base_speed', value: 2.5 }),
      await ask({ type: 'set_param', node: 'trajectory_controller', name: 'base_speed', value: 'fast' }),
      await ask({ type: 'set_param', node: 'trajectory_controller', name: 'base_speed', value: 2.5 }),
    ]
  })
  if (refused.slice(0, 3).some((r) => r.ok)) fail(`a bad set_param was accepted: ${JSON.stringify(refused)}`)
  if (!refused[3].ok) fail(`a good set_param was refused: ${refused[3].message}`)
  result.set_param = refused.map((r) => `${r.ok ? 'ok' : 'refused'}: ${r.message.slice(0, 70)}`)

  const before = (await snap()).readyInfo
  await click('sim-reset')
  await nextReady(before)
  const after = await st()
  const opts = (await snap()).readyInfo.launch.options
  if (after.tick > 40) fail(`tick ${after.tick} after reset`)
  if (after.status === 'ESTOP' || after.ecu === 'ESTOP') fail(`still E-stopped after reset: ${after.status}, ECU ${after.ecu}`)
  if (opts.params?.trajectory_controller?.base_speed !== 2.5) fail(`the staged base_speed did not apply on reset: ${JSON.stringify(opts.params)}`)
  result.reset = `tick ${after.tick} after reset, staged base_speed 2.5 applied`

  if (errors.length) fail(`console errors:\n${errors.join('\n')}`)
  console.log(JSON.stringify({ ok: true, ...result, wall_s: (Date.now() - t0) / 1e3, out: OUT }, null, 1))
  console.log('E2E OK')
} catch (e) {
  console.error(e.message ?? e)
  if (errors.length) console.error(errors.join('\n'))
  console.log('E2E FAILED')
  process.exitCode = 1
} finally {
  await browser?.close()
  server.kill()
}
