// Smoke test of the built page: serve dist/ with vite preview, load it headless, wait for the
// map, set a goal by clicking the map, start, and check that the mock shuttle moves.
// Takes dark, light and 3D screenshots.
//
// Then the panels, all through the page's own buttons and the map (with the mock worker):
// pause and step n, obstacle place by click, drag to move, delete, perception on (perceived
// outlines), a GNSS fault preset that ends in SAFE_STOP and Resume, manual mode with keyboard
// teleop, DBW E-stop and power cycle, set_param staged and applied on reset, a goal dragged with a
// heading, and a scenario saved to the URL and replayed in a fresh page (same events, goal and
// obstacles). Screenshots of each step at 1400 and 1280 px. --panels false skips this part.
//
// The 3D world: the World button loads it (lazily, with progress), then top-down and 3D
// screenshots along the route in both themes, a goal clicked on the draped map, pick and project
// against the ground, and the frame rate with and without it (SwiftShader here, so only a floor).
// --world false skips this part.
//
//   npm run build && npm run smoke -- [--out <dir>] [--sim mock|pyodide] [--keep]
//
// Puppeteer uses its own Chrome (npx puppeteer browsers install chrome) or PUPPETEER_EXECUTABLE_PATH.

import { spawn } from 'node:child_process'
import { mkdirSync } from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import puppeteer from 'puppeteer'

const here = path.dirname(fileURLToPath(import.meta.url))
const root = path.resolve(here, '..')
const arg = (name, def) => {
  const i = process.argv.indexOf(`--${name}`)
  return i > 0 ? process.argv[i + 1] : def
}
const OUT = path.resolve(arg('out', path.join(os.tmpdir(), 'nift-playground-smoke')))
const SIM = arg('sim', 'mock')
const PORT = Number(arg('port', 4173))
const PANELS = arg('panels', 'true') !== 'false'
const BASE_SPEED = arg('base-speed', '2') // set_param value; a whole number checks the int/float round trip
const WORLD = arg('world', 'true') !== 'false'
mkdirSync(OUT, { recursive: true })

const sleep = (ms) => new Promise((r) => setTimeout(r, ms))
const fail = (msg) => {
  throw new Error(`SMOKE FAIL: ${msg}`)
}

async function waitForServer(url, ms = 20000) {
  const t0 = Date.now()
  while (Date.now() - t0 < ms) {
    try {
      const r = await fetch(url)
      if (r.ok) return
    } catch {
      // not up yet
    }
    await sleep(200)
  }
  fail(`server at ${url} did not start`)
}

const viteBin = path.join(root, 'node_modules/vite/bin/vite.js')
const server = spawn(process.execPath, [viteBin, 'preview', '--port', String(PORT), '--strictPort'], { cwd: root, stdio: 'ignore' })
let browser
const errors = []
try {
  const url = `http://localhost:${PORT}/`
  await waitForServer(url)
  console.error('server up')
  const mapRes = await fetch(`${url}maps/lanelet2_odr2024.osm`)
  if (!mapRes.ok) fail('map not served by the build')

  browser = await puppeteer.launch({ headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] })
  console.error('browser up')
  const page = await browser.newPage()
  await page.setViewport({ width: 1400, height: 860 })
  page.on('console', (m) => {
    if (m.type() === 'error') errors.push(m.text())
  })
  page.on('pageerror', (e) => errors.push(String(e)))

  const t0 = Date.now()
  await page.goto(`${url}?sim=${SIM}&theme=dark`, { waitUntil: 'load' })
  await page.waitForFunction(() => window.__playground?.mapReady === true, { timeout: 30000 })
  const tMap = Date.now() - t0
  await page.waitForFunction(() => window.__playground?.client?.snapshot?.ready === true, { timeout: 120000 })
  const tReady = Date.now() - t0
  const lanelets = await page.evaluate(() => window.__playground.map.lanelets.length)
  if (lanelets < 100) fail(`only ${lanelets} lanelets drawn`)
  await page.waitForFunction(() => window.__playground.client.snapshot.state !== null)
  const start = await page.evaluate(() => window.__playground.client.snapshot.state.pose)

  // Pick goal candidates on lanelet centrelines 40 to 120 m away, nearest the middle of that band
  const candidates = await page.evaluate((s) => {
    const out = []
    for (const l of window.__playground.map.lanelets) {
      const c = l.centerline[Math.floor(l.centerline.length / 2)]
      const d = Math.hypot(c[0] - s.x, c[1] - s.y)
      if (d > 40 && d < 120) out.push([Math.abs(d - 80), c[0], c[1]])
    }
    return out.sort((a, b) => a[0] - b[0]).slice(0, 12).map((c) => [c[1], c[2]])
  }, start)
  if (candidates.length === 0) fail('no goal candidates')

  let goal = null
  for (const [gx, gy] of candidates) {
    await page.click('[data-testid="tool-goal"]')
    const px = await page.evaluate((x, y) => window.__playground.scene.project(x, y), gx, gy)
    await page.mouse.click(px.x, px.y)
    await sleep(300)
    const path = await page.evaluate(() => window.__playground.client.snapshot.path.length)
    if (path > 1) {
      goal = [gx, gy]
      break
    }
  }
  if (!goal) fail('no clicked goal produced a path')
  const sent = await page.evaluate(() => window.__playground.client.snapshot.goal)
  if (Math.hypot(sent[0] - goal[0], sent[1] - goal[1]) > 2.0) fail(`goal from click ${sent} is far from ${goal}`)

  // An obstacle beside the route, so the screenshots show one
  await page.evaluate(() => {
    const c = window.__playground.client
    const p = c.snapshot.path[Math.floor(c.snapshot.path.length / 2)]
    const o = [p[0] + 6, p[1] + 6]
    c.send({ type: 'set_obstacles', obstacles: [{ oid: 'smoke', poly: [[o[0] - 1, o[1] - 1], [o[0] + 1.2, o[1] - 0.8], [o[0] + 0.9, o[1] + 1.1], [o[0] - 0.8, o[1] + 0.9]] }] })
  })

  await page.click('[data-testid="cmd-start"]')
  await page.evaluate(() => window.__playground.client.send({ type: 'speed', factor: 4 }))
  await sleep(3000)
  const now = await page.evaluate(() => window.__playground.client.snapshot.state)
  const moved = Math.hypot(now.pose.x - start.x, now.pose.y - start.y)
  if (moved < 3) fail(`shuttle moved only ${moved.toFixed(2)} m (mode ${now.mode})`)
  if (now.mode !== 'AUTONOMOUS' && now.mode !== 'IDLE') fail(`unexpected mode ${now.mode}`)

  await page.evaluate(() => window.__playground.client.send({ type: 'pause' }))
  await sleep(300)
  await page.screenshot({ path: path.join(OUT, 'playground_dark.png') })
  await page.click('[data-testid="theme-toggle"]')
  await sleep(400)
  await page.screenshot({ path: path.join(OUT, 'playground_light.png') })
  await page.click('[data-testid="theme-toggle"]')
  await sleep(300)
  // Close-ups around the shuttle: top-down, then 3D
  const shot = async (name, cam) => {
    // Switch through the tool bar so the buttons show the mode, then frame the shuttle
    await page.evaluate((label) => {
      for (const b of document.querySelectorAll('button')) if (b.textContent === label) b.click()
    }, cam === 'ortho' ? 'Top-down' : '3D')
    await sleep(200)
    await page.evaluate((cam) => {
      const s = window.__playground.scene
      const p = window.__playground.client.snapshot.state.pose
      s.focus(p.x, p.y, cam === 'ortho' ? 40 : 25)
    }, cam)
    await sleep(500)
    await page.screenshot({ path: path.join(OUT, name) })
  }
  await shot('playground_dark_closeup.png', 'ortho')
  await page.click('[data-testid="theme-toggle"]')
  await shot('playground_light_closeup.png', 'ortho')
  await page.click('[data-testid="theme-toggle"]')
  await shot('playground_3d.png', 'persp')

  const world = WORLD ? await worldChecks(page, url) : null
  const panels = PANELS ? await panelChecks(browser, page, url) : null

  if (errors.length) fail(`console errors:\n${errors.join('\n')}`)
  console.log(
    JSON.stringify({ ok: true, sim: SIM, lanelets, mapMs: tMap, readyMs: tReady, goal, moved: +moved.toFixed(2), mode: now.mode, world, panels, out: OUT }),
  )
} catch (e) {
  console.error(e.message ?? e)
  if (errors.length) console.error(errors.join('\n'))
  process.exitCode = 1
} finally {
  await browser?.close()
  server.kill()
}

// ------------------------------------------------------------------------------------- world

async function worldChecks(page, url) {
  const out = {}
  const clickLabel = (label) =>
    page.evaluate((label) => {
      for (const b of document.querySelectorAll('button')) if (b.textContent === label) b.click()
    }, label)
  const shotW = async (name) => page.screenshot({ path: path.join(OUT, name) })
  const fps = async () => {
    const a = await page.evaluate(() => [window.__playground.scene.stats(), performance.now()])
    await sleep(2000)
    const b = await page.evaluate(() => [window.__playground.scene.stats(), performance.now()])
    return { fps: +(((b[0].frames - a[0].frames) * 1000) / (b[1] - a[1])).toFixed(1), calls: b[0].calls, triangles: b[0].triangles }
  }

  // Off by default, and nothing of it fetched before the button
  const before = await page.evaluate(() => performance.getEntriesByType('resource').filter((r) => r.name.includes('/world/')).length)
  if (before !== 0) fail(`world files fetched before the World button: ${before}`)
  await clickLabel('Top-down')
  await page.evaluate(() => window.__playground.scene.fit())
  out.fpsNoWorld = await fps()

  const t0 = Date.now()
  await page.click('[data-testid="toggle-world"]')
  const sawProgress = await page
    .waitForFunction(() => /World \d+%/.test(document.querySelector('[data-testid="toggle-world"]')?.textContent ?? '') || window.__playground?.worldReady, { timeout: 10000 })
    .then(() => true)
  await page.waitForFunction(() => window.__playground?.worldReady === true, { timeout: 60000 })
  out.loadMs = Date.now() - t0
  out.info = await page.evaluate(() => window.__playground.worldInfo)
  out.progressShown = sawProgress
  const fetched = await page.evaluate(() =>
    performance.getEntriesByType('resource').filter((r) => r.name.includes('/world/')).map((r) => [r.name.split('/world/')[1], r.transferSize || r.encodedBodySize]),
  )
  out.fetched = Object.fromEntries(fetched)
  for (const f of ['world.json', 'world.glb', 'height.bin.gz', 'trees.json', 'draco/draco_decoder.wasm']) if (!(f in out.fetched)) fail(`world file not fetched: ${f}`)
  const noticeRes = await fetch(`${url}world/NOTICE`)
  if (!noticeRes.ok || !(await noticeRes.text()).includes('MIT License')) fail('world/NOTICE missing or without the MIT notice')
  await sleep(800)

  // Top-down, whole site, dark and light
  await page.evaluate(() => window.__playground.scene.fit())
  await sleep(600)
  out.fpsWorldTop = await fps()
  await shotW('world_top_dark.png')
  await page.click('[data-testid="theme-toggle"]')
  await sleep(500)
  await shotW('world_top_light.png')
  await page.click('[data-testid="theme-toggle"]')

  // Close-up on the shuttle and its path, top-down
  const st = await page.evaluate(() => window.__playground.client.snapshot.state.pose)
  await page.evaluate((p) => window.__playground.scene.focus(p.x, p.y, 50), st)
  await sleep(600)
  await shotW('world_top_closeup.png')

  // The drawing sits on the ground: the shuttle and the path are lifted by the height grid
  const lift = await page.evaluate(() => {
    const s = window.__playground.scene
    const p = window.__playground.client.snapshot.state.pose
    return { ground: s.groundAt(p.x, p.y), shuttleZ: s.shuttle.position.z }
  })
  if (Math.abs(lift.shuttleZ - lift.ground) > 0.3) fail(`shuttle not on the ground: z ${lift.shuttleZ} ground ${lift.ground}`)
  out.groundUnderShuttle = +lift.ground.toFixed(2)

  // A goal clicked on the draped map lands where it was clicked (top-down)
  const target = await page.evaluate(() => {
    const path = window.__playground.client.snapshot.path
    return path[Math.floor(path.length * 0.8)]
  })
  await page.evaluate((t) => window.__playground.scene.focus(t[0], t[1], 60), target)
  await sleep(400)
  const tp = await page.evaluate((t) => window.__playground.scene.project(t[0], t[1]), target)
  const picked = await page.evaluate((x, y) => window.__playground.scene.pick(x, y), tp.x, tp.y)
  out.pickErrTopM = +Math.hypot(picked[0] - target[0], picked[1] - target[1]).toFixed(3)
  if (out.pickErrTopM > 0.3) fail(`top-down pick on the world is off by ${out.pickErrTopM} m`)

  // 3D, behind the shuttle looking along the route, dark and light
  await clickLabel('3D')
  await sleep(300)
  const view = await page.evaluate(() => {
    const snap = window.__playground.client.snapshot
    const p = snap.state.pose
    const path = snap.path
    let k = 0
    let best = Infinity
    path.forEach((q, i) => {
      const d = Math.hypot(q[0] - p.x, q[1] - p.y)
      if (d < best) [best, k] = [d, i]
    })
    const ahead = path[Math.min(path.length - 1, k + 40)] ?? [p.x + 20 * Math.cos(p.yaw), p.y + 20 * Math.sin(p.yaw)]
    const dx = ahead[0] - p.x
    const dy = ahead[1] - p.y
    const n = Math.hypot(dx, dy) || 1
    return { k, eye: [p.x - (14 * dx) / n, p.y - (14 * dy) / n, 7], target: [ahead[0], ahead[1], 0] }
  })
  await page.evaluate((v) => window.__playground.scene.setView(v.eye, v.target), view)
  await sleep(800)
  out.fpsWorld3d = await fps()
  await shotW('world_3d_route_dark.png')
  // pick in 3D: project two path points the camera looks along, pick the pixels back. A point far
  // down the route can sit behind a rise of the site, where no pick on the ground can reach it.
  const p3 = await page.evaluate((ks) => {
    const path = window.__playground.client.snapshot.path
    const s = window.__playground.scene
    return ks.map((k) => {
      const t = path[Math.min(path.length - 1, k)]
      const px = s.project(t[0], t[1])
      return { t, got: s.pick(px.x, px.y) }
    })
  }, [view.k + 20, view.k + 40])
  out.pickErr3dM = +Math.max(...p3.map((p) => (p.got ? Math.hypot(p.got[0] - p.t[0], p.got[1] - p.t[1]) : Infinity))).toFixed(3)
  if (out.pickErr3dM > 0.5) fail(`3D pick on the world is off by ${out.pickErr3dM} m`)
  await page.click('[data-testid="theme-toggle"]')
  await sleep(500)
  await shotW('world_3d_route_light.png')
  await page.click('[data-testid="theme-toggle"]')
  // A wide oblique of the site
  await page.evaluate(() => {
    const b = window.__playground.map.bounds
    const cx = (b.minX + b.maxX) / 2
    const cy = (b.minY + b.maxY) / 2
    window.__playground.scene.setView([cx - 120, b.minY - 220, 230], [cx, cy - 20, 0])
  })
  await sleep(800)
  await shotW('world_3d_site_dark.png')

  // Off again: the grid is back, the map is flat
  await page.click('[data-testid="toggle-world"]')
  await sleep(300)
  const off = await page.evaluate(() => window.__playground.scene.isWorldVisible())
  if (off) fail('the world is still shown after the toggle')
  await clickLabel('Top-down')
  await page.evaluate(() => window.__playground.scene.fit())
  await sleep(300)
  out.fpsAfterOff = await fps()
  return out
}

// ------------------------------------------------------------------------------------ panels

async function panelChecks(browser, page, url) {
  const out = {}
  const snap = () => page.evaluate(() => window.__playground.client.snapshot)
  const st = () => page.evaluate(() => window.__playground.client.snapshot.state)
  const text = (sel) => page.$eval(sel, (e) => e.textContent ?? '')
  const value = (sel) => page.$eval(sel, (e) => e.value)
  const until = async (fn, what, ms = 15000) => {
    const t0 = Date.now()
    while (Date.now() - t0 < ms) {
      if (await fn()) return
      await sleep(100)
    }
    const s = await st().catch(() => null)
    fail(`timed out waiting for ${what} (status ${s?.status}, t ${s?.t}, paused ${s?.paused}, gnss ${JSON.stringify(s?.gnss)})`)
  }
  const shotP = async (name) => page.screenshot({ path: path.join(OUT, name) })
  /** Opens a folding section (by its testid) when the probe inside it is not shown yet. */
  const openSection = async (section, probe) => {
    if (await page.$(`[data-testid="${section}"] [data-testid="${probe}"]`)) return
    await page.click(`[data-testid="${section}"] button`)
    await page.waitForSelector(`[data-testid="${section}"] [data-testid="${probe}"]`, { timeout: 3000 })
  }
  const px = (x, y) => page.evaluate((x, y) => window.__playground.scene.project(x, y), x, y)
  const status = async () => (await st()).status
  const reset = async (launch) => {
    const before = await page.evaluate(() => window.__playground.client.snapshot.readyInfo)
    if (launch) await page.evaluate((l) => window.__playground.client.send({ type: 'reset', launch: l }), launch)
    else await page.click('[data-testid="sim-reset"]')
    await page.waitForFunction((b) => window.__playground.client.snapshot.ready && window.__playground.client.snapshot.readyInfo !== b, {}, before)
  }
  const goalAhead = async (dist = 60) => {
    // A centreline point about `dist` m ahead of the shuttle, clicked on the map
    const s = await st()
    const cand = await page.evaluate((s, dist) => {
      let best = null
      for (const l of window.__playground.map.lanelets)
        for (const c of l.centerline) {
          const dx = c[0] - s.pose.x
          const dy = c[1] - s.pose.y
          const ahead = dx * Math.cos(s.pose.yaw) + dy * Math.sin(s.pose.yaw)
          const side = Math.abs(-dx * Math.sin(s.pose.yaw) + dy * Math.cos(s.pose.yaw))
          const score = Math.abs(ahead - dist) + side * 3
          if (ahead > 0 && (!best || score < best[0])) best = [score, c[0], c[1]]
        }
      return best && [best[1], best[2]]
    }, s, dist)
    if (!cand) fail('no goal ahead of the shuttle')
    await page.evaluate(() => window.__playground.scene.fit())
    await sleep(300)
    await page.click('[data-testid="tool-goal"]')
    const p = await px(cand[0], cand[1])
    await page.mouse.click(p.x, p.y)
    await until(async () => (await snap()).path.length > 1, 'a path after the goal click')
    return cand
  }
  const runFast = async () => {
    await page.click('[data-testid="speed-max"]')
    if ((await st()).paused) await page.click('[data-testid="sim-pause"]')
  }
  const pause = async () => {
    if (!(await st()).paused) await page.click('[data-testid="sim-pause"]')
    await until(async () => (await st()).paused, 'pause')
  }

  // ---- fresh stack, normal speed, top-down view
  await page.evaluate(() => {
    for (const b of document.querySelectorAll('button')) if (b.textContent === 'Top-down') b.click()
  })
  await reset()
  // The real stack keeps the page's obstacles across a reset (they are the page's world): clear them
  if ((await snap()).obstacles.length) {
    await page.evaluate(() => {
      for (const b of document.querySelectorAll('[data-testid="obstacles-section"] button')) if (b.textContent === 'Clear') b.click()
    })
    await until(async () => (await snap()).obstacles.length === 0, 'the obstacles to clear')
  }
  if ((await st()).paused) await page.click('[data-testid="sim-pause"]')
  await page.click('[data-testid="speed-1"]')

  // ---- goal by click and start through the operator panel
  await goalAhead(70)
  await page.click('[data-testid="cmd-start"]')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING after Start')
  await until(async () => (await text('[data-testid="last-reply"]')).includes('Navigation started'), 'the start reply')
  out.start = 'ok'

  // ---- pause and step n
  await pause()
  const tick0 = (await st()).tick
  await page.click('[data-testid="sim-step-n"]', { clickCount: 3 })
  await page.type('[data-testid="sim-step-n"]', '25')
  await page.click('[data-testid="sim-step"]')
  await until(async () => (await st()).tick === tick0 + 25, `tick ${tick0 + 25} after step 25`, 5000)
  await sleep(400)
  if ((await st()).tick !== tick0 + 25) fail('the sim ran on while paused')
  out.step = `${tick0} -> ${tick0 + 25}`

  // ---- obstacles: place by click (Obstacle tool), drag to move, delete from the list
  const s0 = await st()
  await page.evaluate((p) => window.__playground.scene.focus(p.x, p.y, 45), s0.pose)
  await sleep(400)
  const place = [s0.pose.x + 8 * Math.cos(s0.pose.yaw) - 5 * Math.sin(s0.pose.yaw), s0.pose.y + 8 * Math.sin(s0.pose.yaw) + 5 * Math.cos(s0.pose.yaw)]
  await page.click('[data-testid="tool-obstacle"]')
  let p = await px(place[0], place[1])
  await page.mouse.click(p.x, p.y)
  await until(async () => (await snap()).obstacles.length === 1, 'an obstacle after the click')
  await page.keyboard.press('Escape')
  const centre = (poly) => poly.reduce((a, q) => [a[0] + q[0] / poly.length, a[1] + q[1] / poly.length], [0, 0])
  const c0 = centre((await snap()).obstacles[0].poly)
  if (Math.hypot(c0[0] - place[0], c0[1] - place[1]) > 0.3) fail(`obstacle placed at ${c0}, clicked ${place}`)
  // drag it 6 m east
  p = await px(c0[0], c0[1])
  const q = await px(c0[0] + 6, c0[1])
  await page.mouse.move(p.x, p.y)
  await page.mouse.down()
  await page.mouse.move((p.x + q.x) / 2, (p.y + q.y) / 2, { steps: 5 })
  await shotP('panels_obstacle_drag.png')
  await page.mouse.move(q.x, q.y, { steps: 5 })
  await page.mouse.up()
  await until(async () => {
    const c = centre((await snap()).obstacles[0].poly)
    return Math.hypot(c[0] - c0[0] - 6, c[1] - c0[1]) < 0.3
  }, 'the obstacle to move 6 m east')
  const camAfter = await page.evaluate((x, y) => window.__playground.scene.project(x, y), c0[0], c0[1])
  if (Math.hypot(camAfter.x - p.x, camAfter.y - p.y) > 2) fail('the camera panned during the obstacle drag')
  const id = (await snap()).obstacles[0].oid
  await shotP('panels_obstacle.png')
  await page.click(`[data-testid="obstacle-del-${id}"]`)
  await until(async () => (await snap()).obstacles.length === 0, 'the obstacle to go')
  out.obstacles = `placed, dragged 6 m, deleted ${id}`

  // ---- set_param staged, then the launch form: perception on, ekf; reset applies both
  await openSection('params-section', 'param-base_speed')
  await page.type('[data-testid="param-base_speed"]', BASE_SPEED)
  await page.click('[data-testid="param-set-base_speed"]')
  await until(async () => (await text('[data-testid="params-section"]')).includes('1 staged'), 'base_speed staged')
  await openSection('launch-section', 'launch-perception')
  await page.select('[data-testid="launch-perception"]', 'true')
  await page.select('[data-testid="launch-localization"]', 'ekf')
  const before = await page.evaluate(() => window.__playground.client.snapshot.readyInfo)
  await page.click('[data-testid="launch-apply"]')
  await page.waitForFunction((b) => window.__playground.client.snapshot.ready && window.__playground.client.snapshot.readyInfo !== b, {}, before)
  const opts = await page.evaluate(() => window.__playground.client.snapshot.readyInfo.launch.options ?? window.__playground.client.snapshot.readyInfo.launch)
  if (opts.localization !== 'ekf' || opts.perception !== true) fail(`launch options not applied: ${JSON.stringify(opts)}`)
  if (opts.params?.trajectory_controller?.base_speed !== Number(BASE_SPEED)) fail(`base_speed not applied on reset: ${JSON.stringify(opts.params)}`)
  // A reset keeps the pacing: the sim was paused for the step above
  if ((await st()).paused) {
    await page.click('[data-testid="sim-pause"]')
    await until(async () => !(await st()).paused, 'the sim to run after the reset', 5000)
  }
  await until(async () => (await st()).locStatus === 'OK', 'locStatus OK in ekf')
  out.params = `base_speed ${BASE_SPEED} staged, applied on reset; ekf + perception`

  // ---- perceived vs placed: an obstacle ahead in the lane (perception on)
  if ((await st()).paused) await page.click('[data-testid="sim-pause"]')
  await page.evaluate(() => {
    for (const b of document.querySelectorAll('[data-testid="obstacles-section"] button')) if (b.textContent === 'Ahead') b.click()
  })
  await until(async () => ((await snap()).perceived ?? []).length === 1, 'the obstacle to be perceived')
  const s1 = await st()
  await page.evaluate((p) => window.__playground.scene.focus(p.x + 6 * Math.cos(p.yaw), p.y + 6 * Math.sin(p.yaw), 40), s1.pose)
  await sleep(400)
  await shotP('panels_perceived.png')
  out.perceived = (await text('[data-testid="obstacle-list"]')).includes('seen') ? 'seen' : fail('the list does not mark it seen')
  await page.click('[data-testid="obstacles-section"] [data-testid^="obstacle-del-"]')
  await until(async () => (await snap()).obstacles.length === 0, 'the obstacle to go')

  // ---- GNSS fault preset: dropout 3 s while navigating -> SAFE_STOP, then Resume
  await goalAhead(80)
  await page.click('[data-testid="cmd-start"]')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING')
  await openSection('gnss-section', 'fault-live')
  await page.select('[data-testid="fault-live"]', 'dropout')
  await page.evaluate(() => {
    const el = [...document.querySelectorAll('[data-testid="gnss-section"] label')].find((l) => l.textContent.startsWith('duration_s'))
    const input = el.querySelector('input')
    const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set
    set.call(input, '3')
    input.dispatchEvent(new Event('input', { bubbles: true }))
  })
  await page.click('[data-testid="fault-apply"]')
  await until(async () => (await status()) === 'SAFE_STOP', 'SAFE_STOP after the GNSS dropout')
  await page.waitForSelector('[data-testid="safe-stop-banner"]')
  await page.click('[data-testid="dock-tab-events"]')
  await until(async () => (await text('[data-testid="feed-list"]')).includes('GNSS fault'), 'the fault in the events tab')
  await sleep(300)
  await shotP('panels_safe_stop.png')
  await page.click('[data-testid="dock-tab-log"]')
  await until(async () => (await st()).gnss.stale === false, 'fresh GNSS after the dropout')
  await page.click('[data-testid="cmd-resume"]')
  await until(async () => (await status()) === 'IDLE', 'IDLE after Resume')
  out.gnss_fault = 'dropout 3 s -> SAFE_STOP -> Resume -> IDLE'

  // ---- manual mode and keyboard teleop (W for a second, then Space)
  await page.click('[data-testid="cmd-manual"]')
  await until(async () => (await status()) === 'MANUAL', 'MANUAL')
  await page.click('[data-testid="teleop-pad"]')
  await page.keyboard.down('w')
  await until(async () => (await st()).speed > 0.5, 'teleop speed', 8000)
  await page.keyboard.down('a')
  await sleep(500)
  const steerTele = (await st()).steer
  await page.keyboard.up('a')
  await page.keyboard.up('w')
  await page.keyboard.press(' ')
  await until(async () => Math.abs((await st()).speed) < 0.05, 'teleop stop', 8000)
  if (!(steerTele > 0)) fail(`A should steer left (+), got ${steerTele}`)
  await page.click('[data-testid="cmd-manual"]')
  await until(async () => (await status()) === 'IDLE', 'IDLE after manual off')
  out.teleop = `W then A: steer ${((steerTele * 180) / Math.PI).toFixed(1)} deg, Space stops`

  // ---- DBW E-stop latches the ECU; power cycle releases it
  await openSection('dbw-section', 'cmd-dbw_estop')
  await page.click('[data-testid="cmd-dbw_estop"]')
  await until(async () => (await st()).ecu === 'ESTOP', 'ECU ESTOP')
  await page.waitForSelector('[data-testid="dbw-latched-banner"]')
  await page.click('[data-testid="cmd-power_cycle"]')
  await until(async () => (await st()).ecu !== 'ESTOP', 'ECU released')
  out.dbw = 'latched, power cycled'

  // ---- a goal dragged with a heading (RViz 2D Goal); screenshot mid-drag shows the arrow
  await reset({ localization: 'direct', perception: false, filter_type: '' })
  const sg = await st()
  await page.evaluate((p) => window.__playground.scene.focus(p.x, p.y, 60), sg.pose)
  await sleep(300)
  await page.click('[data-testid="tool-goal"]')
  const g0 = [sg.pose.x + 15 * Math.cos(sg.pose.yaw), sg.pose.y + 15 * Math.sin(sg.pose.yaw)]
  const a = await px(g0[0], g0[1])
  const b2 = await px(g0[0] + 5 * Math.cos(sg.pose.yaw), g0[1] + 5 * Math.sin(sg.pose.yaw))
  await page.evaluate(() => {
    const c = window.__playground.client
    const orig = c.send.bind(c)
    window.__sent = []
    c.send = (m) => {
      window.__sent.push(m)
      return orig(m)
    }
  })
  await page.mouse.move(a.x, a.y)
  await page.mouse.down()
  await page.mouse.move(b2.x, b2.y, { steps: 6 })
  await shotP('panels_goal_arrow.png')
  await page.mouse.up()
  const goalMsg = await page.evaluate(() => window.__sent.find((m) => m.type === 'goal'))
  if (!goalMsg || typeof goalMsg.yaw !== 'number') fail(`dragged goal sent no yaw: ${JSON.stringify(goalMsg)}`)
  const dyaw = Math.atan2(Math.sin(goalMsg.yaw - sg.pose.yaw), Math.cos(goalMsg.yaw - sg.pose.yaw))
  if (Math.abs(dyaw) > 0.1) fail(`goal yaw ${goalMsg.yaw} not the drag direction ${sg.pose.yaw}`)
  out.goal_drag = `yaw ${goalMsg.yaw.toFixed(3)} (drag ${sg.pose.yaw.toFixed(3)})`

  // ---- scenario: goal, start, an obstacle, run; save to the URL; replay in a fresh page
  await reset()
  await page.click('[data-testid="speed-2"]')
  await goalAhead(90)
  await sleep(300)
  await page.click('[data-testid="cmd-start"]')
  await until(async () => (await status()) === 'NAVIGATING', 'NAVIGATING')
  await sleep(1500)
  await page.evaluate(() => {
    for (const b of document.querySelectorAll('[data-testid="obstacles-section"] button')) if (b.textContent === 'Ahead') b.click()
  })
  await until(async () => (await snap()).obstacles.length === 1, 'the scenario obstacle')
  await sleep(800)
  await pause()
  await openSection('scenario-section', 'scenario-share')
  await page.click('[data-testid="scenario-share"]')
  await page.waitForSelector('[data-testid="scenario-link"]')
  const link = await value('[data-testid="scenario-link"]')
  if (!link.includes('#scenario=')) fail(`no scenario link: ${link}`)
  if ((await page.url()) !== link) fail('the page URL was not updated with the scenario')
  const saved = await page.evaluate(async () => {
    const c = window.__playground.client
    return new Promise((res) => {
      const off = c.onMessage((m) => m.type === 'scenario' && (off(), res(m.scenario)))
      c.send({ type: 'scenario_save' })
    })
  })
  const a1 = await snap()
  const lastTick = Math.max(...saved.events.map((e) => e.tick))
  await shotP('panels_scenario_saved.png')

  const page2 = await browser.newPage()
  await page2.setViewport({ width: 1280, height: 800 })
  page2.on('pageerror', (e) => errors.push(String(e)))
  page2.on('console', (m) => m.type() === 'error' && errors.push(m.text()))
  await page2.goto(link, { waitUntil: 'load' })
  // The real stack boots Pyodide again in the new page (from the browser cache)
  await page2.waitForFunction(() => window.__playground?.mapReady && window.__playground?.client?.snapshot?.ready, { timeout: SIM === 'mock' ? 30000 : 180000 })
  await page2.waitForFunction((t) => (window.__playground.client.snapshot.state?.tick ?? 0) > t + 2, { timeout: 60000 }, lastTick).catch(async (e) => {
    const s = await page2.evaluate(async () => {
      const raw = []
      const off = window.__playground.client.onMessage((m) => raw.push(m))
      await new Promise((r) => setTimeout(r, 2000))
      off()
      const c = window.__playground.client.snapshot
      return { tick: c.state?.tick, paused: c.state?.paused, status: c.state?.status, error: c.error, log: c.log.filter((l) => l.level !== 'info').slice(-8),
               raw: raw.slice(-6).map((m) => (m.type === 'state' ? { type: 'state', tick: m.tick, paused: m.paused, pose: m.pose } : m.type)) }
    })
    fail(`the replay did not pass tick ${lastTick}: ${e.message}\n${JSON.stringify(s)}\n${JSON.stringify(saved)}`)
  })
  await page2.evaluate(() => window.__playground.client.send({ type: 'pause' }))
  const b1 = await page2.evaluate(() => window.__playground.client.snapshot)
  const resaved = await page2.evaluate(async () => {
    const c = window.__playground.client
    return new Promise((res) => {
      const off = c.onMessage((m) => m.type === 'scenario' && (off(), res(m.scenario)))
      c.send({ type: 'scenario_save' })
    })
  })
  if (JSON.stringify(resaved.events) !== JSON.stringify(saved.events)) fail(`replayed events differ:\n${JSON.stringify(saved.events)}\n${JSON.stringify(resaved.events)}`)
  if (JSON.stringify(b1.goal) !== JSON.stringify(a1.goal)) fail(`replayed goal ${b1.goal} != ${a1.goal}`)
  if (JSON.stringify(b1.obstacles) !== JSON.stringify(a1.obstacles)) fail(`replayed obstacles differ:\n${JSON.stringify(a1.obstacles)}\n${JSON.stringify(b1.obstacles)}\n${JSON.stringify(saved.events)}`)
  if (b1.state.status !== 'NAVIGATING' && b1.state.status !== 'SAFE_STOP' && b1.state.status !== 'IDLE') fail(`replay status ${b1.state.status}`)
  await page2.evaluate((p) => window.__playground.scene.focus(p.x, p.y, 60), b1.state.pose)
  await sleep(500)
  await page2.screenshot({ path: path.join(OUT, 'panels_scenario_replayed_1280.png') })
  await page2.click('[data-testid="theme-toggle"]')
  await sleep(400)
  await page2.screenshot({ path: path.join(OUT, 'panels_light_1280.png') })
  await page2.close()
  out.scenario = `${saved.events.length} events, ${link.length} chars, replayed equal`

  // ---- the whole page, dark, sections open
  for (const [sec, probe] of [['gnss-section', 'fault-live'], ['launch-section', 'launch-perception'], ['diag-section', 'diag-list']]) await openSection(sec, probe)
  await page.evaluate(() => window.__playground.scene.fit())
  await sleep(400)
  await shotP('panels_dark.png')
  return out
}
