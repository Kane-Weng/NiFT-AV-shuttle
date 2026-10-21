/*
 * File: MapScene.ts
 *
 * Description:
 *   The three.js view of the playground, drawn to look like our RViz: dark background, grid,
 *   lanelet borders, the planned path, a goal marker, obstacles and the box shuttle with its
 *   TF axes. Everything is in the map frame (x east, y north, z up, metres).
 *
 *   Camera: top-down orthographic by default (left drag pans, right drag rotates, wheel zooms),
 *   or perspective (right drag also tilts). A click without a drag reports the map point, which
 *   the page uses for the goal. A tool can take a left-button gesture (onDragStart returns true):
 *   the camera then stays still and the tool gets the drag (2D Goal and Set pose with their
 *   heading, moving an obstacle).
 *
 *   Follow keeps the shuttle in the middle and turns the view with its heading, like a game
 *   camera. Top-down, the map turns so the shuttle always points up the screen. In 3D it is a
 *   chase camera, behind and above the shuttle and looking along its heading. The camera glides
 *   after the shuttle (follower.ts) instead of copying each 10 Hz state, so GNSS noise and the
 *   small weave it gives the shuttle do not shake the view. It follows the sim's true pose; the
 *   stack's estimate jumps with every noisy fix. While following, left drag does not pan, the
 *   wheel zooms about the shuttle and right drag cannot turn the view (the heading does). In 3D
 *   right drag still tilts it. Turning the follow on glides the view onto the shuttle and turns
 *   it to the heading. Turning it off leaves the view as it is.
 *
 *   The shuttle and the stack's pose outline are drawn from states that come about 10 times a
 *   second. They are played back smoothly between states, a little behind the newest one
 *   (interp.ts), so the box moves every frame instead of stepping. Paused, stepping, after a reset
 *   or a teleport they jump straight to the newest state. The trail and the follow use the same
 *   drawn pose, so they never run ahead of the box.
 *
 *   The scene has no React in it. The page creates one MapScene per canvas and feeds it data.
 *
 *   The 3D world (world.ts) is optional. When it is shown, everything the page draws is lifted
 *   onto its ground with the height grid, display only (the stack stays 2D). It renders first;
 *   in the top-down view the depth buffer is then cleared, so lanes, path, goal and the shuttle
 *   stay visible under trees and roofs, as RViz overlays would. In 3D they are depth tested
 *   against the world.
 */

import * as THREE from 'three'
import { MapControls } from 'three/examples/jsm/controls/MapControls.js'
import { LineSegments2 } from 'three/examples/jsm/lines/LineSegments2.js'
import { LineSegmentsGeometry } from 'three/examples/jsm/lines/LineSegmentsGeometry.js'
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js'
import { Line2 } from 'three/examples/jsm/lines/Line2.js'
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js'
import type { LaneletMap, XY, BorderKind } from '../map/lanelets'
import type { Pose2D } from '../protocol'
import { VEHICLE } from '../geo/site'
import { THEMES, type SceneTheme, type ThemeName } from './theme'
import type { LoadedWorld } from './world'
import { FollowSmoother, wrapAngle } from './follower'
import { PoseInterpolator } from './interp'

export type CameraMode = 'ortho' | 'persp'

export interface MapSceneEvents {
  onClick?: (x: number, y: number) => void
  onHover?: (x: number, y: number) => void
  /** Left button down on the map. Return true to take the gesture from the camera. */
  onDragStart?: (x: number, y: number) => boolean
  onDrag?: (x: number, y: number) => void
  onDragEnd?: (x: number, y: number) => void
}

export interface ArrowSpec {
  x: number
  y: number
  yaw: number
  kind: 'goal' | 'pose'
}

// Draw order heights [m]: everything flat sits a little above the grid so nothing z-fights
const Z_GRID = -0.02
const Z_LANES = 0.0
const Z_CENTER = 0.01
const Z_OBST = 0.02
const Z_TRAIL = 0.03
const Z_PATH = 0.05
const Z_GOAL = 0.06

const TRAIL_MAX = 4000
// Lift above the world's ground: clears the road mesh, its compression error and the curbs
const DRAPE = 0.15
// Longest straight piece of a draped line [m] (the height grid has 1 m cells)
const DRAPE_STEP = 1.0
// The 3D chase camera's tilt from straight down [rad]: about 25 degrees above the horizon
const CHASE_POLAR = 1.13

type HeightFn = ((x: number, y: number) => number) | null

// On the world the lanes lie on dark asphalt in both themes, so they keep the dark theme's colours
const ASPHALT_KEYS = new Set<keyof SceneTheme>(['edge', 'divider', 'center', 'virtual', 'centerline'])

/** Splits a polyline so no piece is longer than `step`, for following the ground. */
function densify(p: XY[], step: number): XY[] {
  const out: XY[] = [p[0]]
  for (let i = 1; i < p.length; i++) {
    const [ax, ay] = p[i - 1]
    const [bx, by] = p[i]
    const n = Math.ceil(Math.hypot(bx - ax, by - ay) / step)
    for (let k = 1; k <= n; k++) out.push([ax + ((bx - ax) * k) / n, ay + ((by - ay) * k) / n])
  }
  return out
}

function segmentsOf(lines: XY[][], z: number, h: HeightFn = null): number[] {
  const out: number[] = []
  for (const line of lines) {
    if (line.length < 2) continue
    const p = h ? densify(line, DRAPE_STEP) : line
    let pz = h ? h(p[0][0], p[0][1]) + z : z
    for (let i = 1; i < p.length; i++) {
      const qz = h ? h(p[i][0], p[i][1]) + z : z
      out.push(p[i - 1][0], p[i - 1][1], pz, p[i][0], p[i][1], qz)
      pz = qz
    }
  }
  return out
}

function lineMat(color: number, width: number, dashed = false, dash = 3, gap = 3): LineMaterial {
  const m = new LineMaterial({ color, linewidth: width, worldUnits: false, dashed, dashSize: dash, gapSize: gap })
  m.depthWrite = false
  return m
}

/** RViz-style TF axes: red x, green y, blue z. */
function makeAxes(length: number, radius: number): THREE.Group {
  const g = new THREE.Group()
  const geo = new THREE.CylinderGeometry(radius, radius, length, 12)
  geo.translate(0, length / 2, 0)
  const make = (color: number, rot: (m: THREE.Mesh) => void) => {
    const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color }))
    rot(m)
    g.add(m)
  }
  make(0xff0000, (m) => m.rotateZ(-Math.PI / 2)) // +y -> +x
  make(0x00ff00, () => {}) // +y
  make(0x0000ff, (m) => m.rotateX(Math.PI / 2)) // +y -> +z
  return g
}

/** OrbitControls' azimuth (z up) that puts the camera behind a shuttle heading `yaw`, so the heading points up the screen. */
function azimuthOf(yaw: number): number {
  return yaw - Math.PI / 2
}

export class MapScene {
  readonly renderer: THREE.WebGLRenderer
  readonly scene = new THREE.Scene()
  private ortho: THREE.OrthographicCamera
  private persp: THREE.PerspectiveCamera
  private controls: MapControls
  private mode: CameraMode = 'ortho'
  private theme: SceneTheme = THEMES.dark
  private themeName: ThemeName = 'dark'
  private width = 1
  private height = 1
  private raf = 0
  private resizeObs: ResizeObserver
  private lineMats: { mat: LineMaterial; key: keyof SceneTheme }[] = []
  private laneMats = new Set<LineMaterial>()
  private meshMats: { mat: { color: THREE.Color }; key: keyof SceneTheme }[] = []

  private gridGroup = new THREE.Group()
  private laneGroup = new THREE.Group()
  private centerGroup = new THREE.Group()
  private obstacleGroup = new THREE.Group()
  private pathLine: Line2
  private pathGeo = new LineGeometry()
  private trailLine: THREE.Line
  private trailPos = new Float32Array(TRAIL_MAX * 3)
  private trailCount = 0
  private goalGroup = new THREE.Group()
  private shuttle = new THREE.Group()
  private mapAxes: THREE.Group
  private bounds = { minX: -50, minY: -50, maxX: 50, maxY: 50 }
  private follow = false
  private follower = new FollowSmoother()
  private lastFrame = 0
  /** The glide onto the shuttle when the follow turns on: from this target, span, azimuth and tilt, over `ms`. */
  private glide: {
    t0: number
    ms: number
    from: THREE.Vector3
    fromSpan: number
    toSpan: number
    fromAz: number
    fromPolar: number
  } | null = null
  private lastPose: Pose2D | null = null
  /** Plays the 10 Hz states back smoothly; `shown` is the pose drawn this frame, `shownEst` the estimate. */
  private interp = new PoseInterpolator()
  private shown: Pose2D | null = null
  private shownEst: Pose2D | null = null
  private down: { x: number; y: number; t: number } | null = null
  private arrowKind: 'goal' | 'pose' = 'goal'
  private dragging = false
  private perceivedGroup = new THREE.Group()
  private arrow = new THREE.Group()
  private arrowMat: THREE.MeshBasicMaterial
  private poseEst: THREE.LineLoop
  private obstacleMats: THREE.Material[] = []
  private perceivedMat: LineMaterial | null = null
  // The optional 3D world and what is drawn on it, kept so a toggle can redraw it draped
  private worldScene = new THREE.Scene()
  private world: LoadedWorld | null = null
  private worldOn = false
  private worldLights: { hemi: THREE.HemisphereLight; sun: THREE.DirectionalLight }
  private map: LaneletMap | null = null
  private last: {
    path: XY[]
    goal: XY | null
    obstacles: { polys: XY[][]; selected: number }
    perceived: XY[][] | null
    pose: Pose2D | null
    poseEst: Pose2D | null
  } = { path: [], goal: null, obstacles: { polys: [], selected: -1 }, perceived: null, pose: null, poseEst: null }
  private frames = 0

  constructor(
    private canvas: HTMLCanvasElement,
    private events: MapSceneEvents = {},
  ) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true })
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    // The loop clears and renders the world and the overlays itself (see loop())
    this.renderer.autoClear = false
    this.renderer.info.autoReset = false

    this.ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, -1000, 5000)
    this.ortho.up.set(0, 0, 1)
    this.ortho.position.set(0, 0, 500)
    this.persp = new THREE.PerspectiveCamera(50, 1, 0.5, 10000)
    this.persp.up.set(0, 0, 1)

    this.controls = new MapControls(this.ortho, canvas)
    this.controls.enableDamping = true
    this.controls.dampingFactor = 0.15
    this.controls.zoomToCursor = true
    this.configureControls()

    this.scene.add(this.gridGroup, this.laneGroup, this.centerGroup, this.obstacleGroup, this.perceivedGroup, this.goalGroup)

    // RViz's pose arrows: a flat arrow, 3 m long, for the goal or pose being dragged. Drawn last
    // and without the depth test, so the ground, the world and the lanes never hide it
    this.arrowMat = new THREE.MeshBasicMaterial({ color: this.theme.goal, side: THREE.DoubleSide, depthWrite: false, depthTest: false })
    const shape = new THREE.Shape()
    shape.moveTo(0, -0.18)
    shape.lineTo(2.1, -0.18)
    shape.lineTo(2.1, -0.55)
    shape.lineTo(3.0, 0)
    shape.lineTo(2.1, 0.55)
    shape.lineTo(2.1, 0.18)
    shape.lineTo(0, 0.18)
    shape.closePath()
    const arrowMesh = new THREE.Mesh(new THREE.ShapeGeometry(shape), this.arrowMat)
    arrowMesh.renderOrder = 100
    this.arrow.add(arrowMesh)
    this.arrow.visible = false
    this.scene.add(this.arrow)

    // The stack's pose (TF map -> base_link) as an outline of the footprint, beside the true box
    const front = VEHICLE.wheelBase / 2 + VEHICLE.frontOverhang
    const rear = VEHICLE.wheelBase / 2 + VEHICLE.rearOverhang
    const w = VEHICLE.width / 2
    const estMat = new THREE.LineBasicMaterial({ color: this.theme.poseEst })
    this.meshMats.push({ mat: estMat, key: 'poseEst' })
    this.poseEst = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(front, w, 0.08),
        new THREE.Vector3(-rear, w, 0.08),
        new THREE.Vector3(-rear, -w, 0.08),
        new THREE.Vector3(front, -w, 0.08),
        new THREE.Vector3(front + 0.8, 0, 0.08),
      ]),
      estMat,
    )
    this.poseEst.visible = false
    this.scene.add(this.poseEst)

    // Path (nav_msgs/Path in RViz: a green line)
    const pathMat = this.track(lineMat(this.theme.path, 4), 'path')
    this.pathLine = new Line2(this.pathGeo, pathMat)
    this.pathLine.visible = false
    this.pathLine.renderOrder = 5
    this.scene.add(this.pathLine)

    // Trail of driven poses (like an Odometry display with a long history)
    const trailGeo = new THREE.BufferGeometry()
    trailGeo.setAttribute('position', new THREE.BufferAttribute(this.trailPos, 3))
    trailGeo.setDrawRange(0, 0)
    const trailMat = new THREE.LineBasicMaterial({ color: this.theme.trail, transparent: true, opacity: 0.6 })
    this.meshMats.push({ mat: trailMat, key: 'trail' })
    this.trailLine = new THREE.Line(trailGeo, trailMat)
    this.trailLine.frustumCulled = false
    this.scene.add(this.trailLine)

    const hemi = new THREE.HemisphereLight(0xdfe8f5, 0x5a5040, 1.6)
    hemi.position.set(0, 0, 1)
    const sun = new THREE.DirectionalLight(0xffffff, 1.5)
    sun.position.set(-120, -200, 300)
    this.worldLights = { hemi, sun }
    this.worldScene.add(hemi, sun)

    this.buildGoal()
    this.buildShuttle()
    this.mapAxes = makeAxes(4, 0.12)
    this.scene.add(this.mapAxes)
    this.buildGrid()

    // Capture phase on the parent: a tool sees the press before MapControls starts a pan
    canvas.parentElement?.addEventListener('pointerdown', this.onToolDown, true)
    canvas.addEventListener('pointerdown', this.onPointerDown)
    canvas.addEventListener('pointerup', this.onPointerUp)
    canvas.addEventListener('pointermove', this.onPointerMove)
    this.resizeObs = new ResizeObserver(() => this.resize())
    this.resizeObs.observe(canvas.parentElement ?? canvas)
    this.resize()
    this.setTheme('dark')
    this.loop(performance.now())
  }

  // ------------------------------------------------------------ public API

  setTheme(name: ThemeName) {
    this.themeName = name
    this.theme = THEMES[name]
    // The world is a little dimmer on the dark background, so the RViz colours stay readable
    this.worldLights.hemi.intensity = name === 'dark' ? 1.25 : 1.6
    this.worldLights.sun.intensity = name === 'dark' ? 1.1 : 1.5
    for (const { mat, key } of this.lineMats) mat.color.setHex(this.colourOf(key))
    for (const { mat, key } of this.meshMats) mat.color.setHex(this.theme[key])
    this.arrowMat.color.setHex(this.theme[this.arrowKind])
  }

  getTheme(): ThemeName {
    return this.themeName
  }

  /** Draws the lanelet map and fits the view to it. */
  setMap(map: LaneletMap) {
    this.map = map
    this.drawLanes()
    this.bounds = map.bounds
    this.buildGrid()
    this.fit()
  }

  private drawLanes() {
    const map = this.map
    if (!map) return
    for (const g of [this.laneGroup, this.centerGroup])
      for (const c of g.children) (c as LineSegments2).geometry.dispose()
    this.lineMats = this.lineMats.filter((e) => !this.laneMats.has(e.mat))
    for (const m of this.laneMats) m.dispose()
    this.laneMats.clear()
    this.laneGroup.clear()
    this.centerGroup.clear()
    const h = this.heightFn()
    const byKind = new Map<string, XY[][]>()
    for (const b of map.borders) {
      const k = `${b.kind}:${b.dashed ? 'd' : 's'}`
      const list = byKind.get(k) ?? []
      list.push(b.points)
      byKind.set(k, list)
    }
    const widths: Record<BorderKind, number> = { edge: 1.6, divider: 1.3, center: 1.6, virtual: 1, stop_line: 2.5 }
    for (const [k, lines] of byKind) {
      const [kind, style] = k.split(':') as [BorderKind, string]
      const key: keyof SceneTheme = kind === 'stop_line' ? 'edge' : kind
      const geo = new LineSegmentsGeometry().setPositions(segmentsOf(lines, Z_LANES + this.lift(), h))
      const mat = this.track(lineMat(this.colourOf(key), widths[kind], style === 'd', 3, 3), key)
      this.laneMats.add(mat)
      const obj = new LineSegments2(geo, mat)
      if (style === 'd') obj.computeLineDistances()
      this.laneGroup.add(obj)
    }
    const cgeo = new LineSegmentsGeometry().setPositions(segmentsOf(map.lanelets.map((l) => l.centerline), Z_CENTER + this.lift(), h))
    const cmat = this.track(lineMat(this.colourOf('centerline'), 1, true, 1, 1.5), 'centerline')
    this.laneMats.add(cmat)
    const cl = new LineSegments2(cgeo, cmat)
    cl.computeLineDistances()
    this.centerGroup.add(cl)
  }

  // ------------------------------------------------------------ the 3D world

  /** Adds the loaded world (world.ts) and shows it. */
  setWorld(world: LoadedWorld) {
    if (this.world) {
      this.worldScene.remove(this.world.root)
      this.world.dispose()
    }
    this.world = world
    this.worldScene.add(world.root)
    this.setWorldVisible(true)
  }

  setWorldVisible(v: boolean) {
    const on = v && this.world !== null
    if (on === this.worldOn) return
    this.worldOn = on
    this.gridGroup.visible = !on
    this.redrape()
  }

  isWorldVisible(): boolean {
    return this.worldOn
  }

  setTreesVisible(v: boolean) {
    if (this.world) this.world.trees.visible = v
  }

  /** Ground height under a map point: the world's when it is shown, else 0. */
  groundAt(x: number, y: number): number {
    return this.worldOn && this.world ? this.world.height.at(x, y) : 0
  }

  /** Frames drawn so far and what the last frame cost (for the perf check). */
  stats() {
    const r = this.renderer.info.render
    return { frames: this.frames, calls: r.calls, triangles: r.triangles, worldOn: this.worldOn }
  }

  private heightFn(): HeightFn {
    const w = this.world
    return this.worldOn && w ? (x, y) => w.height.at(x, y) : null
  }

  private colourOf(key: keyof SceneTheme): number {
    return this.worldOn && ASPHALT_KEYS.has(key) ? THEMES.dark[key] : this.theme[key]
  }

  private lift(): number {
    return this.worldOn ? DRAPE : 0
  }

  /** Redraws everything that sits on the ground after the world was shown or hidden. */
  private redrape() {
    this.drawLanes()
    this.setPath(this.last.path)
    this.setGoal(this.last.goal)
    this.setObstacles(this.last.obstacles.polys, this.last.obstacles.selected)
    this.setPerceived(this.last.perceived)
    if (this.shown) this.placeShuttle(this.shown)
    this.drawPoseEst(this.shownEst)
    for (let i = 0; i < this.trailCount; i++) {
      this.trailPos[3 * i + 2] = this.groundAt(this.trailPos[3 * i], this.trailPos[3 * i + 1]) + Z_TRAIL + this.lift()
    }
    ;(this.trailLine.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true
    this.controls.target.z = this.groundAt(this.controls.target.x, this.controls.target.y)
  }

  setCenterlinesVisible(v: boolean) {
    this.centerGroup.visible = v
  }

  setPath(points: XY[]) {
    this.last.path = points
    if (points.length < 2) {
      this.pathLine.visible = false
      return
    }
    const pos: number[] = []
    const h = this.heightFn()
    for (const p of h ? densify(points, DRAPE_STEP) : points) pos.push(p[0], p[1], Z_PATH + this.lift() + (h ? h(p[0], p[1]) : 0))
    this.pathGeo.dispose()
    this.pathGeo = new LineGeometry()
    this.pathGeo.setPositions(pos)
    this.pathLine.geometry = this.pathGeo
    this.pathLine.computeLineDistances()
    this.pathLine.visible = true
  }

  setGoal(goal: XY | null) {
    this.last.goal = goal
    this.goalGroup.visible = goal !== null
    if (goal) this.goalGroup.position.set(goal[0], goal[1], Z_GOAL + this.lift() + this.groundAt(goal[0], goal[1]))
  }

  /** The world's obstacles; the one at `selected` gets a strong outline. */
  setObstacles(polys: XY[][], selected = -1) {
    this.last.obstacles = { polys, selected }
    for (const c of this.obstacleGroup.children) (c as THREE.Mesh).geometry.dispose()
    for (const m of this.obstacleMats) m.dispose()
    this.obstacleGroup.clear()
    const fill = new THREE.MeshBasicMaterial({ color: this.theme.obstacle, transparent: true, opacity: 0.35, depthWrite: false })
    const edge = new THREE.LineBasicMaterial({ color: this.theme.obstacle })
    const sel = new THREE.LineBasicMaterial({ color: this.theme.selected })
    this.obstacleMats = [fill, edge, sel]
    this.meshMats = this.meshMats.filter((e) => e.key !== 'obstacle' && e.key !== 'selected')
    this.meshMats.push({ mat: fill, key: 'obstacle' }, { mat: edge, key: 'obstacle' }, { mat: sel, key: 'selected' })
    polys.forEach((poly, i) => {
      if (poly.length < 3) return
      // On the world, an obstacle sits flat at the highest ground under its corners
      const z0 = this.worldOn ? Math.max(...poly.map(([x, y]) => this.groundAt(x, y))) + this.lift() : 0
      const shape = new THREE.Shape(poly.map(([x, y]) => new THREE.Vector2(x, y)))
      const mesh = new THREE.Mesh(new THREE.ShapeGeometry(shape), fill)
      mesh.position.z = Z_OBST + z0
      this.obstacleGroup.add(mesh)
      const pts = [...poly, poly[0]].map(([x, y]) => new THREE.Vector3(x, y, Z_OBST + z0 + 0.01))
      this.obstacleGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), i === selected ? sel : edge))
    })
  }

  /** What /obstacles held, in the map frame, dashed. null or [] hides them. */
  setPerceived(polys: XY[][] | null) {
    this.last.perceived = polys
    for (const c of this.perceivedGroup.children) (c as LineSegments2).geometry.dispose()
    this.perceivedGroup.clear()
    if (!polys || polys.length === 0) return
    // One fat dashed material for every call (tracked once for the theme and the resolution)
    this.perceivedMat ??= this.track(lineMat(this.theme.perceived, 2.5, true, 0.5, 0.35), 'perceived')
    const closed = polys.filter((p) => p.length >= 2).map((p) => [...p, p[0]])
    const geo = new LineSegmentsGeometry().setPositions(segmentsOf(closed, Z_OBST + 0.03 + this.lift(), this.heightFn()))
    const line = new LineSegments2(geo, this.perceivedMat)
    line.computeLineDistances()
    line.renderOrder = 6
    this.perceivedGroup.add(line)
  }

  /** The arrow of a 2D Goal or Set pose being placed, or null to hide it. */
  setArrow(a: ArrowSpec | null) {
    this.arrow.visible = a !== null
    if (!a) return
    this.arrowKind = a.kind
    this.arrowMat.color.setHex(this.theme[a.kind])
    this.arrow.position.set(a.x, a.y, Z_GOAL + 0.02 + this.lift() + this.groundAt(a.x, a.y))
    this.arrow.rotation.set(0, 0, a.yaw)
    // 3 m long, or about 70 px when zoomed out, so it reads at any zoom
    const k = Math.max(1, (70 * this.metresPerPixel()) / 3)
    this.arrow.scale.set(k, k, 1)
  }

  /**
   * A state from the sim: sim time `t` [s], the true pose (the box), the pose the stack believes
   * (an outline, null hides it) and whether the sim is paused. Both are drawn interpolated between
   * states, a little behind the newest (interp.ts).
   */
  setState(t: number, pose: Pose2D, poseEst: Pose2D | null, paused: boolean) {
    this.last.pose = pose
    this.last.poseEst = poseEst
    this.interp.push(t, pose, poseEst, paused, performance.now())
  }

  /** Shows the shuttle at `p` at once, without interpolating (the estimate stays as it is). */
  setPose(p: Pose2D) {
    this.last.pose = p
    this.interp.reset()
    this.interp.push(0, p, this.last.poseEst, true, performance.now())
  }

  private drawPoseEst(p: Pose2D | null) {
    this.poseEst.visible = p !== null
    if (!p) return
    this.poseEst.position.set(p.x, p.y, this.groundAt(p.x, p.y) + this.lift())
    this.poseEst.rotation.set(0, 0, p.yaw)
  }

  /** The box on the ground: on the world it follows the slope under its wheels (display only). */
  private placeShuttle(p: Pose2D) {
    if (!this.worldOn) {
      this.shuttle.position.set(p.x, p.y, 0)
      this.shuttle.rotation.set(0, 0, p.yaw)
      return
    }
    const c = Math.cos(p.yaw)
    const s = Math.sin(p.yaw)
    const half = VEHICLE.wheelBase / 2
    const side = VEHICLE.width / 2
    const zf = this.groundAt(p.x + half * c, p.y + half * s)
    const zb = this.groundAt(p.x - half * c, p.y - half * s)
    const zl = this.groundAt(p.x - side * s, p.y + side * c)
    const zr = this.groundAt(p.x + side * s, p.y - side * c)
    const pitch = Math.atan2(zf - zb, 2 * half)
    const roll = Math.atan2(zl - zr, 2 * side)
    this.shuttle.position.set(p.x, p.y, (zf + zb + zl + zr) / 4 + 0.02)
    this.shuttle.rotation.set(roll, -pitch, p.yaw, 'ZYX')
  }

  /** Draws the shuttle (base_link) at the pose of this frame and extends the trail. */
  private showPose(p: Pose2D, est: Pose2D | null) {
    this.shown = p
    this.shownEst = est
    this.drawPoseEst(est)
    this.shuttle.visible = true
    this.placeShuttle(p)
    const last = this.lastPose
    if (!last || Math.hypot(p.x - last.x, p.y - last.y) > 0.25) {
      if (last && Math.hypot(p.x - last.x, p.y - last.y) > 20) this.clearTrail()
      if (this.trailCount < TRAIL_MAX) {
        this.trailPos.set([p.x, p.y, Z_TRAIL + this.lift() + this.groundAt(p.x, p.y)], this.trailCount * 3)
        this.trailCount++
        const attr = this.trailLine.geometry.getAttribute('position') as THREE.BufferAttribute
        attr.needsUpdate = true
        this.trailLine.geometry.setDrawRange(0, this.trailCount)
      }
      this.lastPose = { ...p }
    }
  }

  clearTrail() {
    this.trailCount = 0
    this.trailLine.geometry.setDrawRange(0, 0)
  }

  /**
   * Follows the shuttle and turns with its heading. The view glides onto it over `ms`, turning to
   * the heading on the way (and in 3D tilting to the chase view), and from a view wider than 200 m
   * it also zooms in to 60 m around it. Panning and zooming to the cursor would move the target
   * away from the shuttle for a frame, so both are off while following. The heading holds the
   * view's azimuth: OrbitControls clamps it to one angle, so right drag cannot turn the view but
   * can still tilt it in 3D. Turning the follow off frees the azimuth and leaves the view as it is.
   */
  setFollow(v: boolean, ms = 900) {
    this.follow = v
    this.controls.enablePan = !v
    this.controls.zoomToCursor = !v
    this.glide = null
    this.follower.reset()
    if (!v) {
      this.controls.minAzimuthAngle = -Infinity
      this.controls.maxAzimuthAngle = Infinity
      return
    }
    if (!this.last.pose) return
    const span = this.metresPerPixel() * this.height
    this.glide = {
      t0: performance.now(),
      ms,
      from: this.controls.target.clone(),
      fromSpan: span,
      toSpan: span > 200 ? 60 : span,
      fromAz: this.controls.getAzimuthalAngle(),
      fromPolar: this.controls.getPolarAngle(),
    }
  }

  /**
   * One frame of the follow: the smoothed pose moves the target (and the camera with it) and its
   * heading sets the azimuth. During the glide the view eases from where it was towards the
   * smoothed pose, so it lands on a moving shuttle.
   */
  private stepFollow(now: number, dt: number) {
    const p = this.shown
    if (!p) return
    const s = this.follower.update(p, dt)
    const to = new THREE.Vector3(s.x, s.y, this.groundAt(s.x, s.y))
    const toAz = azimuthOf(s.yaw)
    const g = this.glide
    if (!g) {
      this.moveTarget(to)
      this.setAzimuth(toAz)
      return
    }
    const u = Math.min(1, (now - g.t0) / g.ms)
    const k = u < 0.5 ? 4 * u * u * u : 1 - (-2 * u + 2) ** 3 / 2 // ease in and out
    const target = g.from.clone().lerp(to, k)
    const span = Math.exp(Math.log(g.fromSpan) + (Math.log(g.toSpan) - Math.log(g.fromSpan)) * k)
    const az = g.fromAz + wrapAngle(toAz - g.fromAz) * k
    this.moveTarget(target)
    this.setAzimuth(az)
    if (this.mode === 'ortho') {
      this.ortho.zoom = this.height / span
      this.ortho.updateProjectionMatrix()
    } else {
      const polar = g.fromPolar + (CHASE_POLAR - g.fromPolar) * k
      this.orbit(az, polar, span / (2 * Math.tan((this.persp.fov * Math.PI) / 360)))
    }
    if (u >= 1) this.glide = null
  }

  /** Moves the controls' target to `to`, and the camera by the same step. */
  private moveTarget(to: THREE.Vector3) {
    const d = to.clone().sub(this.controls.target)
    this.controls.target.add(d)
    this.camera().position.add(d)
  }

  /** Holds the view's azimuth at `az` (OrbitControls' angle); the next controls.update() turns to it. */
  private setAzimuth(az: number) {
    const a = wrapAngle(az)
    this.controls.minAzimuthAngle = a
    this.controls.maxAzimuthAngle = a
  }

  /** Puts the 3D camera `dist` from the target at OrbitControls' azimuth `az` and tilt `polar` from straight down. */
  private orbit(az: number, polar: number, dist: number) {
    const t = this.controls.target
    const h = Math.sin(polar) * dist
    // OrbitControls' azimuth 0 looks north from the south; it grows towards the west
    this.persp.position.set(t.x + h * Math.sin(az), t.y - h * Math.cos(az), t.z + Math.cos(polar) * dist)
  }

  /** Centres the view on a map point with about `span` metres visible vertically. */
  focus(x: number, y: number, span: number) {
    this.glide = null
    const d = new THREE.Vector3(x, y, this.groundAt(x, y)).sub(this.controls.target)
    this.controls.target.add(d)
    if (this.mode === 'ortho') {
      this.ortho.position.set(x, y, 500)
      this.ortho.zoom = this.height / span
      this.ortho.updateProjectionMatrix()
    } else {
      const offset = this.persp.position.clone().sub(this.controls.target.clone().sub(d)).normalize()
      const dist = span / (2 * Math.tan((this.persp.fov * Math.PI) / 360))
      this.persp.position.copy(this.controls.target).addScaledVector(offset, dist)
    }
    this.controls.update()
  }

  setCameraMode(mode: CameraMode) {
    if (mode === this.mode) return
    const target = this.controls.target.clone()
    const metresPerPx = this.metresPerPixel()
    this.mode = mode
    if (mode === 'persp') {
      // Same footprint as the ortho view, seen from the south and above, or the chase view when following
      const span = metresPerPx * this.height
      const dist = span / (2 * Math.tan((this.persp.fov * Math.PI) / 360))
      this.persp.position.set(target.x, target.y - dist * 0.55, dist * 0.85)
      if (this.follow && !this.glide && isFinite(this.controls.minAzimuthAngle)) {
        this.controls.target.copy(target)
        this.orbit(this.controls.minAzimuthAngle, CHASE_POLAR, dist)
      }
    } else {
      this.ortho.position.set(target.x, target.y, 500)
      this.ortho.zoom = this.ortho.zoom || 1
    }
    this.controls.object = this.camera()
    this.configureControls()
    this.controls.target.copy(target)
    this.controls.update()
  }

  /** 3D view from `eye` towards `target` (map frame, metres; z above the ground under each point). */
  setView(eye: [number, number, number], target: [number, number, number]) {
    this.glide = null
    this.setCameraMode('persp')
    this.controls.target.set(target[0], target[1], target[2] + this.groundAt(target[0], target[1]))
    this.persp.position.set(eye[0], eye[1], eye[2] + this.groundAt(eye[0], eye[1]))
    this.controls.update()
  }

  getCameraMode(): CameraMode {
    return this.mode
  }

  /** Fits the camera to the map bounds, top-down and north up. */
  fit() {
    this.glide = null
    // Held at north up for this one update, then free again (the page stops the follow with it)
    this.controls.minAzimuthAngle = 0
    this.controls.maxAzimuthAngle = 0
    const b = this.bounds
    const cx = (b.minX + b.maxX) / 2
    const cy = (b.minY + b.maxY) / 2
    const w = (b.maxX - b.minX) * 1.08
    const h = (b.maxY - b.minY) * 1.08
    this.controls.target.set(cx, cy, this.groundAt(cx, cy))
    if (this.mode === 'ortho') {
      this.ortho.position.set(cx, cy, 500)
      this.ortho.zoom = Math.min(this.width / w, this.height / h)
      this.ortho.updateProjectionMatrix()
    } else {
      const span = Math.max(h, (w * this.height) / this.width)
      const dist = span / (2 * Math.tan((this.persp.fov * Math.PI) / 360))
      this.persp.position.set(cx, cy - dist * 0.55, this.controls.target.z + dist * 0.85)
    }
    this.controls.update()
    if (!this.follow) {
      this.controls.minAzimuthAngle = -Infinity
      this.controls.maxAzimuthAngle = Infinity
    }
  }

  /**
   * Map point under a canvas pixel, on the ground. That is the plane z = 0, or the world's ground
   * when it is shown. Buildings, bridges and trees are ignored, so a click behind them still
   * lands on the ground there.
   */
  pick(clientX: number, clientY: number): XY | null {
    const r = this.canvas.getBoundingClientRect()
    const ndc = new THREE.Vector2(((clientX - r.left) / r.width) * 2 - 1, -((clientY - r.top) / r.height) * 2 + 1)
    const ray = new THREE.Raycaster()
    ray.setFromCamera(ndc, this.camera())
    if (this.worldOn && this.world) {
      const hit = this.world.height.rayHit(ray.ray.origin, ray.ray.direction, this.lift())
      return hit ? [hit.x, hit.y] : null
    }
    const hit = new THREE.Vector3()
    if (!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), hit)) return null
    return [hit.x, hit.y]
  }

  /** Canvas pixel of a map point (for tests and overlays). */
  project(x: number, y: number): { x: number; y: number } {
    const v = new THREE.Vector3(x, y, this.worldOn ? this.groundAt(x, y) + this.lift() : 0).project(this.camera())
    const r = this.canvas.getBoundingClientRect()
    return { x: r.left + ((v.x + 1) / 2) * r.width, y: r.top + ((1 - v.y) / 2) * r.height }
  }

  dispose() {
    cancelAnimationFrame(this.raf)
    this.world?.dispose()
    this.resizeObs.disconnect()
    this.controls.dispose()
    this.canvas.parentElement?.removeEventListener('pointerdown', this.onToolDown, true)
    this.canvas.removeEventListener('pointerdown', this.onPointerDown)
    this.canvas.removeEventListener('pointerup', this.onPointerUp)
    this.canvas.removeEventListener('pointermove', this.onPointerMove)
    this.renderer.dispose()
  }

  // ------------------------------------------------------------ internals

  private camera(): THREE.Camera {
    return this.mode === 'ortho' ? this.ortho : this.persp
  }

  private metresPerPixel(): number {
    if (this.mode === 'ortho') return 1 / this.ortho.zoom
    const dist = this.persp.position.distanceTo(this.controls.target)
    return (2 * dist * Math.tan((this.persp.fov * Math.PI) / 360)) / this.height
  }

  private configureControls() {
    const c = this.controls
    if (this.mode === 'ortho') {
      // Top-down: right drag only spins about z, like RViz's TopDownOrtho view
      c.minPolarAngle = 0
      c.maxPolarAngle = 0
      c.minZoom = 0.2
      c.maxZoom = 400
    } else {
      c.minPolarAngle = 0
      c.maxPolarAngle = Math.PI * 0.47
      c.minDistance = 3
      c.maxDistance = 4000
    }
  }

  private track<T extends LineMaterial>(m: T, key: keyof SceneTheme): T {
    m.resolution.set(this.width, this.height)
    this.lineMats.push({ mat: m, key })
    return m
  }

  private buildGrid() {
    this.gridGroup.clear()
    this.meshMats = this.meshMats.filter((e) => e.key !== 'gridMinor' && e.key !== 'gridMajor')
    const b = this.bounds
    const minor = 5
    const major = 50
    const pad = 250
    const x0 = Math.floor((b.minX - pad) / major) * major
    const x1 = Math.ceil((b.maxX + pad) / major) * major
    const y0 = Math.floor((b.minY - pad) / major) * major
    const y1 = Math.ceil((b.maxY + pad) / major) * major
    const minorPts: number[] = []
    const majorPts: number[] = []
    for (let x = x0; x <= x1; x += minor) (x % major === 0 ? majorPts : minorPts).push(x, y0, Z_GRID, x, y1, Z_GRID)
    for (let y = y0; y <= y1; y += minor) (y % major === 0 ? majorPts : minorPts).push(x0, y, Z_GRID, x1, y, Z_GRID)
    const mk = (pts: number[], key: 'gridMinor' | 'gridMajor') => {
      const geo = new THREE.BufferGeometry()
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3))
      const mat = new THREE.LineBasicMaterial({ color: this.theme[key], depthWrite: false })
      this.meshMats.push({ mat, key })
      const l = new THREE.LineSegments(geo, mat)
      l.renderOrder = -1
      return l
    }
    this.gridGroup.add(mk(minorPts, 'gridMinor'), mk(majorPts, 'gridMajor'))
  }

  private buildGoal() {
    const ringMat = new THREE.MeshBasicMaterial({ color: this.theme.goal, side: THREE.DoubleSide })
    const discMat = new THREE.MeshBasicMaterial({ color: this.theme.goal, transparent: true, opacity: 0.25, depthWrite: false })
    this.meshMats.push({ mat: ringMat, key: 'goal' }, { mat: discMat, key: 'goal' })
    this.goalGroup.add(new THREE.Mesh(new THREE.RingGeometry(1.0, 1.35, 40), ringMat))
    this.goalGroup.add(new THREE.Mesh(new THREE.CircleGeometry(1.0, 40), discMat))
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 2.5, 8), ringMat)
    pole.rotateX(Math.PI / 2)
    pole.position.z = 1.25
    this.goalGroup.add(pole)
    this.goalGroup.visible = false
  }

  /** The box shuttle: real length, width and height, base_link mid-wheelbase, an orange nose. */
  private buildShuttle() {
    const front = VEHICLE.wheelBase / 2 + VEHICLE.frontOverhang
    const rear = VEHICLE.wheelBase / 2 + VEHICLE.rearOverhang
    const nose = 0.35
    const bodyLen = front + rear - nose
    const bodyMat = new THREE.MeshLambertMaterial({ color: this.theme.body })
    const frontMat = new THREE.MeshLambertMaterial({ color: this.theme.front })
    this.meshMats.push({ mat: bodyMat, key: 'body' })
    this.meshMats.push({ mat: frontMat, key: 'front' })
    const body = new THREE.Mesh(new THREE.BoxGeometry(bodyLen, VEHICLE.width, VEHICLE.height), bodyMat)
    body.position.set(-rear + bodyLen / 2, 0, VEHICLE.height / 2)
    const cap = new THREE.Mesh(new THREE.BoxGeometry(nose, VEHICLE.width, VEHICLE.height), frontMat)
    cap.position.set(front - nose / 2, 0, VEHICLE.height / 2)
    const outline = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(front + rear, VEHICLE.width, VEHICLE.height)),
      new THREE.LineBasicMaterial({ color: this.theme.bodyEdge }),
    )
    this.meshMats.push({ mat: outline.material as THREE.LineBasicMaterial, key: 'bodyEdge' })
    outline.position.set((front - rear) / 2, 0, VEHICLE.height / 2)
    const axes = makeAxes(2.2, 0.07)
    axes.position.z = VEHICLE.height + 0.05 // base_link axes, lifted so they show from above
    this.shuttle.add(body, cap, outline, axes)
    this.shuttle.visible = false
    this.scene.add(this.shuttle)
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.6))
    const sun = new THREE.DirectionalLight(0xffffff, 1.4)
    sun.position.set(-30, -50, 100)
    this.scene.add(sun)
  }

  private resize() {
    const parent = this.canvas.parentElement
    const w = Math.max(1, parent?.clientWidth ?? this.canvas.clientWidth)
    const h = Math.max(1, parent?.clientHeight ?? this.canvas.clientHeight)
    this.width = w
    this.height = h
    this.renderer.setSize(w, h, true)
    this.ortho.left = -w / 2
    this.ortho.right = w / 2
    this.ortho.top = h / 2
    this.ortho.bottom = -h / 2
    this.ortho.updateProjectionMatrix()
    this.persp.aspect = w / h
    this.persp.updateProjectionMatrix()
    for (const { mat } of this.lineMats) mat.resolution.set(w, h)
  }

  private loop = (now: number) => {
    this.raf = requestAnimationFrame(this.loop)
    const dt = this.lastFrame ? (now - this.lastFrame) / 1000 : 0
    this.lastFrame = now
    const s = this.interp.sample(now)
    if (s) this.showPose(s.pose, s.est)
    if (this.follow) this.stepFollow(now, dt)
    this.controls.update()
    const r = this.renderer
    const cam = this.camera()
    r.info.reset()
    r.setClearColor(this.theme.background)
    r.clear()
    if (this.worldOn) {
      r.render(this.worldScene, cam)
      // Top-down: the map's own drawing goes over roofs and trees, like RViz overlays
      if (this.mode === 'ortho') r.clearDepth()
    }
    r.render(this.scene, cam)
    this.frames++
  }

  private onToolDown = (e: PointerEvent) => {
    if (e.target !== this.canvas || e.button !== 0 || !this.events.onDragStart) return
    const p = this.pick(e.clientX, e.clientY)
    if (!p || !this.events.onDragStart(p[0], p[1])) return
    // Disabled before MapControls sees this press, so it neither pans nor captures the pointer
    this.controls.enabled = false
    this.dragging = true
    this.canvas.setPointerCapture?.(e.pointerId)
  }

  private onPointerDown = (e: PointerEvent) => {
    this.down = this.dragging ? null : { x: e.clientX, y: e.clientY, t: performance.now() }
  }

  private onPointerUp = (e: PointerEvent) => {
    if (this.dragging) {
      this.dragging = false
      this.controls.enabled = true
      const p = this.pick(e.clientX, e.clientY)
      if (p) this.events.onDragEnd?.(p[0], p[1])
      return
    }
    const d = this.down
    this.down = null
    if (!d || e.button !== 0) return
    if (Math.hypot(e.clientX - d.x, e.clientY - d.y) > 4 || performance.now() - d.t > 600) return
    const p = this.pick(e.clientX, e.clientY)
    if (p) this.events.onClick?.(p[0], p[1])
  }

  private onPointerMove = (e: PointerEvent) => {
    const p = this.pick(e.clientX, e.clientY)
    if (!p) return
    this.events.onHover?.(p[0], p[1])
    if (this.dragging) this.events.onDrag?.(p[0], p[1])
  }
}
