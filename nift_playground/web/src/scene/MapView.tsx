/*
 * File: MapView.tsx
 *
 * Description:
 *   The map area: the three.js canvas with an RViz-like tool bar on top. It feeds the MapScene from
 *   the SimClient without re-rendering React on every state.
 *
 *   Map tools (panels/ui.ts), as in RViz: 2D Goal (G) and Set pose (P) take a click, or a drag
 *   that gives the heading; Obstacle (O) places a car per click, and a click on an obstacle removes
 *   it. Select (Escape) is no tool: a press on an obstacle selects it and a drag moves it (one
 *   obstacle_move on release), Delete removes it. Anything else pans and rotates the view. Follow keeps
 *   the shuttle in the middle and turns the view with its heading: top-down the shuttle points up the
 *   screen, in 3D the camera chases it from behind. The view glides after it, so GNSS noise does not
 *   shake it. While following, the pan and the turn are locked, the wheel zooms about the shuttle
 *   and in 3D right drag still tilts. The page starts on the whole map, then glides onto the shuttle
 *   and follows it, so a first visit finds it. ?follow=0 stays on the map.
 *
 *   The settings window turns the centerlines and the 3D world on and off through panels/ui.ts.
 *   The world (scene/world.ts) is off by default. The first time it is turned on, the module and
 *   its files (about 2 MB) load, with progress over the map. ?world=1 turns it on at start.
 */

import { useEffect, useRef, useState } from 'react'
import { MapScene, type CameraMode } from './MapScene'
import type { LaneletMap } from '../map/lanelets'
import type { ThemeName } from './theme'
import { useSimClient } from '../sim/useSim'
import { getUi, setUi, subscribeUi as subscribeSelection, toggleTool, useUi, OBSTACLE_SIZES, type WorldLoad } from '../panels/ui'
import { FitIcon, FollowIcon, GoalIcon, ObstacleIcon, PerspIcon, PoseIcon, SelectIcon, TopDownIcon } from '../panels/icons'
import { historyOf } from '../panels/history'
import { centroid, nextObstacleId, pointInPolygon, translate } from '../panels/geom'

interface Props {
  map: LaneletMap | null
  theme: ThemeName
}

/** A drag shorter than this sends no heading (the goal) or keeps the shuttle's (the pose). */
const MIN_ARROW_M = 0.5

type Gesture =
  | { kind: 'goal' | 'pose'; x: number; y: number; yaw: number | null }
  | { kind: 'move'; id: string; x0: number; y0: number; dx: number; dy: number }

declare global {
  interface Window {
    __playground?: Record<string, unknown>
  }
}

const BAR = 'flex rounded-md border border-gray-300 dark:border-gray-700 bg-white/90 dark:bg-gray-900/90 shadow-sm overflow-hidden'

/** A button of a tool bar group: an icon, and a label from the sm breakpoint up. */
function ToolButton(props: { active?: boolean; onClick: () => void; icon: React.ReactNode; label: string; title?: string; testid?: string }) {
  return (
    <button
      title={props.title}
      aria-label={props.label}
      aria-pressed={props.active}
      data-testid={props.testid}
      onClick={props.onClick}
      className={`flex items-center gap-1.5 px-2 py-1 text-xs transition-colors [&+&]:border-l border-gray-300 dark:border-gray-700 ${
        props.active ? 'bg-sky-600 text-white' : 'text-gray-700 dark:text-gray-200 hover:bg-gray-100 dark:hover:bg-gray-800'
      }`}
    >
      {props.icon}
      <span className="hidden sm:inline">{props.label}</span>
    </button>
  )
}

export default function MapView({ map, theme }: Props) {
  const client = useSimClient()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const sceneRef = useRef<MapScene | null>(null)
  const { tool, centerlines, world, worldLoad } = useUi()
  const [cam, setCam] = useState<CameraMode>(() => (new URLSearchParams(location.search).get('cam') === 'persp' ? 'persp' : 'ortho'))
  const [follow, setFollow] = useState(false)
  const setWorldLoad = (worldLoad: WorldLoad) => setUi({ worldLoad })

  useEffect(() => {
    const h = historyOf(client)
    let gesture: Gesture | null = null
    // Obstacles as drawn: the snapshot's, with a drag (or a sent move not yet echoed) applied
    let preview: { id: string; dx: number; dy: number } | null = null
    const drawObstacles = () => {
      const { obstacles } = client.snapshot
      const sel = getUi().selected
      scene.setObstacles(
        obstacles.map((o) => (preview && o.oid === preview.id ? translate(o.poly, preview.dx, preview.dy) : o.poly)),
        obstacles.findIndex((o) => o.oid === sel),
      )
    }
    const hit = (x: number, y: number): string | null => {
      const obs = client.snapshot.obstacles
      for (let i = obs.length - 1; i >= 0; i--) if (pointInPolygon([x, y], obs[i].poly)) return obs[i].oid
      return null
    }

    const scene = new MapScene(canvasRef.current!, {
      onDragStart: (x, y) => {
        const t = getUi().tool
        if (t === 'goal' || t === 'pose') {
          gesture = { kind: t, x, y, yaw: null }
          return true
        }
        if (t === 'obstacle') {
          const there = hit(x, y)
          if (there) {
            void h.command({ type: 'obstacle_remove', oid: there }, `Remove ${there}`)
            if (getUi().selected === there) setUi({ selected: null })
            return true
          }
          const id = nextObstacleId(client.snapshot.obstacles)
          const sz = OBSTACLE_SIZES[getUi().obstacleSize]
          const yaw = client.snapshot.state?.pose.yaw ?? 0
          void h.command({ type: 'obstacle_add', oid: id, box: { x, y, yaw, length: sz.length, width: sz.width } }, `Add ${id}`)
          setUi({ selected: id })
          return true
        }
        const id = hit(x, y)
        if (!id) return false
        setUi({ selected: id })
        drawObstacles()
        gesture = { kind: 'move', id, x0: x, y0: y, dx: 0, dy: 0 }
        return true
      },
      onDrag: (x, y) => {
        const g = gesture
        if (!g) return
        if (g.kind === 'move') {
          g.dx = x - g.x0
          g.dy = y - g.y0
          preview = { id: g.id, dx: g.dx, dy: g.dy }
          drawObstacles()
        } else {
          const far = Math.hypot(x - g.x, y - g.y) >= MIN_ARROW_M
          g.yaw = far ? Math.atan2(y - g.y, x - g.x) : null
          scene.setArrow(far ? { x: g.x, y: g.y, yaw: g.yaw!, kind: g.kind } : null)
        }
      },
      onDragEnd: () => {
        const g = gesture
        gesture = null
        scene.setArrow(null)
        if (!g) return
        if (g.kind === 'goal') {
          client.send(g.yaw === null ? { type: 'goal', x: g.x, y: g.y } : { type: 'goal', x: g.x, y: g.y, yaw: g.yaw })
          setUi({ tool: null })
        } else if (g.kind === 'pose') {
          const yaw = g.yaw ?? client.snapshot.state?.pose.yaw ?? 0
          void h.command({ type: 'initialpose', x: g.x, y: g.y, yaw }, `Initial pose (${g.x.toFixed(1)}, ${g.y.toFixed(1)})`)
          setUi({ tool: null })
        } else if (g.kind === 'move') {
          const o = client.snapshot.obstacles.find((q) => q.oid === g.id)
          if (!o || Math.hypot(g.dx, g.dy) <= 0.05) {
            preview = null
            return
          }
          const c = centroid(o.poly)
          void h.command({ type: 'obstacle_move', oid: g.id, x: c[0] + g.dx, y: c[1] + g.dy }, `Move ${g.id}`)
        }
      },
      onClick: () => {
        // A click on the empty map drops the obstacle selection
        if (getUi().selected) setUi({ selected: null })
      },
    })
    sceneRef.current = scene
    scene.setCenterlinesVisible(false)
    // Pose on every state; path, goal and obstacles only when the snapshot holds a new one
    let prev: Partial<typeof client.snapshot> = {}
    const unsub = client.subscribe((s) => {
      if (s.state && s.state !== prev.state) {
        scene.setState(s.state.t, s.state.pose, s.state.poseEst, s.state.paused)
      }
      if (s.path !== prev.path) scene.setPath(s.path)
      if (s.goal !== prev.goal) scene.setGoal(s.goal)
      if (s.obstacles !== prev.obstacles) {
        preview = null // the worker's echo replaces the drag preview
        drawObstacles()
      }
      if (s.perceived !== prev.perceived) scene.setPerceived(s.perceived)
      prev = s
    })
    let selPrev = getUi().selected
    const unsubUi = subscribeSelection(() => {
      if (getUi().selected !== selPrev) {
        selPrev = getUi().selected
        drawObstacles()
      }
    })
    window.__playground = { ...(window.__playground ?? {}), scene, client }
    return () => {
      unsub()
      unsubUi()
      scene.dispose()
      sceneRef.current = null
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [client])

  useEffect(() => {
    if (map && sceneRef.current) {
      sceneRef.current.setMap(map)
      window.__playground = { ...(window.__playground ?? {}), map, mapReady: true }
    }
  }, [map])
  useEffect(() => sceneRef.current?.setTheme(theme), [theme])
  useEffect(() => sceneRef.current?.setCameraMode(cam), [cam])
  useEffect(() => sceneRef.current?.setCenterlinesVisible(centerlines), [centerlines])
  useEffect(() => sceneRef.current?.setFollow(follow), [follow])

  // Once the map and the first pose are in, a short look at the whole map, then onto the shuttle
  useEffect(() => {
    if (!map || new URLSearchParams(location.search).get('follow') === '0') return
    let timer = 0
    let started = false
    const off = client.subscribe((s) => {
      if (!s.state || started) return
      started = true
      timer = window.setTimeout(() => setFollow(true), 700)
    })
    return () => {
      off()
      clearTimeout(timer)
    }
  }, [map, client])

  // The world loads once, on first use; later toggles only show or hide it
  useEffect(() => {
    const scene = sceneRef.current
    if (!scene) return
    if (!world) {
      scene.setWorldVisible(false)
      return
    }
    if (worldLoad.state === 'ready') {
      scene.setWorldVisible(true)
      return
    }
    if (worldLoad.state === 'loading') return
    setWorldLoad({ state: 'loading', pct: 0 })
    const t0 = performance.now()
    import('./world')
      .then(({ loadWorld }) => loadWorld((f) => setWorldLoad({ state: 'loading', pct: Math.round(100 * f) })))
      .then((w) => {
        if (sceneRef.current !== scene) return w.dispose()
        scene.setWorld(w)
        scene.setWorldVisible(getUi().world)
        setWorldLoad({ state: 'ready', pct: 100 })
        window.__playground = {
          ...(window.__playground ?? {}),
          worldReady: true,
          worldInfo: { ...w.manifest.stats, loadMs: Math.round(performance.now() - t0), bytes: w.bytes },
        }
      })
      .catch((e: unknown) => {
        console.warn('world failed to load', e)
        setWorldLoad({ state: 'error', pct: 0, error: String(e) })
        setUi({ world: false })
      })
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [world, client])

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target
      if (t instanceof HTMLInputElement || t instanceof HTMLSelectElement || t instanceof HTMLTextAreaElement) return
      if (e.ctrlKey || e.metaKey || e.altKey) return
      const k = e.key.toLowerCase()
      if (k === 'g') toggleTool('goal')
      else if (k === 'p') toggleTool('pose')
      else if (k === 'o') toggleTool('obstacle')
      else if (e.key === 'Escape') setUi({ tool: null, selected: null })
      else if ((e.key === 'Delete' || e.key === 'Backspace') && getUi().selected) {
        const id = getUi().selected!
        void historyOf(client).command({ type: 'obstacle_remove', oid: id }, `Remove ${id}`)
        setUi({ selected: null })
      }
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [client])

  return (
    <div className="relative w-full h-full flex flex-col">
      <div className={`relative flex-1 min-h-0 ${tool ? 'cursor-crosshair' : follow ? 'cursor-default' : 'cursor-grab active:cursor-grabbing'}`}>
        <canvas ref={canvasRef} data-testid="map-canvas" className="block w-full h-full" />
        <div className="absolute top-2 left-2 right-2 flex flex-wrap gap-2 pointer-events-none [&>*]:pointer-events-auto">
          <div className={BAR} role="radiogroup" aria-label="Map tools" data-tour="tools">
            <ToolButton testid="tool-select" active={tool === null} onClick={() => setUi({ tool: null })} icon={<SelectIcon />} label="Select" title="Select (Esc): drag the view, or drag an obstacle to move it. Delete removes the selected one" />
            <ToolButton testid="tool-goal" active={tool === 'goal'} onClick={() => toggleTool('goal')} icon={<GoalIcon />} label="2D Goal" title="2D Goal (G): click the map, or drag to give the heading" />
            <ToolButton testid="tool-pose" active={tool === 'pose'} onClick={() => toggleTool('pose')} icon={<PoseIcon />} label="Set pose" title="Set pose (P): click and drag for the shuttle's position and heading. The sim teleports and the stack gets /initialpose" />
            <ToolButton testid="tool-obstacle" active={tool === 'obstacle'} onClick={() => toggleTool('obstacle')} icon={<ObstacleIcon />} label="Obstacle" title="Obstacle (O): click the road to place a car, click an obstacle to remove it" />
          </div>
          <div className={BAR} role="radiogroup" aria-label="Camera" data-tour="camera">
            <ToolButton testid="cam-ortho" active={cam === 'ortho'} onClick={() => setCam('ortho')} icon={<TopDownIcon />} label="Top-down" title="Top-down view. Right drag rotates" />
            <ToolButton testid="cam-persp" active={cam === 'persp'} onClick={() => setCam('persp')} icon={<PerspIcon />} label="3D" title="3D view. Right drag rotates and tilts" />
          </div>
          <div className={BAR}>
            <ToolButton
              testid="view-fit"
              onClick={() => {
                setFollow(false)
                sceneRef.current?.fit()
              }}
              icon={<FitIcon />}
              label="Fit"
              title="Fit the whole map (stops Follow)"
            />
            <ToolButton testid="view-follow" active={follow} onClick={() => setFollow(!follow)} icon={<FollowIcon />} label="Follow" title="Follow the shuttle: it stays in the middle and the view turns with it (top-down it points up, 3D chases it from behind). The wheel zooms about it; dragging does not pan or turn" />
          </div>
        </div>
        {worldLoad.state === 'loading' && (
          <div data-testid="world-loading" className="absolute bottom-2 left-2 px-2 py-1 text-xs rounded bg-white/90 dark:bg-gray-900/90 border border-gray-300 dark:border-gray-700 text-gray-700 dark:text-gray-200">
            Loading the 3D world… {worldLoad.pct}%
          </div>
        )}
        {worldLoad.state === 'error' && (
          <div data-testid="world-error" className="absolute bottom-2 left-2 px-2 py-1 text-xs rounded bg-red-50 dark:bg-red-950 border border-red-300 dark:border-red-800 text-red-700 dark:text-red-300">
            The 3D world did not load: {worldLoad.error}
          </div>
        )}
        {!map && (
          <div className="absolute inset-0 flex items-center justify-center text-sm text-gray-400">Loading map…</div>
        )}
      </div>
    </div>
  )
}
