/*
 * File: MapView.tsx
 *
 * Description:
 *   The map area: the three.js canvas, an RViz-like tool bar on top and a status bar below.
 *   It feeds the MapScene from the SimClient without re-rendering React on every state.
 *
 *   Map tools (panels/ui.ts), as in RViz: 2D Goal (G) and 2D Pose Estimate (P) take a click, or a
 *   drag that gives the heading; Obstacle (O) places one per click. With no tool, a press on an
 *   obstacle selects it and a drag moves it (one obstacle_move on release), Delete removes it,
 *   Escape drops the tool. Anything else pans and rotates the view.
 */

import { useEffect, useRef, useState } from 'react'
import { MapScene, type CameraMode } from './MapScene'
import type { LaneletMap } from '../map/lanelets'
import type { ThemeName } from './theme'
import { useSimClient } from '../sim/useSim'
import { getUi, setUi, subscribeUi as subscribeSelection, toggleTool, useUi, OBSTACLE_SIZES } from '../panels/ui'
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

function ToolButton(props: { active?: boolean; onClick: () => void; children: React.ReactNode; title?: string; testid?: string }) {
  return (
    <button
      title={props.title}
      data-testid={props.testid}
      onClick={props.onClick}
      className={`px-2.5 py-1 text-xs rounded border transition-colors ${
        props.active
          ? 'bg-sky-600 border-sky-500 text-white'
          : 'bg-white/90 dark:bg-gray-900/90 border-gray-300 dark:border-gray-700 text-gray-700 dark:text-gray-200 hover:bg-gray-100 dark:hover:bg-gray-800'
      }`}
    >
      {props.children}
    </button>
  )
}

export default function MapView({ map, theme }: Props) {
  const client = useSimClient()
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const sceneRef = useRef<MapScene | null>(null)
  const cursorRef = useRef<HTMLSpanElement>(null)
  const { tool } = useUi()
  const [cam, setCam] = useState<CameraMode>(() => (new URLSearchParams(location.search).get('cam') === 'persp' ? 'persp' : 'ortho'))
  const [centerlines, setCenterlines] = useState(false)
  const [follow, setFollow] = useState(false)

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
      onHover: (x, y) => {
        if (cursorRef.current) cursorRef.current.textContent = `x ${x.toFixed(2)}  y ${y.toFixed(2)}`
      },
    })
    sceneRef.current = scene
    scene.setCenterlinesVisible(false)
    // Pose on every state; path, goal and obstacles only when the snapshot holds a new one
    let prev: Partial<typeof client.snapshot> = {}
    const unsub = client.subscribe((s) => {
      if (s.state && s.state !== prev.state) {
        scene.setPose(s.state.pose)
        scene.setPoseEst(s.state.poseEst)
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
      <div className={`relative flex-1 min-h-0 ${tool ? 'cursor-crosshair' : 'cursor-grab active:cursor-grabbing'}`}>
        <canvas ref={canvasRef} data-testid="map-canvas" className="block w-full h-full" />
        <div className="absolute top-2 left-2 flex flex-wrap gap-1.5">
          <ToolButton testid="tool-goal" active={tool === 'goal'} onClick={() => toggleTool('goal')} title="Click the map for a goal, drag for its heading (G)">
            2D Goal
          </ToolButton>
          <ToolButton testid="tool-pose" active={tool === 'pose'} onClick={() => toggleTool('pose')} title="Click and drag: the shuttle's position and heading, the sim teleports (P)">
            2D Pose Estimate
          </ToolButton>
          <ToolButton testid="tool-obstacle" active={tool === 'obstacle'} onClick={() => toggleTool('obstacle')} title="Click the map to place an obstacle; drag one to move it, Delete removes it (O)">
            Obstacle
          </ToolButton>
          <span className="w-px bg-gray-400/40 mx-0.5" />
          <ToolButton active={cam === 'ortho'} onClick={() => setCam('ortho')} title="Top-down orthographic">
            Top-down
          </ToolButton>
          <ToolButton active={cam === 'persp'} onClick={() => setCam('persp')} title="Perspective, right drag tilts">
            3D
          </ToolButton>
          <ToolButton onClick={() => sceneRef.current?.fit()} title="Fit the map">
            Fit
          </ToolButton>
          <ToolButton active={follow} onClick={() => setFollow(!follow)} title="Keep the shuttle in view">
            Follow
          </ToolButton>
          <span className="w-px bg-gray-400/40 mx-0.5" />
          <ToolButton active={centerlines} onClick={() => setCenterlines(!centerlines)} title="Lanelet centrelines">
            Centerlines
          </ToolButton>
        </div>
        {!map && (
          <div className="absolute inset-0 flex items-center justify-center text-sm text-gray-400">Loading map…</div>
        )}
      </div>
      <div className="flex items-center gap-4 px-3 py-1 text-[11px] font-mono border-t border-gray-200 dark:border-gray-800 bg-white dark:bg-gray-900 text-gray-500 dark:text-gray-400 shrink-0">
        <span>Fixed frame: map</span>
        <span className="hidden lg:inline">
          <span className="text-gray-400">shuttle box: truth · </span>
          <span className="text-cyan-600 dark:text-cyan-400">outline: stack's pose</span>
        </span>
        <span ref={cursorRef} data-testid="cursor">x -  y -</span>
        <span className="ml-auto">{map ? `${map.lanelets.length} lanelets` : ''}</span>
      </div>
    </div>
  )
}
