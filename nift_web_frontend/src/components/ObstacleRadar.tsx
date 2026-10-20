/*
 * File: ObstacleRadar.tsx
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Canvas-based forward proximity radar panel for the right sidebar.
 *   Subscribes to /obstacles (geometry_msgs/PolygonStamped) which the lidar
 *   node publishes in the base_link frame (vehicle-relative coordinates).
 *   Because the data is already relative, no coordinate transforms are needed.
 *
 *   Canvas coordinate mapping (ROS base_link → screen pixels):
 *     canvas_x = VX − ros_y × SCALE   (ROS +y = left  → screen left)
 *     canvas_y = VY − ros_x × SCALE   (ROS +x = fwd   → screen up)
 *   Vehicle origin (VX, VY) sits near the bottom of the canvas so the
 *   forward hemisphere fills the majority of the display area.
 *
 *   Distance model: all ring radii represent clearance from the vehicle's
 *   bounding edge (half-diagonal ≈ 1.72 m), matching last_obs_dist in
 *   the telemetry feed.  Rings are drawn at (ring.m + VEHICLE_RADIUS) from
 *   the vehicle center so they align with the physical bounding circle.
 *
 *   Layers (bottom → top):
 *     1. Background
 *     2. Corridor half-width guide rails
 *     3. Bounding circle (faint) + distance arcs (upper semicircle only)
 *     4. Obstacle polygon (red)
 *     5. Shuttle rectangle + forward arrow
 *     6. Direction labels + status badge
 */

import { useEffect, useRef, useState } from 'react'
import { useRosTopic } from '../hooks/useRosTopic'
import type { PolygonStamped } from '../types/ros'

const CANVAS_W = 220
const CANVAS_H = 140
const VX = CANVAS_W / 2          // vehicle x-origin in canvas (center)
const VY = CANVAS_H - 12         // vehicle y-origin in canvas (near bottom)
const RANGE_M = 11                // meters visible ahead
const SCALE = (VY - 10) / RANGE_M  // px per metre ≈ 10.7

// Shuttle physical dimensions [m]
const V_LEN = 3.055   // owner: nift_common/vehicle.yaml length (TS copy, tested)
const V_WID = 1.588   // owner: nift_common/vehicle.yaml width (TS copy, tested)
const VEHICLE_RADIUS_M = Math.sqrt(V_LEN ** 2 + V_WID ** 2) / 2  // ≈ 1.72 m

// Mirrors filter_utils._CORRIDOR_HALF
const CORRIDOR_HALF_M = V_WID / 2 + 0.3

// Rings represent clearance from bounding edge (matching last_obs_dist)
const RINGS = [
  { m: 0.8, color: '#7c3aed', dash: [2, 2] as number[] },  // collision zone → SAFE_STOP
  { m: 2,   color: '#dc2626', dash: [3, 3] as number[] },   // hard-stop → BLOCKED
  { m: 10,  color: '#374151', dash: [2, 4] as number[] },   // influence radius (filter boundary)
]
// Obstacles beyond the influence radius are filter-CLEAR; badge reflects this
const INFLUENCE_RADIUS_M = 10.0

function toCanvas(rosX: number, rosY: number): [number, number] {
  return [VX - rosY * SCALE, VY - rosX * SCALE]
}

export default function ObstacleRadar() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [showInfo, setShowInfo] = useState(false)
  const obstacles = useRosTopic<PolygonStamped>(
    '/obstacles',
    'geometry_msgs/PolygonStamped',
  )

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // ── Background ──────────────────────────────────────────────────────────
    ctx.clearRect(0, 0, CANVAS_W, CANVAS_H)
    ctx.fillStyle = '#030712'
    ctx.fillRect(0, 0, CANVAS_W, CANVAS_H)

    // ── Path corridor guide rails ───────────────────────────────────────────
    const corrPx = CORRIDOR_HALF_M * SCALE
    ctx.strokeStyle = '#1e3a5f'
    ctx.lineWidth = 1
    ctx.setLineDash([3, 5])
    ctx.beginPath()
    ctx.moveTo(VX - corrPx, 0); ctx.lineTo(VX - corrPx, VY)
    ctx.moveTo(VX + corrPx, 0); ctx.lineTo(VX + corrPx, VY)
    ctx.stroke()
    ctx.setLineDash([])

    // ── Bounding circle (faint) ─────────────────────────────────────────────
    ctx.beginPath()
    ctx.arc(VX, VY, VEHICLE_RADIUS_M * SCALE, Math.PI, 0, false)
    ctx.strokeStyle = '#1e3050'
    ctx.lineWidth = 1
    ctx.stroke()

    // ── Distance arcs (upper semicircle, origin = vehicle edge) ────────────
    for (const ring of RINGS) {
      const r = (ring.m + VEHICLE_RADIUS_M) * SCALE
      ctx.beginPath()
      ctx.arc(VX, VY, r, Math.PI, 0, false)
      ctx.strokeStyle = ring.color
      ctx.lineWidth = ring.m === 2 || ring.m === 0.8 ? 1.5 : 1
      ctx.setLineDash(ring.dash)
      ctx.stroke()
      ctx.setLineDash([])
      // Label at 3 o'clock on each arc
      ctx.fillStyle = ring.m === 0.8 ? '#7c3aed' : ring.m === 2 ? '#ef4444' : '#4b5563'
      ctx.font = '8px monospace'
      ctx.textAlign = 'left'
      ctx.fillText(`${ring.m}m`, VX + r + 2, VY - 2)
    }

    // ── Obstacle polygon ────────────────────────────────────────────────────
    const pts = obstacles?.polygon.points ?? []
    const hasObstacle = pts.length >= 3

    if (hasObstacle) {
      ctx.beginPath()
      const [x0, y0] = toCanvas(pts[0].x, pts[0].y)
      ctx.moveTo(x0, y0)
      for (let i = 1; i < pts.length; i++) {
        const [xi, yi] = toCanvas(pts[i].x, pts[i].y)
        ctx.lineTo(xi, yi)
      }
      ctx.closePath()
      ctx.fillStyle = 'rgba(239,68,68,0.18)'
      ctx.fill()
      ctx.strokeStyle = '#ef4444'
      ctx.lineWidth = 1.5
      ctx.stroke()
    }

    // ── Shuttle rectangle ───────────────────────────────────────────────────
    const vHalfL = (V_LEN / 2) * SCALE
    const vHalfW = (V_WID / 2) * SCALE

    ctx.fillStyle = '#1e3a8a'
    ctx.strokeStyle = '#3b82f6'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.rect(VX - vHalfW, VY - vHalfL, vHalfW * 2, vHalfL * 2)
    ctx.fill()
    ctx.stroke()

    // Forward arrowhead on shuttle nose
    ctx.fillStyle = '#60a5fa'
    ctx.beginPath()
    ctx.moveTo(VX,     VY - vHalfL - 5)
    ctx.lineTo(VX - 4, VY - vHalfL + 1)
    ctx.lineTo(VX + 4, VY - vHalfL + 1)
    ctx.closePath()
    ctx.fill()

    // ── Direction labels ────────────────────────────────────────────────────
    ctx.font = 'bold 8px monospace'
    ctx.fillStyle = '#4b5563'
    ctx.textAlign = 'center'
    ctx.fillText('FWD', VX, 10)
    ctx.textAlign = 'right'
    ctx.fillText('L', 11, VY - 4)
    ctx.textAlign = 'left'
    ctx.fillText('R', CANVAS_W - 2, VY - 4)

    // ── Status badge ────────────────────────────────────────────────────────
    // Compute edge clearance to match last_obs_dist (center_dist - VEHICLE_RADIUS)
    let minCenterDist = Infinity
    for (const pt of pts) {
      minCenterDist = Math.min(minCenterDist, Math.sqrt(pt.x * pt.x + pt.y * pt.y))
    }
    const edgeDist = Math.max(0, minCenterDist - VEHICLE_RADIUS_M)

    const obsActive   = hasObstacle && edgeDist < INFLUENCE_RADIUS_M
    const statusText  = obsActive ? `OBS  ${edgeDist.toFixed(1)} m` : 'CLEAR'
    const statusColor = obsActive ? '#ef4444' : '#22c55e'
    const badgeBg     = obsActive ? 'rgba(239,68,68,0.12)' : 'rgba(34,197,94,0.10)'
    const badgeW = 96, badgeH = 16

    ctx.fillStyle = badgeBg
    ctx.fillRect(VX - badgeW / 2, CANVAS_H - badgeH - 2, badgeW, badgeH)
    ctx.fillStyle = statusColor
    ctx.font = 'bold 10px monospace'
    ctx.textAlign = 'center'
    ctx.fillText(statusText, VX, CANVAS_H - 5)

  }, [obstacles])

  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center gap-1.5">
        <span className="text-xs font-semibold text-gray-400 uppercase tracking-wide">
          Proximity Radar
        </span>
        <button
          onClick={() => setShowInfo(v => !v)}
          className="w-4 h-4 rounded-full border border-gray-600 text-gray-500 hover:text-gray-300 hover:border-gray-400 text-xs leading-none flex items-center justify-center transition-colors"
          aria-label="Radar info"
        >
          ?
        </button>
      </div>

      {showInfo && (
        <div className="bg-gray-800 rounded p-2 text-xs text-gray-400 space-y-1">
          <div className="font-medium text-gray-300 mb-1">Radar key</div>
          {RINGS.map(r => (
            <div key={r.m} className="flex items-center gap-2">
              <span
                className="inline-block w-5 h-px shrink-0"
                style={{ background: r.color, border: `1px dashed ${r.color}` }}
              />
              <span>
                <span className="font-mono">{r.m} m</span>
                {r.m === 0.8 && ' — collision zone → SAFE_STOP'}
                {r.m === 2   && ' — hard-stop → BLOCKED (speed zeroed)'}
                {r.m === 10  && ' — influence radius (avoidance begins)'}
              </span>
            </div>
          ))}
          <div className="flex items-center gap-2 pt-0.5">
            <span className="inline-block w-5 shrink-0 border-l border-dashed border-blue-900" style={{ height: 12 }} />
            <span>
              <span className="font-mono">±{CORRIDOR_HALF_M.toFixed(1)} m</span>
              {' '}— driving corridor (speed penalty only applies inside)
            </span>
          </div>
          <p className="text-gray-600 pt-0.5">
            Distances measured from vehicle bounding edge (~{VEHICLE_RADIUS_M.toFixed(2)} m radius).
            Coordinates: base_link frame.
          </p>
        </div>
      )}

      <div className="flex justify-center">
        <canvas
          ref={canvasRef}
          width={CANVAS_W}
          height={CANVAS_H}
          className="rounded-md border border-gray-800"
        />
      </div>
    </div>
  )
}