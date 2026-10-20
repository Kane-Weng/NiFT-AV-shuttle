/*
 * File: MapView.tsx
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   The main 2D map panel. Uses the react-leaflet library (a React wrapper
 *   around Leaflet.js) to render an interactive OpenStreetMap tile layer
 *   and overlay live data streamed from the ROS2 stack.
 *
 *   Layers drawn on the map (bottom to top):
 *     1. OpenStreetMap tiles      — background satellite/road imagery
 *     2. Lanelet2 road network    — blue lines from /lanelet_map_markers
 *                                   (the HD map the planner routes on)
 *     3. Planned trajectory       — solid cyan line from /planned_path
 *                                   (the route the vehicle is following)
 *     4. Obstacle polygon         — red filled shape from /obstacles
 *                                   (the convex hull of detected LiDAR clusters)
 *     5. Goal marker              — green dot placed by the user's click
 *     6. Vehicle arrow            — SVG arrow from /gnss/fix (position) and
 *                                   /gnss/heading_deg (rotation); points in the
 *                                   direction of travel so heading is obvious
 *
 *   Goal-setting flow:
 *     ControlPanel toggles goalMode → MapView adds class 'goal-mode-active'
 *     (changes cursor to crosshair) → user clicks → GoalClickHandler fires
 *     → lat/lon converted to ROS map-frame via latLonToMap() → published
 *     to /goal_pose as geometry_msgs/PoseStamped → waypoint_node plans route.
 *
 *   All ROS map-frame coordinates (meters from Mcity origin) are converted
 *   to [lat, lon] via mapToLatLon() before being passed to Leaflet.
 *
 * Usage:
 *   <MapView goalMode={goalMode} onGoalModeChange={setGoalMode} />
 */

import { useCallback, useMemo, useState } from 'react'
import {
  MapContainer,
  TileLayer,
  Polyline,
  Polygon,
  Marker,
  useMapEvents,
} from 'react-leaflet'
import L from 'leaflet'
import { useRosTopic } from '../hooks/useRosTopic'
import { useRosPublisher } from '../hooks/useRosPublisher'
import { mapToLatLon, latLonToMap, MAP_ORIGIN_LAT, MAP_ORIGIN_LON } from '../utils/coordinates'
import type { NavSatFix, Path, PolygonStamped, MarkerArray, Float64Msg } from '../types/ros'

const MAP_CENTER: [number, number] = [MAP_ORIGIN_LAT, MAP_ORIGIN_LON]
const LINE_STRIP = 4
const LINE_LIST = 5

const goalIcon = L.divIcon({
  className: '',
  html: `<div style="
    width:14px;height:14px;
    background:#22c55e;
    border:2.5px solid #15803d;
    border-radius:50%;
    transform:translate(-50%,-50%);
    box-shadow:0 0 6px #22c55e88;
  "></div>`,
  iconAnchor: [0, 0],
  iconSize: [0, 0],
})

function GoalClickHandler({
  onGoalSet,
  active,
}: {
  onGoalSet: (lat: number, lon: number) => void
  active: boolean
}) {
  useMapEvents({
    click(e) {
      if (active) onGoalSet(e.latlng.lat, e.latlng.lng)
    },
  })
  return null
}

export default function MapView({
  goalMode,
  onGoalModeChange,
}: {
  goalMode: boolean
  onGoalModeChange: (v: boolean) => void
}) {
  const [goalMarker, setGoalMarker] = useState<[number, number] | null>(null)

  const gnssFix = useRosTopic<NavSatFix>('/gnss/fix', 'sensor_msgs/NavSatFix')
  const headingMsg = useRosTopic<Float64Msg>('/gnss/heading_deg', 'std_msgs/Float64')
  const plannedPath = useRosTopic<Path>('/planned_path', 'nav_msgs/Path')
  const obstacles = useRosTopic<PolygonStamped>('/obstacles', 'geometry_msgs/PolygonStamped')
  // Lanelet map markers update at 1 Hz — no throttle needed
  const laneletMarkers = useRosTopic<MarkerArray>(
    '/lanelet_map_markers',
    'visualization_msgs/msg/MarkerArray',
  )

  const publishGoal = useRosPublisher('/goal_pose', 'geometry_msgs/PoseStamped')

  const pathPositions = useMemo<[number, number][]>(() => {
    if (!plannedPath) return []
    return plannedPath.poses.map(p => mapToLatLon(p.pose.position.x, p.pose.position.y))
  }, [plannedPath])

  // Accumulate all lanelet geometry into one multi-polyline for performance.
  // LINE_STRIP: all points form one continuous line.
  // LINE_LIST:  each consecutive pair is an independent segment.
  // action !== 0 markers (DELETE / DELETEALL) are skipped.
  const laneletLines = useMemo<[number, number][][]>(() => {
    if (!laneletMarkers?.markers) return []
    const lines: [number, number][][] = []
    for (const m of laneletMarkers.markers) {
      if (m.action !== 0) continue
      if (!Array.isArray(m.points) || m.points.length < 2) continue
      if (m.type === LINE_STRIP) {
        lines.push(m.points.map(p => mapToLatLon(p.x, p.y)))
      } else if (m.type === LINE_LIST) {
        for (let i = 0; i + 1 < m.points.length; i += 2) {
          lines.push([
            mapToLatLon(m.points[i].x,     m.points[i].y),
            mapToLatLon(m.points[i + 1].x, m.points[i + 1].y),
          ])
        }
      }
    }
    return lines
  }, [laneletMarkers])

  const obstaclePositions = useMemo<[number, number][]>(() => {
    if (!obstacles || obstacles.polygon.points.length < 3) return []
    return obstacles.polygon.points.map(p => mapToLatLon(p.x, p.y))
  }, [obstacles])

  const vehiclePos: [number, number] | null = gnssFix
    ? [gnssFix.latitude, gnssFix.longitude]
    : null

  const vehicleIcon = useMemo(() => {
    const deg = headingMsg?.data ?? 0
    return L.divIcon({
      className: '',
      html: `<div style="transform:rotate(${deg}deg);width:28px;height:28px;">
        <svg width="28" height="28" viewBox="0 0 28 28" xmlns="http://www.w3.org/2000/svg">
          <polygon points="14,2 25,24 14,19 3,24" fill="#3b82f6" stroke="#ffffff" stroke-width="2" stroke-linejoin="round"/>
        </svg>
      </div>`,
      iconAnchor: [14, 14],
      iconSize: [28, 28],
    })
  }, [headingMsg?.data])

  const handleGoalSet = useCallback(
    (lat: number, lon: number) => {
      setGoalMarker([lat, lon])
      onGoalModeChange(false)
      const { x, y } = latLonToMap(lat, lon)
      publishGoal({
        header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x, y, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        },
      })
    },
    [publishGoal, onGoalModeChange],
  )

  return (
    <div className={`relative w-full h-full ${goalMode ? 'goal-mode-active' : ''}`}>
      <MapContainer
        center={MAP_CENTER}
        zoom={18}
        style={{ width: '100%', height: '100%' }}
        zoomControl
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />

        <GoalClickHandler onGoalSet={handleGoalSet} active={goalMode} />

        {/* Lanelet2 road network — single multi-polyline is far cheaper than N components */}
        {laneletLines.length > 0 && (
          <Polyline
            positions={laneletLines}
            color="#60a5fa"
            weight={1.5}
            opacity={0.55}
          />
        )}

        {/* Planned trajectory */}
        {pathPositions.length > 1 && (
          <Polyline
            positions={pathPositions}
            color="#06b6d4"
            weight={4}
            opacity={0.9}
          />
        )}

        {/* Latest obstacle footprint */}
        {obstaclePositions.length >= 3 && (
          <Polygon
            positions={obstaclePositions}
            color="#f87171"
            fillColor="#f87171"
            fillOpacity={0.25}
            weight={2}
          />
        )}

        {/* Goal marker */}
        {goalMarker && <Marker position={goalMarker} icon={goalIcon} />}

        {/* Vehicle — rendered last so it sits on top */}
        {vehiclePos && <Marker position={vehiclePos} icon={vehicleIcon} />}
      </MapContainer>

      {/* Map legend */}
      <div className="absolute bottom-10 left-2 z-[1000] bg-gray-900/80 text-xs text-gray-300 rounded px-2 py-1 space-y-0.5 pointer-events-none">
        <div className="flex items-center gap-1.5">
          <span className="inline-block w-3 h-0.5 bg-blue-400 opacity-60" />
          Lanelets
        </div>
        <div className="flex items-center gap-1.5">
          <span className="inline-block w-3 h-0.5" style={{ background: '#06b6d4', height: '2px' }} />
          Planned path
        </div>
        <div className="flex items-center gap-1.5">
          <span className="inline-block" style={{ fontSize: 10 }}>▲</span>
          Vehicle
        </div>
        <div className="flex items-center gap-1.5">
          <span className="inline-block w-3 h-3 bg-red-400 opacity-50" />
          Obstacle
        </div>
      </div>
    </div>
  )
}
