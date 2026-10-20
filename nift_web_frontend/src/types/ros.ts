/*
 * File: ros.ts
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   TypeScript type definitions that mirror the ROS2 message types used in
 *   the NIFT stack. When rosbridge delivers a message over WebSocket it
 *   arrives as a plain JSON object. These interfaces tell TypeScript what
 *   shape that object has, so the rest of the code gets autocompletion and
 *   type checking instead of working with untyped 'any' objects.
 *
 *   Each interface maps 1-to-1 with its ROS2 counterpart:
 *     NavSatFix        → sensor_msgs/NavSatFix        (GPS position)
 *     TwistStamped     → geometry_msgs/TwistStamped   (velocity)
 *     Path             → nav_msgs/Path                (planned route)
 *     PolygonStamped   → geometry_msgs/PolygonStamped (obstacle hull)
 *     MarkerArray      → visualization_msgs/MarkerArray (lanelet2 lines)
 *     Float64Msg       → std_msgs/Float64              (heading degrees)
 *     StringMsg        → std_msgs/String               (nav status text)
 *     BoolMsg          → std_msgs/Bool                 (e-stop / offroad flags)
 *     DiagnosticStatus → diagnostic_msgs/DiagnosticStatus (per-node health)
 *     DiagnosticArray  → diagnostic_msgs/DiagnosticArray  (all node health)
 *
 * Usage:
 *   Import the specific type you need in any component or hook:
 *     import type { NavSatFix } from '../types/ros'
 */

export interface Header {
  stamp: { sec: number; nanosec: number }
  frame_id: string
}

export interface NavSatFix {
  header: Header
  latitude: number
  longitude: number
  altitude: number
  status: { status: number; service: number }
}

export interface TwistStamped {
  header: Header
  twist: {
    linear: { x: number; y: number; z: number }
    angular: { x: number; y: number; z: number }
  }
}

export interface Point {
  x: number
  y: number
  z: number
}

export interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

export interface Pose {
  position: Point
  orientation: Quaternion
}

export interface PoseStamped {
  header: Header
  pose: Pose
}

export interface Path {
  header: Header
  poses: PoseStamped[]
}

export interface PolygonStamped {
  header: Header
  polygon: {
    points: Point[]
  }
}

export interface ColorRGBA {
  r: number
  g: number
  b: number
  a: number
}

export interface Marker {
  header: Header
  ns: string
  id: number
  type: number  // 4 = LINE_STRIP, 5 = LINE_LIST
  action: number
  pose: Pose
  scale: Point
  color: ColorRGBA
  points: Point[]
}

export interface MarkerArray {
  markers: Marker[]
}

export interface Float64Msg {
  data: number
}

export interface StringMsg {
  data: string
}

export interface BoolMsg {
  data: boolean
}

export interface DiagnosticStatus {
  level: number  // 0=OK, 1=WARN, 2=ERROR, 3=STALE
  name: string
  message: string
  hardware_id: string
  values: { key: string; value: string }[]
}

export interface DiagnosticArray {
  header: Header
  status: DiagnosticStatus[]
}
