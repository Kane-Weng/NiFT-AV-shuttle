/*
 * File: obstacles.ts
 *
 * Description:
 *   Collects the obstacle polygons of one LiDAR frame and moves them into the
 *   map frame, the same way trajectory_controller does.
 *
 *   lidar_node publishes /obstacles in base_link, one PolygonStamped per
 *   obstacle. All polygons of one LiDAR frame carry the frame's stamp, and a
 *   frame with no obstacle sends one empty polygon. So a polygon with the
 *   current stamp joins the current frame. A new stamp starts a new frame. An
 *   empty polygon clears the frame.
 *
 *   The map needs map coordinates. Each polygon is moved with the latest
 *   map → base_link transform when it arrives. base_link stays level, so a
 *   2D rotation by the yaw plus the translation is exact. Without a transform
 *   a polygon keeps only its base_link points. The radar still shows it, the
 *   map does not.
 *
 *   No React and no roslib here, so test/obstacles.test.ts runs it in Node.
 *
 * Usage:
 *   import { addObstacleMessage, poseFromTf } from '../utils/obstacles'
 */

import type { PolygonStamped, TFMessage } from '../types/ros'

/** Pose of base_link in map. x and y in metres, yaw in radians (0 = +x, counter-clockwise) */
export interface Pose2D {
  x: number
  y: number
  yaw: number
}

export type XY = [number, number]

/** One obstacle: its outline in base_link, and in map when a pose was known */
export interface Obstacle {
  base: XY[]
  map: XY[] | null
}

/** The obstacles of the latest LiDAR frame */
export interface ObstacleFrame {
  stamp: string
  obstacles: Obstacle[]
}

export const EMPTY_FRAME: ObstacleFrame = { stamp: '', obstacles: [] }

const stripSlash = (frame: string) => (frame.startsWith('/') ? frame.slice(1) : frame)

/** The last parent → child transform in a /tf message as a 2D pose, or null if it has none */
export function poseFromTf(msg: TFMessage, parent = 'map', child = 'base_link'): Pose2D | null {
  let pose: Pose2D | null = null
  for (const t of msg.transforms ?? []) {
    if (stripSlash(t.header.frame_id) !== parent || stripSlash(t.child_frame_id) !== child) continue
    const q = t.transform.rotation
    pose = {
      x: t.transform.translation.x,
      y: t.transform.translation.y,
      yaw: Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
    }
  }
  return pose
}

/** base_link points → map points, for base_link at pose */
export function baseToMap(points: XY[], pose: Pose2D): XY[] {
  const c = Math.cos(pose.yaw)
  const s = Math.sin(pose.yaw)
  return points.map(([x, y]) => [c * x - s * y + pose.x, s * x + c * y + pose.y])
}

/** Add one /obstacles message to the frame. pose is the latest map → base_link, or null */
export function addObstacleMessage(
  frame: ObstacleFrame,
  msg: PolygonStamped,
  pose: Pose2D | null,
): ObstacleFrame {
  const stamp = `${msg.header.stamp.sec}.${msg.header.stamp.nanosec}`
  const points = msg.polygon?.points ?? []
  if (points.length === 0) return { stamp, obstacles: [] }
  const base: XY[] = points.map(p => [p.x, p.y])
  const obstacle: Obstacle = { base, map: pose ? baseToMap(base, pose) : null }
  if (stamp === frame.stamp) return { stamp, obstacles: [...frame.obstacles, obstacle] }
  return { stamp, obstacles: [obstacle] }
}
