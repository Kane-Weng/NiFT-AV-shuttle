// Tests for src/utils/obstacles.ts. Run with `npm test` (Node's own test runner, no extra packages).
import { test } from 'node:test'
import assert from 'node:assert/strict'

import { addObstacleMessage, baseToMap, EMPTY_FRAME, poseFromTf } from '../src/utils/obstacles.ts'
import type { Pose2D, XY } from '../src/utils/obstacles.ts'
import type { PolygonStamped, TFMessage } from '../src/types/ros.ts'

function polygon(sec: number, points: XY[]): PolygonStamped {
  return {
    header: { stamp: { sec, nanosec: 0 }, frame_id: 'base_link' },
    polygon: { points: points.map(([x, y]) => ({ x, y, z: 0 })) },
  }
}

function tf(parent: string, child: string, x: number, y: number, yaw: number): TFMessage {
  return {
    transforms: [{
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: parent },
      child_frame_id: child,
      transform: {
        translation: { x, y, z: 0 },
        rotation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
      },
    }],
  }
}

function close(actual: XY[], expected: XY[]) {
  assert.equal(actual.length, expected.length)
  actual.forEach(([x, y], i) => {
    assert.ok(Math.abs(x - expected[i][0]) < 1e-9 && Math.abs(y - expected[i][1]) < 1e-9,
      `point ${i}: got ${x}, ${y}, expected ${expected[i]}`)
  })
}

test('an obstacle ahead of the shuttle lands ahead of it on the map, not near the map origin', () => {
  // Shuttle at (100, 50) facing north (map yaw 90 deg). Obstacle 6 m ahead in base_link.
  const pose: Pose2D = { x: 100, y: 50, yaw: Math.PI / 2 }
  const frame = addObstacleMessage(EMPTY_FRAME, polygon(1, [[6, -1], [7, -1], [7, 1], [6, 1]]), pose)
  assert.equal(frame.obstacles.length, 1)
  close(frame.obstacles[0].base, [[6, -1], [7, -1], [7, 1], [6, 1]])
  close(frame.obstacles[0].map!, [[101, 56], [101, 57], [99, 57], [99, 56]])
})

test('base_link to map is the same 2D transform as trajectory_controller._obstacle_callback', () => {
  const pose: Pose2D = { x: -3, y: 4, yaw: 0.7 }
  const c = Math.cos(0.7), s = Math.sin(0.7)
  close(baseToMap([[2, 1]], pose), [[c * 2 - s * 1 - 3, s * 2 + c * 1 + 4]])
})

test('polygons with the same stamp form one frame, a new stamp replaces it, an empty one clears it', () => {
  let frame = addObstacleMessage(EMPTY_FRAME, polygon(1, [[5, 0], [6, 0], [6, 1]]), null)
  frame = addObstacleMessage(frame, polygon(1, [[8, 2], [9, 2], [9, 3]]), null)
  assert.equal(frame.obstacles.length, 2, 'both obstacles of frame 1')

  frame = addObstacleMessage(frame, polygon(2, [[4, 0], [5, 0], [5, 1]]), null)
  assert.equal(frame.obstacles.length, 1, 'frame 2 replaces frame 1')
  close(frame.obstacles[0].base, [[4, 0], [5, 0], [5, 1]])

  frame = addObstacleMessage(frame, polygon(3, []), null)
  assert.equal(frame.obstacles.length, 0, 'the clear polygon empties the frame')
})

test('without a pose an obstacle has no map outline but keeps its base_link one for the radar', () => {
  const frame = addObstacleMessage(EMPTY_FRAME, polygon(1, [[5, 0], [6, 0], [6, 1]]), null)
  assert.equal(frame.obstacles[0].map, null)
  assert.equal(frame.obstacles[0].base.length, 3)
})

test('poseFromTf picks map -> base_link and its yaw', () => {
  const pose = poseFromTf(tf('map', 'base_link', 12, -7, 1.2))
  assert.ok(pose)
  assert.equal(pose.x, 12)
  assert.equal(pose.y, -7)
  assert.ok(Math.abs(pose.yaw - 1.2) < 1e-12)
  assert.ok(poseFromTf(tf('/map', '/base_link', 1, 2, 0)), 'a leading slash is tolerated')
})

test('poseFromTf ignores other transforms', () => {
  assert.equal(poseFromTf(tf('base_link', 'velodyne', 1, 0, 0)), null)
  assert.equal(poseFromTf({ transforms: [] }), null)
})
