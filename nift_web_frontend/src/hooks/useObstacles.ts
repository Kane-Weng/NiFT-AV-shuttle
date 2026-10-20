/*
 * File: useObstacles.ts
 *
 * Description:
 *   React hook that returns the obstacles of the latest LiDAR frame from
 *   /obstacles. utils/obstacles.ts explains how the polygons of one frame
 *   are collected.
 *
 *   With inMap = true the hook also reads /tf and keeps the latest
 *   map → base_link transform in a ref. It changes often and nothing needs
 *   to re-render for it. Each polygon is then moved into map with the
 *   transform at its arrival, as trajectory_controller does.
 *
 *   /obstacles is never throttled. A throttle would drop polygons of a frame.
 *
 * Usage:
 *   const frame = useObstacles(true)   // frame.obstacles[i].map for the map
 *   const frame = useObstacles()       // frame.obstacles[i].base for the radar
 */

import { useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRos } from './useRos'
import { addObstacleMessage, EMPTY_FRAME, poseFromTf } from '../utils/obstacles'
import type { ObstacleFrame, Pose2D } from '../utils/obstacles'
import type { PolygonStamped, TFMessage } from '../types/ros'

export function useObstacles(inMap = false): ObstacleFrame {
  const ros = useRos()
  const poseRef = useRef<Pose2D | null>(null)
  const [frame, setFrame] = useState<ObstacleFrame>(EMPTY_FRAME)

  useEffect(() => {
    if (!ros || !inMap) return
    const tf = new ROSLIB.Topic({ ros, name: '/tf', messageType: 'tf2_msgs/TFMessage' })
    const callback = (msg: ROSLIB.Message) => {
      const pose = poseFromTf(msg as unknown as TFMessage)
      if (pose) poseRef.current = pose
    }
    tf.subscribe(callback)
    return () => {
      tf.unsubscribe(callback)
    }
  }, [ros, inMap])

  useEffect(() => {
    if (!ros) return
    const topic = new ROSLIB.Topic({ ros, name: '/obstacles', messageType: 'geometry_msgs/PolygonStamped' })
    const callback = (msg: ROSLIB.Message) => {
      const pose = poseRef.current   // the pose at arrival, not when React runs the update
      setFrame(prev => addObstacleMessage(prev, msg as unknown as PolygonStamped, pose))
    }
    topic.subscribe(callback)
    return () => {
      topic.unsubscribe(callback)
    }
  }, [ros])

  return frame
}
