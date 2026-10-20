/*
 * File: useRosPublisher.ts
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Generic React hook for publishing messages to a ROS2 topic from the
 *   browser. The complement to useRosTopic — where that hook listens,
 *   this one sends.
 *
 *   How it works:
 *     1. Gets the shared ROS connection from useRos().
 *     2. Creates a roslib Topic object and stores it in a ref (a ref is like
 *        a variable that React doesn't re-render on — good for objects).
 *     3. Returns a stable publish(message) function that components can call
 *        whenever they want to send data to ROS2.
 *     4. Silently does nothing if the connection is not yet established
 *        (topicRef.current will be null), so callers don't need to guard.
 *
 *   Unlike useRosTopic, publishing does not involve React state — there is
 *   nothing to display, so no re-render is triggered.
 *
 * Usage:
 *   import { useRosPublisher } from '../hooks/useRosPublisher'
 *   const publish = useRosPublisher('/goal_pose', 'geometry_msgs/PoseStamped')
 *   publish({ header: ..., pose: ... })   // sends message to ROS2
 */

import { useCallback, useEffect, useRef } from 'react'
import ROSLIB from 'roslib'
import { useRos } from './useRos'

export function useRosPublisher(topicName: string, messageType: string) {
  const ros = useRos()
  const topicRef = useRef<ROSLIB.Topic | null>(null)

  useEffect(() => {
    if (!ros) {
      topicRef.current = null
      return
    }
    topicRef.current = new ROSLIB.Topic({ ros, name: topicName, messageType })
    return () => {
      topicRef.current = null
    }
  }, [ros, topicName, messageType])

  return useCallback((message: object) => {
    topicRef.current?.publish(new ROSLIB.Message(message))
  }, [])
}
