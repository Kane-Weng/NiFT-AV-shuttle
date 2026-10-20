/*
 * File: useRosTopic.ts
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Generic React hook for subscribing to a ROS2 topic and receiving its
 *   latest message as React state. Wraps the roslib Topic.subscribe() API
 *   so components never need to manage subscriptions manually.
 *
 *   How it works:
 *     1. Gets the shared ROS connection from useRos().
 *     2. Opens a roslib Topic subscription when the connection is available.
 *     3. Each incoming message triggers a React state update, which causes
 *        the component using this hook to re-render with the new data.
 *     4. Automatically unsubscribes (cleans up) when the component unmounts
 *        or when the topic name / connection changes.
 *
 *   The generic type parameter <T> lets callers declare what shape the
 *   message will have, e.g. useRosTopic<NavSatFix>(...), giving full
 *   TypeScript autocompletion on the returned value.
 *
 *   throttleRate (ms): optional rosbridge-side throttle. When set to e.g.
 *   100, rosbridge will only forward one message per 100 ms even if the
 *   topic publishes faster. Useful for high-frequency topics.
 *
 * Usage:
 *   import { useRosTopic } from '../hooks/useRosTopic'
 *   const fix = useRosTopic<NavSatFix>('/gnss/fix', 'sensor_msgs/NavSatFix')
 *   // fix is null until the first message arrives
 */

import { useEffect, useState } from 'react'
import ROSLIB from 'roslib'
import { useRos } from './useRos'

export function useRosTopic<T>(
  topicName: string,
  messageType: string,
  throttleRate = 0,
): T | null {
  const ros = useRos()
  const [message, setMessage] = useState<T | null>(null)

  useEffect(() => {
    if (!ros) return

    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType,
      throttle_rate: throttleRate,
    })

    const callback = (msg: ROSLIB.Message) => {
      setMessage(msg as unknown as T)
    }

    topic.subscribe(callback)

    return () => {
      topic.unsubscribe(callback)
    }
  }, [ros, topicName, messageType, throttleRate])

  return message
}
