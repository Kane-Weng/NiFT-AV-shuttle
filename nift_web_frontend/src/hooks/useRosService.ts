/*
 * File: useRosService.ts
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Generic React hook for calling a ROS2 service from the browser.
 *   Complements useRosPublisher (one-way topic publish) for request/response
 *   interactions — primarily used here for std_srvs/Trigger calls.
 *
 *   How it works:
 *     1. Gets the shared ROS connection from useRos().
 *     2. Creates a ROSLIB.Service object and stores it in a ref.
 *     3. Returns a stable call(request) function. Passing an empty object {}
 *        is correct for Trigger services that take no request fields.
 *     4. Silently does nothing if the connection is not yet established.
 *
 * Usage:
 *   import { useRosService } from '../hooks/useRosService'
 *   const callTrigger = useRosService('/start_navigation', 'std_srvs/Trigger')
 *   callTrigger()   // fires the service call
 */

import { useCallback, useEffect, useRef } from 'react'
import ROSLIB from 'roslib'
import { useRos } from './useRos'

export function useRosService(serviceName: string, serviceType: string) {
  const ros = useRos()
  const serviceRef = useRef<ROSLIB.Service | null>(null)

  useEffect(() => {
    if (!ros) {
      serviceRef.current = null
      return
    }
    serviceRef.current = new ROSLIB.Service({ ros, name: serviceName, serviceType })
    return () => {
      serviceRef.current = null
    }
  }, [ros, serviceName, serviceType])

  return useCallback((request: object = {}) => {
    serviceRef.current?.callService(new ROSLIB.ServiceRequest(request), () => {})
  }, [])
}
