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
 *     3. Returns a stable call(request, onResult) function. Passing an empty
 *        object {} is correct for Trigger services that take no request fields.
 *     4. onResult gets exactly one ServiceOutcome. It holds the service's
 *        response, or an error when there is no connection, rosbridge reports
 *        a failure (e.g. the service doesn't exist), or no reply comes within
 *        SERVICE_TIMEOUT_MS.
 *
 * Usage:
 *   import { useRosService } from '../hooks/useRosService'
 *   const callTrigger = useRosService<TriggerResponse>('/start_navigation', 'std_srvs/Trigger')
 *   callTrigger({}, outcome => { if (outcome.ok) console.log(outcome.response.message) })
 */

import { useCallback, useEffect, useRef } from 'react'
import ROSLIB from 'roslib'
import { useRos } from './useRos'

// The controller answers at once. A call still open after this long has failed.
export const SERVICE_TIMEOUT_MS = 5000

export type ServiceOutcome<T> =
  | { ok: true; response: T }
  | { ok: false; error: string }

export function useRosService<TResponse = unknown>(serviceName: string, serviceType: string) {
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

  return useCallback((request: object = {}, onResult?: (outcome: ServiceOutcome<TResponse>) => void) => {
    const service = serviceRef.current
    if (!service) {
      onResult?.({ ok: false, error: 'not connected to rosbridge' })
      return
    }
    let done = false
    let timer = 0
    const finish = (outcome: ServiceOutcome<TResponse>) => {
      if (done) return
      done = true
      window.clearTimeout(timer)
      onResult?.(outcome)
    }
    timer = window.setTimeout(
      () => finish({ ok: false, error: `no reply from ${serviceName} in ${SERVICE_TIMEOUT_MS / 1000} s` }),
      SERVICE_TIMEOUT_MS,
    )
    service.callService(
      new ROSLIB.ServiceRequest(request),
      (response: TResponse) => finish({ ok: true, response }),
      (error: string) => finish({ ok: false, error: String(error) }),
    )
  }, [serviceName])
}
