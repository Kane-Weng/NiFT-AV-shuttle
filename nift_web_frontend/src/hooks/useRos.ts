/*
 * File: useRos.ts
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Provides shared access to the single ROSLIB.Ros WebSocket connection
 *   that App.tsx opens at startup.
 *
 *   In React, "Context" is a way to pass a value (here, the ros connection
 *   object) down through the component tree without having to manually pass
 *   it as a prop to every component. App.tsx wraps the whole app in
 *   <RosContext.Provider value={ros}>, and any component that needs the
 *   connection just calls useRos() to get it.
 *
 *   Returns null if the connection has not been established yet (e.g. still
 *   connecting or rosbridge is offline). Callers should guard against null
 *   before using the returned value.
 *
 * Usage:
 *   import { useRos } from '../hooks/useRos'
 *   const ros = useRos()   // null until connected
 */

import { createContext, useContext } from 'react'
import ROSLIB from 'roslib'

export const RosContext = createContext<ROSLIB.Ros | null>(null)

export function useRos(): ROSLIB.Ros | null {
  return useContext(RosContext)
}
