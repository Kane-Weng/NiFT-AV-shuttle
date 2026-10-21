/*
 * File: ui.ts
 *
 * Description:
 *   Page state the map and the panels share, outside React: the active map tool (2D Goal,
 *   Set pose, Obstacle, or none: Select), the selected obstacle and the view options of the settings window
 *   (centerlines, the 3D world and its loading, the ROS log under the map). The map view reads the tool to decide what a
 *   click or a drag does. The panels and the settings window set the rest.
 */

import { useSyncExternalStore } from 'react'

export type MapTool = 'goal' | 'pose' | 'obstacle' | null

/** Footprints the Obstacle tool places, length x width in metres */
export const OBSTACLE_SIZES = {
  car: { length: 4.5, width: 1.9, label: 'car' },
  box: { length: 1.0, width: 1.0, label: 'box 1 m' },
  person: { length: 0.6, width: 0.6, label: 'person' },
} as const
export type ObstacleSize = keyof typeof OBSTACLE_SIZES

export interface WorldLoad {
  state: 'idle' | 'loading' | 'ready' | 'error'
  pct: number
  error?: string
}

export interface UiState {
  tool: MapTool
  selected: string | null // obstacle id
  obstacleSize: ObstacleSize
  centerlines: boolean
  world: boolean // wanted; the map view loads it on first use
  worldLoad: WorldLoad
  log: boolean // the log dock under the map; off by default, remembered in this browser
  win: 'settings' | 'stack' | 'tour' | null // the header window that is open
}

/** ?log=1 or ?log=0 wins over the saved choice. */
function savedLog(): boolean {
  const p = new URLSearchParams(location.search).get('log')
  if (p === '1' || p === '0') return p === '1'
  try {
    return localStorage.getItem('playground.log') === '1'
  } catch {
    return false
  }
}

let state: UiState = {
  tool: null,
  selected: null,
  obstacleSize: 'car',
  centerlines: false,
  world: new URLSearchParams(location.search).get('world') === '1',
  worldLoad: { state: 'idle', pct: 0 },
  log: savedLog(),
  win: null,
}
const listeners = new Set<() => void>()

export function getUi(): UiState {
  return state
}

export function setUi(patch: Partial<UiState>) {
  state = { ...state, ...patch }

  for (const fn of listeners) fn()
}

/** The log switch of the settings window: shows or hides the log and remembers the choice. */
export function setLog(log: boolean) {
  setUi({ log })
  try {
    localStorage.setItem('playground.log', log ? '1' : '0')
  } catch {
    // storage blocked: the choice lasts for this page only
  }
}

/** Toggles a tool: picking the active one again turns it off. */
export function toggleTool(tool: Exclude<MapTool, null>) {
  setUi({ tool: state.tool === tool ? null : tool })
}

export function subscribeUi(fn: () => void): () => void {
  listeners.add(fn)
  return () => listeners.delete(fn)
}

export function useUi(): UiState {
  return useSyncExternalStore(subscribeUi, getUi)
}
