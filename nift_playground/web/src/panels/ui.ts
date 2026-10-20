/*
 * File: ui.ts
 *
 * Description:
 *   Page state the map and the panels share, outside React: the active map tool (2D Goal,
 *   2D Pose Estimate, Obstacle) and the selected obstacle. The map view reads the tool to decide
 *   what a click or a drag does. The panels set it from their buttons.
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

export interface UiState {
  tool: MapTool
  selected: string | null // obstacle id
  obstacleSize: ObstacleSize
}

let state: UiState = { tool: null, selected: null, obstacleSize: 'car' }
const listeners = new Set<() => void>()

export function getUi(): UiState {
  return state
}

export function setUi(patch: Partial<UiState>) {
  state = { ...state, ...patch }
  for (const fn of listeners) fn()
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
