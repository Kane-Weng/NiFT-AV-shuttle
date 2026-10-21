/*
 * File: presets.ts
 *
 * Description:
 *   Reads ready.presets and ready.launch for the panels. The runner may send the GNSS faults as one
 *   list with a kind on each item (ARCH.md 8.2, launch_set.presets) or as {scenarios, live}
 *   (sim_backend.gnss_fault_presets). Both shapes are read here, so the panels see one.
 *
 *   Also the parameter whitelist the page offers for set_param, with the defaults of
 *   nift_control/config/planning.yaml (a copy for display only: the node reads the YAML).
 */

import type { FaultPreset, LiveFaultPreset, ReadyMsg, RoutePreset, ScheduledFaultPreset } from '../protocol'

export function faultPresets(ready: ReadyMsg | null): { live: LiveFaultPreset[]; scheduled: ScheduledFaultPreset[] } {
  const raw = ready?.presets?.gnss_faults as unknown
  const live: LiveFaultPreset[] = []
  const scheduled: ScheduledFaultPreset[] = []
  const take = (p: FaultPreset | Record<string, unknown>, fallback: 'scheduled' | 'live') => {
    const kind = (p as { kind?: string }).kind
    if (kind === 'sim' || kind === 'link') live.push(p as LiveFaultPreset)
    else if (kind === 'scheduled' || fallback === 'scheduled') scheduled.push({ ...(p as ScheduledFaultPreset), kind: 'scheduled' })
    else live.push({ ...(p as LiveFaultPreset), kind: 'sim' })
  }
  if (Array.isArray(raw)) for (const p of raw) take(p, 'scheduled')
  else if (raw && typeof raw === 'object') {
    const o = raw as { scenarios?: unknown[]; live?: unknown[] }
    for (const p of o.scenarios ?? []) take(p as Record<string, unknown>, 'scheduled')
    for (const p of o.live ?? []) take(p as Record<string, unknown>, 'live')
  }
  return { live, scheduled }
}

export function routePresets(ready: ReadyMsg | null): RoutePreset[] {
  const r = ready?.presets?.routes
  return Array.isArray(r) ? r.filter((x) => x && Array.isArray(x.spawn) && Array.isArray(x.goal)) : []
}

/** The resolved launch options (ARCH.md 6.4) from ready.launch, wherever the runner put them. */
export function launchOptions(ready: ReadyMsg | null): Record<string, unknown> {
  const l = (ready?.launch ?? {}) as Record<string, unknown>
  const o = (l.options ?? l.opts ?? l) as Record<string, unknown>
  return o && typeof o === 'object' ? o : {}
}

export interface ParamSpec {
  node: string
  name: string
  def: number
  min: number
  max: number
  step: number
  unit: string
  doc: string
}

/** set_param whitelist (proposed for ARCH.md 8.1). Only parameters a playground user would try. */
export const PARAMS: ParamSpec[] = [
  { node: 'trajectory_controller', name: 'base_speed', def: 3.5, min: 0.5, max: 6, step: 0.1, unit: 'm/s', doc: 'nominal cruising speed' },
  { node: 'trajectory_controller', name: 'pure_pursuit_ld_min', def: 4.0, min: 1, max: 15, step: 0.5, unit: 'm', doc: 'lookahead at low speed' },
  { node: 'trajectory_controller', name: 'pure_pursuit_ld_max', def: 8.0, min: 1, max: 20, step: 0.5, unit: 'm', doc: 'lookahead at high speed' },
  { node: 'trajectory_controller', name: 'goal_tolerance', def: 2.0, min: 0.2, max: 5, step: 0.1, unit: 'm', doc: 'distance that counts as arrived' },
  { node: 'trajectory_controller', name: 'decel_distance', def: 10.0, min: 1, max: 30, step: 0.5, unit: 'm', doc: 'braking starts this far from the goal' },
  { node: 'trajectory_controller', name: 'cte_tolerance', def: 1.5, min: 0.2, max: 5, step: 0.1, unit: 'm', doc: 'cross-track error that slows the shuttle' },
  { node: 'trajectory_controller', name: 'min_safe_dist', def: 2.0, min: 0.5, max: 10, step: 0.1, unit: 'm', doc: 'fixed obstacle margin (perception on)' },
  { node: 'trajectory_controller', name: 'collision_zone_dist', def: 0.8, min: 0.2, max: 5, step: 0.1, unit: 'm', doc: 'obstacle this close: SAFE_STOP (perception on)' },
]

export const LAUNCH_CHOICES = {
  localization: ['direct', 'ekf', 'shadow', 'off'],
  gnss_feed: ['ideal', 'real'],
  gnss_checks: ['true', 'false'],
  perception: ['false', 'true'],
  filter_type: ['', 'distance_stop', 'cbf', 'potential_field', 'none'],
} as const
