/*
 * File: StatusPill.tsx
 *
 * Description:
 *   The controller status, localization and sim clock for the page header, so SAFE_STOP or a lost
 *   localization shows even while the sidebar is scrolled to the playground controls.
 */

import { useSimSnapshot } from '../sim/useSim'
import { Badge } from './Panel'
import { STATUS_TONE, ecuTone, locTone } from './OperatorPanel'

export default function StatusPill() {
  const { state } = useSimSnapshot()
  if (!state) return null
  return (
    <div className="hidden sm:flex items-center gap-1.5 text-xs" data-testid="header-status">
      <Badge tone={state.status ? STATUS_TONE[state.status] : 'muted'}>{state.status ?? 'IDLE (start-up)'}</Badge>
      {state.locStatus && <Badge tone={locTone(state)}>{`loc ${state.locStatus}`}</Badge>}
      {state.gnss.stale && <Badge tone="error">GNSS stale</Badge>}
      {state.ecu === 'ESTOP' && <Badge tone={ecuTone(state.ecu)}>ECU E-stop</Badge>}
      <span className="font-mono tabular-nums text-gray-500 dark:text-gray-400 w-24 text-right">
        t {state.t.toFixed(1)} s{state.paused ? ' ⏸' : ''}
      </span>
    </div>
  )
}
