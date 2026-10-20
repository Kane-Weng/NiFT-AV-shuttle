/*
 * File: useTrigger.ts
 *
 * Description:
 *   React hook for a std_srvs/Trigger button. Returns run() and the state of
 *   the last call, so a button can show what the service actually answered.
 *
 *   state is null when there is nothing to show. It is 'pending' while the
 *   call is open, then 'ok', 'refused' (success false, with the service's
 *   message) or 'error' (no connection, no service, no reply). The result
 *   stays for FEEDBACK_MS. A newer press of the same button replaces it.
 *
 * Usage:
 *   const [startNav, startState] = useTrigger('/start_navigation')
 *   <button onClick={startNav}>...</button>
 */

import { useCallback, useEffect, useRef, useState } from 'react'
import { useRosService } from './useRosService'
import { triggerFeedback } from '../utils/control'
import type { FeedbackKind } from '../utils/control'
import type { TriggerResponse } from '../types/ros'

export const FEEDBACK_MS = 8000

export interface TriggerState {
  phase: 'pending' | FeedbackKind
  text: string
}

export function useTrigger(serviceName: string): [() => void, TriggerState | null] {
  const call = useRosService<TriggerResponse>(serviceName, 'std_srvs/Trigger')
  const [state, setState] = useState<TriggerState | null>(null)
  const seqRef = useRef(0)
  const timerRef = useRef(0)

  useEffect(() => () => window.clearTimeout(timerRef.current), [])

  const run = useCallback(() => {
    const seq = ++seqRef.current
    window.clearTimeout(timerRef.current)
    setState({ phase: 'pending', text: '' })
    call({}, outcome => {
      if (seq !== seqRef.current) return   // a newer press owns the display
      const feedback = triggerFeedback(outcome)
      setState({ phase: feedback.kind, text: feedback.text })
      timerRef.current = window.setTimeout(() => setState(null), FEEDBACK_MS)
    })
  }, [call])

  return [run, state]
}
