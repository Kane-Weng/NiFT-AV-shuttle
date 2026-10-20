// Tests for src/utils/control.ts. Run with `npm test` (Node's own test runner, no extra packages).
import { test } from 'node:test'
import assert from 'node:assert/strict'

import {
  buildTwist, controlView, ESTOP_LATCH_NOTE, navStatus, NO_KEYS, stopButton, teleopStep, triggerFeedback, ZERO_TWIST,
} from '../src/utils/control.ts'
import type { DirFlags } from '../src/utils/control.ts'

// /nav_telemetry as trajectory_controller._publish_telemetry formats it
const telemetry = (status: string) =>
  `STATUS:    ${status}\nTarget V:  0.00 m/s\nCurr V:    0.00 m/s\nObs Dist:  10.00 m`

test('navStatus reads the STATUS line of /nav_telemetry', () => {
  assert.equal(navStatus(telemetry('NAVIGATING')), 'NAVIGATING')
  assert.equal(navStatus('Target V:  1.00 m/s\nSTATUS:  MANUAL'), 'MANUAL')
  assert.equal(navStatus('Target V:  1.00 m/s'), null)
  assert.equal(navStatus(undefined), null)
})

test('the latched ESTOP gets its own view, not AUTO', () => {
  assert.equal(controlView(navStatus(telemetry('ESTOP'))), 'estop')
})

test('every controller mode maps to its view', () => {
  assert.equal(controlView(navStatus(telemetry('MANUAL'))), 'manual')
  assert.equal(controlView(navStatus(telemetry('SAFE_STOP'))), 'safe_stop')
  assert.equal(controlView(navStatus(telemetry('IDLE'))), 'auto')
  assert.equal(controlView(navStatus(telemetry('NAVIGATING'))), 'auto')
  assert.equal(controlView(null), 'auto')
})

test('a refused /start_navigation shows the controller reason, not a success', () => {
  const fb = triggerFeedback({ ok: true, response: { success: false, message: 'Cannot start: no valid path loaded.' } })
  assert.deepEqual(fb, { kind: 'refused', text: 'Cannot start: no valid path loaded.' })
})

test('an accepted call is ok, and a failed call is an error', () => {
  assert.deepEqual(
    triggerFeedback({ ok: true, response: { success: true, message: 'Navigation started successfully.' } }),
    { kind: 'ok', text: 'Navigation started successfully.' })
  assert.equal(triggerFeedback({ ok: false, error: 'not connected to rosbridge' }).kind, 'error')
  assert.equal(triggerFeedback({ ok: true, response: { success: false, message: '' } }).kind, 'refused')
})

test('Stop (/safe_stop) is enabled while navigating and in MANUAL', () => {
  assert.equal(stopButton(navStatus(telemetry('NAVIGATING'))), 'enabled')
  assert.equal(stopButton(navStatus(telemetry('MANUAL'))), 'enabled')
})

test('Stop is hidden in SAFE_STOP and ESTOP, and disabled in IDLE or without a status', () => {
  assert.equal(stopButton(navStatus(telemetry('SAFE_STOP'))), 'hidden')
  assert.equal(stopButton(navStatus(telemetry('ESTOP'))), 'hidden')
  assert.equal(stopButton(navStatus(telemetry('IDLE'))), 'disabled')
  assert.equal(stopButton(null), 'disabled')
})

test('a /safe_stop refused in ESTOP shows the reason as a refusal', () => {
  const message = 'Cannot safe-stop: Vehicle is E-STOPPED. Restart this node, then power-cycle the shuttle, to recover.'
  const fb = triggerFeedback({ ok: true, response: { success: false, message } })
  assert.deepEqual(fb, { kind: 'refused', text: message })
})

test('the E-STOP note says it latches and how to recover', () => {
  assert.match(ESTOP_LATCH_NOTE, /Latches/)
  assert.match(ESTOP_LATCH_NOTE, /restart/)
  assert.match(ESTOP_LATCH_NOTE, /power cycle/)
})

const W: DirFlags = { ...NO_KEYS, fwd: true }

test('manual driving publishes nothing while no key is held', () => {
  let driving = false
  for (let i = 0; i < 50; i++) {
    const step = teleopStep(driving, NO_KEYS, 1.0, 10)
    assert.equal(step.publish, null, `tick ${i}`)
    driving = step.driving
  }
})

test('manual driving publishes while a key is held, then one zero Twist, then nothing', () => {
  const sent = []
  let driving = false
  const keys = [W, W, W, NO_KEYS, NO_KEYS, NO_KEYS]
  for (const flags of keys) {
    const step = teleopStep(driving, flags, 1.0, 10)
    driving = step.driving
    sent.push(step.publish)
  }
  assert.deepEqual(sent.slice(0, 3), [buildTwist(W, 1.0, 10), buildTwist(W, 1.0, 10), buildTwist(W, 1.0, 10)])
  assert.deepEqual(sent[3], ZERO_TWIST, 'the stop, once')
  assert.deepEqual(sent.slice(4), [null, null])
})

test('the Twist carries speed and a steering angle in radians, mirrored in reverse', () => {
  const left = buildTwist({ ...NO_KEYS, fwd: true, left: true }, 1.5, 10)
  assert.equal(left.linear.x, 1.5)
  assert.ok(Math.abs(left.angular.z - 10 * Math.PI / 180) < 1e-12)
  const reverseLeft = buildTwist({ ...NO_KEYS, rev: true, left: true }, 1.5, 10)
  assert.equal(reverseLeft.linear.x, -1.5)
  assert.ok(Math.abs(reverseLeft.angular.z + 10 * Math.PI / 180) < 1e-12)
})
