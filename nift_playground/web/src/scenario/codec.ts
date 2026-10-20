/*
 * File: codec.ts
 *
 * Description:
 *   Scenarios in the URL (ARCH.md 8.5): base64url of the deflated JSON, in the hash as
 *   #scenario=<code>. Python reads the same code with zlib.decompress(base64.urlsafe_b64decode(...)).
 *   The page also accepts ?scenario=<code>, and a code that is plain base64url JSON (no deflate),
 *   so a hand-written link works too.
 */

import type { Scenario } from '../protocol'

function toBase64Url(bytes: Uint8Array): string {
  let bin = ''
  for (let i = 0; i < bytes.length; i += 0x8000) bin += String.fromCharCode(...bytes.subarray(i, i + 0x8000))
  return btoa(bin).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '')
}

function fromBase64Url(code: string): Uint8Array {
  const b64 = code.replace(/-/g, '+').replace(/_/g, '/')
  const bin = atob(b64 + '='.repeat((4 - (b64.length % 4)) % 4))
  const out = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i)
  return out
}

async function pipe(bytes: Uint8Array, stream: CompressionStream | DecompressionStream): Promise<Uint8Array> {
  const out = new Response(new Blob([bytes]).stream().pipeThrough(stream))
  return new Uint8Array(await out.arrayBuffer())
}

/** Checks the shape and returns the scenario, or throws with a reason a person can read. */
export function validateScenario(x: unknown): Scenario {
  if (!x || typeof x !== 'object') throw new Error('not a scenario object')
  const s = x as Partial<Scenario>
  if (s.v !== 1) throw new Error(`unknown scenario version ${String(s.v)}`)
  if (!s.launch || typeof s.launch !== 'object') throw new Error('scenario has no launch options')
  if (!Array.isArray(s.events)) throw new Error('scenario has no events list')
  for (const e of s.events) {
    if (typeof e?.tick !== 'number' || !e.msg || typeof e.msg.type !== 'string') throw new Error('bad scenario event')
  }
  return s as Scenario
}

export async function encodeScenario(s: Scenario): Promise<string> {
  const json = new TextEncoder().encode(JSON.stringify(s))
  return toBase64Url(await pipe(json, new CompressionStream('deflate')))
}

export async function decodeScenario(code: string): Promise<Scenario> {
  const bytes = fromBase64Url(code.trim())
  let text: string
  try {
    text = new TextDecoder().decode(await pipe(bytes, new DecompressionStream('deflate')))
  } catch {
    text = new TextDecoder().decode(bytes) // not deflated: plain JSON
  }
  let parsed: unknown
  try {
    parsed = JSON.parse(text)
  } catch {
    throw new Error('the scenario code is neither deflated nor plain JSON')
  }
  return validateScenario(parsed)
}

/** The scenario code in a URL (hash first, then the query), or null. */
export function scenarioCodeFrom(href: string): string | null {
  const u = new URL(href)
  const hash = new URLSearchParams(u.hash.replace(/^#/, ''))
  return hash.get('scenario') ?? u.searchParams.get('scenario')
}

/** The page URL with this scenario code in the hash (and none in the query). */
export function urlWithScenario(href: string, code: string): string {
  const u = new URL(href)
  u.searchParams.delete('scenario')
  u.hash = `scenario=${code}`
  return u.toString()
}

/**
 * Reads a scenario from what a person pastes: a link with #scenario= or ?scenario=, a bare code,
 * or the JSON itself.
 */
export async function parseScenarioInput(text: string): Promise<Scenario> {
  const t = text.trim()
  if (t.startsWith('{')) return validateScenario(JSON.parse(t))
  if (/^https?:\/\//.test(t)) {
    const code = scenarioCodeFrom(t)
    if (!code) throw new Error('the link has no scenario in it')
    return decodeScenario(code)
  }
  return decodeScenario(t.replace(/^#?scenario=/, ''))
}
