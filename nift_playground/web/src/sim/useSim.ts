/*
 * File: useSim.ts
 *
 * Description:
 *   React access to the SimClient: a context for the client and a hook for its snapshot.
 */

import { createContext, useContext, useEffect, useState } from 'react'
import type { SimClient, SimSnapshot } from './SimClient'

export const SimContext = createContext<SimClient | null>(null)

export function useSimClient(): SimClient {
  const c = useContext(SimContext)
  if (!c) throw new Error('useSimClient outside SimContext')
  return c
}

/** The latest snapshot. Re-renders on every worker update (state comes at about 20 Hz). */
export function useSimSnapshot(): SimSnapshot {
  const client = useSimClient()
  const [snap, setSnap] = useState(client.snapshot)
  useEffect(() => client.subscribe(setSnap), [client])
  return snap
}
