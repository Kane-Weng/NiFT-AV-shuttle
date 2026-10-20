/*
 * File: loadMap.ts
 *
 * Description:
 *   Fetches the Lanelet2 map and builds it in the map frame of the stack.
 */

import { parseOsm } from './osm'
import { buildLaneletMap, type LaneletMap } from './lanelets'
import { MapProjector } from '../geo/utm'
import { MAP_FILE, MAP_ORIGIN_LAT, MAP_ORIGIN_LON } from '../geo/site'

/** The map URL under the page's base path (the build copies the repo's maps/ file there). */
export function mapUrl(): string {
  return new URL(`${import.meta.env.BASE_URL}maps/${MAP_FILE}`, window.location.href).href
}

export async function loadLaneletMap(url: string): Promise<LaneletMap> {
  const res = await fetch(url)
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`)
  const osm = parseOsm(await res.text())
  return buildLaneletMap(osm, new MapProjector(MAP_ORIGIN_LAT, MAP_ORIGIN_LON))
}
