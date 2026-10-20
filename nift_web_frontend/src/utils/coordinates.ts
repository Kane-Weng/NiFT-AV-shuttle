/*
 * File: coordinates.ts
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Coordinate conversion utilities that match the math used inside the
 *   ROS2 stack (shuttle_tf_node.py → Lanelet2 UtmProjector).
 *
 *   The ROS2 map frame is a local East-North-Up (ENU) coordinate system
 *   centered at the Mcity map origin. All positions in that frame are in
 *   meters measured from the origin — so (x=10, y=5) means 10 m east and
 *   5 m north of the origin.
 *
 *   Leaflet and OpenStreetMap work in WGS84 latitude / longitude (degrees).
 *   These two functions bridge between them:
 *
 *     mapToLatLon(x, y)    — convert a ROS map-frame point to [lat, lon]
 *                            so it can be drawn on the Leaflet map.
 *                            Used for: planned path, lanelet lines, obstacles.
 *
 *     latLonToMap(lat, lon) — convert a Leaflet map click back to ROS map-
 *                            frame {x, y} in meters.
 *                            Used for: publishing /goal_pose when the user
 *                            clicks the map.
 *
 *   Both functions use the proj4 library with UTM Zone 17N projection — the
 *   same projection Lanelet2 uses internally — so goal poses land exactly
 *   where expected on the road network.
 *
 * Usage:
 *   import { mapToLatLon, latLonToMap } from '../utils/coordinates'
 */

import proj4 from 'proj4'

// Map origin (Mcity), the default of the map_origin_lat / map_origin_lon launch arguments
export const MAP_ORIGIN_LAT = 42.3005    // owner: nift_common/site.py MAP_ORIGIN_LAT (TS copy, tested)
export const MAP_ORIGIN_LON = -83.6987   // owner: nift_common/site.py MAP_ORIGIN_LON (TS copy, tested)

// UTM Zone 17N: the zone Lanelet2's UtmProjector picks for the origin above. Change it with the origin.
const WGS84 = 'EPSG:4326'
const UTM_17N = '+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs'   // zone of MAP_ORIGIN_LON (tested)

// Compute origin UTM coordinates once at module load (same as shuttle_tf_node map_origin_x/y)
const [ORIGIN_UTM_X, ORIGIN_UTM_Y] = proj4(WGS84, UTM_17N, [MAP_ORIGIN_LON, MAP_ORIGIN_LAT])

/** Map frame (meters, ENU from origin) → [lat, lon] for Leaflet */
export function mapToLatLon(x: number, y: number): [number, number] {
  const [lon, lat] = proj4(UTM_17N, WGS84, [ORIGIN_UTM_X + x, ORIGIN_UTM_Y + y])
  return [lat, lon]
}

/** [lat, lon] → map frame {x, y} in meters — used when publishing goal_pose */
export function latLonToMap(lat: number, lon: number): { x: number; y: number } {
  const [utmX, utmY] = proj4(WGS84, UTM_17N, [lon, lat])
  return { x: utmX - ORIGIN_UTM_X, y: utmY - ORIGIN_UTM_Y }
}

/** Quaternion → yaw in degrees (for display) */
export function quatToYawDeg(x: number, y: number, z: number, w: number): number {
  const siny = 2 * (w * z + x * y)
  const cosy = 1 - 2 * (y * y + z * z)
  return (Math.atan2(siny, cosy) * 180) / Math.PI
}
