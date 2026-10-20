/*
 * File: site.ts
 *
 * Description:
 *   The site and vehicle facts the page needs. Each value is a copy of the owner named next to it.
 */

// owner: nift_common/nift_common/site.py MAP_ORIGIN_LAT / MAP_ORIGIN_LON (TS copy)
export const MAP_ORIGIN_LAT = 42.3005
export const MAP_ORIGIN_LON = -83.6987

// The map the playground loads (maps/README.md). The build serves it from the repo's maps/ folder.
export const MAP_FILE = 'lanelet2_odr2024.osm'

// owner: nift_common/nift_common/vehicle.yaml (TS copy) [m]
export const VEHICLE = {
  length: 3.055,
  width: 1.588,
  height: 1.905,
  wheelBase: 2.082,
  frontOverhang: 0.4830572,
  rearOverhang: 0.48977042,
  wheelRadius: 0.228,
  baseLinkHeight: 0.9525,
  maxSteeringDeg: 15.0,
  maxSpeed: 4.91744,
}
