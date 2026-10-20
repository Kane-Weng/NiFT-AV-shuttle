/*
 * File: utm.ts
 *
 * Description:
 *   WGS84 lat/lon to UTM, the projection Lanelet2's UtmProjector uses (GeographicLib underneath).
 *   It uses Karney's 6th order Krueger series, which GeographicLib's TransverseMercator also uses.
 *   It matches Lanelet2 to well under a millimetre across the map (test/utm.test.ts).
 *
 *   The stack's map frame is UTM minus the UTM of the map origin, in the zone of the origin
 *   (waypoint_node, shuttle_tf_node). MapProjector.toMapXY() does exactly that.
 */

const A_WGS84 = 6378137.0
const F_WGS84 = 1 / 298.257223563
const K0 = 0.9996
const FALSE_EASTING = 500000.0
const FALSE_NORTHING_SOUTH = 10000000.0

const n = F_WGS84 / (2 - F_WGS84)
const e = Math.sqrt(F_WGS84 * (2 - F_WGS84))
const n2 = n * n
const n3 = n2 * n
const n4 = n3 * n
const n5 = n4 * n
const n6 = n5 * n

// Rectifying radius and the alpha coefficients of the forward series (Karney 2011, eq. 14 and 35)
const RECT_A = (A_WGS84 / (1 + n)) * (1 + n2 / 4 + n4 / 64 + n6 / 256)
const ALPHA = [
  n / 2 - (2 * n2) / 3 + (5 * n3) / 16 + (41 * n4) / 180 - (127 * n5) / 288 + (7891 * n6) / 37800,
  (13 * n2) / 48 - (3 * n3) / 5 + (557 * n4) / 1440 + (281 * n5) / 630 - (1983433 * n6) / 1935360,
  (61 * n3) / 240 - (103 * n4) / 140 + (15061 * n5) / 26880 + (167603 * n6) / 181440,
  (49561 * n4) / 161280 - (179 * n5) / 168 + (6601661 * n6) / 7257600,
  (34729 * n5) / 80640 - (3418889 * n6) / 1995840,
  (212378941 * n6) / 319334400,
]

const DEG = Math.PI / 180

/** The standard UTM zone of a longitude. It skips the Norway and Svalbard exceptions. */
export function utmZone(lonDeg: number): number {
  const lon = ((((lonDeg + 180) % 360) + 360) % 360)
  return Math.floor(lon / 6) + 1
}

/** Lat/lon in degrees to UTM easting and northing in metres, in the given zone. */
export function latLonToUtm(latDeg: number, lonDeg: number, zone: number): [number, number] {
  const lon0 = (zone - 1) * 6 - 180 + 3
  let dLon = lonDeg - lon0
  dLon = ((((dLon + 180) % 360) + 360) % 360) - 180
  const phi = latDeg * DEG
  const lam = dLon * DEG

  // Conformal latitude through tau' (Karney 2011, eq. 7 to 9)
  const tau = Math.tan(phi)
  const sigma = Math.sinh(e * Math.atanh((e * tau) / Math.sqrt(1 + tau * tau)))
  const tauP = tau * Math.sqrt(1 + sigma * sigma) - sigma * Math.sqrt(1 + tau * tau)
  const cosLam = Math.cos(lam)
  const xiP = Math.atan2(tauP, cosLam)
  const etaP = Math.asinh(Math.sin(lam) / Math.sqrt(tauP * tauP + cosLam * cosLam))

  let xi = xiP
  let eta = etaP
  for (let j = 1; j <= 6; j++) {
    const a = ALPHA[j - 1]
    xi += a * Math.sin(2 * j * xiP) * Math.cosh(2 * j * etaP)
    eta += a * Math.cos(2 * j * xiP) * Math.sinh(2 * j * etaP)
  }
  const x = FALSE_EASTING + K0 * RECT_A * eta
  const y = (latDeg < 0 ? FALSE_NORTHING_SOUTH : 0) + K0 * RECT_A * xi
  return [x, y]
}

/** Projects lat/lon into a map frame whose origin and UTM zone come from one lat/lon point. */
export class MapProjector {
  readonly zone: number
  readonly originUtm: [number, number]

  constructor(
    readonly originLat: number,
    readonly originLon: number,
  ) {
    this.zone = utmZone(originLon)
    this.originUtm = latLonToUtm(originLat, originLon, this.zone)
  }

  /** Lat/lon to map-frame metres (east, north), like Lanelet2's UtmProjector with this origin. */
  toMapXY(latDeg: number, lonDeg: number): [number, number] {
    const [x, y] = latLonToUtm(latDeg, lonDeg, this.zone)
    return [x - this.originUtm[0], y - this.originUtm[1]]
  }
}
