"""UTM on WGS84 in pure Python, a step-by-step port of GeographicLib 1.52 (UTMUPS, TransverseMercator, Math).

Lanelet2's UtmProjector calls GeographicLib::UTMUPS::Forward, Reverse and Transfer. Ubuntu 22.04 (ROS Humble) ships
GeographicLib 1.52 built with GEOGRAPHICLIB_TRANSVERSEMERCATOR_ORDER 6. This module repeats its floating point steps
in the same order (Horner coefficients, sincosd with remquo, Clenshaw sums on complex numbers), so on the same C math
library the results are the same to the last bit or within one or two units in the last place. That matters: the
centerline algorithm compares distances, and a near tie can flip on a nanometre. UPS (the polar caps) is not supported.

Third-party portions: the transverse Mercator series, its coefficient tables and the UTM zone rules are translated from
GeographicLib 1.52, Copyright (c) Charles Karney (2008-2020) <charles@karney.com>, MIT License, see
LICENSES/MIT-GeographicLib.txt. _hypot is Python's math.hypot.
"""
import math

_EPS = 2.0 ** -52
_DEGREE = math.pi / 180

WGS84_A = 6378137.0
WGS84_F = 1 / (298257223563.0 / 1000000000)
UTM_K0 = 9996.0 / 10000
FALSE_EASTING = 500000.0
FALSE_NORTHING = {True: 0.0, False: 10000000.0}
UTM_N_SHIFT = 10000000.0
MAXPOW = 6

_B1COEFF = [1, 4, 64, 256, 256]
_ALPCOEFF = [
    31564, -66675, 34440, 47250, -100800, 75600, 151200,
    -1983433, 863232, 748608, -1161216, 524160, 1935360,
    670412, 406647, -533952, 184464, 725760,
    6601661, -7732800, 2230245, 7257600,
    -13675556, 3438171, 7983360,
    212378941, 319334400,
]
_BETCOEFF = [
    384796, -382725, -6720, 932400, -1612800, 1209600, 2419200,
    -1118711, 1695744, -1174656, 258048, 80640, 3870720,
    22276, -16929, -15984, 12852, 362880,
    -830251, -158400, 197865, 7257600,
    -435388, 453717, 15966720,
    20648693, 638668800,
]


# hypot(a, b) is Python's own math.hypot. It can differ from the C library of ROS Humble by one unit in the last
# place in rare cases, which moves a projected coordinate by about 1e-9 m. For the shipped map the node cache
# (_nodecache.py) supplies the real Lanelet2 coordinates, so this only matters for other maps.
_hypot = math.hypot


def _polyval(n, p, off, x):
    y = 0.0 if n < 0 else float(p[off])
    i = off + 1
    while n > 0:
        n -= 1
        y = y * x + p[i]
        i += 1
    return y


def _eatanhe(x, es):
    return es * math.atanh(es * x) if es > 0 else -es * math.atan(es * x)


def _taupf(tau, es):
    if math.isfinite(tau):
        tau1 = _hypot(1.0, tau)
        sig = math.sinh(_eatanhe(tau / tau1, es))
        return _hypot(1.0, sig) * tau - sig * tau1
    return tau


def _tauf(taup, es):
    tol = math.sqrt(_EPS) / 10
    taumax = 2 / math.sqrt(_EPS)
    e2m = 1.0 - es * es
    tau = taup * math.exp(_eatanhe(1.0, es)) if abs(taup) > 70 else taup / e2m
    stol = tol * max(1.0, abs(taup))
    if not abs(tau) < taumax:
        return tau
    for _ in range(5):
        taupa = _taupf(tau, es)
        dtau = (taup - taupa) * (1 + e2m * (tau * tau)) / (e2m * _hypot(1.0, tau) * _hypot(1.0, taupa))
        tau += dtau
        if not abs(dtau) >= stol:
            break
    return tau


def _sincosd(x):
    r = math.remainder(x, 90.0)
    q = int(round((x - r) / 90.0))
    r *= _DEGREE
    s = math.sin(r)
    c = math.cos(r)
    q &= 3
    if q == 0:
        sinx, cosx = s, c
    elif q == 1:
        sinx, cosx = c, -s
    elif q == 2:
        sinx, cosx = -s, -c
    else:
        sinx, cosx = -c, s
    if x != 0:
        sinx += 0.0
        cosx += 0.0
    return sinx, cosx


def _atan2d(y, x):
    q = 0
    if abs(y) > abs(x):
        x, y = y, x
        q = 2
    if x < 0:
        x = -x
        q += 1
    ang = math.atan2(y, x) / _DEGREE
    if q == 1:
        ang = (180 if y >= 0 else -180) - ang
    elif q == 2:
        ang = 90 - ang
    elif q == 3:
        ang = -90 + ang
    return ang


def _sum(u, v):
    s = u + v
    up = s - v
    vpp = s - up
    up -= u
    vpp -= v
    return s, -(up + vpp)


def _ang_normalize(x):
    x = math.remainder(x, 360.0)
    return x if x != -180 else 180.0


def _ang_diff(x, y):
    d, t = _sum(math.remainder(-x, 360.0), math.remainder(y, 360.0))
    d = _ang_normalize(d)
    s, _ = _sum(-180.0 if (d == 180 and t > 0) else d, t)
    return s


class _TransverseMercator:
    def __init__(self, a, f, k0):
        self.a = a
        self.f = f
        self.k0 = k0
        self.e2 = f * (2 - f)
        self.es = (-1 if f < 0 else 1) * math.sqrt(abs(self.e2))
        self.e2m = 1 - self.e2
        self.n = f / (2 - f)
        m = MAXPOW // 2
        self.b1 = _polyval(m, _B1COEFF, 0, self.n * self.n) / (_B1COEFF[m + 1] * (1 + self.n))
        self.a1 = self.b1 * a
        self.alp = [0.0] * (MAXPOW + 1)
        self.bet = [0.0] * (MAXPOW + 1)
        o = 0
        d = self.n
        for l in range(1, MAXPOW + 1):
            m = MAXPOW - l
            self.alp[l] = d * _polyval(m, _ALPCOEFF, o, self.n) / _ALPCOEFF[o + m + 1]
            self.bet[l] = d * _polyval(m, _BETCOEFF, o, self.n) / _BETCOEFF[o + m + 1]
            o += m + 2
            d *= self.n

    def forward(self, lon0, lat, lon):
        if abs(lat) > 90:
            lat = float('nan')
        lon = _ang_diff(lon0, lon)
        latsign = -1 if lat < 0 else 1
        lonsign = -1 if lon < 0 else 1
        lon *= lonsign
        lat *= latsign
        backside = lon > 90
        if backside:
            if lat == 0:
                latsign = -1
            lon = 180 - lon
        sphi, cphi = _sincosd(lat)
        slam, clam = _sincosd(lon)
        if lat != 90:
            tau = sphi / cphi
            taup = _taupf(tau, self.es)
            xip = math.atan2(taup, clam)
            etap = math.asinh(slam / _hypot(taup, clam))
        else:
            xip = math.pi / 2
            etap = 0.0
        c0 = math.cos(2 * xip)
        ch0 = math.cosh(2 * etap)
        s0 = math.sin(2 * xip)
        sh0 = math.sinh(2 * etap)
        a = complex(2 * c0 * ch0, -2 * s0 * sh0)
        n = MAXPOW
        alp = self.alp
        y0 = complex(alp[n] if n & 1 else 0.0, 0.0)
        y1 = 0j
        if n & 1:
            n -= 1
        while n:
            y1 = a * y0 - y1 + alp[n]
            n -= 1
            y0 = a * y1 - y0 + alp[n]
            n -= 1
        a = complex(s0 * ch0, c0 * sh0)
        y1 = complex(xip, etap) + a * y0
        xi = y1.real
        eta = y1.imag
        y = self.a1 * self.k0 * (math.pi - xi if backside else xi) * latsign
        x = self.a1 * self.k0 * eta * lonsign
        return x, y

    def reverse(self, lon0, x, y):
        xi = y / (self.a1 * self.k0)
        eta = x / (self.a1 * self.k0)
        xisign = -1 if xi < 0 else 1
        etasign = -1 if eta < 0 else 1
        xi *= xisign
        eta *= etasign
        backside = xi > math.pi / 2
        if backside:
            xi = math.pi - xi
        c0 = math.cos(2 * xi)
        ch0 = math.cosh(2 * eta)
        s0 = math.sin(2 * xi)
        sh0 = math.sinh(2 * eta)
        a = complex(2 * c0 * ch0, -2 * s0 * sh0)
        n = MAXPOW
        bet = self.bet
        y0 = complex(-bet[n] if n & 1 else 0.0, 0.0)
        y1 = 0j
        if n & 1:
            n -= 1
        while n:
            y1 = a * y0 - y1 - bet[n]
            n -= 1
            y0 = a * y1 - y0 - bet[n]
            n -= 1
        a = complex(s0 * ch0, c0 * sh0)
        y1 = complex(xi, eta) + a * y0
        xip = y1.real
        etap = y1.imag
        s = math.sinh(etap)
        c = max(0.0, math.cos(xip))
        r = _hypot(s, c)
        if r != 0:
            lon = _atan2d(s, c)
            sxip = math.sin(xip)
            tau = _tauf(sxip / r, self.es)
            lat = _atan2d(tau, 1.0)
        else:
            lat = 90.0
            lon = 0.0
        lat *= xisign
        if backside:
            lon = 180 - lon
        lon *= etasign
        lon = _ang_normalize(lon + lon0)
        return lat, lon


_UTM = _TransverseMercator(WGS84_A, WGS84_F, UTM_K0)


def central_meridian(zone):
    return 6.0 * zone - 183


def standard_zone(lat, lon):
    """UTM zone with the Norway and Svalbard exceptions (UTMUPS::StandardZone, MGRS::LatitudeBand)."""
    if not (-80 <= lat < 84):
        raise ValueError('Latitude %r is in the UPS area, which the stand-in does not support' % lat)
    ilon = int(math.floor(_ang_normalize(lon)))
    if ilon == 180:
        ilon = -180
    zone = (ilon + 186) // 6
    ilat = int(math.floor(lat))
    band = max(-10, min(9, (ilat + 80) // 8 - 10))
    if band == 7 and zone == 31 and ilon >= 3:
        zone = 32
    elif band == 9 and 0 <= ilon < 42:
        zone = 2 * ((ilon + 183) // 12) + 1
    return zone


def _forward_in_zone(lat, lon, zone):
    lon0 = central_meridian(zone)
    dlon = lon - lon0
    dlon = abs(dlon - 360 * math.floor((dlon + 180) / 360))
    if not dlon <= 60:
        raise ValueError('Longitude %r more than 60d from center of UTM zone %d' % (lon, zone))
    x, y = _UTM.forward(lon0, lat, lon)
    northp = lat >= 0
    return northp, x + FALSE_EASTING, y + FALSE_NORTHING[northp]


def utm_forward(lat, lon):
    """UTMUPS::Forward: (lat, lon) -> (zone, northp, easting, northing) in the standard zone."""
    if abs(lat) > 90:
        raise ValueError('Latitude %r not in [-90, 90]' % lat)
    zone = standard_zone(lat, lon)
    northp, x, y = _forward_in_zone(lat, lon, zone)
    return zone, northp, x, y


def utm_reverse(zone, northp, x, y):
    """UTMUPS::Reverse: (zone, northp, easting, northing) -> (lat, lon)."""
    return _UTM.reverse(central_meridian(zone), x - FALSE_EASTING, y - FALSE_NORTHING[northp])


def utm_transfer(zone_in, northp_in, x, y, zone_out, northp_out):
    """UTMUPS::Transfer between UTM zones and hemispheres. Returns (x, y)."""
    northp = northp_in
    if zone_in != zone_out:
        lat, lon = utm_reverse(zone_in, northp_in, x, y)
        northp, x, y = _forward_in_zone(lat, lon, zone_out)
    if northp != northp_out:
        y += (-1 if northp_out else 1) * UTM_N_SHIFT
    return x, y
