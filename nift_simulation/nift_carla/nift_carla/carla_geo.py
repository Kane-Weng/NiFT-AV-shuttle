"""
Convert coordinates between the map frame, WGS84 and the CARLA world.

File: carla_geo.py
Author: Kane Weng

Description:
  Coordinate conversions between the three frames involved in the CARLA backend.

    map frame   — the NiFT "map" frame: local metres produced by Lanelet2's
                  UtmProjector with the Mcity datum as origin (UTM zone 17N,
                  offset so the datum is (0, 0)).  This is exactly what
                  shuttle_tf_node, waypoint_node and the web dashboard use.
    WGS84       — latitude / longitude.
    CARLA world — Unreal's left-handed frame.  CARLA loads the OpenDRIVE
                  (right-handed, x east / y north in the tmerc projection of
                  the .xodr <geoReference>) and flips y:  carla = (x, -y).
                  Yaw is in degrees and clockwise-positive when seen from above.

  Conversion chain:

    map (x, y) --UtmProjector.reverse--> (lat, lon) --pyproj tmerc--> (tx, ty)
               --y flip--> CARLA (tx, -ty)

  Reference numbers for Mcity (see test/test_carla_geo.py):
    datum (42.3005, -83.6987)  ->  CARLA (48.090, +10.376)
    map +y (UTM grid north)    ->  bearing 91.82 deg in tmerc  (1.82 deg grid convergence)
    map yaw pi/2               ->  CARLA yaw ~ -91.82 deg
"""

import math
import re
from typing import Optional, Tuple

from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
import pyproj

try:  # lanelet2 comes with ROS; on a bare GPU host (smoke test) fall back to pyproj UTM
    from lanelet2.core import BasicPoint3d, GPSPoint
    from lanelet2.io import Origin
    from lanelet2.projection import UtmProjector
    HAVE_LANELET2 = True
except ImportError:  # pragma: no cover - exercised only where ROS is absent
    HAVE_LANELET2 = False

# Coordinate system code for pyproj Transformer
WGS84_EPSG = 'EPSG:4326'


class PyprojUtmProjector:
    """
    Project like lanelet2.projection.UtmProjector(Origin(lat, lon)) with useOffset=True.

    UTM zone of the origin, origin subtracted so it maps to (0, 0).  Agrees with lanelet2
    to well under a millimetre (see test_carla_geo.py); used when lanelet2 is not importable.
    """

    def __init__(self, lat: float, lon: float):
        zone = int((lon + 180.0) // 6.0) + 1
        south = ' +south' if lat < 0.0 else ''

        # Coordinate Reference System, Transformers, and Transformation
        crs = pyproj.CRS.from_proj4(
            f'+proj=utm +zone={zone} +datum=WGS84 +units=m +no_defs{south}'
        )
        self._fwd = pyproj.Transformer.from_crs(WGS84_EPSG, crs, always_xy=True)
        self._inv = pyproj.Transformer.from_crs(crs, WGS84_EPSG, always_xy=True)
        self._ox, self._oy = self._fwd.transform(lon, lat)

    def forward_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        x, y = self._fwd.transform(lon, lat)
        return x - self._ox, y - self._oy

    def reverse_xy(self, x: float, y: float) -> Tuple[float, float]:
        lon, lat = self._inv.transform(x + self._ox, y + self._oy)
        return lat, lon


# <geoReference> of mcity-digital-twin CARLA/packaged_version/*/OpenDrive/McityMap_Main.xodr
# (mcity_map_v4.xodr, RoadRunner export).  Used as a fallback when the running
# server's OpenDRIVE cannot be parsed.
MCITY_TMERC = ('+proj=tmerc +lat_0=42.3005934157 +lon_0=-83.69928318881136 '
               '+k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs')

# NiFT map datum (nift_common.site; the web dashboard's coordinates.ts must match)
DEFAULT_DATUM_LAT = MAP_ORIGIN_LAT
DEFAULT_DATUM_LON = MAP_ORIGIN_LON

# Regular expression that extracts georeferencing data from OpenDRIVE map files
_GEOREF_RE = re.compile(
    r'<geoReference>\s*(?:<!\[CDATA\[)?(.*?)(?:\]\]>)?\s*</geoReference>', re.S)

# proj tokens that need external grid files or only affect the vertical axis
_DROP_TOKENS = ('+geoidgrids=', '+vunits=')


def parse_georeference(xodr: str) -> Optional[str]:
    """Extract the proj4 string from an OpenDRIVE <geoReference> block, or None."""
    m = _GEOREF_RE.search(xodr or '')
    if not m:
        return None
    s = m.group(1).strip()
    return sanitize_proj4(s) if s else None


def sanitize_proj4(proj4: str) -> str:
    """Remove tokens that require grid files (geoid) so pyproj can build the CRS offline."""
    tokens = [t for t in proj4.split() if not t.startswith(_DROP_TOKENS)]
    return ' '.join(tokens)


class CarlaGeo:
    """map frame <-> WGS84 <-> CARLA world conversions for one map/datum pair."""

    def __init__(self,
                 datum_lat: float = DEFAULT_DATUM_LAT,
                 datum_lon: float = DEFAULT_DATUM_LON,
                 georef_proj4: Optional[str] = None,
                 force_pyproj: bool = False):
        self.datum_lat = float(datum_lat)
        self.datum_lon = float(datum_lon)
        self.georef = sanitize_proj4(georef_proj4) if georef_proj4 else MCITY_TMERC
        self.georef_is_fallback = georef_proj4 is None

        # Same projector as nift_sensor/shuttle_tf_node.py and nift_waypoint/waypoint_node.py.
        # UtmProjector(useOffset=True) already subtracts the datum, so forward(datum) == (0, 0).
        self.using_lanelet2 = HAVE_LANELET2 and not force_pyproj
        if self.using_lanelet2:
            self.projector = UtmProjector(Origin(self.datum_lat, self.datum_lon))
        else:
            self.projector = PyprojUtmProjector(self.datum_lat, self.datum_lon)

        # create Coordinate Reference System, used to convert between coordinate systems
        crs = pyproj.CRS.from_proj4(self.georef)
        self._wgs2tm = pyproj.Transformer.from_crs(WGS84_EPSG, crs, always_xy=True)
        self._tm2wgs = pyproj.Transformer.from_crs(crs, WGS84_EPSG, always_xy=True)

    # ------------------------------------------------------------------ #
    #  Construction helpers                                              #
    # ------------------------------------------------------------------ #
    @classmethod
    def from_opendrive(cls, xodr: str,
                       datum_lat: float = DEFAULT_DATUM_LAT,
                       datum_lon: float = DEFAULT_DATUM_LON) -> 'CarlaGeo':
        """Build a `CarlaGeo` from carla.Map.to_opendrive() output; falls back to MCITY_TMERC."""
        return cls(datum_lat, datum_lon, parse_georeference(xodr))

    # ------------------------------------------------------------------ #
    #  map <-> WGS84                                                     #
    # ------------------------------------------------------------------ #
    def map_to_wgs(self, x: float, y: float) -> Tuple[float, float]:
        if self.using_lanelet2:
            g = self.projector.reverse(BasicPoint3d(float(x), float(y), 0.0))
            return g.lat, g.lon
        return self.projector.reverse_xy(float(x), float(y))

    def wgs_to_map(self, lat: float, lon: float) -> Tuple[float, float]:
        if self.using_lanelet2:
            p = self.projector.forward(GPSPoint(float(lat), float(lon), 0.0))
            return p.x, p.y
        return self.projector.forward_xy(float(lat), float(lon))

    # ------------------------------------------------------------------ #
    #  WGS84 <-> CARLA                                                   #
    # ------------------------------------------------------------------ #
    def wgs_to_carla(self, lat: float, lon: float, z: float = 0.0) -> Tuple[float, float, float]:
        tx, ty = self._wgs2tm.transform(float(lon), float(lat))
        return tx, -ty, float(z)

    def carla_to_wgs(self, cx: float, cy: float) -> Tuple[float, float]:
        lon, lat = self._tm2wgs.transform(float(cx), -float(cy))
        return lat, lon

    # ------------------------------------------------------------------ #
    #  map <-> CARLA                                                     #
    # ------------------------------------------------------------------ #
    def map_to_carla(self, x: float, y: float, z: float = 0.0) -> Tuple[float, float, float]:
        lat, lon = self.map_to_wgs(x, y)
        return self.wgs_to_carla(lat, lon, z)

    def carla_to_map(self, cx: float, cy: float) -> Tuple[float, float]:
        lat, lon = self.carla_to_wgs(cx, cy)
        return self.wgs_to_map(lat, lon)

    # ------------------------------------------------------------------ #
    #  Headings                                                          #
    #  (Pattern: transform a unit direction vector ->                    #
    #            measure angle in target frame)                          #
    # ------------------------------------------------------------------ #
    def map_yaw_to_carla_yaw_deg(self, x: float, y: float, yaw_rad: float) -> float:
        """ROS yaw (rad, CCW from map +x) at (x, y) -> CARLA/UE yaw (deg)."""
        c0 = self.map_to_carla(x, y)
        c1 = self.map_to_carla(x + math.cos(yaw_rad), y + math.sin(yaw_rad))
        return math.degrees(math.atan2(c1[1] - c0[1], c1[0] - c0[0]))

    def carla_pose_to_map(
        self, cx: float, cy: float, fx: float, fy: float
    ) -> Tuple[float, float, float]:
        """CARLA location + forward vector (UE frame) -> (x, y, yaw_rad) in the map frame."""
        m0 = self.carla_to_map(cx, cy)
        m1 = self.carla_to_map(cx + fx, cy + fy)
        return m0[0], m0[1], math.atan2(m1[1] - m0[1], m1[0] - m0[0])

    def carla_yaw_to_map_yaw(self, cx: float, cy: float, yaw_deg: float) -> float:
        r = math.radians(yaw_deg)
        return self.carla_pose_to_map(cx, cy, math.cos(r), math.sin(r))[2]

    @staticmethod
    def compass_deg_from_carla_yaw(yaw_deg: float) -> float:
        """
        Convert UE yaw (deg, CW+) to compass heading (deg, 0 = north, CW).

        Approximate (ignores convergence).
        """
        return (90.0 + yaw_deg) % 360.0

    def grid_convergence_deg(self) -> float:
        """
        Return the angle from tmerc +y to map +y at the datum, in degrees.

        tmerc +y is true north at lon_0, map +y is UTM grid north.
        """
        c0 = self.map_to_carla(0.0, 0.0)
        c1 = self.map_to_carla(0.0, 100.0)
        # in the un-flipped tmerc frame the bearing of map north measured from +x
        bearing = math.degrees(math.atan2(-(c1[1] - c0[1]), c1[0] - c0[0]))
        return bearing - 90.0
