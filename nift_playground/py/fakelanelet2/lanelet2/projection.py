"""Projections of the Lanelet2 stand-in: UtmProjector, as lanelet2_projection/src/UTM.cpp in Lanelet2 1.2.3.

UtmProjector(origin) picks the UTM zone of the origin. With useOffset (the default) it subtracts the origin's UTM
coordinates, so forward(origin) is (0, 0). Points in a neighbouring zone are transferred into the origin's zone.

Third-party portions: translated from Lanelet2 1.2.3 (https://github.com/fzi-forschungszentrum-informatik/Lanelet2),
BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik. See ../LICENSES/BSD-3-Clause-Lanelet2.txt and
../NOTICE.
"""
import math

from . import _utm
from .core import BasicPoint3d, GPSPoint


class ForwardProjectionError(RuntimeError):
    pass


class ReverseProjectionError(RuntimeError):
    pass


class Projector:
    def __init__(self, origin=None):
        from .io import Origin
        self._origin = origin if origin is not None else Origin()

    def origin(self):
        return self._origin

    def forward(self, gps):
        raise NotImplementedError

    def reverse(self, point):
        raise NotImplementedError


class UtmProjector(Projector):
    def __init__(self, origin, useOffset=True, throwInPaddingArea=False):
        super().__init__(origin)
        pos = self._origin.position
        zone, northp, x, y = _utm.utm_forward(pos.lat, pos.lon)
        self._zone = zone
        self._northp = northp
        self._use_offset = bool(useOffset)
        self._throw_in_padding = bool(throwInPaddingArea)
        self._x_offset = x if useOffset else 0.0
        self._y_offset = y if useOffset else 0.0

    def forward_xy(self, lat, lon):
        """(lat, lon) -> (x, y) without building point objects. Same numbers as forward()."""
        try:
            zone, northp, x, y = _utm.utm_forward(lat, lon)
            if zone != self._zone or northp != self._northp:
                if self._throw_in_padding:
                    raise ForwardProjectionError('You have left the UTM zone or changed the hemisphere!')
                x, y = _utm.utm_transfer(zone, northp, x, y, self._zone, self._northp)
        except ValueError as e:
            raise ForwardProjectionError(str(e)) from None
        if self._use_offset:
            x -= self._x_offset
            y -= self._y_offset
        return x, y

    def forward(self, gps):
        x, y = self.forward_xy(gps.lat, gps.lon)
        return BasicPoint3d(x, y, gps.ele)

    def reverse(self, point):
        x = point.x + self._x_offset if self._use_offset else point.x
        y = point.y + self._y_offset if self._use_offset else point.y
        try:
            lat, lon = _utm.utm_reverse(self._zone, self._northp, x, y)
        except ValueError as e:
            raise ReverseProjectionError(str(e)) from None
        gps = GPSPoint(lat, lon, getattr(point, 'z', 0.0))
        if self._throw_in_padding:
            try:
                self.forward(gps)
            except ForwardProjectionError as e:
                raise ReverseProjectionError(str(e)) from None
        return gps


class SphericalMercatorProjector(Projector):
    """Lanelet2's default projector (lanelet2_io/Projection.h). Imprecise, kept for API completeness."""
    EarthRadius = 6378137.0

    def forward(self, gps):
        scale = math.cos(self._origin.position.lat * math.pi / 180.0)
        x = scale * gps.lon * math.pi * self.EarthRadius / 180.0
        y = scale * self.EarthRadius * math.log(math.tan((90.0 + gps.lat) * math.pi / 360.0))
        return BasicPoint3d(x, y, gps.ele)

    def reverse(self, point):
        scale = math.cos(self._origin.position.lat * math.pi / 180.0)
        lon = point.x * 180.0 / (math.pi * self.EarthRadius * scale)
        lat = 360.0 * math.atan(math.exp(point.y / (self.EarthRadius * scale))) / math.pi - 90.0
        return GPSPoint(lat, lon, getattr(point, 'z', 0.0))
