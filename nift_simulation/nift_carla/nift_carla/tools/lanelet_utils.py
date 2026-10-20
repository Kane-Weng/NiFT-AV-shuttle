"""
Load the Lanelet2 map the way the stack does and extract centerlines.

File: lanelet_utils.py

Description:
  Small helpers shared by the nift_carla tools: load the NiFT Lanelet2 map with
  the same projector/datum the stack uses and extract centerlines in the map frame.
"""

import math
import os
from typing import List, Tuple

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

# the sim's map (nift_bringup profiles.DEFAULT_MAPS['sim']): generated from the OpenDRIVE that
# CARLA loads
# the twin's 2024 world, which a stock CARLA server with the twin loads
SIM_MAP = 'lanelet2_odr2024.osm'


def default_map_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('nift_waypoint'), 'maps', SIM_MAP)
        if os.path.exists(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', '..', '..', '..', 'maps', SIM_MAP))


def load_map(path: str, datum_lat: float, datum_lon: float):
    return lanelet2.io.load(path, UtmProjector(Origin(datum_lat, datum_lon)))


def centerline_xy(ll) -> List[Tuple[float, float]]:
    return [(p.x, p.y) for p in ll.centerline]


def polyline_length(pts: List[Tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def heading_spread_deg(pts: List[Tuple[float, float]]) -> float:
    """Max absolute deviation of segment headings from the overall start->end bearing."""
    if len(pts) < 2:
        return 0.0
    base = math.atan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0])
    worst = 0.0
    for a, b in zip(pts, pts[1:]):
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        d = math.degrees(abs(math.atan2(math.sin(h - base), math.cos(h - base))))
        worst = max(worst, d)
    return worst


def resample(pts: List[Tuple[float, float]], step: float) -> List[Tuple[float, float]]:
    """Resample a polyline every `step` metres (keeps the last point)."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    carry = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg == 0.0:
            continue
        d = step - carry
        while d <= seg:
            t = d / seg
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            d += step
        carry = seg - (d - step)
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out
