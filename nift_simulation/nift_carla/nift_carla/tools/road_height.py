"""
Return the road height under a CARLA (x, y) from the driving waypoints.

File: road_height.py

Description:
  Height of the road under a CARLA (x, y), from the map's driving waypoints.

  The Mcity roads sit at z 266..279 m. Map.get_waypoint(project_to_road=True) returns the nearest
  lane in 3-D, so a query at z = 0 lands on whatever road is nearest to the ground plane: 15..33 m
  away from the right one at Mcity (2026-10-11), which made carla_check_alignment fail for any map.
  Query at RoadHeight(cmap)(x, y) instead.
"""

import carla
import numpy as np


class RoadHeight:
    """height(cx, cy) -> z of the nearest driving waypoint (2-D), sampled every `spacing` m."""

    def __init__(self, cmap, spacing: float = 1.0):
        wps = [
            w for w in cmap.generate_waypoints(spacing) if w.lane_type == carla.LaneType.Driving
        ]
        self._xy = np.array([(w.transform.location.x, w.transform.location.y) for w in wps])
        self._z = np.array([w.transform.location.z for w in wps])

    def __call__(self, cx: float, cy: float) -> float:
        d2 = (self._xy[:, 0] - cx) ** 2 + (self._xy[:, 1] - cy) ** 2
        return float(self._z[np.argmin(d2)])
