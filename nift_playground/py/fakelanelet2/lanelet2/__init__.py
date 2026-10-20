"""A pure-Python stand-in for the Lanelet2 1.2.3 Python API, so the NiFT nodes run unchanged in the browser.

Put nift_playground/py/fakelanelet2 on sys.path and `import lanelet2` gets this package. It needs only the standard
library (no numpy, no C extensions), so it runs in Pyodide.

It covers what nift_waypoint, nift_sensor and nift_carla's carla_geo use: io.load with a UtmProjector, the core
types, geometry.findNearest and friends, traffic_rules.create(Germany, Vehicle) and routing.RoutingGraph with
getRoute / shortestPath. The tests in nift_playground/tests/lanelet2 check it against saved results of the real
Lanelet2. It matches real Lanelet2 within the tested tolerances: projection and map nodes within 1 mm, centerline
points within 1 cm, and the same nearest lanelet (ties included), graph links and routes on the NiFT map and on
Lanelet2's example map.

How it stays close to Lanelet2 (ROS Humble build):
- _utm.py repeats GeographicLib 1.52's UTM maths step by step
- _centerline.py, geometry.py, routing.py and traffic_rules.py follow Lanelet2's own algorithms
- _rtree.py repeats the boost R-tree that decides ties in findNearest, and _stdorder.py the container order it is
  built from (written from CLEANROOM_SPEC.md, see NOTICE)
- _nodecache.py takes map node coordinates from a cache made by the real Lanelet2, when the map ships one, so the
  browser's C library cannot move a node by a nanometre

This is an independent reimplementation, not part of the Lanelet2 project and not endorsed by FZI. Parts of it are
translated from Lanelet2 (BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik), Boost.Geometry (Boost
Software License 1.0) and GeographicLib (MIT, Charles Karney). NOTICE lists the third-party portions per file, and
LICENSES/ holds the licence texts (both next to this package).
"""
from . import core, geometry, io, projection, routing, traffic_rules  # noqa: F401

__version__ = '1.2.3+nift.stand_in'
IS_STAND_IN = True
