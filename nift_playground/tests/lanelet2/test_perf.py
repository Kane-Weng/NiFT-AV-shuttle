"""Load time of the NiFT map with the stand-in, natively. Pyodide is slower; the numbers go in the report.

The budget is loose (CI machines vary). Run with -s to see the numbers.
"""
import os
import time

import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.projection import UtmProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants

MAP = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'maps', 'lanelet2_odr2024.osm'))


def test_load_budget():
    t0 = time.perf_counter()
    m = lanelet2.io.load(MAP, UtmProjector(lanelet2.io.Origin(42.3005, -83.6987)))
    t1 = time.perf_counter()
    n = sum(len(ll.centerline) for ll in m.laneletLayer)
    t2 = time.perf_counter()
    g = RoutingGraph(m, lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle))
    t3 = time.perf_counter()
    res = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(0.0, 0.0), 1)
    t4 = time.perf_counter()
    a, b = m.laneletLayer[1], m.laneletLayer[300]
    g.getRoute(a, b, 0)
    t5 = time.perf_counter()
    print('\nodr2024: load %.2f s, centerlines %.2f s (%d points), graph %.3f s, first findNearest %.3f s, '
          'route %.4f s' % (t1 - t0, t2 - t1, n, t3 - t2, t4 - t3, t5 - t4))
    assert res and n > 40000
    assert t3 - t0 < 10.0
