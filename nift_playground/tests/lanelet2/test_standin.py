"""Unit tests of the Lanelet2 stand-in: API surface, projection, centerline, traffic rules, routing, C++ order.

The equivalence against the real Lanelet2 is in test_equivalence.py; these tests pin the pieces on small inputs.
"""
import gzip
import json
import math
import os

import pytest

import lanelet2
from lanelet2 import _stdorder, _utm
from lanelet2.core import BasicPoint2d, BasicPoint3d, GPSPoint
from lanelet2.projection import UtmProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
ODR2024 = os.path.join(REPO, 'maps', 'lanelet2_odr2024.osm')
SITE_ORIGIN = (42.3005, -83.6987)  # nift_common.site MAP_ORIGIN_LAT / LON


def test_is_the_stand_in():
    assert lanelet2.IS_STAND_IN
    assert 'fakelanelet2' in lanelet2.__file__


def test_api_names_the_nodes_use():
    # waypoint_node, LaneletPlanner, WaypointVisualizer, shuttle_tf_node, localization_node, carla_geo
    for mod, names in (
            (lanelet2.io, ['Origin', 'load', 'loadRobust']),
            (lanelet2.projection, ['UtmProjector']),
            (lanelet2.core, ['GPSPoint', 'BasicPoint2d', 'BasicPoint3d', 'Lanelet', 'ConstLanelet', 'LineString3d',
                             'ConstLineString3d', 'Point3d', 'LaneletMap']),
            (lanelet2.geometry, ['findNearest', 'distance', 'toArcCoordinates', 'approximatedLength2d', 'length2d',
                                 'follows', 'leftOf', 'rightOf', 'inside', 'boundingBox2d']),
            (lanelet2.traffic_rules, ['create', 'Locations', 'Participants']),
            (lanelet2.routing, ['RoutingGraph', 'Route', 'LaneletPath', 'RoutingCostDistance',
                                'RoutingCostTravelTime'])):
        for n in names:
            assert hasattr(mod, n), '%s.%s' % (mod.__name__, n)


# ---------------------------------------------------------------- projection

def test_projection_origin_is_zero_and_round_trips():
    p = UtmProjector(lanelet2.io.Origin(*SITE_ORIGIN))
    o = p.forward(GPSPoint(SITE_ORIGIN[0], SITE_ORIGIN[1], 0.0))
    assert (o.x, o.y) == (0.0, 0.0)
    for lat, lon in ((42.3021188854, -83.6974999742), (42.29, -83.71), (42.31, -83.69)):
        q = p.forward(GPSPoint(lat, lon, 12.5))
        assert q.z == 12.5
        g = p.reverse(BasicPoint3d(q.x, q.y, q.z))
        assert abs(g.lat - lat) < 1e-11 and abs(g.lon - lon) < 1e-11
        assert g.alt == g.ele == 12.5


def test_projection_reference_values():
    # real Lanelet2 1.2.3 (GeographicLib 1.52) in nift_pub_deps:humble, bit for bit
    p = UtmProjector(lanelet2.io.Origin(*SITE_ORIGIN))
    assert p.forward_xy(42.3021188854, -83.6974999742) == (104.62535970879253, 176.63394920155406)
    zone, northp, x, y = _utm.utm_forward(42.3021188854, -83.6974999742)
    assert (zone, northp) == (17, True)
    assert x == 277651.45062129333 and abs(y - 4686845.1639857134) < 1e-9


def test_projection_other_zone_is_transferred():
    p = UtmProjector(lanelet2.io.Origin(*SITE_ORIGIN))
    a = p.forward(GPSPoint(42.3, -84.0001, 0))   # zone 16
    b = p.forward(GPSPoint(42.3, -83.9999, 0))   # zone 17
    # 0.0002 degrees of longitude at 42.3 N is about 16.5 m (grid convergence turns it 2 degrees); no jump
    assert math.hypot(b.x - a.x, b.y - a.y) == pytest.approx(16.49, abs=0.05)
    with pytest.raises(lanelet2.projection.ForwardProjectionError):
        UtmProjector(lanelet2.io.Origin(*SITE_ORIGIN), True, True).forward(GPSPoint(42.3, -84.5, 0))


def _cxx_truth():
    with gzip.open(os.path.join(HERE, 'fixtures', 'cxx_ground_truth.json.gz'), 'rt') as f:
        return json.load(f)


def test_std_sort_and_nth_element_match_libstdcxx():
    for case in _cxx_truth()['sort_cases']:
        a = [(v, i) for i, v in enumerate(case['values'])]
        less = lambda p, q: p[0] < q[0]  # noqa: E731
        b = list(a)
        _stdorder.sort_range(b, less)
        assert [i for _, i in b] == case['sort']
        c = list(a)
        _stdorder.select_range(c, 0, case['k'], len(c), less)
        assert [i for _, i in c] == case['nth_element']


def test_unordered_map_order_matches_libstdcxx():
    for name, t in _cxx_truth()['unordered_map'].items():
        order, n_bkt = _stdorder.hash_map_order(t['keys_ascending'])
        assert n_bkt == t['bucket_count'], name
        assert order == t['order'], name


# ---------------------------------------------------------------- a small synthetic map

def _osm(lanes_change='dashed', second_change='solid', one_way=None):
    """Two lanes going +x for 200 m in two 100 m pieces. Lane 1 (left, ids 1 and 3), lane 2 (right, 2 and 4).
    The line between them is line_thin with the given subtype (first piece / second piece)."""
    lat0, lon0 = 49.0, 8.4
    m_per_deg_lat = 111132.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
    nodes = []
    ways = []

    def node(nid, x, y):
        nodes.append("<node id='%d' lat='%.10f' lon='%.10f'/>" % (nid, lat0 + y / m_per_deg_lat,
                                                                   lon0 + x / m_per_deg_lon))

    def line(wid, y, x0, nid0, tags, first_nid=None):
        ids = []
        for k in range(11):
            nid = nid0 + k
            if k == 0 and first_nid is not None:
                ids.append(first_nid)
                continue
            node(nid, x0 + 10.0 * k, y)
            ids.append(nid)
        ways.append("<way id='%d'>%s%s</way>" % (wid, ''.join("<nd ref='%d'/>" % i for i in ids),
                                                 ''.join("<tag k='%s' v='%s'/>" % kv for kv in tags)))
        return ids

    a = line(101, 7.0, 0.0, 1000, [('type', 'line_thin'), ('subtype', 'solid')])
    b = line(102, 3.5, 0.0, 1100, [('type', 'line_thin'), ('subtype', lanes_change)])
    c = line(103, 0.0, 0.0, 1200, [('type', 'line_thin'), ('subtype', 'solid')])
    line(201, 7.0, 100.0, 2000, [('type', 'line_thin'), ('subtype', 'solid')], first_nid=a[-1])
    line(202, 3.5, 100.0, 2100, [('type', 'line_thin'), ('subtype', second_change)], first_nid=b[-1])
    line(203, 0.0, 100.0, 2200, [('type', 'line_thin'), ('subtype', 'solid')], first_nid=c[-1])
    rels = []
    for rid, left, right in ((1, 101, 102), (2, 102, 103), (3, 201, 202), (4, 202, 203)):
        tags = [('type', 'lanelet'), ('subtype', 'road'), ('location', 'urban')]
        if one_way is not None and rid == 4:
            tags.append(('one_way', one_way))
        rels.append("<relation id='%d'><member type='way' ref='%d' role='left'/>"
                    "<member type='way' ref='%d' role='right'/>%s</relation>"
                    % (rid, left, right, ''.join("<tag k='%s' v='%s'/>" % kv for kv in tags)))
    return ("<?xml version='1.0'?><osm version='0.6'>%s%s%s</osm>"
            % (''.join(nodes), ''.join(ways), ''.join(rels))).encode()


def _load(data):
    m = lanelet2.io.load(data, UtmProjector(lanelet2.io.Origin(49.0, 8.4)))
    rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
    return m, rules, RoutingGraph(m, rules)


def test_load_and_centerline():
    m, _, _ = _load(_osm())
    assert len(m.laneletLayer) == 4
    ll = m.laneletLayer[1]
    assert ll.leftBound.id == 101 and ll.rightBound.id == 102
    cl = list(ll.centerline)
    assert len(cl) >= 11
    for p in cl:
        # halfway between the lines y=7 and y=3.5 (the projection bends the lines a little)
        lp = lanelet2.geometry.distance(ll.leftBound, BasicPoint2d(p.x, p.y))
        rp = lanelet2.geometry.distance(ll.rightBound, BasicPoint2d(p.x, p.y))
        assert abs(lp - rp) < 1e-3
    inv = ll.invert()
    assert inv.inverted() and inv.id == ll.id and inv != ll and inv.invert() == ll
    assert inv.leftBound == ll.rightBound.invert()
    assert [(p.x, p.y) for p in inv.centerline] == [(p.x, p.y) for p in reversed(cl)]


def test_load_errors_raise_like_lanelet2():
    bad = _osm().replace(b"<member type='way' ref='103' role='right'/>", b'')
    with pytest.raises(lanelet2.io.ParseError):
        _load(bad)
    m, errors = lanelet2.io.loadRobust(bad, UtmProjector(lanelet2.io.Origin(49.0, 8.4)))
    assert errors and len(m.laneletLayer) == 4
    with pytest.raises(lanelet2.io.ParseError):
        lanelet2.io.load(_osm(), UtmProjector(lanelet2.io.Origin()))  # Lanelet2 refuses the default origin


def test_lane_change_follows_line_marking():
    m, rules, g = _load(_osm('dashed', 'solid'))
    l1, l2, l3, l4 = (m.laneletLayer[i] for i in (1, 2, 3, 4))
    assert rules.canChangeLane(l2, l1) and rules.canChangeLane(l1, l2)
    assert not rules.canChangeLane(l4, l3)
    assert [x.id for x in g.following(l2, False)] == [4]
    assert sorted(x.id for x in g.following(l2, True)) == [1, 4]
    assert g.left(l2) == l1 and g.right(l1) == l2 and g.left(l4) is None
    path = g.getRoute(l2, l3, 0).shortestPath()
    assert [x.id for x in path] == [2, 1, 3]
    assert g.shortestPath(l2, l3, 0, False) is None
    # cost: one lane change (10) plus the successor edge (half of each lanelet's approximate length)
    want = 10 + (lanelet2.geometry.approximatedLength2d(l1) + lanelet2.geometry.approximatedLength2d(l3)) / 2
    assert abs(g.pathCost(path) - want) < 1e-9

    m, rules, g = _load(_osm('dashed_solid', 'solid'))   # only to the right, seen from the line's direction
    l1, l2 = m.laneletLayer[1], m.laneletLayer[2]
    assert rules.canChangeLane(l1, l2) and not rules.canChangeLane(l2, l1)
    m, rules, g = _load(_osm('solid', 'solid'))
    assert g.getRoute(m.laneletLayer[2], m.laneletLayer[3], 0) is None


def test_one_way_and_inverted_routing():
    m, rules, g = _load(_osm(one_way='no'))
    l4 = m.laneletLayer[4]
    assert rules.canPass(l4.invert()) and not rules.canPass(m.laneletLayer[3].invert())
    assert not rules.isOneWay(l4) and rules.isOneWay(m.laneletLayer[3])
    # the inverted lanelet 4 ends where lanelet 2 ends, so nothing follows it here
    assert g.following(l4.invert(), False) == []
    assert rules.speedLimit(l4).speedLimit == pytest.approx(50.0)  # German urban road default, km/h


def _left_normal(ls, i):
    a, b = ls[i], ls[i + 1]
    d = math.hypot(b.x - a.x, b.y - a.y)
    return -(b.y - a.y) / d, (b.x - a.x) / d


def test_find_nearest():
    m, _, _ = _load(_osm())
    l1 = m.laneletLayer[1]
    c = list(l1.centerline)[3]
    res = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(c.x, c.y), 1)
    assert len(res) == 1 and res[0][0] == 0.0 and res[0][1] == l1
    nx, ny = _left_normal(l1.centerline, 3)
    res = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(c.x + 20 * nx, c.y + 20 * ny), 2)
    # 20 m left of the centerline is 18.25 m left of the left line (lane 3.5 m wide)
    assert [ll.id for _, ll in res] == [1, 2] and res[0][0] == pytest.approx(18.25, abs=0.01)


def test_to_arc_coordinates():
    m, _, _ = _load(_osm())
    ll = m.laneletLayer[1]
    p0 = ll.centerline[0]
    p1 = ll.centerline[len(ll.centerline) - 1]
    d = math.hypot(p1.x - p0.x, p1.y - p0.y)
    tx, ty = (p1.x - p0.x) / d, (p1.y - p0.y) / d
    arc = lanelet2.geometry.toArcCoordinates(ll.centerline, BasicPoint2d(p0.x + 25 * tx - ty, p0.y + 25 * ty + tx))
    assert arc.length == pytest.approx(25.0, abs=0.05)
    assert arc.distance == pytest.approx(1.0, abs=0.05)  # left of the centerline is positive


# ---------------------------------------------------------------- the NiFT map

@pytest.fixture(scope='module')
def odr():
    m = lanelet2.io.load(ODR2024, UtmProjector(lanelet2.io.Origin(*SITE_ORIGIN)))
    rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
    return m, rules, RoutingGraph(m, rules)


def test_odr2024_links_like_real_lanelet2(odr):
    m, _, g = odr
    assert len(m.laneletLayer) == 507
    assert m._node_cache_used  # node_cache/a148057a7545a8fd.bin, made by make_node_cache.py
    # real Lanelet2 links 314 and 317 to 74 although their ends are 34 m apart: they share the end node ids
    for a in (314, 317):
        assert 74 in [x.id for x in g.following(m.laneletLayer[a], False)]


def test_waypoint_planner_runs_on_the_stand_in(odr, playground):
    """LaneletPlanner from nift_waypoint, unchanged, on the stand-in: same lanelets as real Lanelet2's getRoute."""
    from nift_waypoint.utils.lanelet_planner import LaneletPlanner
    import equivalence as eq
    m, _, g = odr
    fixture = eq.load_fixture('odr2024')

    class Log:
        def info(self, *a, **k):
            pass
        warn = warning = error = debug = info

    planner = LaneletPlanner(routing_graph=g, logger=Log(), max_search_expansions=3000, max_candidate_routes=5,
                             allow_lane_changes_in_search=False, curvature_check_point_step=1,
                             planning_min_turn_radius_m=4.0)
    checked = 0
    for (s, gl), real in zip(fixture['queries']['point_pairs'], fixture['real']['pipeline']):
        start = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(*s), 1)[0][1]
        goal = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(*gl), 1)[0][1]
        lanelets, _ = planner.find_feasible_lanelet_path(start, goal)
        got = None if lanelets is None else ['%d%s' % (x.id, 'i' if x.inverted() else '') for x in lanelets]
        assert got == real['route']
        if lanelets:
            pts = planner.build_path_points_abs(lanelets, BasicPoint2d(*s), BasicPoint2d(*gl))
            assert len(pts) >= 1
        checked += 1
    assert checked >= 200
