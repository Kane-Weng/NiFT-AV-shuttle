"""Dump what the NiFT nodes ask of Lanelet2, for one map and one query set, to JSON.

The same script runs against the real Lanelet2 (in Docker, nift_pub_deps:humble) and against the stand-in (natively,
with nift_playground/py/fakelanelet2 on sys.path). equivalence.py compares two dumps.

    python3 lanelet2_dump.py queries.json out.json [map_path]

map_path overrides queries['map'] (the map sits at a different path inside Docker).

Python 3.10 compatible. Uses only the public lanelet2 Python API, so both sides run the same code.
"""
import json
import sys
import time


def _ll_key(ll):
    return '%d%s' % (ll.id, 'i' if ll.inverted() else '')


def dump(lanelet2, queries):
    from lanelet2.core import BasicPoint2d, BasicPoint3d, GPSPoint
    from lanelet2.projection import UtmProjector
    from lanelet2.routing import RoutingGraph
    from lanelet2.traffic_rules import Locations, Participants

    out = {'timing': {}}
    origin = lanelet2.io.Origin(queries['origin'][0], queries['origin'][1])
    projector = UtmProjector(origin)

    # projection, both ways, as waypoint_node / shuttle_tf_node / localization_node / carla_geo use it
    fwd = []
    for lat, lon, ele in queries['gps']:
        p = projector.forward(GPSPoint(lat, lon, ele))
        fwd.append([p.x, p.y, p.z])
    out['forward'] = fwd
    rev = []
    for x, y in queries['xy_reverse']:
        g = projector.reverse(BasicPoint3d(x, y, 0.0))
        rev.append([g.lat, g.lon])
    out['reverse'] = rev

    t0 = time.time()
    m = lanelet2.io.load(queries['map'], projector)
    out['timing']['load'] = time.time() - t0

    t0 = time.time()
    lanelets = {}
    for ll in m.laneletLayer:
        cl = [[p.x, p.y, p.z] for p in ll.centerline]
        lb = ll.leftBound
        rb = ll.rightBound
        lanelets[str(ll.id)] = {
            'left': lb.id, 'left_inv': lb.inverted(), 'right': rb.id, 'right_inv': rb.inverted(),
            'left_first': [lb[0].x, lb[0].y, lb[0].z] if len(lb) else None,
            'right_first': [rb[0].x, rb[0].y, rb[0].z] if len(rb) else None,
            'centerline': cl,
            'alen': lanelet2.geometry.approximatedLength2d(ll),
        }
    out['timing']['centerlines'] = time.time() - t0
    out['lanelets'] = lanelets
    out['counts'] = {'lanelets': len(m.laneletLayer), 'linestrings': len(m.lineStringLayer),
                     'points': len(m.pointLayer), 'areas': len(m.areaLayer),
                     'regulatory_elements': len(m.regulatoryElementLayer)}

    rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
    t0 = time.time()
    graph = RoutingGraph(m, rules)
    out['timing']['graph'] = time.time() - t0

    # traffic rules and graph structure
    structure = {}
    for ll in m.laneletLayer:
        for cand in (ll, ll.invert()):
            entry = {
                'canPass': rules.canPass(cand),
                'following': sorted(_ll_key(x) for x in graph.following(cand, False)),
                'following_lc': sorted(_ll_key(x) for x in graph.following(cand, True)),
            }
            left = graph.left(cand)
            right = graph.right(cand)
            entry['left'] = _ll_key(left) if left is not None else None
            entry['right'] = _ll_key(right) if right is not None else None
            sl = rules.speedLimit(cand)
            entry['speed_limit'] = sl.speedLimit if isinstance(sl.speedLimit, float) else float(sl.speedLimit)
            structure[_ll_key(cand)] = entry
    out['structure'] = structure

    # nearest lanelet for each query point (waypoint_node: findNearest(laneletLayer, point, 1))
    t0 = time.time()
    nearest = []
    for x, y in queries['points']:
        res = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(x, y), 3)
        nearest.append([[d, _ll_key(ll)] for d, ll in res])
    out['timing']['nearest'] = time.time() - t0
    out['nearest'] = nearest

    def path_keys(path):
        return None if path is None else [_ll_key(ll) for ll in path]

    # routes between lanelet ids, as LaneletPlanner.find_feasible_lanelet_path does: getRoute(a, b, 0)
    t0 = time.time()
    routes = []
    for a, b in queries['lanelet_pairs']:
        la = m.laneletLayer[a]
        lb_ = m.laneletLayer[b]
        route = graph.getRoute(la, lb_, 0)
        entry = {'route0': path_keys(route.shortestPath()) if route is not None else None,
                 'length2d': route.length2d() if route is not None else None}
        sp1 = graph.shortestPath(la, lb_, 1)
        entry['shortest1'] = path_keys(sp1)
        sp0n = graph.shortestPath(la, lb_, 0, False)
        entry['shortest0_nolc'] = path_keys(sp0n)
        routes.append(entry)
    out['timing']['routes'] = time.time() - t0
    out['routes'] = routes

    # the whole chain from two points: nearest start, nearest goal, getRoute
    pipeline = []
    for (sx, sy), (gx, gy) in queries['point_pairs']:
        s = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(sx, sy), 1)
        g = lanelet2.geometry.findNearest(m.laneletLayer, BasicPoint2d(gx, gy), 1)
        entry = {'start': _ll_key(s[0][1]), 'goal': _ll_key(g[0][1])}
        if s[0][1].id == g[0][1].id:
            entry['route'] = [_ll_key(s[0][1])]
        else:
            route = graph.getRoute(s[0][1], g[0][1], 0)
            entry['route'] = path_keys(route.shortestPath()) if route is not None else None
        pipeline.append(entry)
    out['pipeline'] = pipeline

    # node projection spot check
    pts = sorted(p.id for p in m.pointLayer)
    out['points'] = {str(i): [m.pointLayer[i].x, m.pointLayer[i].y, m.pointLayer[i].z] for i in pts[::37]}
    return out


def main():
    import lanelet2
    with open(sys.argv[1]) as f:
        queries = json.load(f)
    if len(sys.argv) > 3:
        queries['map'] = sys.argv[3]
    out = dump(lanelet2, queries)
    out['is_stand_in'] = bool(getattr(lanelet2, 'IS_STAND_IN', False))
    with open(sys.argv[2], 'w') as f:
        json.dump(out, f)


if __name__ == '__main__':
    main()
