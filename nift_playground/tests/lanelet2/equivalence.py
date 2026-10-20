"""Query generation and comparison for the Lanelet2 equivalence check (stand-in against the real Lanelet2).

Tolerances (from the U11 plan): projection within 1 mm, centerline points within 1 cm, the same nearest lanelet and
the same route. Where Lanelet2's own result depends on an unordered container (equal distances, equal route costs),
"same" means the same distance or the same cost; the report counts these cases separately. compare(strict=True)
accepts no nearest-lanelet ties, only equal-cost route ties.
"""
import gzip
import json
import math
import os
import random

PY_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', 'py', 'fakelanelet2'))
REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
FIXTURES = os.path.join(os.path.dirname(__file__), 'fixtures')

# map name -> (path relative to the repo or absolute, origin lat, lon)
MAPS = {
    'odr2024': ('maps/lanelet2_odr2024.osm', 42.3005, -83.6987),  # nift_common.site MAP_ORIGIN_LAT/LON
    'mapping_example': ('nift_playground/tests/lanelet2/maps/mapping_example.osm', 49.0, 8.4),  # Lanelet2 examples
}

TOL_PROJ_M = 1e-3
TOL_CENTER_M = 1e-2
TOL_TIE_DIST = 1e-9  # a point on a shared border: distances 0 and 1e-12 are the same answer
TOL_TIE_COST = 1e-6


def map_path(name):
    rel = MAPS[name][0]
    return rel if os.path.isabs(rel) else os.path.join(REPO, rel)


def make_queries(name, lanelet2, n_points=500, n_pairs=250, seed=20261020):
    """Random but reproducible queries, generated with the stand-in (only map extents and ids are used)."""
    from lanelet2.projection import UtmProjector
    rel, lat0, lon0 = MAPS[name]
    rng = random.Random(seed)
    m = lanelet2.io.load(map_path(name), UtmProjector(lanelet2.io.Origin(lat0, lon0)))
    ids = sorted(ll.id for ll in m.laneletLayer)
    xs = [p.x for p in m.pointLayer]
    ys = [p.y for p in m.pointLayer]
    x0, x1, y0, y1 = min(xs) - 20, max(xs) + 20, min(ys) - 20, max(ys) + 20

    points = []
    for k in range(n_points):
        if k % 2 == 0:
            ll = m.laneletLayer[rng.choice(ids)]
            lb = list(ll.leftBound)
            rb = list(ll.rightBound)
            t = rng.random()
            pl = lb[min(int(t * len(lb)), len(lb) - 1)]
            pr = rb[min(int(t * len(rb)), len(rb) - 1)]
            u = rng.uniform(-0.2, 1.2)
            points.append([pl.x + u * (pr.x - pl.x) + rng.gauss(0, 0.3),
                           pl.y + u * (pr.y - pl.y) + rng.gauss(0, 0.3)])
        else:
            points.append([rng.uniform(x0, x1), rng.uniform(y0, y1)])
    point_pairs = [[points[2 * i], points[2 * i + 1]] for i in range(n_points // 2)]
    rng.shuffle(point_pairs)
    # half the pairs at random, half where the stand-in's graph says the goal is reachable (so routes get tested on
    # maps where most random pairs have no route); the real side decides the route either way
    lanelet_pairs = [[rng.choice(ids), rng.choice(ids)] for _ in range(n_pairs - n_pairs // 2)]
    from lanelet2.routing import RoutingGraph
    from lanelet2.traffic_rules import Locations, Participants
    graph = RoutingGraph(m, lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle))
    passable = [i for i in ids if graph._vertex(m.laneletLayer[i]) is not None]
    while len(lanelet_pairs) < n_pairs:
        a = rng.choice(passable)
        seen = {a}
        frontier = [m.laneletLayer[a]]
        while frontier:
            nxt = []
            for ll in frontier:
                for f in graph.following(ll, True):
                    if not f.inverted() and f.id not in seen:
                        seen.add(f.id)
                        nxt.append(f)
            frontier = nxt
        seen.discard(a)
        if seen:
            lanelet_pairs.append([a, rng.choice(sorted(seen))])
    gps = [[lat0 + rng.uniform(-0.02, 0.02), lon0 + rng.uniform(-0.02, 0.02), rng.uniform(-5, 300)]
           for _ in range(300)]
    # a few far points, some in the neighbouring UTM zone (transfer into the origin's zone)
    gps += [[lat0 + rng.uniform(-1.0, 1.0), lon0 + rng.uniform(-1.5, 1.5), 0.0] for _ in range(40)]
    xy_reverse = [[rng.uniform(-5000, 5000), rng.uniform(-5000, 5000)] for _ in range(300)]
    return {
        'name': name, 'map': rel, 'origin': [lat0, lon0], 'seed': seed,
        'gps': gps, 'xy_reverse': xy_reverse, 'points': points, 'point_pairs': point_pairs,
        'lanelet_pairs': lanelet_pairs,
    }


def _round_dump(real):
    """Shrink a real dump for the fixture: centerlines to 0.1 mm (the check is 1 cm)."""
    out = dict(real)
    lls = {}
    for k, v in real['lanelets'].items():
        v = dict(v)
        v['centerline'] = [[round(c, 4) for c in p] for p in v['centerline']]
        lls[k] = v
    out['lanelets'] = lls
    return out


def save_fixture(name, queries, real):
    os.makedirs(FIXTURES, exist_ok=True)
    path = os.path.join(FIXTURES, name + '.json.gz')
    with gzip.open(path, 'wt') as f:
        json.dump({'queries': queries, 'real': _round_dump(real)}, f, separators=(',', ':'))
    return path


def load_fixture(name):
    with gzip.open(os.path.join(FIXTURES, name + '.json.gz'), 'rt') as f:
        return json.load(f)


def _parse_key(key):
    return (int(key[:-1]), True) if key.endswith('i') else (int(key), False)


class _CostOracle:
    """Route costs on the stand-in's routing graph, to tell a different route from an equal-cost tie."""

    def __init__(self, lanelet2, queries):
        from lanelet2.projection import UtmProjector
        from lanelet2.routing import RoutingGraph
        from lanelet2.traffic_rules import Locations, Participants
        lat0, lon0 = queries['origin']
        self.m = lanelet2.io.load(map_path(queries['name']), UtmProjector(lanelet2.io.Origin(lat0, lon0)))
        self.g = RoutingGraph(self.m, lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle))

    def lanelet(self, key):
        i, inv = _parse_key(key)
        ll = self.m.laneletLayer[i]
        return ll.invert() if inv else ll

    def cost(self, keys, cost_id=0):
        if keys is None:
            return None
        return self.g.pathCost([self.lanelet(k) for k in keys], cost_id)

    def distance(self, key, xy):
        import lanelet2
        return lanelet2.geometry.distance(self.lanelet(key), lanelet2.core.BasicPoint2d(*xy))


def compare(real, stand, queries, oracle, strict=False):
    """Returns (stats dict, list of failure strings).

    strict=True fails on any nearest-lanelet mismatch, ties included: findNearest must return the same lanelets in the
    same order (all n results), and the point-to-point chain must start and end on the same lanelets. The stand-in
    repeats Lanelet2's R-tree to get ties right, so the tests and the Pyodide check run strict. Equal-cost route ties
    are still accepted (Lanelet2's boost heap decides those).
    """
    fails = []
    stats = {}

    # projection
    d_fwd = 0.0
    for a, b in zip(real['forward'], stand['forward']):
        d_fwd = max(d_fwd, max(abs(x - y) for x, y in zip(a, b)))
    stats['forward_max_diff_m'] = d_fwd
    if d_fwd > TOL_PROJ_M:
        fails.append('forward projection differs by %.3g m' % d_fwd)
    d_rev = 0.0
    for a, b in zip(real['reverse'], stand['reverse']):
        dn = abs(a[0] - b[0]) * 111320.0
        de = abs(a[1] - b[1]) * 111320.0 * math.cos(math.radians(a[0]))
        d_rev = max(d_rev, dn, de)
    stats['reverse_max_diff_m'] = d_rev
    if d_rev > TOL_PROJ_M:
        fails.append('reverse projection differs by %.3g m' % d_rev)
    d_pts = 0.0
    for k, v in real['points'].items():
        w = stand['points'].get(k)
        if w is None:
            fails.append('point %s missing' % k)
            continue
        d_pts = max(d_pts, max(abs(x - y) for x, y in zip(v, w)))
    stats['node_max_diff_m'] = d_pts
    if d_pts > TOL_PROJ_M:
        fails.append('map node projection differs by %.3g m' % d_pts)

    if real['counts'] != stand['counts']:
        fails.append('layer counts differ: %r vs %r' % (real['counts'], stand['counts']))

    # lanelets and centerlines
    d_cl = 0.0
    n_cl_pts = 0
    bad_cl = 0
    if set(real['lanelets']) != set(stand['lanelets']):
        fails.append('lanelet id sets differ')
    for k, r in real['lanelets'].items():
        s = stand['lanelets'].get(k)
        if s is None:
            continue
        for f in ('left', 'left_inv', 'right', 'right_inv'):
            if r[f] != s[f]:
                fails.append('lanelet %s: %s %r vs %r' % (k, f, r[f], s[f]))
        if abs(r['alen'] - s['alen']) > 1e-6:
            fails.append('lanelet %s: approximatedLength2d %r vs %r' % (k, r['alen'], s['alen']))
        rc, sc = r['centerline'], s['centerline']
        if len(rc) != len(sc):
            bad_cl += 1
            fails.append('lanelet %s: centerline has %d points, stand-in %d' % (k, len(rc), len(sc)))
            continue
        for a, b in zip(rc, sc):
            n_cl_pts += 1
            d_cl = max(d_cl, math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b))))
    stats['centerline_points'] = n_cl_pts
    stats['centerline_max_diff_m'] = d_cl
    stats['centerline_count_mismatch'] = bad_cl
    if d_cl > TOL_CENTER_M:
        fails.append('centerline points differ by up to %.3g m' % d_cl)

    # traffic rules and graph structure
    n_struct = 0
    for k, r in real['structure'].items():
        s = stand['structure'].get(k)
        n_struct += 1
        if s is None:
            fails.append('structure: %s missing' % k)
            continue
        for f in ('canPass', 'following', 'following_lc', 'left', 'right'):
            if r[f] != s[f]:
                fails.append('structure %s: %s %r vs %r' % (k, f, r[f], s[f]))
        if abs(r['speed_limit'] - s['speed_limit']) > 1e-9:
            fails.append('structure %s: speed limit %r vs %r' % (k, r['speed_limit'], s['speed_limit']))
    stats['structure_vertices'] = n_struct

    # nearest
    exact = ties = list_diff = 0
    d_near = 0.0
    for i, (r, s) in enumerate(zip(real['nearest'], stand['nearest'])):
        d_near = max(d_near, abs(r[0][0] - s[0][0]))
        if [e[1] for e in r] != [e[1] for e in s]:
            list_diff += 1
            if strict and r[0][1] == s[0][1]:
                fails.append('nearest %d at %r (strict, order of all results): real %r, stand-in %r'
                             % (i, queries['points'][i], r, s))
        if r[0][1] == s[0][1]:
            exact += 1
        elif abs(oracle.distance(r[0][1], queries['points'][i]) - s[0][0]) <= TOL_TIE_DIST:
            ties += 1
            if strict:
                fails.append('nearest %d at %r (strict, tie picked differently): real %r, stand-in %r'
                             % (i, queries['points'][i], r, s))
        else:
            fails.append('nearest %d at %r: real %r, stand-in %r' % (i, queries['points'][i], r, s))
    stats['nearest_points'] = len(real['nearest'])
    stats['nearest_same_id'] = exact
    stats['nearest_equal_distance_tie'] = ties
    stats['nearest_result_list_differs'] = list_diff
    stats['nearest_max_dist_diff_m'] = d_near
    if d_near > 1e-6:
        fails.append('nearest distances differ by up to %.3g m' % d_near)

    # routes
    def same_route(rk, sk, cost_id):
        if rk == sk:
            return 'same'
        if rk is None or sk is None:
            return None
        cr = oracle.cost(rk, cost_id)
        cs = oracle.cost(sk, cost_id)
        if cr is not None and cs is not None and abs(cr - cs) <= TOL_TIE_COST * max(1.0, cr):
            return 'tie'
        return None

    counts = {'same': 0, 'tie': 0, 'no_route': 0}
    for i, (r, s) in enumerate(zip(real['routes'], stand['routes'])):
        for f, cid in (('route0', 0), ('shortest1', 1), ('shortest0_nolc', 0)):
            res = same_route(r[f], s[f], cid)
            if res is None:
                fails.append('route %d %r %s: real %r, stand-in %r' % (i, queries['lanelet_pairs'][i], f, r[f], s[f]))
            else:
                counts[res] += 1
                if r[f] is None:
                    counts['no_route'] += 1
        if r['length2d'] is not None and res == 'same' and abs(r['length2d'] - s['length2d']) > 1e-2:
            fails.append('route %d length2d %r vs %r' % (i, r['length2d'], s['length2d']))
    stats['routes_checked'] = 3 * len(real['routes'])
    stats['routes_same'] = counts['same']
    stats['routes_equal_cost_tie'] = counts['tie']
    stats['routes_none_both'] = counts['no_route']

    # the whole chain
    p_same = p_tie = p_near_tie = 0
    for i, (r, s) in enumerate(zip(real['pipeline'], stand['pipeline'])):
        if r == s:
            p_same += 1
            continue
        if r['start'] == s['start'] and r['goal'] == s['goal']:
            if same_route(r['route'], s['route'], 0):
                p_tie += 1
                continue
        else:
            # a different start or goal lanelet is fine only if the point is equally far from both
            (sx, sy), (gx, gy) = queries['point_pairs'][i]
            ok = True
            for f, xy in (('start', (sx, sy)), ('goal', (gx, gy))):
                if r[f] != s[f] and abs(oracle.distance(r[f], xy) - oracle.distance(s[f], xy)) > TOL_TIE_DIST:
                    ok = False
            if ok:
                p_near_tie += 1
                if not strict:
                    continue
                fails.append('pipeline %d (strict, nearest tie picked differently): real %r, stand-in %r' % (i, r, s))
                continue
        fails.append('pipeline %d: real %r, stand-in %r' % (i, r, s))
    stats['pipeline_pairs'] = len(real['pipeline'])
    stats['pipeline_same'] = p_same
    stats['pipeline_route_tie'] = p_tie
    stats['pipeline_nearest_tie'] = p_near_tie
    return stats, fails
