# Copyright 2026 The NiFT authors
# SPDX-License-Identifier: Apache-2.0
"""The fast paths of findNearest give exactly what the plain code gives.

- _rtree.sort_by_first against _stdorder.sort_range and the clean-room sort vectors.
- RTree.nearest, which replays or resumes the previous walk when it can prove the walk is the same, against a fresh
  walk from the same point, on point sequences like a driving shuttle's, with interleaved queries too.
- geometry._point_in_ring with its quick segment skip against the same test without the skip, and _ring_distance
  against the plain segment loop.

test_equivalence.py checks the whole stand-in against the real Lanelet2.
"""
import gzip
import json
import math
import os
import random
from itertools import islice

import pytest

import lanelet2
from lanelet2 import _rtree, _stdorder, geometry
from lanelet2.core import BasicPoint2d
from lanelet2.projection import UtmProjector

HERE = os.path.dirname(os.path.abspath(__file__))
ODR2024 = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'maps', 'lanelet2_odr2024.osm'))


def _less(a, b):
    return a[0] < b[0]


def test_sort_by_first_on_the_cleanroom_vectors():
    with gzip.open(os.path.join(HERE, 'fixtures', 'cleanroom_vectors.json.gz'), 'rt', encoding='utf-8') as f:
        cases = json.load(f)['sort']
    assert sum(c['depth_fallback'] for c in cases) > 0
    for case in cases:
        items = [(v, i) for i, v in enumerate(case['values'])]
        lo, hi = case['lo'], case['hi']
        part = items[lo:hi]  # the sort only depends on the positions inside the range
        _rtree.sort_by_first(part)
        items[lo:hi] = part
        assert [i for _, i in items] == case['result']


@pytest.mark.parametrize('seed', range(4))
def test_sort_by_first_equals_sort_range(seed):
    rng = random.Random(seed)
    for _ in range(3000):
        n = rng.choice([0, 1, 2, 15, 16, 17, 18, 33, 64, 100, 157, 300, 507])
        kind = rng.randrange(6)
        if kind == 0:
            vals = [rng.random() for _ in range(n)]                             # distinct
        elif kind == 1:
            vals = [rng.randint(0, 3) * 1.0 for _ in range(n)]                  # few values
        elif kind == 2:
            vals = [rng.choice([0.0, -0.0, 0.25, rng.random()]) for _ in range(n)]
        elif kind == 3:
            pool = [rng.random() for _ in range(max(1, n // 3))]
            vals = [rng.choice(pool) for _ in range(n)]                         # many small tie groups
        elif kind == 4:
            vals = sorted(rng.choice([rng.random(), 0.5]) for _ in range(n))   # sorted list plus a tail, as the walk
            vals += [rng.choice([0.5, rng.random()]) for _ in range(min(16, n))]
        else:
            vals = [float(min(i, n - 1 - i)) for i in range(n)]                # organ pipe
        a = [(v, i) for i, v in enumerate(vals)]
        b = list(a)
        _stdorder.sort_range(a, _less)
        _rtree.sort_by_first(b)
        assert a == b


@pytest.fixture(scope='module')
def odr():
    return lanelet2.io.load(ODR2024, UtmProjector(lanelet2.io.Origin(42.3005, -83.6987)))


def _fresh(tree, x, y, k):
    """The walk from the start, without the record of earlier queries."""
    return list(islice(tree._walk(x, y, _rtree._WalkRecord(), None), k))


def test_nearest_replay_equals_a_fresh_walk(odr):
    layer = odr.laneletLayer
    tree = layer._rtree()
    lanelets = sorted(layer, key=lambda ll: ll.id)
    rng = random.Random(11)
    seen = {'replay': 0, 'resume': 0}
    matches = _rtree._WalkRecord.matches
    for _ in range(12):
        # drive along a lanelet's centerline in small steps, as the GNSS fixes do
        cl = list(rng.choice(lanelets).centerline)
        for i in range(len(cl) - 1):
            for t in (0.0, 0.3, 0.6):
                x = cl[i].x + t * (cl[i + 1].x - cl[i].x) + rng.gauss(0, 0.02)
                y = cl[i].y + t * (cl[i + 1].y - cl[i].y) + rng.gauss(0, 0.02)
                k = rng.choice([1, 2, 3, 5, 8, 30, 600])
                rec = tree._last
                hit = rec is not None and matches(rec, x, y, len(rec.nodes), len(rec.vals)) is not None
                n_rec = len(rec.yields) if rec is not None else 0
                got = list(islice(tree.nearest(x, y), k))
                assert [id(v) for v in got] == [id(v) for v in _fresh(tree, x, y, k)]
                if hit:
                    seen['replay'] += 1
                    seen['resume'] += k > n_rec and not rec.done
    assert seen['replay'] > 300 and seen['resume'] > 100, seen


def test_nearest_interleaved_queries(odr):
    tree = odr.laneletLayer._rtree()
    rng = random.Random(3)
    x, y = 0.0, 0.0
    for _ in range(150):
        x += rng.gauss(0, 0.05)
        y += rng.gauss(0, 0.05)
        pts = [(x, y), (x + rng.gauss(0, 0.03), y + rng.gauss(0, 0.03))]
        gens = [tree.nearest(*p) for p in pts]
        got = [[], []]
        for _ in range(rng.choice([1, 3, 10, 40])):
            for j in rng.sample([0, 1], 2):
                v = next(gens[j], None)
                if v is not None:
                    got[j].append(v)
        for j, p in enumerate(pts):
            assert got[j] == _fresh(tree, p[0], p[1], len(got[j]))


def test_find_nearest_on_a_route_like_track(odr):
    layer = odr.laneletLayer
    rng = random.Random(4)
    ll = layer[sorted(layer._items)[100]]
    cl = list(ll.centerline)
    for i in range(len(cl) - 1):
        for t in (0.0, 0.5):
            p = BasicPoint2d(cl[i].x + t * (cl[i + 1].x - cl[i].x) + rng.gauss(0, 0.5),
                             cl[i].y + t * (cl[i + 1].y - cl[i].y) + rng.gauss(0, 0.5))
            for n in (1, 3):
                got = geometry.findNearest(layer, p, n)
                layer._rtree()._last = None  # the same query with no record to replay
                exp = geometry.findNearest(layer, p, n)
                assert [(d, l.id) for d, l in got] == [(d, l.id) for d, l in exp]


def _ring_distance_plain(poly, x, y):
    if geometry._point_in_ring(poly, x, y, ([q[0] for q in poly], 1e300)):
        return 0.0
    n = len(poly)
    best = geometry._seg_cdist(x, y, *poly[0], *poly[1])
    for i in range(1, n - 1):
        d = geometry._seg_cdist(x, y, *poly[i], *poly[i + 1])
        if d == 0:
            return 0.0
        if d < best:
            best = d
    if n > 2:
        d = geometry._seg_cdist(x, y, *poly[n - 1], *poly[0])
        if d < best:
            best = d
    return math.sqrt(best)


def test_point_in_ring_skip_and_ring_distance(odr):
    rng = random.Random(9)
    for ll in sorted(odr.laneletLayer, key=lambda ll: ll.id)[::10]:
        aux = geometry._lanelet_cache(ll)
        poly, ring_x = aux['poly'], aux['ring_x']
        xs = ring_x[0]
        # With a huge magnitude bound no segment is skipped, which is the test without the skip.
        no_skip = (xs, 1e300)
        pts = [q for q in poly[::2]]                                                # on vertices
        pts += [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(poly, poly[1:])][::2]  # on edges
        pts += [(q[0] + rng.gauss(0, 1), q[1] + rng.gauss(0, 1)) for q in poly[::2]]
        pts += [(q[0], q[1] + rng.gauss(0, 1)) for q in poly[::3]]                  # same x as a vertex
        for x, y in pts:
            assert geometry._point_in_ring(poly, x, y, ring_x) == geometry._point_in_ring(poly, x, y, no_skip)
            assert geometry._point_in_ring(poly, x, y) == geometry._point_in_ring(poly, x, y, no_skip)
            assert geometry._ring_distance(poly, x, y, ring_x) == _ring_distance_plain(poly, x, y)
