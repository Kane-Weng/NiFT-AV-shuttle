"""Geometry functions of the Lanelet2 stand-in (lanelet2.geometry), ported from Lanelet2 1.2.3.

Third-party portions: translated from Lanelet2 1.2.3 (https://github.com/fzi-forschungszentrum-informatik/Lanelet2),
BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik. See ../LICENSES/BSD-3-Clause-Lanelet2.txt and
../NOTICE.
The helpers _beq, _bless_pt, _side_value, _side, _point_in_ring, _seg_cdist and _ring_distance translate
Boost.Geometry 1.74 (util/math.hpp, policies/compare.hpp, strategies/cartesian/side_by_triangle.hpp,
strategies/cartesian/point_in_poly_winding.hpp, algorithms/detail/within/point_in_geometry.hpp,
strategies/cartesian/distance_projected_point.hpp, algorithms/detail/distance/point_to_geometry.hpp), Boost Software
License 1.0. The copyright holders of each header are listed in ../LICENSES/BSL-1.0-Boost.txt.

Covered: findNearest, distance (point/point, lanelet/point, line string/point), boundingBox2d, approximatedLength2d,
length2d, length, follows, leftOf, rightOf, inside, toArcCoordinates, project, distanceToCenterline2d, and the helper
signedDistance used by the OSM loader to align lanelet bounds.
"""
import math

from .core import Area, BasicPoint2d, BasicPoint3d, BoundingBox2d, Lanelet, LineString3d


class ArcCoordinates:
    __slots__ = ('length', 'distance')

    def __init__(self, length=0.0, distance=0.0):
        self.length = length
        self.distance = distance

    def __repr__(self):
        return 'ArcCoordinates(%r, %r)' % (self.length, self.distance)


# ---------------------------------------------------------------- point/segment helpers

def _project_on_segment(l1, l2, p):
    """projectedPoint(l1, l2, p) from LineStringGeometry.cpp, for tuples of any equal dimension."""
    v = [b - a for a, b in zip(l1, l2)]
    w = [c - a for a, c in zip(l1, p)]
    c1 = sum(a * b for a, b in zip(w, v))
    if c1 <= 0:
        return tuple(l1)
    c2 = sum(a * a for a in v)
    if c2 <= c1:
        return tuple(l2)
    b = c1 / c2
    return tuple(vi * b + a for vi, a in zip(v, l1))


def _norm(v):
    return math.sqrt(sum(a * a for a in v))


def _closest_segment(pts, p):
    """projectedPointL2PImpl: (index of segment start, projected point). First segment with the strictly smallest
    distance wins; a zero distance stops the search."""
    n = len(pts)
    if n == 1:
        return 0, tuple(pts[0])
    best = None
    best_i = 0
    best_proj = None
    for i in range(n - 1):
        proj = _project_on_segment(pts[i], pts[i + 1], p)
        d = _norm([a - b for a, b in zip(proj, p)])
        if best is None or best > d:
            best = d
            best_i = i
            best_proj = proj
        if best == 0.0:
            break
    return best_i, best_proj


def _left_of(s1, s2, p):
    return (s2[0] - s1[0]) * (p[1] - s1[1]) - (s2[1] - s1[1]) * (p[0] - s1[0]) > 0


def _signed_distance_impl(pts, p):
    """signedDistanceImpl: (signed distance, segment index, projected point). Left of the line is positive."""
    i, proj = _closest_segment(pts, p)
    d = _norm([a - b for a, b in zip(proj, p)])
    if len(pts) == 1:
        seg1 = seg2 = tuple(pts[0])
    else:
        seg1 = tuple(pts[i])
        seg2 = tuple(pts[i + 1])
    is_left = _left_of(seg1, seg2, p)
    if seg2 == tuple(proj):
        # find the first point equal to seg2, then look at the next segment (stackoverflow 10583212)
        k = next((j for j, q in enumerate(pts) if tuple(q) == seg2), None)
        if k is not None and k + 1 < len(pts):
            nxt = tuple(pts[k + 1])
            if is_left != _left_of(seg2, nxt, p) and is_left == _left_of(seg1, seg2, nxt):
                is_left = not is_left
    return (d if is_left else -d), i, proj


def signedDistance(linestring, point):
    """Signed distance from a 2d or 3d point to a line string (left is positive)."""
    if len(point) >= 3 and hasattr(point, 'z'):
        pts = [(q.x, q.y, q.z) for q in linestring]
        return _signed_distance_impl(pts, (point.x, point.y, point.z))[0]
    pts = [(q.x, q.y) for q in linestring]
    return _signed_distance_impl(pts, (point.x, point.y))[0]


def align(left, right):
    """geometry::align(left, right): invert bounds so that right is right of left and left is left of right."""
    if (len(left) <= 1 and len(right) <= 1) or len(right) == 0 or len(left) == 0:
        return left, right

    def middle(ls):
        if len(ls) > 2:
            q = ls[len(ls) // 2]
            return (q.x, q.y, q.z)
        a, b = ls[0], ls[len(ls) - 1]
        return ((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5)

    def sd(ls, p):
        return _signed_distance_impl([(q.x, q.y, q.z) for q in ls], p)[0]

    right_of_left = sd(left, middle(right)) < 0
    if not right_of_left and len(left) > 1:
        left = left.invert()
    left_of_right = sd(right, middle(left)) > 0
    if not left_of_right and len(right) > 1:
        right = right.invert()
    return left, right


# ---------------------------------------------------------------- bounding boxes and polygons

def _lanelet_cache(ll):
    aux = ll._data.aux
    if 'poly' not in aux:
        d = ll._data
        # CompoundPolygon2d(leftBound, rightBound.invert()); the ring is the same for the inverted lanelet
        poly = [(p.x, p.y) for p in d.left] + [(p.x, p.y) for p in reversed(list(d.right))]
        xs = [q[0] for q in poly]
        ys = [q[1] for q in poly]
        aux['poly'] = poly
        aux['bbox'] = (min(xs), min(ys), max(xs), max(ys)) if poly else None
        aux['ring_x'] = (xs, max(abs(v) for v in xs) if xs else 0.0)  # for _point_in_ring's quick skip
    return aux


def _bbox_of(pts):
    if not pts:
        return None
    return (min(q[0] for q in pts), min(q[1] for q in pts), max(q[0] for q in pts), max(q[1] for q in pts))


def boundingBox2d(prim):
    """The 2d bounding box. A primitive without points gets an empty box (min +inf, max -inf), as an empty Eigen
    AlignedBox in Lanelet2; the layer's R-tree leaves such primitives out."""
    if isinstance(prim, Lanelet):
        b = _lanelet_cache(prim)['bbox']
    elif isinstance(prim, Area):
        b = _bbox_of([(p.x, p.y) for p in prim.outerBoundPolygon()])
    elif isinstance(prim, LineString3d):
        b = _bbox_of([(p.x, p.y) for p in prim])
    elif hasattr(prim, 'parameters'):
        pts = []
        for members in prim.parameters.values():
            for m in members:
                if isinstance(m, Lanelet):
                    bb = boundingBox2d(m)
                    if bb.min.x <= bb.max.x:  # skip empty boxes
                        pts += [(bb.min.x, bb.min.y), (bb.max.x, bb.max.y)]
                elif isinstance(m, LineString3d):
                    pts += [(p.x, p.y) for p in m]
                elif hasattr(m, 'x'):
                    pts.append((m.x, m.y))
        b = _bbox_of(pts)
    else:
        b = (prim.x, prim.y, prim.x, prim.y)
    if b is None:
        return BoundingBox2d()
    return BoundingBox2d(BasicPoint2d(b[0], b[1]), BasicPoint2d(b[2], b[3]))


_EPS = 2.0 ** -52


def _beq(a, b):
    """boost::geometry::math::equals for doubles."""
    if a == b:
        return True
    return abs(a - b) <= _EPS * max(abs(a), abs(b), 1.0)


def _bless_pt(p, q):
    """compare::cartesian<less>: lexicographic, coordinates equal by math::equals."""
    if not _beq(p[0], q[0]):
        return p[0] < q[0]
    if not _beq(p[1], q[1]):
        return p[1] < q[1]
    return False


def _side_value(p1, p2, p):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dpx = p[0] - p1[0]
    dpy = p[1] - p1[1]
    return dx * dpy - dy * dpx, max(abs(dx), abs(dy), abs(dpx), abs(dpy), 1.0)


def _side(p1, p2, p):
    """strategy::side::side_by_triangle<>::apply (boost 1.74): 1 left, -1 right, 0 collinear."""
    if (_beq(p1[0], p2[0]) and _beq(p1[1], p2[1])) or (_beq(p1[0], p[0]) and _beq(p1[1], p[1])) \
            or (_beq(p2[0], p[0]) and _beq(p2[1], p[1])):
        return 0
    if _bless_pt(p, p1):
        if _bless_pt(p, p2):
            s, f = _side_value(p, p1, p2)
        else:
            s, f = _side_value(p2, p, p1)
    elif _bless_pt(p1, p2):
        s, f = _side_value(p1, p2, p)
    else:
        s, f = _side_value(p2, p, p1)
    if s == 0 or abs(s) <= _EPS * f:
        return 0
    return 1 if s > 0 else -1


def _point_in_ring(poly, x, y, ring_x=None):
    """boost::geometry within(point, ring) with the cartesian_winding strategy: True only strictly inside.

    ring_x is (the ring's x coordinates, their largest absolute value), cached by _lanelet_cache, or None.
    """
    n = len(poly)
    if n < 3:
        return False
    if ring_x is None:
        xs = [q[0] for q in poly]
        xmag = max(abs(v) for v in xs)
    else:
        xs, xmag = ring_x
    # Quick skip of a segment whose ends are both clearly left or both clearly right of x. For such a segment
    # neither end equals x by _beq (|x - s| > m > _EPS * max(|s|, |x|, 1)) and x is not strictly between the ends,
    # so the full test below would give c == 0 and go on to the next segment. Skipping it changes nothing.
    m = 2.0 * _EPS * max(xmag, abs(x), 1.0)
    count = 0
    p = (x, y)
    for i in range(n):
        j = i + 1 if i + 1 < n else 0
        d1 = x - xs[i]
        d2 = x - xs[j]
        if (d1 > m and d2 > m) or (d1 < -m and d2 < -m):
            continue
        s1 = poly[i]
        s2 = poly[j]
        eq1 = _beq(s1[0], x)
        eq2 = _beq(s2[0], x)
        if eq1 and eq2:
            if (s1[1] <= y <= s2[1]) or (s2[1] <= y <= s1[1]):
                return False  # touches the boundary
            continue
        if eq1:
            c = 1 if s2[0] > x else -1
        elif eq2:
            c = -1 if s1[0] > x else 1
        elif s1[0] < x < s2[0]:
            c = 2
        elif s2[0] < x < s1[0]:
            c = -2
        else:
            c = 0
        if c == 0:
            continue
        if c in (1, -1):
            se = s1 if eq1 else s2
            side = 0 if _beq(y, se[1]) else (-c if y < se[1] else c)
        else:
            side = _side(s1, s2, p)
        if side == 0:
            return False  # touches
        if side * c > 0:
            count += c
    return count != 0


def _seg_cdist(x, y, x1, y1, x2, y2):
    """Comparable (squared) distance from a point to a segment, boost projected_point with pythagoras."""
    vx = x2 - x1
    vy = y2 - y1
    wx = x - x1
    wy = y - y1
    c1 = wx * vx + wy * vy
    if c1 <= 0:
        dx = x - x1
        dy = y - y1
        return dx * dx + dy * dy
    c2 = vx * vx + vy * vy
    if c2 <= c1:
        dx = x - x2
        dy = y - y2
        return dx * dx + dy * dy
    b = c1 / c2
    px = x1 + vx * b
    py = y1 + vy * b
    dx = x - px
    dy = y - py
    return dx * dx + dy * dy


def _point_segment_distance(x, y, x1, y1, x2, y2):
    return math.sqrt(_seg_cdist(x, y, x1, y1, x2, y2))


def _ring_distance(poly, x, y, ring_x=None):
    """boost distance(point, open ring): 0 when strictly within, else the distance to the nearest segment."""
    if _point_in_ring(poly, x, y, ring_x):
        return 0.0
    n = len(poly)
    if n == 1:
        return math.sqrt(_seg_cdist(x, y, *poly[0], *poly[0]))
    best = _seg_cdist(x, y, *poly[0], *poly[1])
    x1, y1 = poly[1]
    for i in range(2, n):
        # _seg_cdist(x, y, *poly[i - 1], *poly[i]) written inline, the same operations in the same order
        x2, y2 = poly[i]
        vx = x2 - x1
        vy = y2 - y1
        wx = x - x1
        wy = y - y1
        c1 = wx * vx + wy * vy
        if c1 <= 0:
            dx = x - x1
            dy = y - y1
        else:
            c2 = vx * vx + vy * vy
            if c2 <= c1:
                dx = x - x2
                dy = y - y2
            else:
                b = c1 / c2
                dx = x - (x1 + vx * b)
                dy = y - (y1 + vy * b)
        d = dx * dx + dy * dy
        if d == 0:
            return 0.0
        if d < best:
            best = d
        x1, y1 = x2, y2
    if n > 2:
        d = _seg_cdist(x, y, *poly[n - 1], *poly[0])
        if d < best:
            best = d
    return math.sqrt(best)


def _polyline_distance(pts, x, y):
    if len(pts) == 1:
        return math.sqrt((pts[0][0] - x) ** 2 + (pts[0][1] - y) ** 2)
    return min(_point_segment_distance(x, y, *pts[i], *pts[i + 1]) for i in range(len(pts) - 1))


def distance2d(a, b):
    """2d distance between a lanelet / area / line string / point and a point (either order)."""
    if not isinstance(b, (BasicPoint2d, BasicPoint3d)) and isinstance(a, (BasicPoint2d, BasicPoint3d)) \
            and not isinstance(b, (int, float)):
        a, b = b, a
    x, y = b.x, b.y
    if isinstance(a, Lanelet):
        aux = _lanelet_cache(a)
        return _ring_distance(aux['poly'], x, y, aux['ring_x'])
    if isinstance(a, Area):
        return _ring_distance([(p.x, p.y) for p in a.outerBoundPolygon()], x, y)
    if isinstance(a, LineString3d):
        return _polyline_distance([(p.x, p.y) for p in a], x, y)
    return math.sqrt((a.x - x) ** 2 + (a.y - y) ** 2)


def distance(a, b):
    return distance2d(a, b)


def inside(lanelet, point):
    aux = _lanelet_cache(lanelet)
    return _point_in_ring(aux['poly'], point.x, point.y, aux['ring_x'])


def findNearest(layer, point, n):
    """The n primitives of a layer closest to point, as [(distance, primitive)] sorted by distance.

    Same search as Lanelet2 (LaneletMap.cpp findNearestImpl): candidates come from the layer's R-tree in order of
    bounding-box distance, the search stops once the box distance exceeds the n-th best real distance, and a new
    candidate goes before existing ones at the same distance (NSmallestElements). The R-tree is built and walked like
    boost's (see _rtree.py), so among lanelets at exactly the same distance (a point inside overlapping lanelets) the
    pick is the same as Lanelet2's.
    """
    from ._rtree import box_point_distance
    x, y = point.x, point.y
    values = []
    for box, prim in layer._rtree().nearest(x, y):
        dbox = box_point_distance(box, x, y)
        if len(values) >= n and dbox > values[-1][0]:
            break
        d = distance2d(prim, point)
        pos = 0
        while pos < len(values) and values[pos][0] < d:
            pos += 1
        if pos < len(values) or len(values) < n:
            if len(values) >= n:
                values.pop()
            values.insert(pos, (d, prim))
    return values


# ---------------------------------------------------------------- lengths and relations

def _pt_dist2d(p, q):
    return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2)


def approximatedLength2d(lanelet):
    """approximatedLength2d: length of the left bound sampled at about 10 points (used by the routing costs)."""
    aux = lanelet._data.aux
    key = ('alen', lanelet.inverted())
    if key in aux:
        return aux[key]
    line = list(lanelet.leftBound)
    n = len(line)
    step = max(1, n // 10)
    length = 0.0
    i1 = 0
    i2 = step
    while i2 < n:
        length += _pt_dist2d(line[i1], line[i2])
        if i2 + step >= n:
            length += _pt_dist2d(line[i2], line[n - 1])
        i1 += step
        i2 += step
    aux[key] = length
    return length


def _polyline_length2d(pts):
    return sum(_pt_dist2d(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def length2d(prim):
    if isinstance(prim, Lanelet):
        return _polyline_length2d(list(prim.centerline))
    return _polyline_length2d(list(prim))


def length3d(prim):
    pts = list(prim.centerline) if isinstance(prim, Lanelet) else list(prim)
    return sum(math.sqrt((pts[i].x - pts[i + 1].x) ** 2 + (pts[i].y - pts[i + 1].y) ** 2
                         + (pts[i].z - pts[i + 1].z) ** 2) for i in range(len(pts) - 1))


def length(prim):
    return length3d(prim)


def follows(prev, nxt):
    pl, pr, nl, nr = prev.leftBound, prev.rightBound, nxt.leftBound, nxt.rightBound
    return (len(pl) > 0 and len(pr) > 0 and len(nl) > 0 and len(nr) > 0
            and pl[len(pl) - 1] is nl[0] and pr[len(pr) - 1] is nr[0])


def leftOf(left, right):
    return left.rightBound == right.leftBound


def rightOf(right, left):
    return leftOf(left, right)


def toArcCoordinates(linestring, point):
    """Arc length along the line string to the projection of point, and signed lateral distance (left positive)."""
    pts = [(q.x, q.y) for q in linestring]
    d, i, proj = _signed_distance_impl(pts, (point.x, point.y))
    first = pts[i]
    length = 0.0
    for k in range(len(pts) - 1):
        if pts[k] == first:
            break
        length += math.sqrt((pts[k + 1][0] - pts[k][0]) ** 2 + (pts[k + 1][1] - pts[k][1]) ** 2)
    length += math.sqrt((proj[0] - first[0]) ** 2 + (proj[1] - first[1]) ** 2)
    return ArcCoordinates(length, d)


def project(linestring, point):
    if hasattr(point, 'z') and isinstance(point, BasicPoint3d):
        _, proj = _closest_segment([(q.x, q.y, q.z) for q in linestring], (point.x, point.y, point.z))
        return BasicPoint3d(*proj)
    _, proj = _closest_segment([(q.x, q.y) for q in linestring], (point.x, point.y))
    return BasicPoint2d(*proj)


def distanceToCenterline2d(lanelet, point):
    return _polyline_distance([(q.x, q.y) for q in lanelet.centerline], point.x, point.y)
