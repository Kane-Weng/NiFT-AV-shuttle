"""Lanelet centerline, a line-by-line port of calculateCenterline() in Lanelet2 1.2.3 (lanelet2_core/src/Lanelet.cpp).

Third-party portions: translated from Lanelet2 1.2.3 (https://github.com/fzi-forschungszentrum-informatik/Lanelet2),
BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik. See ../LICENSES/BSD-3-Clause-Lanelet2.txt and
../NOTICE.

The algorithm walks both bounds at once. Each step it looks, on each bound, for the next point that is closest to the
current point on the other bound and whose connection does not cross the bounds, and adds the midpoint of the pair.
Lanelet2 uses boost R-trees for the "nearest point" and "segments that intersect" queries. Here a minimum search and a
uniform grid with a box test do the same job. The results are identical, except when two candidate points have exactly
the same distance, where the R-tree order is not defined.
"""
import math


def _orient(ax, ay, bx, by, cx, cy):
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def seg_intersects(ax, ay, bx, by, cx, cy, dx, dy):
    """True if segments AB and CD share at least one point (touching counts), like boost::geometry::intersects."""
    if (ax if ax > bx else bx) < (cx if cx < dx else dx) or (cx if cx > dx else dx) < (ax if ax < bx else bx):
        return False
    if (ay if ay > by else by) < (cy if cy < dy else dy) or (cy if cy > dy else dy) < (ay if ay < by else by):
        return False
    d1 = _orient(cx, cy, dx, dy, ax, ay)
    d2 = _orient(cx, cy, dx, dy, bx, by)
    if (d1 > 0 and d2 > 0) or (d1 < 0 and d2 < 0):
        return False
    d3 = _orient(ax, ay, bx, by, cx, cy)
    d4 = _orient(ax, ay, bx, by, dx, dy)
    if (d3 > 0 and d4 > 0) or (d3 < 0 and d4 < 0):
        return False
    # Either a proper crossing, a touch, or collinear with overlapping boxes (checked above).
    return True


def _any_hit(segs, sx1, sy1, sx2, sy2, skip_first=None, skip_end=None):
    """True if a segment in segs intersects S. skip_first: ignore segments starting at that point (x, y).
    skip_end: ignore segments that start or end at that point."""
    bx0 = sx1 if sx1 < sx2 else sx2
    bx1 = sx2 if sx1 < sx2 else sx1
    by0 = sy1 if sy1 < sy2 else sy2
    by1 = sy2 if sy1 < sy2 else sy1
    for s in segs:
        x1, y1, x2, y2, mnx, mny, mxx, mxy = s
        if mxx < bx0 or bx1 < mnx or mxy < by0 or by1 < mny:
            continue
        # orientation tests (see seg_intersects)
        d1 = (x2 - x1) * (sy1 - y1) - (y2 - y1) * (sx1 - x1)
        d2 = (x2 - x1) * (sy2 - y1) - (y2 - y1) * (sx2 - x1)
        if (d1 > 0 and d2 > 0) or (d1 < 0 and d2 < 0):
            continue
        d3 = (sx2 - sx1) * (y1 - sy1) - (sy2 - sy1) * (x1 - sx1)
        d4 = (sx2 - sx1) * (y2 - sy1) - (sy2 - sy1) * (x2 - sx1)
        if (d3 > 0 and d4 > 0) or (d3 < 0 and d4 < 0):
            continue
        if skip_first is not None and x1 == skip_first[0] and y1 == skip_first[1]:
            continue
        if skip_end is not None and ((x1 == skip_end[0] and y1 == skip_end[1])
                                     or (x2 == skip_end[0] and y2 == skip_end[1])):
            continue
        return True
    return False


def _segments(pts):
    out = []
    for i in range(len(pts) - 1):
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        out.append((x1, y1, x2, y2, min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))
    return out


_CELL = 4.0  # metres; bound points are about 0.5 m apart, connections a few metres long


class _SegGrid:
    """Segments of one bound in a uniform grid. A query returns the segments in the cells the query box touches
    (a segment may come twice; that is fine for "does any segment intersect")."""

    def __init__(self, pts):
        self.segs = _segments(pts)
        grid = {}
        for s in self.segs:
            for i in range(int(math.floor(s[4] / _CELL)), int(math.floor(s[6] / _CELL)) + 1):
                for j in range(int(math.floor(s[5] / _CELL)), int(math.floor(s[7] / _CELL)) + 1):
                    grid.setdefault((i, j), []).append(s)
        self.grid = grid

    def near(self, x1, y1, x2, y2):
        i0 = int(math.floor((x1 if x1 < x2 else x2) / _CELL))
        i1 = int(math.floor((x2 if x1 < x2 else x1) / _CELL))
        j0 = int(math.floor((y1 if y1 < y2 else y2) / _CELL))
        j1 = int(math.floor((y2 if y1 < y2 else y1) / _CELL))
        if i0 == i1 and j0 == j1:
            return self.grid.get((i0, j0), ())
        if (i1 - i0 + 1) * (j1 - j0 + 1) > 16:
            return self.segs
        out = []
        get = self.grid.get
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                cell = get((i, j))
                if cell:
                    out.extend(cell)
        return out


class _BoundChecker:
    """BoundChecker from Lanelet.cpp. Lanelet2 queries R-trees of segments; a grid and a box test give the same
    answers (the questions are all "does any segment intersect")."""

    def __init__(self, left, right):
        self.left = _SegGrid(left)
        self.right = _SegGrid(right)
        self.entry = (right[0][0], right[0][1], left[0][0], left[0][1])
        self.exit = (left[-1][0], left[-1][1], right[-1][0], right[-1][1])

    @staticmethod
    def _crosses_line(line, sx1, sy1, sx2, sy2):
        if seg_intersects(sx1, sy1, sx2, sy2, *line):
            # pointIsLeftOf(line.first, line.second, seg.second)
            return _orient(line[0], line[1], line[2], line[3], sx2, sy2) > 0
        return False

    def intersects(self, sx1, sy1, sx2, sy2):
        first = (sx1, sy1)
        return (_any_hit(self.left.near(sx1, sy1, sx2, sy2), sx1, sy1, sx2, sy2, skip_first=first)
                or _any_hit(self.right.near(sx1, sy1, sx2, sy2), sx1, sy1, sx2, sy2, skip_first=first)
                or self._crosses_line(self.entry, sx1, sy1, sx2, sy2)
                or self._crosses_line(self.exit, sx1, sy1, sx2, sy2))

    def second_crosses_bounds(self, sx1, sy1, sx2, sy2, left):
        grid = self.left if left else self.right
        return _any_hit(grid.near(sx1, sy1, sx2, sy2), sx1, sy1, sx2, sy2, skip_end=(sx2, sy2))


def _closest_nonintersecting(curr_idx, other, pts, bounds, last, is_left):
    """findClosestNonintersectingPoint(): index and half distance of the best candidate, or (None, None).

    Lanelet2 walks the points in order of distance to `other` and keeps the first valid one with the smallest
    distance; its early stop only skips points that could not win. So the answer is the nearest valid point at or
    after curr_idx. We test the nearest first and only sort when it is not valid.
    """
    n = len(pts)
    if curr_idx >= n:
        return None, None
    ox, oy = other
    lx, ly = last
    d2 = [(x - ox) * (x - ox) + (y - oy) * (y - oy) for x, y in pts[curr_idx:]]
    order = [d2.index(min(d2))]
    sorted_all = False
    k = 0
    while True:
        if k >= len(order):
            if sorted_all:
                return None, None
            # sorted() is stable, so order[0] is the minimum tried above
            order = sorted(range(len(d2)), key=d2.__getitem__)
            sorted_all = True
            k = 1
            continue
        j = order[k]
        k += 1
        i = j + curr_idx
        cx, cy = pts[i]
        mx = (ox + cx) * 0.5
        my = (oy + cy) * 0.5
        if (not bounds.intersects(lx, ly, mx, my)
                and not bounds.second_crosses_bounds(ox, oy, cx, cy, is_left)
                and not bounds.second_crosses_bounds(cx, cy, ox, oy, not is_left)):
            return i, math.sqrt((ox - cx) ** 2 + (oy - cy) ** 2) / 2


def _mid(p, q):
    return ((p.x + q.x) * 0.5, (p.y + q.y) * 0.5, (p.z + q.z) * 0.5)


def compute_centerline(left, right):
    """left, right: lists of points with .x .y .z (lanelet bounds, already oriented). Returns [(x, y, z), ...]."""
    if not left or not right:
        return []
    left2 = [(p.x, p.y) for p in left]
    right2 = [(p.x, p.y) for p in right]
    bounds = _BoundChecker(left2, right2)
    center = [_mid(left[0], right[0])]
    n_left = len(left)
    n_right = len(right)
    li = 0
    ri = 0
    if left[0] is right[0]:
        li = 1
    while li != n_left or ri != n_right:
        last = center[-1][:2]
        if ri < n_right:
            lc, ld = _closest_nonintersecting(li + 1, right2[ri], left2, bounds, last, True)
        else:
            lc, ld = None, None
        if li < n_left:
            rc, rd = _closest_nonintersecting(ri + 1, left2[li], right2, bounds, last, False)
        else:
            rc, rd = None, None
        if ld is not None and (rd is None or ld <= rd):
            center.append(_mid(left[lc], right[ri]))
            li = lc
        elif rd is not None and (ld is None or ld > rd):
            center.append(_mid(left[li], right[rc]))
            ri = rc
        else:
            break
    if not (li == n_left - 1 and ri == n_right - 1):
        center.append(_mid(left[-1], right[-1]))
    return center
