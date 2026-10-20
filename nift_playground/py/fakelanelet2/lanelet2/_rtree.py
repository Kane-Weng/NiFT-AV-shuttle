"""The R-tree of a Lanelet2 map layer and its nearest-neighbour iteration, as boost 1.74 builds and walks it.

Lanelet2 keeps each layer in a std::unordered_map and builds a bgi::rtree<std::pair<BoundingBox2d, T>,
bgi::quadratic<16>> from it with the packing constructor. findNearest() walks that tree with a nearest query and keeps
the last of several primitives at exactly the same distance. On the NiFT map many points lie inside two or more
overlapping lanelets (junctions), so the pick depends on the tree. This module builds the same tree from the same
input order (see _stdorder.hash_map_order) and walks it the same way, following
boost/geometry/index/detail/rtree/pack_create.hpp and visitors/distance_query.hpp (distance_query_incremental).

Third-party portions: translated from Boost.Geometry 1.74 (the headers named above, plus index/rtree.hpp and
index/parameters.hpp), Boost Software License 1.0. Copyright Adam Wulkiewicz (2011-2019), Federico J. Fernandez
(2008), Caian Benedicto (2020), Oracle and/or its affiliates (2019). The exact notice of each header is in
../LICENSES/BSL-1.0-Boost.txt, see also ../NOTICE.

Our own code (Apache-2.0), written for speed only: sort_by_first and _coarse0 (a faster path to the result of
_stdorder.sort_range), and _WalkRecord with the replay in RTree.nearest. Neither changes any result. The docstrings
give the reasons, and tests/lanelet2/test_fastpaths.py checks them.
"""
import math
from bisect import bisect_left
from itertools import compress, islice
from operator import eq, itemgetter, le, lt, sub

from ._stdorder import heap_sort, select_range

MAX_ELEMENTS = 16
MIN_ELEMENTS = (16 * 3) // 10  # quadratic<16>
_DBL_MAX = 1.7976931348623157e308


class _Node:
    __slots__ = ('leaf', 'children', 'boxes', 'subnodes')

    def __init__(self, leaf, children):
        self.leaf = leaf
        self.children = children  # leaf: [(box, value)]; internal: [(box, _Node)]
        self.boxes = [c[0] for c in children]
        self.subnodes = None if leaf else [c[1] for c in children]


def _box_distances(boxes, x, y):
    """The comparable distance from (x, y) to each box: the squared distance, 0 inside the box.

    boost adds the squared x gap and then the squared y gap to 0. Adding 0.0 to a square changes nothing, so writing
    it as one sum of the two parts gives the same doubles."""
    return [((b[0] - x) * (b[0] - x) if x < b[0] else (x - b[2]) * (x - b[2]) if x > b[2] else 0.0)
            + ((b[1] - y) * (b[1] - y) if y < b[1] else (y - b[3]) * (y - b[3]) if y > b[3] else 0.0)
            for b in boxes]


_key0 = itemgetter(0)


def _first_less(a, b):
    return a[0] < b[0]


_INF = float('inf')


def _coarse0(items, n, dups):
    """_stdorder's coarse sort phase on items[0:n], for elements compared on item [0], with the partition step written
    inline. Steps that cannot change the final result of sort_by_first are left out (see below). dups is the sorted
    list of the keys that occur more than once.

    _stdorder handles the right part of each cut first, by recursion, then goes on with the left part. Here pending
    ranges wait on a list instead. The order does not matter for the result, because a range's steps only read and
    move elements inside that range, and the ranges are disjoint. Each range keeps the depth budget it was cut with.

    A range with no two equal keys is not processed further. After this phase, sort_by_first sorts stably, so the
    result depends only on the order in which equal elements come out of this phase. For every key, at most one
    element with that key lies in such a range, and its place relative to the equal elements outside the range is set
    by where the range is. So the further steps inside the range, partitions or the heap fallback, cannot change the
    result.

    To find such ranges cheaply, each range carries bounds lb and ub on its keys. A partition step with pivot key p
    leaves keys <= p left of the cut (the pivot itself, elements the left marker passed, and elements swapped in at a
    right marker stop) and keys >= p from the cut on (by the same argument for the right marker). So the left part
    gets [lb, p] and the right part [p, ub]. If no key of dups lies in [lb, ub], the range has no two equal keys.
    """
    nd = len(dups)
    todo = [(0, n, 2 * (n.bit_length() - 1), -_INF, _INF)]
    while todo:
        lo, hi, budget, lb, ub = todo.pop()
        while hi - lo > 16:
            i = bisect_left(dups, lb)
            if i == nd or dups[i] > ub:
                break  # no two equal keys in this range
            if budget == 0:
                heap_sort(items, lo, hi, _first_less)
                break
            budget -= 1
            # the partition step: median of three to lo, then the two-marker scan
            a, m, z = lo + 1, lo + (hi - lo) // 2, hi - 1
            ka, km, kz = items[a][0], items[m][0], items[z][0]
            if ka < km:
                if km < kz:
                    pick = m
                elif ka < kz:
                    pick = z
                else:
                    pick = a
            else:
                if ka < kz:
                    pick = a
                elif km < kz:
                    pick = z
                else:
                    pick = m
            items[lo], items[pick] = items[pick], items[lo]
            pivot = items[lo][0]
            left, right = lo + 1, hi
            while True:
                while items[left][0] < pivot:
                    left += 1
                right -= 1
                while pivot < items[right][0]:
                    right -= 1
                if left >= right:
                    break
                items[left], items[right] = items[right], items[left]
                left += 1
            if hi - left > 16:  # a range of 16 or fewer is left as it is
                todo.append((left, hi, budget, pivot, ub))
            hi = left
            ub = pivot


def sort_by_first(items):
    """The same final order as sort_range(items, lambda a, b: a[0] < b[0]), only faster.

    sort_range runs a coarse phase that only partitions ranges of more than 16 elements, then a straight insertion
    sort over the whole list. That insertion sort never moves an element past an equal one, so it gives the same
    order as Python's stable list.sort. So this runs the same coarse phase (_coarse0) and then list.sort. Lists of 16
    or fewer go straight to list.sort. tests/lanelet2/test_fastpaths.py checks this against sort_range and the
    clean-room vectors.
    """
    n = len(items)
    if n > 16:
        keys = sorted(map(_key0, items))
        dups = list(compress(keys, map(eq, keys, islice(keys, 1, None))))  # each key equal to the next one
        if dups:
            _coarse0(items, n, dups)
    items.sort(key=_key0)


def _expand(box, other):
    if box is None:
        return other
    return (min(box[0], other[0]), min(box[1], other[1]), max(box[2], other[2]), max(box[3], other[3]))


def _subtree_counts(count):
    maxc, smax = 1, MAX_ELEMENTS
    while smax < count:
        maxc = smax
        smax *= MAX_ELEMENTS
    return maxc, MIN_ELEMENTS * (maxc // MAX_ELEMENTS)


def _median_count(count, maxc, minc):
    n = count // maxc
    r = count % maxc
    median = (n // 2) * maxc
    if r != 0:
        if minc <= r:
            median = ((n + 1) // 2) * maxc
        else:
            cmm = count - minc
            n = cmm // maxc
            r = cmm % maxc
            if r == 0:
                median = ((n + 1) // 2) * maxc
            elif n == 0:
                median = r
            else:
                median = ((n + 2) // 2) * maxc
    return median


def _comparable_distances(nodes, x, y):
    """The comparable distances that a walk from (x, y) computes for the children of these nodes, in the same order
    as RTree._walk, followed by _DBL_MAX."""
    out = []
    for node in nodes:
        out += _box_distances(node.boxes, x, y)
    out.append(_DBL_MAX)
    return out


class _WalkRecord:
    """What one nearest walk did, so that a later query can reuse it when that is provably the same walk.

    Every decision of the walk (which node to visit next, when to yield, and every comparison inside the sorts)
    compares two of the comparable distances it has computed so far, or one of them with _DBL_MAX. The rest is list
    lengths and indices. Start a walk from another point. As long as the distances it computes for the same boxes are
    in the same order with the same ties, it makes the same decision at every step, so it visits the same nodes in the
    same order, sorts the same way and yields the same items. matches() checks this condition for all distances
    computed up to the last yield. Then the walk from the new point is, at that yield, in the recorded state with the
    new distances put in. The state keeps the index of each distance in vals for that.
    """
    __slots__ = ('nodes', 'vals', 'yields', 'done', 'pattern', 'stack', 'neighbors', 'current', 'next_vi', 'epoch',
                 'clean')

    def __init__(self):
        self.nodes = []    # the visited nodes, in visit order
        self.vals = []     # the comparable distance of each child of those nodes, in the order they were computed
        self.yields = []   # the items yielded so far
        self.done = False  # the walk ran to its end
        self.pattern = None
        # the walk's state at the last yield (see RTree._walk)
        self.stack = None
        self.neighbors = None
        self.current = -1
        self.next_vi = -1
        self.epoch = 0     # goes up each time the walk resumes after a yield
        self.clean = False  # True only while the walk waits at a yield, so the state above is whole

    def matches(self, x, y, n_nodes, n_vals):
        """The distances from (x, y) to the children of the first n_nodes visited nodes (n_vals values, then _DBL_MAX)
        when they are in exactly the same order, ties included, as the recorded ones. Otherwise None."""
        pat = self.pattern
        if pat is None or pat[0] != n_vals:
            vals = self.vals[:n_vals]
            vals.append(_DBL_MAX)
            order = sorted(range(len(vals)), key=vals.__getitem__)
            w = [vals[i] for i in order]
            # A quick first test on the closest pairs of different distances, the ones most likely to swap. It can
            # only say no early. The full test below decides.
            rises = list(map(lt, w, w[1:]))
            boxes = [b for node in self.nodes[:n_nodes] for b in node.boxes]
            roots = list(map(math.sqrt, w))
            gaps = list(map(sub, roots[1:-1], roots))  # not the last pair, which is _DBL_MAX
            probe = []
            for k in sorted(range(len(gaps)), key=gaps.__getitem__):
                if rises[k]:
                    probe.append((boxes[order[k]], boxes[order[k + 1]]))
                    if len(probe) == 8:
                        break
            pat = self.pattern = (n_vals, order, rises, probe)
        for pair in pat[3]:
            da, db = _box_distances(pair, x, y)
            if not da < db:
                return None
        new = _comparable_distances(self.nodes[:n_nodes], x, y)
        if len(new) != n_vals + 1:
            return None
        w = [new[i] for i in pat[1]]
        # Along the recorded sorted order, the new values must rise where the old ones rose and stay equal where the
        # old ones were equal. Then every pair of values compares the same way as before.
        if list(map(lt, w, w[1:])) == pat[2] and all(map(le, w, w[1:])):
            return new
        return None


class RTree:
    """values: list of (box, value) in the order of the C++ container; box = (minx, miny, maxx, maxy)."""

    def __init__(self, values):
        self.values = list(values)
        self.root = None
        self._last = None  # _WalkRecord of the last nearest walk
        if not self.values:
            return
        entries = []
        hint = None
        for i, (box, _) in enumerate(self.values):
            hint = _expand(hint, box)
            entries.append(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, i))
        self._entries = entries
        maxc, minc = _subtree_counts(len(entries))
        _, self.root = self._per_level(0, len(entries), hint, len(entries), (maxc, minc))
        del self._entries

    def _per_level(self, first, last, hint, count, sc):
        entries = self._entries
        if sc[0] <= 1:
            children = []
            box = None
            for k in range(first, last):
                b, v = self.values[entries[k][2]]
                box = _expand(box, b)
                children.append((b, v))
            return box, _Node(True, children)
        nxt = (sc[0] // MAX_ELEMENTS, sc[1] // MAX_ELEMENTS)
        elements = []
        ebox = [None]
        self._packets(first, last, hint, count, sc, nxt, elements, ebox)
        return ebox[0], _Node(False, elements)

    def _packets(self, first, last, hint, count, sc, nxt, elements, ebox):
        if count <= sc[0]:
            box, node = self._per_level(first, last, hint, count, nxt)
            elements.append((box, node))
            ebox[0] = _expand(ebox[0], box)
            return
        median_count = _median_count(count, sc[0], sc[1])
        median = first + median_count
        # biggest_edge: dimension 0 unless dimension 1 is strictly longer
        length = hint[2] - hint[0]
        dim = 0
        if length < hint[3] - hint[1]:
            dim = 1
        if dim == 0:
            select_range(self._entries, first, median, last, lambda a, b: a[0] < b[0])
            mid = hint[0] + (hint[2] - hint[0]) / 2
            left = (hint[0], hint[1], mid, hint[3])
            right = (mid, hint[1], hint[2], hint[3])
        else:
            select_range(self._entries, first, median, last, lambda a, b: a[1] < b[1])
            mid = hint[1] + (hint[3] - hint[1]) / 2
            left = (hint[0], hint[1], hint[2], mid)
            right = (hint[0], mid, hint[2], hint[3])
        self._packets(first, median, left, median_count, sc, nxt, elements, ebox)
        self._packets(median, last, right, count - median_count, sc, nxt, elements, ebox)

    def nearest(self, x, y):
        """Yield (box, value) in the order of rtree.qbegin(bgi::nearest(point, size())) in boost 1.74.

        A query close to the previous one usually makes the same walk. When _WalkRecord.matches() proves that, the
        recorded items are yielded again without walking, and if the caller wants more, the walk goes on from the
        recorded state with the new distances put in.
        """
        if self.root is None:
            return
        skip = 0
        rec = self._last
        if rec is not None:
            # A snapshot, in case the walk that made the record is resumed while this one runs.
            n_nodes, n_vals, n_yields, done = len(rec.nodes), len(rec.vals), len(rec.yields), rec.done
            epoch, clean = rec.epoch, rec.clean
            new = rec.matches(x, y, n_nodes, n_vals)
            if new is not None:
                yields = rec.yields
                for k in range(n_yields):
                    yield yields[k]
                if done:
                    return
                if clean and rec.epoch == epoch:  # the recorded state is whole and still the one at the snapshot
                    state = ([[[(new[vi], child, vi) for _, child, vi in branches], cb] for branches, cb in rec.stack],
                             [(new[vi], item, vi) for _, item, vi in rec.neighbors],
                             rec.current,
                             new[rec.next_vi] if rec.next_vi >= 0 else _DBL_MAX,
                             rec.next_vi)
                    nrec = self._last = _WalkRecord()
                    nrec.nodes = rec.nodes[:n_nodes]
                    nrec.vals = new[:n_vals]
                    nrec.yields = yields[:n_yields]
                    yield from self._walk(x, y, nrec, state)
                    return
                skip = n_yields
        rec = self._last = _WalkRecord()
        k = 0
        for item in self._walk(x, y, rec, None):
            if k >= skip:
                yield item
            k += 1

    def _walk(self, x, y, rec, state):
        """The walk of boost's distance_query_incremental, from the start (state None) or from a state that
        nearest() rebuilt. Records what it does in rec (a _WalkRecord). Each distance in the state travels with its
        index in rec.vals."""
        max_count = len(self.values)
        rec_nodes = rec.nodes
        rec_vals = rec.vals
        rec_yields = rec.yields
        if state is None:
            stack = []      # [branches, current_branch], branches: [(comparable distance, node, index in vals)]
            neighbors = []  # (comparable distance, (box, value), index in vals)
            current = -1
            next_node = _DBL_MAX
            next_vi = -1    # index in vals of next_node, -1 for _DBL_MAX
        else:
            stack, neighbors, current, next_node, next_vi = state
        rec.stack = stack
        rec.neighbors = neighbors

        def visit(node):
            rec_nodes.append(node)
            ds = _box_distances(node.boxes, x, y)
            base = len(rec_vals)
            rec_vals.extend(ds)
            vis = range(base, base + len(ds))  # the index of each distance in rec.vals
            if node.leaf:
                if len(neighbors) < max_count:
                    neighbors.extend(zip(ds, node.children, vis))
                else:
                    greatest = neighbors[-1][0]
                    neighbors.extend([e for e in zip(ds, node.children, vis) if e[0] < greatest])
                sort_by_first(neighbors)
                if max_count < len(neighbors):
                    del neighbors[max_count:]
                return
            if max_count <= len(neighbors):
                worst = neighbors[-1][0]
                branches = [e for e in zip(ds, node.subnodes, vis) if not worst <= e[0]]
            else:
                branches = list(zip(ds, node.subnodes, vis))
            if branches:
                sort_by_first(branches)
                stack.append([branches, 0])

        if state is None:
            visit(self.root)
        while True:
            # one increment of boost's distance_query_incremental
            nxt = current + 1
            while True:
                if not stack:
                    found = nxt < len(neighbors)
                    break
                top = stack[-1]
                branches, cb = top
                if len(branches) <= cb:
                    stack.pop()
                    continue
                if nxt < len(neighbors) and neighbors[nxt][0] < next_node:
                    found = True
                    break
                if max_count <= len(neighbors) and neighbors[-1][0] <= branches[cb][0]:
                    stack.pop()
                    continue
                top[1] = cb + 1
                visit(branches[cb][1])
                # the distance of the closest branch not visited yet
                next_node = _DBL_MAX
                next_vi = -1
                for bs, c in stack:
                    if c < len(bs) and bs[c][0] < next_node:
                        next_node = bs[c][0]
                        next_vi = bs[c][2]
            if not found:
                rec.done = True
                return
            current = nxt
            item = neighbors[current][1]
            rec_yields.append(item)
            rec.current = current
            rec.next_vi = next_vi
            rec.clean = True
            yield item
            rec.clean = False
            rec.epoch += 1


def box_point_distance(box, x, y):
    dx = box[0] - x if x < box[0] else (x - box[2] if x > box[2] else 0.0)
    dy = box[1] - y if y < box[1] else (y - box[3] if y > box[3] else 0.0)
    return math.sqrt(dx * dx + dy * dy)
