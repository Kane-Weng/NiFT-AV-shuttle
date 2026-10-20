"""Routing of the Lanelet2 stand-in (lanelet2.routing), ported from RoutingGraphBuilder.cpp, RoutingGraph.cpp and
RoutingCost.h in Lanelet2 1.2.3.

Third-party portions: translated from Lanelet2 1.2.3 (https://github.com/fzi-forschungszentrum-informatik/Lanelet2),
BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik. See ../LICENSES/BSD-3-Clause-Lanelet2.txt and
../NOTICE.

The graph is built the same way:
- vertices: every passable lanelet, then the inverted copy of each lanelet that is passable backwards
- Successor edges: lanelets whose start points are this lanelet's end points (follows() and canPass())
- Left/Right edges: neighbours that share a bound, where canChangeLane() allows it; else AdjacentLeft/Right
- Conflicting edges for overlapping lanelets are not built (routing never uses them)
- one edge per routing cost module; the default modules are RoutingCostDistance(10) and RoutingCostTravelTime(5)
Edge cost for a successor is (cost(from) + cost(to)) / 2, where cost is approximatedLength2d or travel time.
shortestPath is Dijkstra on Successor (+ Left/Right with lane changes) edges. Lanelet2 uses a boost heap, so between
routes with exactly the same cost the pick can differ; the cost is the same.
Areas are not routed (German vehicles cannot pass areas).
"""
import heapq
import itertools

from . import geometry


class RelationType:
    None_ = 0
    Successor = 0b1
    Left = 0b10
    Right = 0b100
    AdjacentLeft = 0b1000
    AdjacentRight = 0b10000
    Conflicting = 0b100000
    Area = 0b1000000


Successor = RelationType.Successor
Left = RelationType.Left
Right = RelationType.Right
AdjacentLeft = RelationType.AdjacentLeft
AdjacentRight = RelationType.AdjacentRight
Conflicting = RelationType.Conflicting


class RoutingCost:
    pass


class RoutingCostDistance(RoutingCost):
    def __init__(self, laneChangeCost, minLaneChangeDistance=0.0):
        if laneChangeCost < 0:
            raise ValueError('Lane change cost must be positive, but it is %r' % laneChangeCost)
        self.laneChangeCost = float(laneChangeCost)
        self.minLaneChangeDistance = float(minLaneChangeDistance)

    def _len(self, ll):
        return geometry.approximatedLength2d(ll)

    def getCostSucceeding(self, rules, frm, to):
        return (self._len(frm) + self._len(to)) / 2

    def getCostLaneChange(self, rules, froms, tos):
        if self.minLaneChangeDistance <= 0:
            return self.laneChangeCost
        total = 0.0
        for ll in froms:
            total += self._len(ll)
        return self.laneChangeCost if total >= self.minLaneChangeDistance else float('inf')


class RoutingCostTravelTime(RoutingCost):
    def __init__(self, laneChangeCost, minLaneChangeTime=0.0):
        if laneChangeCost < 0:
            raise ValueError('Lane change cost must be positive, but it is %r' % laneChangeCost)
        self.laneChangeCost = float(laneChangeCost)
        self.minLaneChangeTime = float(minLaneChangeTime)

    @staticmethod
    def _time(rules, ll):
        limit = rules.speedLimit(ll).mps
        if limit == 0.0:
            limit = 1.7976931348623157e308
        return geometry.approximatedLength2d(ll) / limit

    def getCostSucceeding(self, rules, frm, to):
        return (self._time(rules, frm) + self._time(rules, to)) / 2

    def getCostLaneChange(self, rules, froms, tos):
        if self.minLaneChangeTime <= 0:
            return self.laneChangeCost
        total = 0.0
        for ll in froms:
            total += self._time(rules, ll)
        return self.laneChangeCost if total >= self.minLaneChangeTime else float('inf')


def defaultRoutingCosts():
    return [RoutingCostDistance(10.0), RoutingCostTravelTime(5.0)]


class LaneletPath:
    def __init__(self, lanelets=()):
        self._lanelets = list(lanelets)

    def __len__(self):
        return len(self._lanelets)

    def __iter__(self):
        return iter(self._lanelets)

    def __getitem__(self, i):
        return self._lanelets[i]

    def __eq__(self, other):
        return isinstance(other, LaneletPath) and self._lanelets == other._lanelets

    def __repr__(self):
        return 'LaneletPath([%s])' % ', '.join(str(ll.id) + ('i' if ll.inverted() else '') for ll in self._lanelets)


class LaneletRelation:
    def __init__(self, lanelet, relationType):
        self.lanelet = lanelet
        self.relationType = relationType


class Route:
    """The result of getRoute(). Only the shortest path part of Lanelet2's Route is kept."""

    def __init__(self, path):
        self._path = path

    def shortestPath(self):
        return self._path

    def length2d(self):
        return sum(geometry.length2d(ll) for ll in self._path)

    def remainingShortestPath(self, ll):
        lls = list(self._path)
        if ll not in lls:
            return LaneletPath()
        i = lls.index(ll)
        if lls and lls[0] == lls[-1]:
            return LaneletPath(lls[i:] + lls[:i])
        return LaneletPath(lls[i:])


class RoutingGraph:
    def __init__(self, laneletMap, trafficRules, routingCosts=None, config=None):
        self._rules = trafficRules
        self._costs = list(routingCosts) if routingCosts is not None else defaultRoutingCosts()
        self._vertices = []   # vertex index -> lanelet
        self._index = {}      # (id, inverted) -> vertex index
        self._out = []        # vertex -> list of (target, cost_id, relation, cost)
        self._in = []
        self._build(laneletMap)

    # ---------------------------------------------------------------- building

    def _add_vertex(self, ll):
        self._index[(ll.id, ll.inverted())] = len(self._vertices)
        self._vertices.append(ll)
        self._out.append([])
        self._in.append([])

    def _vertex(self, ll):
        return self._index.get((ll.id, ll.inverted()))

    def _add_edge(self, frm, to, cost_id, relation, cost):
        if cost != cost or cost in (float('inf'), float('-inf')):
            return
        if cost < 0:
            raise ValueError('Negative costs calculated by routing cost module!')
        a = self._vertex(frm)
        b = self._vertex(to)
        if a is None or b is None:
            return
        self._out[a].append((b, cost_id, relation, cost))
        self._in[b].append((a, cost_id, relation, cost))

    def _has_edge(self, frm, to):
        a = self._vertex(frm)
        b = self._vertex(to)
        return a is not None and b is not None and any(e[0] == b for e in self._out[a])

    def _assign_costs(self, frm, to, relation):
        for cid, cost in enumerate(self._costs):
            if relation == RelationType.Successor:
                c = cost.getCostSucceeding(self._rules, frm, to)
            elif relation in (RelationType.Left, RelationType.Right):
                c = cost.getCostLaneChange(self._rules, [frm], [to])
            else:
                c = 1.0
            self._add_edge(frm, to, cid, relation, c)

    def _build(self, laneletMap):
        rules = self._rules
        passable = [ll for ll in laneletMap.laneletLayer if rules.canPass(ll)]
        inverted = [ll.invert() for ll in passable if rules.canPass(ll.invert())]
        lanelets = passable + inverted
        points_to_lanelets = {}

        def key(p1, p2):
            return (p1.id, p2.id) if p1.id < p2.id else (p2.id, p1.id)

        for ll in lanelets:
            self._add_vertex(ll)
            lb = ll.leftBound
            rb = ll.rightBound
            if len(lb) == 0 or len(rb) == 0:
                continue
            for k in (key(lb[0], rb[0]), key(lb[len(lb) - 1], rb[len(rb) - 1]),
                      key(lb[0], lb[len(lb) - 1]), key(rb[0], rb[len(rb) - 1])):
                points_to_lanelets.setdefault(k, []).append(ll)
        self._points_to_lanelets = points_to_lanelets

        right_to_left = {}
        left_to_right = {}
        for ll in lanelets:
            lb = ll.leftBound
            rb = ll.rightBound
            if len(lb) == 0 or len(rb) == 0:
                continue
            # following
            for other in points_to_lanelets.get(key(lb[len(lb) - 1], rb[len(rb) - 1]), []):
                if geometry.follows(ll, other) and rules.canPass(ll, other):
                    self._assign_costs(ll, other, RelationType.Successor)
            # sideways
            for bound, relation, collector in ((lb, RelationType.AdjacentLeft, right_to_left),
                                               (rb, RelationType.AdjacentRight, left_to_right)):
                for other in points_to_lanelets.get(key(bound[0], bound[len(bound) - 1]), []):
                    if ll == other or self._has_edge(ll, other):
                        continue
                    if relation == RelationType.AdjacentLeft:
                        directly = geometry.leftOf(other, ll)
                    else:
                        directly = geometry.rightOf(other, ll)
                    if not directly:
                        continue
                    if rules.canChangeLane(ll, other):
                        collector.setdefault((ll.id, ll.inverted()), (ll, other))
                    else:
                        self._assign_costs(ll, other, relation)
        # lane changes; with the default modules (no minimum length or time) every pair gets the plain cost
        for collector, relation in ((right_to_left, RelationType.Left), (left_to_right, RelationType.Right)):
            for frm, to in collector.values():
                for cid, cost in enumerate(self._costs):
                    c = cost.getCostLaneChange(self._rules, [frm], [to])
                    if c in (float('inf'),):
                        adj = RelationType.AdjacentLeft if relation == RelationType.Left else RelationType.AdjacentRight
                        self._add_edge(frm, to, cid, adj, 1.0)
                    else:
                        self._add_edge(frm, to, cid, relation, c)

    # ---------------------------------------------------------------- queries

    def _neighbours(self, ll, relations, cost_id=0, out=True):
        v = self._vertex(ll)
        if v is None:
            return []
        edges = self._out[v] if out else self._in[v]
        return [self._vertices[e[0]] for e in edges if e[1] == cost_id and e[2] & relations]

    def following(self, lanelet, withLaneChanges=True):
        rel = RelationType.Successor | (RelationType.Left | RelationType.Right if withLaneChanges else 0)
        return self._neighbours(lanelet, rel)

    def previous(self, lanelet, withLaneChanges=True):
        rel = RelationType.Successor | (RelationType.Left | RelationType.Right if withLaneChanges else 0)
        return self._neighbours(lanelet, rel, out=False)

    def followingRelations(self, lanelet, withLaneChanges=True):
        return [LaneletRelation(ll, self.routingRelation(lanelet, ll))
                for ll in self.following(lanelet, withLaneChanges)]

    def previousRelations(self, lanelet, withLaneChanges=True):
        return [LaneletRelation(ll, self.routingRelation(ll, lanelet))
                for ll in self.previous(lanelet, withLaneChanges)]

    def _one(self, lanelet, relation):
        res = self._neighbours(lanelet, relation)
        return res[0] if res else None

    def left(self, lanelet):
        return self._one(lanelet, RelationType.Left)

    def right(self, lanelet):
        return self._one(lanelet, RelationType.Right)

    def adjacentLeft(self, lanelet):
        return self._one(lanelet, RelationType.AdjacentLeft)

    def adjacentRight(self, lanelet):
        return self._one(lanelet, RelationType.AdjacentRight)

    def _chain(self, lanelet, relation):
        out = []
        cur = self._one(lanelet, relation)
        while cur is not None and cur not in out:
            out.append(cur)
            cur = self._one(cur, relation)
        return out

    def lefts(self, lanelet):
        return self._chain(lanelet, RelationType.Left)

    def rights(self, lanelet):
        return self._chain(lanelet, RelationType.Right)

    def adjacentLefts(self, lanelet):
        return self._chain(lanelet, RelationType.AdjacentLeft)

    def adjacentRights(self, lanelet):
        return self._chain(lanelet, RelationType.AdjacentRight)

    def besides(self, lanelet):
        lefts = list(reversed(self._chain(lanelet, RelationType.Left | RelationType.AdjacentLeft)))
        rights = self._chain(lanelet, RelationType.Right | RelationType.AdjacentRight)
        return lefts + [lanelet] + rights

    def routingRelation(self, frm, to, includeConflicting=False):
        a = self._vertex(frm)
        b = self._vertex(to)
        if a is None or b is None:
            return None
        for e in self._out[a]:
            if e[0] == b and e[1] == 0:
                return e[2]
        return None

    def conflicting(self, lanelet):
        return []  # conflicting edges are not built by the stand-in

    def passableLaneletSubmap(self):
        from .core import createMapFromLanelets
        return createMapFromLanelets([ll for ll in self._vertices if not ll.inverted()])

    def _dijkstra(self, start, goal, cost_id, relations):
        """Dijkstra from vertex start until goal is examined. Returns (vertex list, cost) or None."""
        dist = {start: 0.0}
        pred = {start: start}
        done = set()
        counter = itertools.count()
        heap = [(0.0, next(counter), start)]
        out = self._out
        while heap:
            d, _, u = heapq.heappop(heap)
            if u in done or d > dist[u]:
                continue
            done.add(u)
            if u == goal:
                path = [u]
                while pred[path[-1]] != path[-1]:
                    path.append(pred[path[-1]])
                path.reverse()
                return path, d
            for v, cid, rel, c in out[u]:
                if cid != cost_id or not rel & relations:
                    continue
                nd = d + c
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    pred[v] = u
                    heapq.heappush(heap, (nd, next(counter), v))
        return None

    def _check_cost_id(self, cost_id):
        if not 0 <= cost_id < len(self._costs):
            raise ValueError('Routing Cost ID is higher than the number of routing modules.')

    def shortestPathWithCost(self, frm, to, routingCostId=0, withLaneChanges=True):
        """Stand-in extension: (LaneletPath, cost) or None. Used by the equivalence tests."""
        self._check_cost_id(routingCostId)
        a = self._vertex(frm)
        b = self._vertex(to)
        if a is None or b is None:
            return None
        rel = RelationType.Successor | (RelationType.Left | RelationType.Right if withLaneChanges else 0)
        res = self._dijkstra(a, b, routingCostId, rel)
        if res is None:
            return None
        verts, cost = res
        return LaneletPath([self._vertices[v] for v in verts]), cost

    def shortestPath(self, frm, to, routingCostId=0, withLaneChanges=True):
        res = self.shortestPathWithCost(frm, to, routingCostId, withLaneChanges)
        return None if res is None else res[0]

    def getRoute(self, frm, to, routingCostId=0, withLaneChanges=True):
        res = self.shortestPathWithCost(frm, to, routingCostId, withLaneChanges)
        if res is None:
            return None
        return Route(res[0])

    def shortestPathWithVia(self, start, via, end, routingCostId=0, withLaneChanges=True):
        points = [start] + list(via) + [end]
        path = []
        for a, b in zip(points, points[1:]):
            res = self.shortestPath(a, b, routingCostId, withLaneChanges)
            if res is None:
                return None
            lls = list(res)
            if path and lls and path[-1] == lls[0]:
                lls = lls[1:]
            path.extend(lls)
        return LaneletPath(path)

    def getRouteVia(self, start, via, end, routingCostId=0, withLaneChanges=True):
        path = self.shortestPathWithVia(start, via, end, routingCostId, withLaneChanges)
        return None if path is None else Route(path)

    def pathCost(self, path, routingCostId=0):
        """Stand-in extension: the cost of a given lanelet sequence on the graph, or None if an edge is missing."""
        total = 0.0
        for a, b in zip(list(path), list(path)[1:]):
            va = self._vertex(a)
            vb = self._vertex(b)
            costs = [e[3] for e in self._out[va] if e[0] == vb and e[1] == routingCostId
                     and e[2] & (RelationType.Successor | RelationType.Left | RelationType.Right)]
            if not costs:
                return None
            total += min(costs)
        return total
