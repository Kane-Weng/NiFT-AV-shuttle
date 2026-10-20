"""Core primitives of the Lanelet2 stand-in: points, line strings, lanelets, areas, regulatory elements, the map.

The classes follow the Python bindings of Lanelet2 1.2.3 (lanelet2.core). Mutable and const flavours are the same
class here (ConstLanelet is Lanelet, and so on), because the nodes only read the map.

Semantics kept from Lanelet2:
- A line string or lanelet is a view on shared data plus an `inverted` flag. Equality means same data, same flag.
- An inverted lanelet swaps and inverts its bounds, and its centerline is the inverted centerline.
- The centerline is computed lazily from the bounds (see _centerline.py) and cached on the data.

Third-party portions: translated from Lanelet2 1.2.3 (https://github.com/fzi-forschungszentrum-informatik/Lanelet2),
BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik. See ../LICENSES/BSD-3-Clause-Lanelet2.txt and
../NOTICE.
"""
from ._centerline import compute_centerline

InvalId = 0


class GPSPoint:
    __slots__ = ('lat', 'lon', 'ele')

    def __init__(self, lat=0.0, lon=0.0, ele=0.0):
        self.lat = float(lat)
        self.lon = float(lon)
        self.ele = float(ele)

    @property
    def alt(self):
        return self.ele

    @alt.setter
    def alt(self, value):
        self.ele = float(value)

    def __repr__(self):
        return 'GPSPoint(%r, %r, %r)' % (self.lat, self.lon, self.ele)


class _BasicVec:
    __slots__ = ()

    def __eq__(self, other):
        try:
            return tuple(self) == tuple(other)
        except TypeError:
            return NotImplemented

    def __hash__(self):
        return hash(tuple(self))


class BasicPoint2d(_BasicVec):
    __slots__ = ('x', 'y')

    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __iter__(self):
        yield self.x
        yield self.y

    def __len__(self):
        return 2

    def __getitem__(self, i):
        return (self.x, self.y)[i]

    def __add__(self, o):
        return BasicPoint2d(self.x + o.x, self.y + o.y)

    def __sub__(self, o):
        return BasicPoint2d(self.x - o.x, self.y - o.y)

    def __mul__(self, s):
        return BasicPoint2d(self.x * s, self.y * s)

    __rmul__ = __mul__

    def __truediv__(self, s):
        return BasicPoint2d(self.x / s, self.y / s)

    def __repr__(self):
        return 'BasicPoint2d(%r, %r)' % (self.x, self.y)


class BasicPoint3d(_BasicVec):
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __len__(self):
        return 3

    def __getitem__(self, i):
        return (self.x, self.y, self.z)[i]

    def __add__(self, o):
        return BasicPoint3d(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o):
        return BasicPoint3d(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, s):
        return BasicPoint3d(self.x * s, self.y * s, self.z * s)

    __rmul__ = __mul__

    def __truediv__(self, s):
        return BasicPoint3d(self.x / s, self.y / s, self.z / s)

    def __repr__(self):
        return 'BasicPoint3d(%r, %r, %r)' % (self.x, self.y, self.z)


class AttributeMap(dict):
    """Tag key -> string value, like lanelet2.core.AttributeMap (a dict with string values)."""

    def __repr__(self):
        return 'AttributeMap(%s)' % dict.__repr__(dict(sorted(self.items())))


def _as_bool(value):
    """Attribute::asBool: '0'/'1' (lexical_cast), then true/yes/false/no. None if not a bool."""
    if value in ('1', '0'):
        return value == '1'
    if value in ('true', 'yes'):
        return True
    if value in ('false', 'no'):
        return False
    return None


def _as_double(value):
    """Attribute::asDouble via boost::lexical_cast: the whole string must be a number."""
    try:
        v = value.strip()
        if v != value or not v:
            return None
        return float(v)
    except (ValueError, AttributeError):
        return None


class Point3d:
    """A map point. x, y, z are in the projected frame. ConstPoint3d is the same class."""
    __slots__ = ('id', 'x', 'y', 'z', 'attributes')

    def __init__(self, id=InvalId, x=0.0, y=0.0, z=0.0, attributes=None):
        self.id = id
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.attributes = attributes if attributes is not None else AttributeMap()

    def basicPoint(self):
        return BasicPoint3d(self.x, self.y, self.z)

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __repr__(self):
        return 'Point3d(%d, %g, %g, %g)' % (self.id, self.x, self.y, self.z)


ConstPoint3d = Point3d
Point2d = Point3d
ConstPoint2d = Point3d


class _LineStringData:
    __slots__ = ('id', 'points', 'attributes')

    def __init__(self, id, points, attributes):
        self.id = id
        self.points = points
        self.attributes = attributes


class LineString3d:
    """A polyline of Point3d. Indexing, len() and iteration respect the inverted flag."""
    __slots__ = ('_data', '_inv')

    def __init__(self, id=InvalId, points=None, attributes=None, _data=None, _inv=False):
        if _data is None:
            _data = _LineStringData(id, list(points) if points is not None else [],
                                    attributes if attributes is not None else AttributeMap())
        self._data = _data
        self._inv = _inv

    @property
    def id(self):
        return self._data.id

    @property
    def attributes(self):
        return self._data.attributes

    def inverted(self):
        return self._inv

    def invert(self):
        return LineString3d(_data=self._data, _inv=not self._inv)

    def _pts(self):
        return self._data.points[::-1] if self._inv else self._data.points

    def __len__(self):
        return len(self._data.points)

    def __iter__(self):
        return iter(reversed(self._data.points)) if self._inv else iter(self._data.points)

    def __getitem__(self, i):
        pts = self._data.points
        if isinstance(i, slice):
            return self._pts()[i]
        n = len(pts)
        if i < 0:
            i += n
        if not 0 <= i < n:
            raise IndexError('line string index out of range')
        return pts[n - 1 - i] if self._inv else pts[i]

    def append(self, point):
        if self._inv:
            self._data.points.insert(0, point)
        else:
            self._data.points.append(point)

    def __eq__(self, other):
        return isinstance(other, LineString3d) and self._data is other._data and self._inv == other._inv

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        # like Lanelet: equal line strings share their data, so they have the same id; stable across runs
        return hash((self._data.id, self._inv))

    def __repr__(self):
        return 'LineString3d(%d, %d points%s)' % (self.id, len(self), ', inverted' if self._inv else '')


ConstLineString3d = LineString3d
LineString2d = LineString3d
ConstLineString2d = LineString3d


class _LaneletData:
    __slots__ = ('id', 'left', 'right', 'attributes', 'regulatoryElements', 'centerline', 'aux')

    def __init__(self, id, left, right, attributes):
        self.id = id
        self.left = left
        self.right = right
        self.attributes = attributes
        self.regulatoryElements = []
        self.centerline = None
        self.aux = {}  # caches for geometry (bounding box, polygon, length)


class Lanelet:
    """A lanelet: left and right bound plus attributes. ConstLanelet is the same class."""
    __slots__ = ('_data', '_inv')

    def __init__(self, id=InvalId, leftBound=None, rightBound=None, attributes=None, _data=None, _inv=False):
        if _data is None:
            _data = _LaneletData(id, leftBound if leftBound is not None else LineString3d(),
                                 rightBound if rightBound is not None else LineString3d(),
                                 attributes if attributes is not None else AttributeMap())
        self._data = _data
        self._inv = _inv

    @property
    def id(self):
        return self._data.id

    @property
    def attributes(self):
        return self._data.attributes

    @property
    def regulatoryElements(self):
        return list(self._data.regulatoryElements)

    def addRegulatoryElement(self, regelem):
        self._data.regulatoryElements.append(regelem)

    def inverted(self):
        return self._inv

    def invert(self):
        return Lanelet(_data=self._data, _inv=not self._inv)

    @property
    def leftBound(self):
        return self._data.right.invert() if self._inv else self._data.left

    @property
    def rightBound(self):
        return self._data.left.invert() if self._inv else self._data.right

    @property
    def centerline(self):
        d = self._data
        if d.centerline is None:
            pts = compute_centerline(list(d.left), list(d.right))
            d.centerline = LineString3d(InvalId, [Point3d(InvalId, x, y, z) for x, y, z in pts])
        return d.centerline.invert() if self._inv else d.centerline

    @centerline.setter
    def centerline(self, ls):
        self._data.centerline = ls

    def resetCache(self):
        self._data.centerline = None
        self._data.aux.clear()

    def speedLimits(self):
        return [r for r in self._data.regulatoryElements if isinstance(r, SpeedLimit)]

    def trafficSigns(self):
        return [r for r in self._data.regulatoryElements if isinstance(r, TrafficSign)]

    def trafficLights(self):
        return [r for r in self._data.regulatoryElements if isinstance(r, TrafficLight)]

    def rightOfWay(self):
        return [r for r in self._data.regulatoryElements if isinstance(r, RightOfWay)]

    def allWayStop(self):
        return [r for r in self._data.regulatoryElements if isinstance(r, AllWayStop)]

    def polygon2d(self):
        """The outline: left bound, then the right bound backwards (CompoundPolygon2d), as a list of BasicPoint2d."""
        return [BasicPoint2d(p.x, p.y) for p in list(self.leftBound) + list(self.rightBound.invert())]

    def polygon3d(self):
        return [BasicPoint3d(p.x, p.y, p.z) for p in list(self.leftBound) + list(self.rightBound.invert())]

    def __eq__(self, other):
        return isinstance(other, Lanelet) and self._data is other._data and self._inv == other._inv

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self._data.id, self._inv))

    def __repr__(self):
        return 'Lanelet(%d%s, left %d, right %d)' % (
            self.id, ', inverted' if self._inv else '', self._data.left.id, self._data.right.id)


ConstLanelet = Lanelet


class Polygon3d(LineString3d):
    __slots__ = ()


ConstPolygon3d = Polygon3d


class Area:
    """A multipolygon area: outer bound (line strings forming one ring), inner bounds, attributes."""

    def __init__(self, id=InvalId, outerBound=None, innerBounds=None, attributes=None):
        self.id = id
        self.outerBound = list(outerBound) if outerBound is not None else []
        self.innerBounds = [list(b) for b in innerBounds] if innerBounds is not None else []
        self.attributes = attributes if attributes is not None else AttributeMap()
        self._regelems = []

    @property
    def regulatoryElements(self):
        return list(self._regelems)

    def addRegulatoryElement(self, regelem):
        self._regelems.append(regelem)

    def outerBoundPolygon(self):
        pts = []
        for ls in self.outerBound:
            pts.extend(ls)
        return pts

    def __repr__(self):
        return 'Area(%d)' % self.id


ConstArea = Area


class RegulatoryElement:
    """A generic regulatory element. parameters maps a role name to a list of primitives."""

    def __init__(self, id=InvalId, parameters=None, attributes=None):
        self.id = id
        self.parameters = parameters if parameters is not None else {}
        self.attributes = attributes if attributes is not None else AttributeMap()

    def __repr__(self):
        return '%s(%d)' % (type(self).__name__, self.id)


GenericRegulatoryElement = RegulatoryElement


class TrafficSign(RegulatoryElement):
    def trafficSigns(self):
        return list(self.parameters.get('refers', []))

    def type(self):
        signs = self.trafficSigns()
        if signs:
            sub = signs[0].attributes.get('subtype')
            if sub is None:
                raise ValueError('Regulatory element has a traffic sign without subtype attribute!')
            return sub
        if 'sign_type' in self.attributes:
            return self.attributes['sign_type']
        raise ValueError('Regulatory element can not determine the type of the traffic sign!')


class SpeedLimit(TrafficSign):
    pass


class TrafficLight(RegulatoryElement):
    pass


class RightOfWay(RegulatoryElement):
    pass


class AllWayStop(RegulatoryElement):
    pass


REGELEM_TYPES = {
    'traffic_sign': TrafficSign,
    'speed_limit': SpeedLimit,
    'traffic_light': TrafficLight,
    'right_of_way': RightOfWay,
    'all_way_stop': AllWayStop,
}


class BoundingBox2d:
    __slots__ = ('min', 'max')

    def __init__(self, pmin=None, pmax=None):
        self.min = pmin if pmin is not None else BasicPoint2d(float('inf'), float('inf'))
        self.max = pmax if pmax is not None else BasicPoint2d(float('-inf'), float('-inf'))

    def __repr__(self):
        return 'BoundingBox2d(%r, %r)' % (self.min, self.max)


class PrimitiveLayer:
    """A map layer: primitives by id, kept in insertion order.

    Iteration follows Lanelet2: its layers are std::unordered_map<Id, T>, so they iterate in the order libstdc++'s hash
    table gives after inserting the ids one by one (io.load inserts in ascending id order, like the C++ loader). The
    stand-in computes that order with _stdorder.hash_map_order, the same order the R-tree is built from. Code that
    needs id order should sort.
    """

    def __init__(self, items=None):
        self._set_items(dict(items) if items else {})

    def _set_items(self, items):
        """Replace the contents (items in insertion order) and drop the derived R-tree and iteration order."""
        self._items = items
        self._tree = None
        self._order = None

    def _iter_order(self):
        if self._order is None:
            from ._stdorder import hash_map_order
            self._order, _ = hash_map_order(list(self._items))
        return self._order

    def __len__(self):
        return len(self._items)

    def __iter__(self):
        items = self._items
        return iter([items[k] for k in self._iter_order()])

    def __getitem__(self, id):
        try:
            return self._items[id]
        except KeyError:
            raise KeyError('No element with id %r in this layer' % (id,)) from None

    def __contains__(self, item):
        if isinstance(item, int):
            return item in self._items
        return getattr(item, 'id', None) in self._items

    def exists(self, id):
        return id in self._items

    def get(self, id):
        return self[id]

    def add(self, prim):
        self._items[prim.id] = prim
        self._tree = None
        self._order = None

    def _rtree(self):
        """The layer's R-tree, built like Lanelet2 builds it after loading (see _rtree.py), from the layer in its
        iteration order."""
        tree = self._tree
        if tree is None:
            from . import geometry
            from ._rtree import RTree
            values = []
            for k in self._iter_order():
                prim = self._items[k]
                b = geometry.boundingBox2d(prim)
                if b.min.x <= b.max.x and b.min.y <= b.max.y:
                    values.append(((b.min.x, b.min.y, b.max.x, b.max.y), prim))
            tree = self._tree = RTree(values)
        return tree

    def search(self, bbox):
        from . import geometry
        out = []
        for prim in self:
            b = geometry.boundingBox2d(prim)
            if not (b.max.x < bbox.min.x or bbox.max.x < b.min.x or b.max.y < bbox.min.y or bbox.max.y < b.min.y):
                out.append(prim)
        return out


class LaneletLayer(PrimitiveLayer):
    pass


PrimitiveLayerLanelet = LaneletLayer


class AreaLayer(PrimitiveLayer):
    pass


PrimitiveLayerArea = AreaLayer


class PointLayer(PrimitiveLayer):
    pass


class LineStringLayer(PrimitiveLayer):
    pass


class PolygonLayer(PrimitiveLayer):
    pass


class RegulatoryElementLayer(PrimitiveLayer):
    pass


class LaneletMap:
    def __init__(self):
        self.laneletLayer = LaneletLayer()
        self.areaLayer = AreaLayer()
        self.regulatoryElementLayer = RegulatoryElementLayer()
        self.polygonLayer = PolygonLayer()
        self.lineStringLayer = LineStringLayer()
        self.pointLayer = PointLayer()

    def add(self, prim):
        if isinstance(prim, Lanelet):
            self.laneletLayer.add(prim)
            for ls in (prim.leftBound, prim.rightBound):
                self.add(ls)
        elif isinstance(prim, Area):
            self.areaLayer.add(prim)
        elif isinstance(prim, RegulatoryElement):
            self.regulatoryElementLayer.add(prim)
        elif isinstance(prim, Polygon3d):
            self.polygonLayer.add(prim)
        elif isinstance(prim, LineString3d):
            self.lineStringLayer.add(prim)
            for p in prim:
                self.pointLayer.add(p)
        elif isinstance(prim, Point3d):
            self.pointLayer.add(prim)
        else:
            raise TypeError('Cannot add %r to a LaneletMap' % (prim,))


LaneletSubmap = LaneletMap


def createMapFromLanelets(lanelets):
    m = LaneletMap()
    for ll in lanelets:
        m.add(ll)
    return m


# The id counter is process-wide and never reset, like Lanelet2's utils::getId / registerId: io.load registers every
# id it reads, so getId() afterwards returns an id that no loaded map uses. Loading a second map keeps counting from
# the highest id seen so far. The start value is the stand-in's own choice.
_next_id = [1000000000]


def getId():
    """A new id, higher than every id registered so far."""
    i = _next_id[0]
    _next_id[0] = i + 1
    return i


def registerId(id):
    """Make sure getId() never returns id."""
    if id >= _next_id[0]:
        _next_id[0] = id + 1
