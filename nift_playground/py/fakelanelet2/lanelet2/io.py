"""Loading Lanelet2 OSM maps in pure Python (lanelet2.io), as OsmFile.cpp and OsmHandlerLoad.cpp in Lanelet2 1.2.3.

Third-party portions: translated from Lanelet2 1.2.3 (https://github.com/fzi-forschungszentrum-informatik/Lanelet2),
BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik. See ../LICENSES/BSD-3-Clause-Lanelet2.txt and
../NOTICE.

What it does, in Lanelet2's order:
- nodes become Point3d, projected with the given projector ('ele' tag is the height, not an attribute)
- ways become LineString3d, or Polygon3d when tagged area=yes
- relations type=lanelet become Lanelet (exactly one left and one right way; bounds aligned with geometry::align;
  an optional centerline way becomes a custom centerline)
- relations type=multipolygon become Area (outer and inner rings assembled from the ways)
- relations type=regulatory_element become regulatory elements and are attached to their lanelets and areas
- elements with action=delete are skipped
Maps that ship a node cache (see _nodecache.py) take the node coordinates from it instead of projecting them.
Errors are collected like Lanelet2 does; load() raises ParseError if there are any, loadRobust() returns them.
"""
import hashlib
import logging
import os
import sys
import xml.etree.ElementTree as ET

from . import _nodecache, geometry
from .core import (REGELEM_TYPES, AttributeMap, Area, GPSPoint, Lanelet, LaneletMap, LineString3d, Point3d,
                   Polygon3d, RegulatoryElement, _as_bool, registerId)
from .projection import ForwardProjectionError

_log = logging.getLogger('lanelet2.io')


class ParseError(RuntimeError):
    pass


class FileNotFoundError_(RuntimeError):
    pass


class Origin:
    def __init__(self, lat=None, lon=0.0, alt=0.0):
        if isinstance(lat, GPSPoint):
            self.position = GPSPoint(lat.lat, lat.lon, lat.ele)
            self.isDefault = False
        elif lat is None:
            self.position = GPSPoint(0.0, 0.0, 0.0)
            self.isDefault = True
        else:
            self.position = GPSPoint(lat, lon, alt)
            self.isDefault = False

    def __repr__(self):
        return 'Origin(%r, %r, %r)' % (self.position.lat, self.position.lon, self.position.ele)


def _tags(elem):
    attrs = AttributeMap()
    for tag in elem.iter('tag'):
        k = tag.get('k', '')
        if k == 'ele':
            continue
        attrs[k] = tag.get('v', '')
    return attrs


def _is_deleted(elem):
    return elem.get('action') == 'delete'


def _as_id(text):
    try:
        return int(text)
    except (TypeError, ValueError):
        return 0


def _as_float(text, default=0.0):
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def _read_osm(root, errors):
    """OsmFileParser::read: nodes, ways, relations keyed by id (std::map, so processed in id order)."""
    nodes = {}
    ways = {}
    relations = {}
    for el in root.iter('node'):
        if _is_deleted(el):
            continue
        nid = _as_id(el.get('id'))
        ele = 0.0
        for tag in el.iter('tag'):
            if tag.get('k') == 'ele':
                ele = _as_float(tag.get('v'))
                break
        nodes[nid] = (_tags(el), _as_float(el.get('lat')), _as_float(el.get('lon')), ele)
    for el in root.iter('way'):
        if _is_deleted(el):
            continue
        wid = _as_id(el.get('id'))
        refs = [_as_id(nd.get('ref')) for nd in el.iter('nd')]
        if any(r not in nodes for r in refs):
            errors.append('Error reading primitive with id %d from file: Way references nonexisting points' % wid)
            refs = []
        ways[wid] = (_tags(el), refs)
    for el in root.iter('relation'):
        if _is_deleted(el):
            continue
        rid = _as_id(el.get('id'))
        members = []
        for m in el.iter('member'):
            mtype = m.get('type', '')
            ref = _as_id(m.get('ref'))
            role = m.get('role', '')
            if mtype == 'node':
                if ref in nodes:
                    members.append((role, 'node', ref))
                else:
                    errors.append('Error reading primitive with id %d from file: Relation has nonexistent member %d'
                                  % (rid, ref))
            elif mtype == 'way':
                if ref in ways:
                    members.append((role, 'way', ref))
                else:
                    errors.append('Error reading primitive with id %d from file: Relation has nonexistent member %d'
                                  % (rid, ref))
            elif mtype == 'relation':
                members.append((role, 'relation', ref))
        relations[rid] = (_tags(el), members)
    # unresolved relation members
    for rid, (attrs, members) in relations.items():
        keep = []
        for mem in members:
            if mem[1] == 'relation' and mem[2] not in relations:
                errors.append('Error reading primitive with id %d from file: Relation references nonexistent '
                              'relation %d' % (rid, mem[2]))
                continue
            keep.append(mem)
        members[:] = keep
    return nodes, ways, relations


class _Loader:
    def __init__(self, projector):
        self.projector = projector
        self.errors = []
        self.points = {}
        self.linestrings = {}
        self.polygons = {}
        self.lanelets = {}
        self.areas = {}
        self.regelems = {}

    def err(self, pid, what):
        self.errors.append('Error parsing primitive %d: %s' % (pid, what))

    def get_or_dummy(self, table, pid, current, factory):
        prim = table.get(pid)
        if prim is None:
            self.err(current, 'Failed to get id %d from map' % pid)
            prim = factory(pid)
        return prim

    def load_nodes(self, nodes, cache=None):
        fwd = getattr(self.projector, 'forward_xy', None)
        for nid in sorted(nodes):
            attrs, lat, lon, ele = nodes[nid]
            try:
                if cache is not None and nid in cache:
                    x, y = cache[nid]
                    z = ele
                elif fwd is not None:
                    x, y = fwd(lat, lon)
                    z = ele
                else:
                    p = self.projector.forward(GPSPoint(lat, lon, ele))
                    x, y, z = p.x, p.y, p.z
            except ForwardProjectionError as e:
                self.err(nid, str(e))
                continue
            self.points[nid] = Point3d(nid, x, y, z, attrs)

    def load_ways(self, ways):
        for wid in sorted(ways):
            attrs, refs = ways[wid]
            pts = [self.get_or_dummy(self.points, r, wid, lambda i: Point3d(i)) for r in refs]
            if not pts:
                self.err(wid, 'Ways must have at least one point!')
                continue
            area = attrs.get('area')
            if area is not None and _as_bool(area):
                self.polygons[wid] = Polygon3d(wid, pts, attrs)
            else:
                self.linestrings[wid] = LineString3d(wid, pts, attrs)

    def border(self, rid, members, role):
        found = [m for m in members if m[0] == role]
        if len(found) != 1:
            self.err(rid, 'Lanelet has not exactly one %s border!' % role)
            return LineString3d(rid)
        if found[0][1] != 'way':
            self.err(rid, 'Lanelet %s border is not of type way!' % role)
            return LineString3d(rid)
        return self.get_or_dummy(self.linestrings, found[0][2], rid, lambda i: LineString3d(i))

    def load_lanelets(self, relations):
        with_regelems = []
        for rid in sorted(relations):
            attrs, members = relations[rid]
            if attrs.get('type') != 'lanelet':
                continue
            left = self.border(rid, members, 'left')
            right = self.border(rid, members, 'right')
            left, right = geometry.align(left, right)
            ll = Lanelet(rid, left, right, attrs)
            if any(m[0] == 'centerline' for m in members):
                ll.centerline = self.border(rid, members, 'centerline')
            self.lanelets[rid] = ll
            if any(m[0] == 'regulatory_element' for m in members):
                with_regelems.append((ll, members))
        return with_regelems

    def linestrings_for(self, members, role, ref_id):
        out = []
        for m in members:
            if m[0] != role:
                continue
            if m[1] != 'way':
                msg = '%s ring must consist of ways but id %d is of type %s!' % (role, m[2], m[1])
                self.err(ref_id, msg[0].upper() + msg[1:])
                continue
            ls = self.linestrings.get(m[2])
            if ls is None:
                self.err(ref_id, 'Failed to get id %d from map' % m[2])
                continue
            out.append(ls)
        return out

    def assemble_boundary(self, lss, aid):
        """OsmHandlerLoad assembleBoundary(): chain ways into closed rings, fix orientation."""
        lss = list(reversed(lss))
        rings = [[]]
        while lss:
            curr = rings[-1]
            if not curr:
                curr.append(lss.pop())
            else:
                last_id = curr[-1][len(curr[-1]) - 1].id
                idx = None
                for j in range(len(lss) - 1, -1, -1):
                    e = lss[j]
                    if e[len(e) - 1].id == last_id or e[0].id == last_id:
                        idx = j
                        break
                if idx is None:
                    self.err(aid, 'Could not complete boundary around linestring %d' % curr[-1].id)
                    rings[-1] = []
                    continue
                new_ls = lss.pop(idx)
                if new_ls[len(new_ls) - 1].id == last_id:
                    new_ls = new_ls.invert()
                curr.append(new_ls)
            curr = rings[-1]
            if curr and curr[-1][len(curr[-1]) - 1].id == curr[0][0].id:
                rings.append([])
        rings.pop()
        return rings

    def load_areas(self, relations):
        with_regelems = []
        for rid in sorted(relations):
            attrs, members = relations[rid]
            if attrs.get('type') != 'multipolygon':
                continue
            outer = self.linestrings_for(members, 'outer', rid)
            if not outer:
                self.err(rid, 'Areas must have at least one outer border!')
                continue
            outer_rings = self.assemble_boundary(outer, rid)
            if len(outer_rings) != 1:
                self.err(rid, 'Areas must have exactly one outer ring!')
                continue
            inner = self.assemble_boundary(self.linestrings_for(members, 'inner', rid), rid)
            area = Area(rid, outer_rings[0], inner, attrs)
            self.areas[rid] = area
            if any(m[0] == 'regulatory_element' for m in members):
                with_regelems.append((area, members))
        return with_regelems

    def load_regelems(self, relations):
        for rid in sorted(relations):
            attrs, members = relations[rid]
            if attrs.get('type') != 'regulatory_element':
                continue
            subtype = attrs.get('subtype')
            if subtype is None:
                self.err(rid, "Regulatory element has no 'subtype' tag.")
                continue
            params = {}
            for role, mtype, ref in members:
                if mtype == 'node':
                    params.setdefault(role, []).append(
                        self.get_or_dummy(self.points, ref, rid, lambda i: Point3d(i)))
                elif mtype == 'way':
                    if ref in self.polygons:
                        params.setdefault(role, []).append(self.polygons[ref])
                    else:
                        params.setdefault(role, []).append(
                            self.get_or_dummy(self.linestrings, ref, rid, lambda i: LineString3d(i)))
                elif mtype == 'relation':
                    rtype = relations[ref][0].get('type')
                    if rtype is None:
                        self.err(rid, 'Relation refers to another relation %d without a type tag!' % ref)
                    elif rtype == 'lanelet':
                        params.setdefault(role, []).append(
                            self.get_or_dummy(self.lanelets, ref, rid, lambda i: Lanelet(i)))
                    elif rtype == 'multipolygon':
                        params.setdefault(role, []).append(self.get_or_dummy(self.areas, ref, rid, lambda i: Area(i)))
                    elif rtype == 'regulatory_element':
                        self.err(rid, 'Regulatory element refers to another regulatory element. This is not '
                                      'supported.')
                    else:
                        self.err(rid, 'Member of regulatory_element has unsupported type ' + rtype)
            cls = REGELEM_TYPES.get(subtype, RegulatoryElement)
            self.regelems[rid] = cls(rid, params, attrs)

    def add_regelems(self, with_regelems):
        for prim, members in with_regelems:
            for role, mtype, ref in members:
                if role != 'regulatory_element':
                    continue
                prim.addRegulatoryElement(
                    self.get_or_dummy(self.regelems, ref, prim.id, lambda i: RegulatoryElement(i)))

    def build(self, nodes, ways, relations, cache=None):
        self.load_nodes(nodes, cache)
        self.load_ways(ways)
        ll_regs = self.load_lanelets(relations)
        ar_regs = self.load_areas(relations)
        self.load_regelems(relations)
        self.add_regelems(ll_regs)
        self.add_regelems(ar_regs)
        m = LaneletMap()
        for table, layer in ((self.lanelets, m.laneletLayer), (self.areas, m.areaLayer),
                             (self.regelems, m.regulatoryElementLayer), (self.polygons, m.polygonLayer),
                             (self.linestrings, m.lineStringLayer), (self.points, m.pointLayer)):
            layer._set_items({k: table[k] for k in sorted(table)})  # C++ emplaces in ascending id order
        return m


def _check_projector(projector):
    if projector is None or getattr(projector.origin(), 'isDefault', False):
        raise ParseError('You must pass an origin when loading a map with georeferenced (lat/lon) data!')


def _read_bytes(source):
    if isinstance(source, (bytes, bytearray)):
        return bytes(source)
    if not os.path.exists(source):
        raise FileNotFoundError_('Could not find lanelet map under ' + str(source))
    with open(source, 'rb') as f:
        return f.read()


def _load(source, projector):
    _check_projector(projector)
    data = _read_bytes(source)
    try:
        root = ET.fromstring(data)
    except ET.ParseError as e:
        raise ParseError('Errors occured while parsing osm file: %s' % e) from None
    read_errors = []
    nodes, ways, relations = _read_osm(root, read_errors)
    cache, why = _nodecache.lookup_with_reason(hashlib.sha256(data).hexdigest(), projector, sorted(nodes))
    if cache is None and sys.platform == 'emscripten':
        # In the browser the C library rounds a few sin/cos/sinh results differently from the one the real Lanelet2
        # uses, so projected nodes can move by a nanometre and a near tie (centerline, nearest lanelet) can flip.
        _log.warning('lanelet2 stand-in: no node cache used (%s). Map nodes are projected in the browser, so '
                     'results may differ slightly from the real Lanelet2. tests/lanelet2/make_node_cache.py makes a '
                     'cache.', why)
    loader = _Loader(projector)
    m = loader.build(nodes, ways, relations, cache)
    m._node_cache_used = cache is not None  # stand-in extension, for tests and the report
    for table in (nodes, ways, relations):
        for pid in table:
            registerId(pid)
    errors = read_errors + loader.errors
    if errors:
        errors = ['Errors ocurred while parsing Lanelet Map:'] + ['\t- ' + e for e in errors]
    return m, errors


def load(filename, projector=None):
    """Load an OSM map. filename may also be the file contents as bytes (stand-in extension, for the browser)."""
    m, errors = _load(filename, projector)
    if errors:
        raise ParseError('\n'.join(errors))
    return m


def loadRobust(filename, projector):
    return _load(filename, projector)
