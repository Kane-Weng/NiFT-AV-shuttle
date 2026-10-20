"""Projected map nodes, as the real Lanelet2 computed them, for maps that ship a cache (see make_node_cache.py).

io.load() uses the cache when the map file's sha256, the origin and the projector settings match, and projects with
_utm otherwise. The cache removes the last source of difference to Lanelet2 in the browser: Pyodide's C library rounds
a few sin/cos/sinh results one unit in the last place differently, which can flip a near tie in the centerline
algorithm. Set LANELET2_STANDIN_NO_CACHE=1 to ignore caches.

File format: one JSON header line, then for each node in ascending id order two little-endian float64 (x, y). The
header holds the node count and a hash of the sorted ids. z is the node's 'ele' tag, as in Lanelet2.
"""
import hashlib
import json
import os
import struct

CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'node_cache')
_REC = struct.Struct('<qdd')  # what make_node_cache.py's real side writes
_XY = struct.Struct('<dd')


def _path(sha):
    return os.path.join(CACHE_DIR, sha[:16] + '.bin')


def _ids_hash(ids):
    return hashlib.sha256(','.join(str(i) for i in ids).encode()).hexdigest()


def write(sha, lat, lon, raw):
    """raw: records (int64 id, float64 x, float64 y) in ascending id order."""
    recs = list(_REC.iter_unpack(raw))
    os.makedirs(CACHE_DIR, exist_ok=True)
    header = {'map_sha256': sha, 'origin': [lat, lon], 'projector': 'UtmProjector(useOffset=True)',
              'count': len(recs), 'ids_sha256': _ids_hash(r[0] for r in recs),
              'source': 'real Lanelet2 1.2.3 in nift_pub_deps:humble'}
    path = _path(sha)
    with open(path, 'wb') as f:
        f.write((json.dumps(header) + '\n').encode())
        f.write(b''.join(_XY.pack(x, y) for _, x, y in recs))
    return path


def lookup(sha, projector, ids):
    """{node id: (x, y)} from the cache, or None. ids: the map's node ids in ascending order."""
    return lookup_with_reason(sha, projector, ids)[0]


def lookup_with_reason(sha, projector, ids):
    """(cache dict or None, why there is no cache or None). Same rules as lookup()."""
    if os.environ.get('LANELET2_STANDIN_NO_CACHE'):
        return None, 'LANELET2_STANDIN_NO_CACHE is set'
    from .projection import UtmProjector
    if type(projector) is not UtmProjector or not projector._use_offset or projector._throw_in_padding:
        return None, 'the cache is only made for UtmProjector(origin) with the default settings'
    path = _path(sha)
    if not os.path.exists(path):
        return None, 'the map ships no node cache (%s)' % os.path.basename(path)
    with open(path, 'rb') as f:
        data = f.read()
    nl = data.find(b'\n')
    if nl < 0:
        return None, 'the node cache %s has no header' % os.path.basename(path)
    header = json.loads(data[:nl].decode())
    pos = projector.origin().position
    if header.get('map_sha256') != sha or header.get('origin') != [pos.lat, pos.lon]:
        return None, 'the node cache was made for another map file or origin'
    if header.get('count') != len(ids) or header.get('ids_sha256') != _ids_hash(ids):
        return None, 'the node cache was made for other node ids'
    payload = data[nl + 1:]
    if len(payload) != len(ids) * _XY.size:
        return None, 'the node cache %s is truncated or too long (%d bytes for %d nodes)' % (
            os.path.basename(path), len(payload), len(ids))
    return dict(zip(ids, _XY.iter_unpack(payload))), None
