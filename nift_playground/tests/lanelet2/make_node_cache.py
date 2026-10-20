#!/usr/bin/env python3
"""Write the node cache for a map: every node's x, y as the real Lanelet2 projects it (UtmProjector, origin).

Why: the stand-in repeats GeographicLib's UTM maths step by step, but sin, cos, sinh and friends come from the C
library, and Pyodide's (musl) rounds a few results one unit in the last place differently from glibc. A nanometre is
enough to flip a near tie in Lanelet2's centerline algorithm (one lanelet of odr2024 then moves 7 cm). With the real
coordinates cached, everything after the projection uses only + - * / and sqrt, which give the same bits everywhere.

    python3 nift_playground/tests/lanelet2/make_node_cache.py                      # odr2024, site origin
    python3 nift_playground/tests/lanelet2/make_node_cache.py MAP.osm LAT LON

Runs Lanelet2 in Docker (nift_pub_deps:humble) and writes py/fakelanelet2/lanelet2/node_cache/<sha256[:16]>.bin.
"""
import hashlib
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', '..', 'py', 'fakelanelet2')))

REAL_SIDE = r'''
import sys, struct, lanelet2
from lanelet2.projection import UtmProjector
path, lat, lon, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
m = lanelet2.io.load(path, UtmProjector(lanelet2.io.Origin(lat, lon)))
ids = sorted(p.id for p in m.pointLayer)
with open(out, 'wb') as f:
    for i in ids:
        p = m.pointLayer[i]
        f.write(struct.pack('<qdd', i, p.x, p.y))
'''


def main():
    from lanelet2 import _nodecache
    args = sys.argv[1:]
    if args:
        map_path, lat, lon = os.path.abspath(args[0]), float(args[1]), float(args[2])
    else:
        map_path, lat, lon = os.path.join(REPO, 'maps', 'lanelet2_odr2024.osm'), 42.3005, -83.6987
    rel = os.path.relpath(map_path, REPO)
    with open(map_path, 'rb') as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    with tempfile.TemporaryDirectory() as wd:
        with open(os.path.join(wd, 'real.py'), 'w') as f:
            f.write(REAL_SIDE)
        subprocess.run(['docker', 'run', '--rm', '--user', '%d:%d' % (os.getuid(), os.getgid()),
                        '-v', '%s:/ws:ro' % REPO, '-v', '%s:/out' % wd, 'nift_pub_deps:humble', 'bash', '-c',
                        'source /opt/ros/humble/setup.bash && python3 /out/real.py /ws/%s %r %r /out/nodes.bin'
                        % (rel, lat, lon)], check=True)
        with open(os.path.join(wd, 'nodes.bin'), 'rb') as f:
            raw = f.read()
    path = _nodecache.write(sha, lat, lon, raw)
    print('wrote %s (%d nodes, %d bytes)' % (path, len(raw) // 24, os.path.getsize(path)))


if __name__ == '__main__':
    main()
