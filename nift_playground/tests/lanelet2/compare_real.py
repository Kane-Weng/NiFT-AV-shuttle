#!/usr/bin/env python3
"""Check the Lanelet2 stand-in against the real Lanelet2 and refresh the test fixtures.

The real side runs lanelet2_dump.py in Docker (image nift_pub_deps:humble, which has Lanelet2 1.2.3). The stand-in
side runs the same script natively. The results are compared (equivalence.py), printed, and the real side is saved
as fixtures/<map>.json.gz, which test_equivalence.py uses so CI needs no Docker.

    python3 nift_playground/tests/lanelet2/compare_real.py            # both maps
    python3 nift_playground/tests/lanelet2/compare_real.py odr2024    # one map
    python3 nift_playground/tests/lanelet2/compare_real.py --no-save  # compare only
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import equivalence as eq  # noqa: E402

IMAGE = 'nift_pub_deps:humble'


def run_real(queries, workdir):
    qpath = os.path.join(workdir, 'queries.json')
    with open(qpath, 'w') as f:
        json.dump(queries, f)
    cmd = ['docker', 'run', '--rm', '--user', '%d:%d' % (os.getuid(), os.getgid()),
           '-v', '%s:/ws:ro' % eq.REPO, '-v', '%s:/out' % workdir, IMAGE, 'bash', '-c',
           'source /opt/ros/humble/setup.bash && python3 /ws/nift_playground/tests/lanelet2/lanelet2_dump.py '
           '/out/queries.json /out/real.json /ws/%s' % queries['map']]
    subprocess.run(cmd, check=True)
    with open(os.path.join(workdir, 'real.json')) as f:
        return json.load(f)


def run_stand_in(queries, workdir):
    qpath = os.path.join(workdir, 'queries.json')
    env = dict(os.environ, PYTHONPATH=eq.PY_DIR)
    subprocess.run([sys.executable, os.path.join(HERE, 'lanelet2_dump.py'), qpath,
                    os.path.join(workdir, 'stand.json'), eq.map_path(queries['name'])], check=True, env=env)
    with open(os.path.join(workdir, 'stand.json')) as f:
        return json.load(f)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    save = '--no-save' not in sys.argv
    names = args or list(eq.MAPS)
    sys.path.insert(0, eq.PY_DIR)
    import lanelet2
    assert getattr(lanelet2, 'IS_STAND_IN', False), 'expected the stand-in on sys.path'
    ok = True
    for name in names:
        queries = eq.make_queries(name, lanelet2)
        with tempfile.TemporaryDirectory() as wd:
            real = run_real(queries, wd)
            stand = run_stand_in(queries, wd)
        assert real['is_stand_in'] is False and stand['is_stand_in'] is True
        stats, fails = eq.compare(real, stand, queries, eq._CostOracle(lanelet2, queries))
        print('== %s' % name)
        for k, v in stats.items():
            print('  %-30s %s' % (k, v))
        print('  %-30s real %s' % ('timing', {k: round(v, 3) for k, v in real['timing'].items()}))
        print('  %-30s stand-in %s' % ('', {k: round(v, 3) for k, v in stand['timing'].items()}))
        if fails:
            ok = False
            print('  %d FAILURES' % len(fails))
            for f in fails[:40]:
                print('   - ' + f)
        else:
            print('  all checks pass')
        if save:
            print('  fixture: %s' % eq.save_fixture(name, queries, real))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
