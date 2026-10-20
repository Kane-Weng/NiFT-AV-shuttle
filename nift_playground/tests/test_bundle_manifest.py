"""
Tests of the Python bundle the browser loads (tools/build_bundle.py, tools/bundle_manifest.txt).

File: test_bundle_manifest.py

Description:
  The main test runs real sessions in a fresh interpreter under sys.addaudithook, records every
  repo file that is opened or imported, and fails if one of them is missing from the manifest. In
  the browser a missing file is an ImportError or a FileNotFoundError that native tests never see.
  It runs each localization mode and the real GNSS feed model, so every node and every config file
  the launch set can reach is loaded.

  The other tests check the builder's refusals (.env files, test data, paths outside the repo,
  stale manifest lines) and that the zip is the same bytes on every build.

  python3 -m pytest nift_playground/tests/test_bundle_manifest.py
"""

import importlib.util
import json
import subprocess
import sys
import textwrap
import zipfile
from pathlib import Path

import pytest

TESTS = Path(__file__).resolve().parent
PLAYGROUND = TESTS.parent
REPO = PLAYGROUND.parent


def _load_builder():
    spec = importlib.util.spec_from_file_location('build_bundle', PLAYGROUND / 'tools' / 'build_bundle.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


bb = _load_builder()

# Files a session reads from the repo that are not in the bundle on purpose.
NOT_BUNDLED = (
    'maps/',                          # the worker writes the map to /nift/maps/ itself (ARCH.md section 9)
    'nift_playground/tests/',         # the test driver
)

# A fresh interpreter, so modules pytest already imported do not hide a missing file.
AUDIT_SCRIPT = textwrap.dedent('''
    import json, os, sys
    repo = os.path.realpath(sys.argv[1])
    scenarios = json.loads(sys.argv[2])
    seen = set()

    def norm(p):
        try:
            p = os.fsdecode(p)
        except TypeError:
            return None
        p = os.path.realpath(p)
        if not p.startswith(repo + os.sep):
            return None
        rel = os.path.relpath(p, repo)
        parts = rel.split(os.sep)
        if '__pycache__' in parts:          # x/__pycache__/m.cpython-312.pyc -> x/m.py
            i = parts.index('__pycache__')
            parts = parts[:i] + [parts[-1].split('.')[0] + '.py']
        return '/'.join(parts)

    def hook(event, args):
        if event == 'open' and args and isinstance(args[0], (str, bytes, os.PathLike)):
            r = norm(args[0])
            if r and os.path.isfile(os.path.join(repo, r)):
                seen.add(r)

    sys.addaudithook(hook)
    sys.path.insert(0, os.path.join(repo, 'nift_playground', 'tests', 'pyodide'))
    import drive
    for sc in scenarios:
        drive.run(sc, repo)
    for m in list(sys.modules.values()):
        f = getattr(m, '__file__', None)
        r = norm(f) if f else None
        if r:
            seen.add(r)
    print(json.dumps(sorted(seen)))
''')


def _scenario(launch, ticks=300):
    return {'v': 1, 'launch': launch, 'ticks': ticks, 'events': [
        {'tick': 20, 'msg': {'type': 'goal', 'x': 60.0, 'y': 12.0}},
        {'tick': 40, 'msg': {'type': 'cmd', 'name': 'start'}},
    ]}


@pytest.fixture(scope='module')
def touched():
    if not (PLAYGROUND / 'py' / 'playground' / 'session.py').exists():
        pytest.skip('playground/session.py has not landed yet')
    scenarios = [
        _scenario({'spawn': '04'}),
        _scenario({'spawn': '04', 'localization': 'ekf', 'gnss_feed': 'real'}),
        _scenario({'spawn': '04', 'localization': 'shadow', 'gnss_faults': 'dropout_3s_turn'}, ticks=100),
        _scenario({'spawn': '04', 'perception': True}, ticks=100),
    ]
    out = subprocess.run([sys.executable, '-c', AUDIT_SCRIPT, str(REPO), json.dumps(scenarios)],
                         capture_output=True, text=True, cwd=str(REPO), timeout=600)
    assert out.returncode == 0, out.stderr[-4000:]
    return json.loads(out.stdout.strip().splitlines()[-1])


def test_session_reads_only_bundled_files(touched):
    bundled = set(bb.collect())
    missing = [p for p in touched if p not in bundled and not p.startswith(NOT_BUNDLED)]
    assert not missing, 'a session reads these repo files, add them to tools/bundle_manifest.txt:\n  ' + '\n  '.join(missing)


def test_every_node_was_loaded(touched):
    # Guards the audit test itself: if these are not seen, the scenarios did not run the stack.
    for f in ('nift_simulation/nift_carla/nift_carla/kinematic_sim_node.py',
              'nift_sensor/nift_sensor/beacon_node.py',
              'nift_sensor/nift_sensor/shuttle_tf_node.py',
              'nift_sensor/nift_sensor/localization_node.py',
              'nift_waypoint/nift_waypoint/waypoint_node.py',
              'nift_control/nift_control/trajectory_controller.py',
              'nift_control/config/planning.yaml',
              'nift_playground/py/fakeros/_generated/spec.json'):
        assert f in touched, f


def test_unused_manifest_entries_are_reported(touched, capsys):
    # Not a failure: an unused file only costs bytes. Printed so the manifest can be trimmed.
    unused = sorted(set(bb.collect()) - set(touched))
    with capsys.disabled():
        if unused:
            print(f'\n[bundle] {len(unused)} bundled files no session read: ' + ', '.join(unused))


def test_no_env_or_test_data_in_bundle():
    files = bb.collect()
    assert files
    assert not [f for f in files if bb.is_env_file(f)]
    assert 'nift_playground/tools/gen_msgs/golden.json' not in files
    assert 'nift_playground/py/fakeros/_generated/spec.json' in files
    assert all(not f.startswith('maps/') for f in files)


def _tiny_repo(tmp_path, files):
    for rel, text in files.items():
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    return tmp_path


def test_refuses_env_files(tmp_path):
    repo = _tiny_repo(tmp_path, {'pkg/a.py': 'x = 1\n', 'pkg/.env': 'MCITY_OCTANE_KEY=secret\n',
                                 'pkg/.env.local': 'K=v\n'})
    for line in ('pkg/*', 'pkg/.env', 'pkg/.env*', 'pkg/{a.py,.env.local}'):
        man = tmp_path / 'm.txt'
        man.write_text(line + '\n')
        with pytest.raises(bb.BundleError, match='.env'):
            bb.collect(repo, man)


def test_refuses_test_data_and_outside_paths(tmp_path):
    repo = _tiny_repo(tmp_path / 'r', {'nift_playground/tools/gen_msgs/golden.json': '{}', 'a.py': ''})
    (tmp_path / 'outside.py').write_text('')
    (repo / 'link.py').symlink_to(tmp_path / 'outside.py')
    man = tmp_path / 'm.txt'
    for line, err in (('nift_playground/tools/gen_msgs/*.json', 'test data'),
                      ('../outside.py', 'outside'),
                      ('link.py', 'outside'),
                      ('nothing/here.py', 'matches no file')):
        man.write_text(line + '\n')
        with pytest.raises(bb.BundleError, match=err):
            bb.collect(repo, man)


def test_brace_expansion():
    assert bb.expand_braces('a/{b,c}/{d,e}.py') == ['a/b/d.py', 'a/b/e.py', 'a/c/d.py', 'a/c/e.py']
    assert bb.expand_braces('plain.py') == ['plain.py']


def test_zip_is_reproducible_and_repo_relative(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    assert bb.main(['--out', str(a)]) == 0
    assert bb.main(['--out', str(b)]) == 0
    za, zb = (a / bb.ZIP_NAME).read_bytes(), (b / bb.ZIP_NAME).read_bytes()
    assert za == zb
    info = json.loads((a / bb.INFO_NAME).read_text())
    with zipfile.ZipFile(a / bb.ZIP_NAME) as zf:
        names = zf.namelist()
    assert names == bb.collect()
    assert info['files'] == len(names)
    assert 'nift_control/nift_control/trajectory_controller.py' in names
