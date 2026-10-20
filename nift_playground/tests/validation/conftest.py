"""
Fixtures for the comparison with the ROS reference runs.

Every run is the real CLI (`python3 -m playground.cli`) in its own process, all of a set at once. Runs are cached per pytest session. The slow set (all 16 cases, twice) runs only with NIFT_PLAYGROUND_SLOW=1.
"""

import concurrent.futures
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.normpath(os.path.join(HERE, '..', '..', 'py'))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from cases import CASES, FAST, MODES, SLOW  # noqa: E402

SLOW_ON = os.environ.get('NIFT_PLAYGROUND_SLOW') == '1'


def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: needs NIFT_PLAYGROUND_SLOW=1 (all reference cases, about a minute)')


def pytest_collection_modifyitems(config, items):
    if SLOW_ON:
        return
    skip = pytest.mark.skip(reason='slow: set NIFT_PLAYGROUND_SLOW=1')
    for item in items:
        if 'slow' in item.keywords:
            item.add_marker(skip)


def _run(label, mode, out_root):
    route, extra = CASES[label]
    out = os.path.join(out_root, mode, label)
    entry = ['-m', 'playground.cli'] if mode == 'shipped' else [os.path.join(HERE, 'ros_order.py')]
    cmd = [sys.executable, *entry, '--route', route, *extra, '--out', out]
    env = {k: v for k, v in os.environ.items() if k not in ('PYTHONPATH', 'AMENT_PREFIX_PATH')}
    p = subprocess.run(cmd, cwd=PY, env=env, capture_output=True, text=True, timeout=900)
    # exit 0 arrived, 1 another outcome (stall, SAFE_STOP), 2 setup failed
    return out, p.returncode, p.stdout[-2000:] + p.stderr[-4000:]


class Runs:
    def __init__(self, root):
        self.root = root
        self.done = {}

    def ensure(self, labels):
        todo = [(lab, m) for lab in labels for m in MODES if (lab, m) not in self.done]
        if not todo:
            return
        workers = max(1, min(len(todo), os.cpu_count() or 2))
        with concurrent.futures.ThreadPoolExecutor(workers) as ex:
            futs = {ex.submit(_run, lab, m, self.root): (lab, m) for lab, m in todo}
            for f in concurrent.futures.as_completed(futs):
                self.done[futs[f]] = f.result()

    def get(self, label, mode):
        self.ensure(FAST + (SLOW if SLOW_ON else ()))
        self.ensure((label,))
        out, code, log = self.done[(label, mode)]
        assert code in (0, 1), f'{mode} {label}: the CLI failed (exit {code})\n{log}'
        return out


@pytest.fixture(scope='session')
def playground_runs(tmp_path_factory):
    return Runs(str(tmp_path_factory.mktemp('playground_runs')))
