"""
Shared setup for the fakeros tests: sys.path as ARCH section 1 gives it, and a fresh world per test.

The expected values in these tests come from Humble (rclpy, tf2_ros, diagnostic_updater and
launch_ros in nift_pub_deps:humble, probed on 2026-10-20), unless a test says it checks a
deliberate fakeros choice.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PLAYGROUND = os.path.dirname(os.path.dirname(HERE))
REPO = os.path.dirname(PLAYGROUND)
PY = os.path.join(PLAYGROUND, 'py')

_ROOTS = [os.path.join(PY, p) for p in ('fakeros', 'fakelanelet2', 'fakesocketio', 'stubs')] + [PY]
_ROOTS += [os.path.join(REPO, p) for p in ('nift_common', 'nift_sensor', 'nift_waypoint', 'nift_control',
                                            'nift_simulation/nift_carla', 'nift_bringup')]
for _p in reversed(_ROOTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fakeros  # noqa: E402
fakeros.install()

START_NS = 1_000_000_000
DT_NS = 50_000_000


@pytest.fixture
def world():
    from fakeros._core import world as w
    w.reset(start_ns=START_NS - DT_NS)
    logs = []
    w.log_sink = lambda level, name, text: logs.append((level, name, text))
    w.logs = logs
    yield w
    w.reset()


@pytest.fixture
def node(world):
    from rclpy.node import Node
    return Node('test_node')
