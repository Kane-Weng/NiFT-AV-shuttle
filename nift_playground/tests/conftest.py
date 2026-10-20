"""
Shared pytest setup for the playground's native tests.

Run them with `python3 -m pytest nift_playground/tests` and no ROS sourced: the stand-ins in `nift_playground/py` must shadow rclpy, lanelet2 and socketio. This file only puts `nift_playground/py` on sys.path and offers fixtures. Nothing here is autouse, so each test folder keeps its own path setup.
"""

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.normpath(os.path.join(HERE, '..', 'py'))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
if PY not in sys.path:
    sys.path.insert(0, PY)


@pytest.fixture(scope='session')
def repo_root():
    return REPO


@pytest.fixture(scope='session')
def playground(repo_root):
    """paths.setup(): the stand-ins and the repo packages on sys.path, fakeros installed. Returns the repo root."""
    from playground import paths
    return paths.setup(repo_root)
