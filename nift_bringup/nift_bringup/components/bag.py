"""
Launch component that records a rosbag of the run.

File: bag.py

Description:
  Component `bag`: ros2 bag record into <repo>/nift_bags/<profile>_<date> (~/nift_ws/src/nift_bags;
  git-ignored). `exec` so Ctrl-C reaches the recorder itself and the bag is closed properly.

  Check
    free disk space where the bags go: FAIL under MIN_FAIL_GB, WARN under MIN_WARN_GB.
"""

from pathlib import Path
import shlex
import shutil

from launch.actions import ExecuteProcess

from .base import Component
from .common import FAIL, OK, repo_root, WARN

TOPICS = ['/gnss/fix', '/gnss/heading_deg', '/gnss/vel', '/gnss/raw/front', '/gnss/raw/rear',
          '/gnss/hygiene',
          '/localization/odom', '/localization/status',          # filter (ekf | shadow)
          '/to_can_bus', '/from_can_bus',
          '/obstacles', '/planned_path', '/nav_telemetry', '/teleop_cmd_vel', '/long_test/state',
          '/diagnostics', '/rosout']
SIM_TOPICS = ['/clock', '/ackermann_like_controller/odom']     # + bridge ground truth
MIN_FAIL_GB, MIN_WARN_GB = 1.0, 5.0


def bag_dir() -> Path:
    """<repo>/nift_bags (common.repo_root: this checkout, else ~/nift_ws/src)."""
    return repo_root() / 'nift_bags'


class Bag(Component):
    name = 'bag'

    def check(self, plan, graph):
        where = bag_dir()
        existing = next(p for p in (where, *where.parents) if p.exists())
        free_gb = shutil.disk_usage(existing).free / 1e9
        if free_gb < MIN_FAIL_GB:
            return [(FAIL, f'{free_gb:.1f} GB free for {where}')]
        return [(WARN if free_gb < MIN_WARN_GB else OK, f'{free_gb:.0f} GB free for {where}')]

    def actions(self, plan):
        where = shlex.quote(str(bag_dir()))
        topics = ' '.join(TOPICS + (SIM_TOPICS if plan.sim else []))
        return [ExecuteProcess(
            cmd=['bash', '-c', f'mkdir -p {where} && exec ros2 bag record '
                               f'-o {where}/{plan.profile}_$(date +%Y%m%d_%H%M%S) {topics}'],
            output='screen')]
