"""
Check levels and small launch helpers shared by the component modules.

File: common.py

Description:
  Shared by the component modules: check result levels and small launch helpers.

  Check levels (preflight.py stops the launch on any FAIL)
    OK    checked and fine
    WARN  worth knowing, does not stop the launch
    FAIL  the component cannot work; fix it, or start anyway with checks:=false
    SKIP  cannot be checked before launch (e.g. sim: the CARLA bridge provides it)
  (NONE is only the placeholder row for a component that has no checks.)
"""

from enum import Enum
import os
from pathlib import Path
import socket
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml


class Level(Enum):
    """Result of one preflight check; .value is what the report prints."""

    OK = 'OK'
    WARN = 'WARN'
    FAIL = 'FAIL'
    SKIP = 'SKIP'
    NONE = '--'


OK, WARN, FAIL, SKIP = Level.OK, Level.WARN, Level.FAIL, Level.SKIP


def share(pkg, *rel):
    return os.path.join(get_package_share_directory(pkg), *rel)


def repo_root() -> Path:
    """
    Return the checkout this file comes from, else ~/nift_ws/src.

    colcon --symlink-install resolves back into src, as bootstrap.sh builds.
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / '.git').exists():
            return parent
    return Path('~/nift_ws/src').expanduser()


def carla_octane():
    """
    Return the CARLA bridge's Octane emulator settings.

    Keys: port, api_key, namespace, front_id, rear_id.
    """
    with open(share('nift_carla', 'config', 'carla_params.yaml')) as f:
        return yaml.safe_load(f)['carla_bridge']['ros__parameters']['octane']


def port_free(port, host='0.0.0.0'):
    """
    Return True if a TCP server could bind host:port now.

    Binds with SO_REUSEADDR, as the servers do (aiohttp for the Octane emulator, rosbridge):
    connections of a server stopped seconds ago, still in TIME_WAIT, do not block it; only a
    socket that listens does. Without it the check failed for ~60 s after every sim run.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, int(port)))
            return True
        except OSError:
            return False


def sim_time(plan):
    return {'use_sim_time': plan.sim}


def node_params(node_name, params):
    """
    Write a params file scoped to `node_name`, for values a node-scoped yaml also sets.

    launch_ros writes a dict parameter as a `/**` wildcard file, and rcl drops a wildcard value
    for a parameter that a node-scoped file (planning.yaml) already sets, whatever the file
    order; a node-scoped file passed later does override it (verified 2026-10-08:
    `filter_type none` never reached trajectory_controller through a dict). Only needed for
    keys the yaml defines.
    """
    f = tempfile.NamedTemporaryFile('w', prefix=f'{node_name}_params_', suffix='.yaml',
                                    delete=False)
    yaml.safe_dump({node_name: {'ros__parameters': params}}, f)
    f.close()
    return f.name


def map_origin(plan):
    return {'map_origin_lat': float(plan.args['map_origin_lat']),
            'map_origin_lon': float(plan.args['map_origin_lon'])}


def once(marker, actions):
    """
    Make an OnProcessIO callback that fires once.

    It returns `actions` the first time `marker` appears in a process's output.
    """
    fired = [False]

    def callback(event):
        if not fired[0] and marker in event.text.decode(errors='replace'):
            fired[0] = True
            return actions
        return None
    return callback
