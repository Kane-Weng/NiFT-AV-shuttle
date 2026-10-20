"""
Launch component for the CAN bridge on the shuttle, or the sim backend in simulation.

File: can.py

Description:
  Component `can`.
    real  ros2_socketcan bridge on can_interface, plus a handler that stops the launch (once) if
          the interface cannot be opened.
    sim   the sim backend, the vehicle itself, always on. It serves GNSS as Octane does and decodes
          /to_can_bus in its virtual ECU.
            sim_backend carla      the CARLA bridge + scenario (nift_carla/carla_bridge.launch.py)
            sim_backend kinematic  kinematic_sim_node (nift_carla/kinematic_sim.launch.py): no
                                   CARLA, no LiDAR, no scenario obstacles (spawn_pose and waypoints
                                   still apply)

  Check
    real  `ip -details -json link show <iface>`: exists, is a CAN interface, UP, 500 kbit/s, not
          BUS-OFF; then 1 s of listening on a raw CAN socket. No frames is a WARN only: whether the
          DBW sends anything at all is still an open question.
    sim   a CARLA server answers on carla_host:carla_port (server vs client version compared), and
          the bridge's Octane emulator port is free (a bridge left running holds it). Kinematic:
          only the Octane port and the scenario file; WARN when the scenario has obstacles (not
          simulated).
"""

import json
import os
import subprocess
import time

from launch.actions import EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
import yaml

from ..profiles import ProfileError
from .base import Component
from .common import carla_octane, FAIL, OK, once, port_free, share, WARN

BITRATE = 500000

# ===================================
#            check (real)
# ===================================


def link_state(iface):
    """iproute2's JSON for iface, or None if it does not exist."""
    # Run `ip -details -json link show <iface>` to pull output as json
    r = subprocess.run(['ip', '-details', '-json', 'link', 'show', iface], capture_output=True,
                       text=True)

    # Interface doesn't exist -> None
    if r.returncode != 0:
        return None

    # Parse & extract the interface dictionary
    return (json.loads(r.stdout) or [None])[0]


def assess_link(link, iface):
    """
    Assess one `ip -details -json link show` entry (None = no such interface).

    Return [(level, message)]. Levels are `OK`, `WARN` and `FAIL`.
    """
    # Missing Interface
    up = f'sudo ip link set {iface} up type can bitrate {BITRATE}'
    if link is None:
        return [(FAIL, f'{iface} does not exist (adapter plugged in?), then: {up}')]

    # Wrong kind of interface
    kind = (link.get('linkinfo') or {}).get('info_kind')
    if kind not in ('can', 'vcan', 'vxcan'):
        return [(FAIL, f'{iface} is not a CAN interface ({kind or link.get("link_type")})')]

    # Interface down
    if 'UP' not in link.get('flags', []):
        return [(FAIL,
                 f'{iface} is DOWN: ' + (up if kind == 'can' else f'sudo ip link set {iface} up'))]

    # Virtual interfaces
    if kind != 'can':
        return [(OK, f'{iface} UP ({kind}, virtual)')]

    # Real `can` interface checks (Bitrate + Controller state)
    data = link['linkinfo'].get('info_data') or {}
    bitrate = (data.get('bittiming') or {}).get('bitrate')
    state = data.get('state', '?')
    out = []
    if bitrate != BITRATE:
        out.append((FAIL, f'{iface} runs at {bitrate} bit/s, expected {BITRATE}: '
                          f'sudo ip link set {iface} down && {up}'))
    if state == 'BUS-OFF':
        out.append((FAIL, f'{iface} is BUS-OFF (wiring, termination, bitrate?): '
                          f'sudo ip link set {iface} down && {up}'))
    elif state in ('ERROR-WARNING', 'ERROR-PASSIVE'):
        out.append((WARN, f'{iface} is {state}: bus errors (wiring, termination, bitrate?)'))

    return out or [(OK, f'{iface} UP, {bitrate // 1000} kbit/s, {state}')]


def sniff(iface, seconds=1.0):
    """Count the CAN frames received on iface in `seconds`."""
    import socket
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as s:
        s.bind((iface,))
        s.settimeout(0.1)
        n, end = 0, time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                s.recv(72)
                n += 1
            except socket.timeout:
                pass
        return n


# ===================================
#            check (sim)
# ===================================
def _check_carla(plan):
    host, port = plan.args['carla_host'], int(plan.args['carla_port'])
    scenario = plan.args['scenario_file']
    if scenario and not os.path.isfile(scenario):
        return [(FAIL, f'scenario_file {scenario} does not exist')]

    try:
        import carla
    except ImportError:
        return [(FAIL, 'python package carla missing: python3 -m pip install --user '
                       '-r nift_simulation/nift_carla/requirements-carla.txt')]

    try:
        client = carla.Client(host, port)
        client.set_timeout(3.0)
        server = client.get_server_version()
    except Exception:
        return [(FAIL,
                 f'no CARLA server at {host}:{port} '
                 '(docker start nift-carla, then wait for the port)')]

    # Version check
    version = client.get_client_version()
    # server == version + '-dirty' is the source build (the shuttle image)
    out = [(OK, f'CARLA {server} at {host}:{port}') if server in (version, version + '-dirty')
           else (WARN, f'CARLA server {server} but client {version}: versions should match')]

    # Port check for octane
    octane_port = carla_octane()['port']
    if not port_free(octane_port, '127.0.0.1'):
        out.append((FAIL, f'port {octane_port} (the bridge\'s Octane emulator) is in use: '
                          'another carla_bridge still running?'))
    return out


def _check_kinematic(plan):
    out = []
    scenario = plan.args['scenario_file']
    if scenario:
        try:
            with open(scenario) as f:
                obstacles = (yaml.safe_load(f) or {}).get('obstacles') or []
        except OSError:
            return [(FAIL, f'scenario_file {scenario} does not exist')]
        if obstacles:
            out.append((WARN,
                        f"the scenario's {len(obstacles)} obstacle(s) are not simulated: "
                        'the kinematic backend has no world to put them in '
                        '(its spawn_pose and waypoints are used)'))
    octane_port = carla_octane()['port']
    if not port_free(octane_port, '127.0.0.1'):
        out.append((FAIL, f'port {octane_port} (the Octane emulator) is in use: '
                          'another kinematic_sim or carla_bridge still running?'))
    return out + [(OK, 'no CARLA server needed')]


# ===================================
#          spawn pose (sim)
# ===================================
def _spawn(args):
    """
    Return the spawn pose for the bridge.

    spawn_x/_y/_yaw, else the scenario's spawn_pose, else '' (carla_params.yaml).
    """
    if args['spawn_x'] and args['spawn_y']:
        return args['spawn_x'], args['spawn_y'], args['spawn_yaw']
    if args['scenario_file']:
        try:
            with open(args['scenario_file']) as f:
                pose = (yaml.safe_load(f) or {}).get('spawn_pose')
        except OSError as e:
            raise ProfileError(f'scenario_file: {e}') from None
        if pose:
            return str(pose.get('x', 0.0)), str(pose.get('y', 0.0)), str(pose.get('yaw', 1.5708))
    return '', '', ''


class Can(Component):
    name = 'can'

    def outputs(self, plan):
        return ['/clock'] if plan.sim else ['/from_can_bus']

    def check(self, plan, graph):
        if plan.kinematic:
            return _check_kinematic(plan)
        if plan.sim:
            return _check_carla(plan)
        iface = plan.args['can_interface']
        out = assess_link(link_state(iface), iface)
        if any(level == FAIL for level, _ in out):
            return out
        try:
            n = sniff(iface)
            out.append((OK, f'{n} frames in 1 s') if n else
                       (WARN,
                        f'no frames on {iface} in 1 s (DBW off? whether it sends any is unknown)'))
        except OSError as e:
            out.append((WARN, f'cannot listen on {iface}: {e}'))
        return out

    def actions(self, plan):
        a = plan.args
        if plan.kinematic:
            x, y, yaw = _spawn(a)
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(share('nift_carla', 'launch',
                                                    'kinematic_sim.launch.py')),
                launch_arguments={
                    'map_origin_lat': a['map_origin_lat'], 'map_origin_lon': a['map_origin_lon'],
                    'spawn_x': x, 'spawn_y': y, 'spawn_yaw': yaw,
                    'time_factor': a['time_factor'], 'lockstep': a['lockstep'],
                    'gnss_feed': a['gnss_feed'], 'gnss_dead_zones': a['gnss_dead_zones'],
                    'gnss_seed': a['gnss_seed'],
                    'gnss_faults': a['gnss_faults'],
                }.items())]
        if plan.sim:
            x, y, yaw = _spawn(a)
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(share('nift_carla', 'launch',
                                                    'carla_bridge.launch.py')),
                launch_arguments={
                    'carla_host': a['carla_host'], 'carla_port': a['carla_port'],
                    'ego_blueprint': a['ego_blueprint'],
                    'scenario_file': a['scenario_file'],
                    'map_origin_lat': a['map_origin_lat'], 'map_origin_lon': a['map_origin_lon'],
                    'spawn_x': x, 'spawn_y': y, 'spawn_yaw': yaw,
                    'time_factor': a['time_factor'], 'lockstep': a['lockstep'],
                    'gnss_feed': a['gnss_feed'], 'gnss_dead_zones': a['gnss_dead_zones'],
                    'gnss_seed': a['gnss_seed'],
                    'gnss_faults': a['gnss_faults'],
                }.items())]

        iface = a['can_interface']
        # sender and receiver both report it
        down = once('Failed to set CAN socket name via ioctl()', [
            LogInfo(msg='============================================================='),
            LogInfo(msg=f"FATAL: CAN INTERFACE '{iface}' IS DOWN OR UNCONFIGURED"),
            LogInfo(msg='   Run in a new terminal (can0):'),
            LogInfo(msg=f'   sudo ip link set can0 up type can bitrate {BITRATE}'),
            LogInfo(msg='============================================================='),
            EmitEvent(event=Shutdown(reason=f'CAN interface {iface} down')),
        ])
        return [
            RegisterEventHandler(OnProcessIO(on_stderr=down)),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(share('ros2_socketcan', 'launch',
                                                 'socket_can_bridge.launch.xml')),
                launch_arguments={'interface': iface}.items()),
        ]
