"""
Test trajectory_controller's E-stop, speed sign, obstacle checks and controller choices.

/trigger_estop leads to ESTOP: the 0x100 frame and full brake, latched. The speed is negative
in BRAKING_TO_FORWARD. A stale obstacle reading no longer causes SAFE_STOP, and MANUAL checks
behind the shuttle while it reverses. lateral_controller picks the path tracker, and the
filters get the braking they plan with. Messages go straight into the callbacks on a
controlled clock; the loops and services are called by hand; CAN and telemetry publishers are
recorded (no DDS round trips). The ROS context runs on its own domain (CONTROL_TEST_DOMAIN,
default 84), never the default one.
"""

import math
import os

from diagnostic_updater import DiagnosticStatusWrapper
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from nift_common.can_protocol import (
    CAN_CMD_MAX,
    CAN_ID_COMMAND,
    CAN_ID_ESTOP,
    decode_command_frame,
    decode_estop_frame,
    DLC_ESTOP,
    Gear,
)
from nift_control.controllers import PurePursuitController, StanleyController
from nift_control.dbw_interface import VehicleMode
from nift_control.filters import KinematicCBFFilter, PotentialFieldAvoidanceFilter
from nift_control.filters.filter_utils import BRAKE_DECEL_MPS2, BRAKE_DELAY_S, VEHICLE_RADIUS
from nift_control.modes import ControlMode
from nift_control.trajectory_controller import TrajectoryController
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init(domain_id=int(os.environ.get('CONTROL_TEST_DOMAIN', '84')))
    yield
    rclpy.shutdown()


class Recorder:

    def __init__(self):
        self.items = []

    def publish(self, x):
        self.items.append(x)


class Clock:
    ns = 1_000_000_000

    def now(self):
        return Time(nanoseconds=self.ns)


@pytest.fixture
def make():
    nodes = []

    def _make(**params):
        params.setdefault('filter_type', 'none')    # no /obstacles watchdog unless a test asks
        node = TrajectoryController(
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        node.path_timer.cancel()
        node.actuate_timer.cancel()
        node.can, node.tele = Recorder(), Recorder()
        node.can_pub, node.telemetry_pub, node.marker_pub = node.can, node.tele, Recorder()
        node.clock = Clock()
        node.get_clock = lambda: node.clock
        node._last_actuate_time = node.clock.now()
        nodes.append(node)
        return node
    yield _make
    for n in nodes:
        n.destroy_node()


# ---------------------------------------------------------------- inputs and outputs

def gnss_vel(node, v=0.0):
    t = TwistStamped()
    t.twist.linear.x = float(v)
    node._velocity_callback(t)


def load_path(node, length=100.0):
    """Load a straight path along +x, and the TF map → base_link at the origin heading +x."""
    msg = Path()
    for i in range(int(length / 2.0) + 1):
        p = PoseStamped()
        p.pose.position.x = 2.0 * i
        msg.poses.append(p)
    node._path_callback(msg)
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = 'map', 'base_link'
    t.transform.rotation.w = 1.0
    node.tf_buffer.set_transform(t, 'test')


def call(node, service):
    return getattr(node, service)(None, Trigger.Response())


def tick(node, v=0.0, dt=0.05):
    """Feed a fresh /gnss/vel, then run the 20 Hz actuation loop once."""
    node.clock.ns += round(dt * 1e9)
    gnss_vel(node, v)
    node._actuate_loop()


def obstacles(node, points, stamp=1):
    """Send one /obstacles polygon in base_link (an empty list clears)."""
    msg = PolygonStamped()
    msg.header.stamp.sec = stamp
    msg.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in points]
    node._obstacle_callback(msg)


def status_lines(node):
    return [m.data.splitlines()[0] for m in node.tele.items]


def diag(node):
    """Return the diagnostics values, the level as an int and the message."""
    stat = DiagnosticStatusWrapper()
    node._diagnostics_callback(stat)
    level = stat.level[0] if isinstance(stat.level, (bytes, bytearray)) else int(stat.level)
    return {kv.key: kv.value for kv in stat.values}, level, stat.message


# ================================================================ E-stop

@pytest.mark.parametrize('enter', ['idle', 'autonomous', 'manual', 'safe_stop'])
def test_trigger_estop_sends_the_estop_frame_and_holds_the_brake(make, enter):
    node = make()
    load_path(node)
    gnss_vel(node, 1.0)
    if enter == 'autonomous':
        assert call(node, '_start_nav_callback').success
    elif enter == 'manual':
        assert call(node, '_toggle_manual_callback').success
    elif enter == 'safe_stop':
        node._request_safe_stop()
    tick(node, 1.0)
    n = len(node.can.items)
    r = call(node, '_estop_service_callback')
    assert r.success and node._mode == ControlMode.ESTOP
    assert 'restart this node, then power-cycle the shuttle' in r.message
    sent = node.can.items[n:]
    # The E-stop frame as the protocol defines it: 0x100, DLC 1, byte 0 = 1, a standard data frame
    estop = sent[0]
    assert (estop.id, estop.dlc, list(estop.data)) == (CAN_ID_ESTOP, DLC_ESTOP, [1] + [0] * 7)
    assert not (estop.is_extended or estop.is_rtr or estop.is_error)
    assert decode_estop_frame(estop) is True
    stop = decode_command_frame(sent[1])
    assert (stop.gear, stop.brake, stop.pedal) == (Gear.NEUTRAL, CAN_CMD_MAX, 0)
    assert len(sent) == 2 and status_lines(node)[-1] == 'STATUS:    ESTOP'


def test_estop_resends_both_frames_every_tick(make):
    node = make()
    call(node, '_estop_service_callback')
    n = len(node.can.items)
    for _ in range(5):
        tick(node, 0.5)
    sent = node.can.items[n:]
    assert [f.id for f in sent] == [CAN_ID_ESTOP, CAN_ID_COMMAND] * 5
    assert all(decode_estop_frame(f) for f in sent[0::2])
    stops = [decode_command_frame(f) for f in sent[1::2]]
    assert all(f.brake == CAN_CMD_MAX and f.gear == Gear.NEUTRAL for f in stops)
    assert all(f.header.stamp.sec == 1 for f in sent)               # stamped with the node clock
    node._path_loop()
    assert status_lines(node)[-1] == 'STATUS:    ESTOP'             # dashboards stay live


def test_estop_is_a_latch(make):
    """Stay in ESTOP whatever happens: /resume, a start, MANUAL, a watchdog, a cleared path."""
    node = make()
    load_path(node)
    gnss_vel(node, 0.0)
    call(node, '_estop_service_callback')
    for service in ('_resume_callback', '_start_nav_callback', '_toggle_manual_callback'):
        r = call(node, service)
        assert not r.success and 'E-STOPPED' in r.message and 'power-cycle' in r.message
        assert node._mode == ControlMode.ESTOP
    node._request_safe_stop()
    node._set_mode(ControlMode.IDLE)
    node._set_mode(ControlMode.AUTONOMOUS)
    node._path_callback(Path())
    node.clock.ns += 2_000_000_000                                  # GNSS stale for 2 s
    node._actuate_loop()
    assert node._mode == ControlMode.ESTOP
    r = call(node, '_estop_service_callback')
    assert r.success and r.message.startswith('Already E-STOPPED')
    values, level, message = diag(node)
    assert values['Control Mode'] == 'ESTOP' and level == 2 and 'power-cycle' in message


def test_estop_steering_follows_the_safe_stop_rule(make):
    """Keep the last steering sent in the stop frames with safe_stop_hold_steering on."""
    node = make(safe_stop_hold_steering=True)
    gnss_vel(node, 1.0)
    assert call(node, '_toggle_manual_callback').success
    node._manual_delta = math.radians(8.0)
    tick(node, 1.0)
    n = len(node.can.items)
    call(node, '_estop_service_callback')
    tick(node, 1.0)
    stops = [decode_command_frame(f) for f in node.can.items[n:] if f.id == CAN_ID_COMMAND]
    assert len(stops) == 2
    assert all(f.steering_angle_deg == pytest.approx(8.0, abs=0.006) for f in stops)
    assert diag(node)[0]['Safe Stop Steering'] == 'held at +8.00 deg'


# ================================================================ /safe_stop (soft, resumable)

SAFE_STOP_ENGAGED = 'SAFE STOP engaged. Call /resume to return to IDLE.'


@pytest.mark.parametrize('enter', ['autonomous', 'manual'])
def test_safe_stop_from_driving_brakes_and_holds_like_a_watchdog(make, enter):
    node = make()
    load_path(node)
    gnss_vel(node, 1.0)
    assert call(node, '_start_nav_callback' if enter == 'autonomous'
                else '_toggle_manual_callback').success
    tick(node, 1.0)
    n = len(node.can.items)
    r = call(node, '_safe_stop_callback')
    assert (r.success, r.message) == (True, SAFE_STOP_ENGAGED)
    assert node._mode == ControlMode.SAFE_STOP
    for _ in range(3):
        tick(node, 0.5)
    sent = node.can.items[n:]
    assert sent and all(f.id == CAN_ID_COMMAND for f in sent)      # no E-stop frame
    stops = [decode_command_frame(f) for f in sent]
    assert all((f.gear, f.brake, f.pedal) == (Gear.NEUTRAL, CAN_CMD_MAX, 0) for f in stops)
    assert status_lines(node)[-1] == 'STATUS:    SAFE_STOP'
    r = call(node, '_resume_callback')                              # Resume becomes available
    assert r.success and node._mode == ControlMode.IDLE


def test_safe_stop_in_safe_stop_changes_nothing(make):
    node = make()
    node._request_safe_stop()
    n = len(node.can.items)
    r = call(node, '_safe_stop_callback')
    assert r.success and r.message == 'Already in SAFE STOP. Call /resume to return to IDLE.'
    assert node._mode == ControlMode.SAFE_STOP and len(node.can.items) == n


@pytest.mark.parametrize('after_drive', [False, True])
def test_safe_stop_in_idle_stays_idle(make, after_drive):
    node = make()
    gnss_vel(node, 0.0)
    if after_drive:                                                 # IDLE that holds the brake
        assert call(node, '_toggle_manual_callback').success
        assert call(node, '_toggle_manual_callback').success
    n = len(node.can.items)
    r = call(node, '_safe_stop_callback')
    assert (r.success, r.message) == (True, 'Already stopped (IDLE).')
    assert node._mode == ControlMode.IDLE and len(node.can.items) == n


def test_safe_stop_in_estop_is_refused(make):
    node = make()
    call(node, '_estop_service_callback')
    n = len(node.can.items)
    r = call(node, '_safe_stop_callback')
    assert not r.success
    assert r.message == ('Cannot safe-stop: Vehicle is E-STOPPED. Restart this node, then '
                         'power-cycle the shuttle, to recover.')
    assert node._mode == ControlMode.ESTOP and len(node.can.items) == n


@pytest.mark.parametrize('enter', ['idle', 'autonomous', 'manual'])
def test_resume_only_from_safe_stop(make, enter):
    node = make()
    load_path(node)
    gnss_vel(node, 0.0)
    if enter == 'autonomous':
        assert call(node, '_start_nav_callback').success
    elif enter == 'manual':
        assert call(node, '_toggle_manual_callback').success
    mode = node._mode
    r = call(node, '_resume_callback')
    assert not r.success and r.message.startswith('Not in SAFE_STOP')
    assert node._mode == mode


def test_safe_stop_service_is_offered(make):
    node = make()
    names = {srv.srv_name.lstrip('/') for srv in node.services}    # relative names, at the root
    assert {'safe_stop', 'trigger_estop', 'resume'} <= names


# ================================================================ speed sign

@pytest.mark.parametrize('state, sign', [
    (VehicleMode.FORWARD, 1), (VehicleMode.BRAKING_TO_REVERSE, 1),
    (VehicleMode.REVERSE, -1), (VehicleMode.BRAKING_TO_FORWARD, -1),
])
@pytest.mark.parametrize('source', ['gnss', 'filter'])
def test_speed_is_negative_while_braking_to_forward(make, state, sign, source):
    """Feed ADRC a negative speed in BRAKING_TO_FORWARD, which still rolls backwards."""
    node = make(localization_source=source)
    node.bridge.state = state
    if source == 'gnss':
        gnss_vel(node, 0.9)
    else:
        o = Odometry()
        o.twist.twist.linear.x = 0.9
        node._loc_odom_callback(o)
    assert node.state.vel == sign * 0.9


def test_manual_reverse_then_forward_keeps_the_speed_continuous(make):
    """
    Reverse at about 1 m/s in MANUAL, then ask for forward.

    Through BRAKING_TO_FORWARD the speed fed to ADRC shrinks towards 0 from below, with no
    jump to +1 m/s.
    """
    node = make()
    gnss_vel(node, 0.0)
    assert call(node, '_toggle_manual_callback').success
    node._manual_v = -1.0
    for _ in range(3):
        tick(node, 1.0)
    assert node.bridge.state == VehicleMode.REVERSE and node.state.vel == -1.0
    node._manual_v = 1.0
    seen = []
    for v in (1.0, 0.8, 0.5, 0.2):
        tick(node, v)
        seen.append((node.bridge.state, node.state.vel))
    assert all(s == VehicleMode.BRAKING_TO_FORWARD for s, _ in seen[1:])
    assert [v for _, v in seen[1:]] == [-0.8, -0.5, -0.2]
    last = [decode_command_frame(f) for f in node.can.items[-3:]]
    assert all(f.gear == Gear.REVERSE and f.pedal == 0 for f in last)


# ================================================================ stale obstacle reading

def test_no_safe_stop_from_a_stale_reading_after_the_obstacle_is_gone(make):
    """
    Start again after /resume once the obstacle is gone.

    The potential field filter kept its last close distance once the obstacles cleared, so the
    start after /resume went straight back to SAFE_STOP.
    """
    node = make(filter_type='potential_field')
    load_path(node)
    gnss_vel(node, 0.0)
    obstacles(node, [(VEHICLE_RADIUS + 0.5, 0.0)], stamp=1)         # 0.5 m ahead of the edge
    assert call(node, '_start_nav_callback').success
    node._path_loop()
    assert node._mode == ControlMode.SAFE_STOP
    obstacles(node, [], stamp=2)                                    # gone: lidar_node's clear
    assert call(node, '_resume_callback').success
    gnss_vel(node, 0.0)
    assert call(node, '_start_nav_callback').success
    node._path_loop()
    assert node._mode == ControlMode.AUTONOMOUS
    assert node.safety_filter.last_obs_dist == float('inf')


# ================================================================ MANUAL while reversing

def manual_with_obstacle(make, x):
    node = make()
    load_path(node)
    gnss_vel(node, 0.0)
    assert call(node, '_toggle_manual_callback').success
    node._obstacles = np.array([[x, 0.0], [x, 0.3]])                # map = base_link here
    return node


@pytest.mark.parametrize('manual_v, bridge_state, stops', [
    (-0.5, VehicleMode.STANDBY, True),          # asked to reverse
    (0.0, VehicleMode.REVERSE, True),           # still rolling back in reverse gear
    (0.5, VehicleMode.BRAKING_TO_FORWARD, True),
    (0.5, VehicleMode.FORWARD, False),          # driving away from it
    (0.0, VehicleMode.STANDBY, False),
])
def test_manual_checks_behind_while_reversing(make, manual_v, bridge_state, stops):
    node = manual_with_obstacle(make, -(VEHICLE_RADIUS + 0.5))      # 0.5 m behind the edge
    node._manual_v = manual_v
    node.bridge.state = bridge_state
    node._path_loop()
    assert (node._mode == ControlMode.SAFE_STOP) == stops


@pytest.mark.parametrize('manual_v', [0.5, -0.5])
def test_manual_still_checks_ahead_both_ways(make, manual_v):
    node = manual_with_obstacle(make, VEHICLE_RADIUS + 0.5)
    node._manual_v = manual_v
    node._path_loop()
    assert node._mode == ControlMode.SAFE_STOP


# ================================================================ lateral_controller

@pytest.mark.parametrize('params, cls', [
    ({}, PurePursuitController),
    ({'lateral_controller': 'pure_pursuit'}, PurePursuitController),
    ({'lateral_controller': 'stanley'}, StanleyController),
])
def test_lateral_controller_parameter(make, params, cls):
    node = make(**params)
    assert node.lateral_ctrl is None
    load_path(node)
    assert type(node.lateral_ctrl) is cls
    gnss_vel(node, 0.0)
    assert call(node, '_start_nav_callback').success
    node._path_loop()                                               # it steers
    assert node._mode == ControlMode.AUTONOMOUS and math.isfinite(node._delta_safe)


def test_unknown_lateral_controller_refuses_to_start(make):
    with pytest.raises(ValueError, match='lateral_controller'):
        make(lateral_controller='mpc')


# ================================================================ braking the filters plan with

def test_filters_get_the_measured_braking_by_default(make):
    node = make(filter_type='cbf')
    f = node.safety_filter
    assert isinstance(f, KinematicCBFFilter)
    assert (f._brake_decel, f._brake_delay) == (BRAKE_DECEL_MPS2, BRAKE_DELAY_S)
    node = make(filter_type='potential_field', brake_decel_mps2=0.4, brake_delay_s=0.9)
    f = node.safety_filter
    assert isinstance(f, PotentialFieldAvoidanceFilter)
    assert (f._brake_decel, f._brake_delay) == (0.4, 0.9)


@pytest.mark.parametrize('params', [{'brake_decel_mps2': 0.0}, {'brake_delay_s': -0.1}])
def test_bad_braking_refuses_to_start(make, params):
    with pytest.raises(ValueError, match='brake_'):
        make(**params)
