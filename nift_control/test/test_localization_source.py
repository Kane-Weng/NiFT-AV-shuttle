"""
Test trajectory_controller's two localization sources.

`gnss` (default): the /gnss/vel watchdog and speed, SAFE_STOP steering 0. `filter`
(localization_node): /localization/status watchdog, DEGRADED speed cap, speed from
/localization/odom, starts need status OK, SAFE_STOP holds the steering. Messages go straight
into the callbacks on a controlled clock; the loops and services are called by hand; CAN and
telemetry publishers are recorded (no DDS round trips). The ROS context runs on its own domain
(CONTROL_TEST_DOMAIN, default 84), never the default one.
"""

import math
import os

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from diagnostic_updater import DiagnosticStatusWrapper
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from nift_common.can_protocol import (
    CAN_CMD_MAX,
    CAN_ID_COMMAND,
    decode_command_frame,
    Gear,
    STEER_MAX_DEG,
)
from nift_control.dbw_interface import VehicleMode
from nift_control.modes import ControlMode
from nift_control.trajectory_controller import TrajectoryController
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger

LEVEL = {
    'OK': DiagnosticStatus.OK,
    'DEGRADED': DiagnosticStatus.WARN,
    'LOST': DiagnosticStatus.ERROR,
    'STALE': DiagnosticStatus.STALE,
}
FILTER = {'localization_source': 'filter'}
# The controller's diagnostics keys at 75b808b (before localization_source): gnss must keep exactly
# these
GNSS_DIAG_KEYS = [
    'Control Mode',
    'Bridge State',
    'Active Waypoints',
    'Current WP IDX',
    'Filter Status',
    'Nominal Speed',
    'Obs Distance',
    'GNSS',
    'Obstacles',
]


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
        # no /obstacles watchdog in the way (tested elsewhere)
        params.setdefault('filter_type', 'none')
        node = TrajectoryController(
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()]
        )
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


# ---------------------------------------------------------------- inputs

def advance(node, s):
    node.clock.ns += round(s * 1e9)


def status(node, level, reason='', age_s='0.050', dist_m='0.00'):
    m = DiagnosticStatus(
        level=LEVEL[level], name='localization', message=f'{level} {reason}'.strip()
    )
    m.values = [KeyValue(key='age_s', value=age_s), KeyValue(key='dist_m', value=dist_m)]
    node._loc_status_callback(m)


def odom(node, v=0.0):
    o = Odometry()
    o.twist.twist.linear.x = float(v)
    node._loc_odom_callback(o)


def gnss_vel(node, v=0.0):
    t = TwistStamped()
    t.twist.linear.x = float(v)
    node._velocity_callback(t)


def fresh(node, level='OK', v=0.0, **kw):
    """One 50 Hz localization_node tick (status + odom) and a /gnss/vel."""
    status(node, level, **kw)
    odom(node, v)
    gnss_vel(node, v)


def load_path(node, length=100.0):
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


def start(node):
    return node._start_nav_callback(None, Trigger.Response())


def toggle_manual(node):
    return node._toggle_manual_callback(None, Trigger.Response())


def tick(node, dt=0.05, level='OK', v=0.0):
    """Feed fresh inputs, then run the 20 Hz actuation loop once."""
    advance(node, dt)
    fresh(node, level, v)
    node._actuate_loop()


def frames(node):
    return [decode_command_frame(f) for f in node.can.items if f.id == CAN_ID_COMMAND]


def diag(node):
    stat = DiagnosticStatusWrapper()
    node._diagnostics_callback(stat)
    return {kv.key: kv.value for kv in stat.values}, stat


def autonomous(node, level='OK'):
    load_path(node)
    fresh(node, 'OK')
    r = start(node)
    assert r.success, r.message
    if level != 'OK':
        fresh(node, level)
    return node


def topics(node):
    return {s.topic_name for s in node.subscriptions}


# ================================================================= gnss (default): today's
# behaviour

def test_gnss_is_the_default(make):
    node = make()
    assert node._localization_source == 'gnss' and not node._loc_filter
    assert node._safe_stop_hold_steering is False
    assert not {'/localization/status', '/localization/odom'} & topics(node)


def test_gnss_start_and_manual_need_fresh_gnss_vel(make):
    node = make()
    load_path(node)
    r = start(node)
    assert (
        not r.success and r.message == 'Cannot start: no fresh GNSS (no /gnss/vel received yet).'
    )
    r = toggle_manual(node)
    assert (
        not r.success
        and r.message == 'Cannot switch to MANUAL: no fresh GNSS (no /gnss/vel received yet).'
    )
    gnss_vel(node, 0.0)
    advance(node, 0.31)
    assert not start(node).success
    gnss_vel(node, 0.0)
    assert start(node).success and node._mode == ControlMode.AUTONOMOUS


def test_gnss_ignores_the_localization_status(make):
    """Even if a LOST status reached the callback (not subscribed with gnss), nothing reads it."""
    node = make()
    load_path(node)
    status(node, 'LOST', 'bridged too long')
    gnss_vel(node, 0.0)
    assert start(node).success
    for _ in range(10):
        advance(node, 0.05)
        status(node, 'LOST')
        gnss_vel(node, 2.0)
        node._actuate_loop()
    assert node._mode == ControlMode.AUTONOMOUS


@pytest.mark.parametrize('enter', ['autonomous', 'manual'])
def test_gnss_watchdog_on_gnss_vel(make, enter):
    node = make()
    load_path(node)
    gnss_vel(node, 1.0)
    assert (start(node) if enter == 'autonomous' else toggle_manual(node)).success
    advance(node, 0.29)
    status(node, 'OK')                       # (ignored with gnss)
    odom(node, 1.0)
    node._actuate_loop()
    assert node._mode != ControlMode.SAFE_STOP
    advance(node, 0.02)                      # /gnss/vel 0.31 s old
    node._actuate_loop()
    assert node._mode == ControlMode.SAFE_STOP


def test_gnss_speed_from_gnss_vel_signed_by_the_bridge(make):
    node = make()
    gnss_vel(node, 1.5)
    assert node.state.vel == 1.5
    node.bridge.state = VehicleMode.REVERSE
    gnss_vel(node, 1.5)
    assert node.state.vel == -1.5
    node.bridge.state = VehicleMode.FORWARD
    gnss_vel(node, -0.7)
    assert node.state.vel == 0.7


def test_gnss_safe_stop_steers_zero(make):
    node = make()
    load_path(node)
    gnss_vel(node, 1.0)
    assert start(node).success
    node._delta_safe = math.radians(10.0)
    node._actuate_loop()
    assert frames(node)[-1].steering_angle_deg == pytest.approx(10.0, abs=0.01)
    node._request_safe_stop()                # what the watchdogs call (/trigger_estop is ESTOP)
    advance(node, 0.05)
    gnss_vel(node, 1.0)
    node._actuate_loop()
    stop = frames(node)[-2:]
    assert all(f.gear == Gear.NEUTRAL and f.brake == CAN_CMD_MAX and f.pedal == 0 for f in stop)
    assert all(f.steering_angle_deg == 0.0 for f in stop)


def test_gnss_diagnostics_and_telemetry_unchanged(make):
    node = make()
    values, _ = diag(node)
    assert list(values) == GNSS_DIAG_KEYS
    node._publish_telemetry('NAVIGATING', 1.0, 2.0, 0.1)
    node._publish_telemetry('MANUAL', 1.0, 0.0, 0.0, manual=True)
    assert all('Loc:' not in m.data and len(m.data.splitlines()) == 8 for m in node.tele.items)


def test_gnss_degraded_never_caps(make):
    node = make()
    status(node, 'DEGRADED', 'bridging')
    assert node._speed_cap() is None and node._limit_speed(3.5, 0.1) == 3.5


# ================================================================= filter

def test_filter_parameters_and_subscriptions(make):
    node = make(**FILTER)
    assert node._loc_filter and node._safe_stop_hold_steering is True
    assert {'/localization/status', '/localization/odom'} <= topics(node)


def test_unknown_source_refused(make):
    with pytest.raises(ValueError, match='localization_source'):
        make(localization_source='ekf')


@pytest.mark.parametrize(
    'setup, expect',
    [
        (lambda n: None, 'no /localization/status received yet'),
        (lambda n: (status(n, 'LOST', 'uninitialised')), 'LOST uninitialised'),
        (
            lambda n: (status(n, 'DEGRADED', 'bridging', age_s='0.620', dist_m='0.90'), odom(n)),
            'DEGRADED bridging, data 0.620 s old, 0.90 m bridged; wait for OK',
        ),
        (lambda n: status(n, 'OK'), 'no /localization/odom received yet'),
        (lambda n: (status(n, 'OK'), odom(n), advance(n, 0.31)), 'localization_node down?'),
        (
            lambda n: (status(n, 'OK'), odom(n), advance(n, 0.31), status(n, 'OK')),
            'last /localization/odom 0.31 s ago',
        ),
        (lambda n: (status(n, 'STALE', 'whatever'), odom(n)), 'LOST STALE whatever'),
    ],
)
def test_filter_start_and_manual_need_status_ok(make, setup, expect):
    node = make(**FILTER)
    load_path(node)
    gnss_vel(node, 0.0)                      # fresh /gnss/vel does not help
    setup(node)
    r = start(node)
    assert (
        not r.success
        and r.message.startswith('Cannot start: localization not OK (')
        and expect in r.message
    )
    r = toggle_manual(node)
    assert not r.success and r.message.startswith('Cannot switch to MANUAL: localization not OK (')
    assert node._mode == ControlMode.IDLE


def test_filter_start_with_status_ok_without_gnss_vel(make):
    node = make(**FILTER)
    load_path(node)
    status(node, 'OK')
    odom(node, 0.0)
    assert start(node).success and node._mode == ControlMode.AUTONOMOUS


def test_filter_manual_toggle_ok_and_back(make):
    node = make(**FILTER)
    status(node, 'OK')
    odom(node)
    assert toggle_manual(node).success and node._mode == ControlMode.MANUAL
    status(node, 'LOST')                     # leaving MANUAL is never refused
    assert toggle_manual(node).success and node._mode == ControlMode.IDLE


@pytest.mark.parametrize('enter', ['autonomous', 'manual'])
def test_filter_lost_safe_stops(make, enter):
    node = make(**FILTER)
    load_path(node)
    fresh(node)
    assert (start(node) if enter == 'autonomous' else toggle_manual(node)).success
    tick(node, level='OK')
    assert node._mode != ControlMode.SAFE_STOP
    tick(node, level='LOST')
    assert node._mode == ControlMode.SAFE_STOP and node._loc_counts['stops'] == 1


def test_filter_unknown_level_counts_as_lost(make):
    node = autonomous(make(**FILTER))
    tick(node, level='STALE')
    assert node._mode == ControlMode.SAFE_STOP


def test_filter_degraded_does_not_stop_and_gnss_vel_is_not_watched(make):
    node = autonomous(make(**FILTER))
    for _ in range(60):                      # 3 s DEGRADED, no /gnss/vel at all
        advance(node, 0.05)
        status(node, 'DEGRADED', 'bridging')
        odom(node, 1.0)
        node._actuate_loop()
    assert node._mode == ControlMode.AUTONOMOUS and node._gnss_stale()


@pytest.mark.parametrize('silent', ['status', 'odom'])
def test_filter_timeout_safe_stops(make, silent):
    node = autonomous(make(**FILTER))
    for dt, expect in ((0.29, ControlMode.AUTONOMOUS), (0.02, ControlMode.SAFE_STOP)):
        advance(node, dt)
        if silent == 'status':
            odom(node, 1.0)
        else:
            status(node, 'OK')
        node._actuate_loop()
        assert node._mode == expect
    assert node._loc_counts['stops'] == 1


def test_filter_watchdog_idle_does_nothing(make):
    node = make(**FILTER)
    advance(node, 5.0)
    node._actuate_loop()
    assert node._mode == ControlMode.IDLE and node._loc_counts['stops'] == 0


def test_filter_lost_safe_stop_latches_until_resume(make):
    node = autonomous(make(**FILTER))
    tick(node, level='LOST')
    for _ in range(5):
        tick(node, level='OK')
    assert node._mode == ControlMode.SAFE_STOP
    assert not start(node).success
    assert (
        node._resume_callback(None, Trigger.Response()).success and node._mode == ControlMode.IDLE
    )
    assert start(node).success


# ---- speed

def test_filter_speed_from_odom_signed_by_the_bridge(make):
    node = make(**FILTER)
    odom(node, 1.2)
    assert node.state.vel == 1.2
    gnss_vel(node, 5.0)                      # not the speed source any more
    assert node.state.vel == 1.2
    # rolling back in FORWARD: ADRC's convention, as with /gnss/vel
    odom(node, -1.2)
    assert node.state.vel == 1.2
    node.bridge.state = VehicleMode.REVERSE
    odom(node, -0.8)
    assert node.state.vel == -0.8
    odom(node, 0.8)
    assert node.state.vel == -0.8


# ---- DEGRADED speed cap

def path_tick(node, level):
    advance(node, 0.1)
    fresh(node, level)
    node._path_loop()
    return node._v_nom


def test_filter_degraded_caps_the_target_speed(make):
    node = autonomous(make(**FILTER))
    assert path_tick(node, 'OK') == pytest.approx(3.5)
    assert path_tick(node, 'DEGRADED') == pytest.approx(1.0)
    assert node._v_safe == pytest.approx(1.0)
    assert path_tick(node, 'DEGRADED') == pytest.approx(1.0)
    assert path_tick(node, 'OK') == pytest.approx(3.5)
    c = node._loc_counts
    assert (c['degraded'], c['cap_bites']) == (1, 1) and c['capped_s'] == pytest.approx(0.2)
    values, stat = diag(node)
    assert (
        values['Degraded Speed Cap']
        == '1.00 m/s; bit in 1 of 1 DEGRADED spells while driving, 0.2 s capped'
    )


def test_filter_cap_counts_only_bites(make):
    node = autonomous(make(**FILTER, base_speed=0.8))
    assert path_tick(node, 'DEGRADED') == pytest.approx(0.8)    # below the cap: a spell, no bite
    path_tick(node, 'OK')
    c = node._loc_counts
    assert (c['degraded'], c['cap_bites'], c['capped_s']) == (1, 0, 0.0)


def test_filter_cap_counts_each_spell_once(make):
    node = autonomous(make(**FILTER))
    for level in ('DEGRADED', 'DEGRADED', 'OK', 'DEGRADED', 'OK', 'OK'):
        path_tick(node, level)
    c = node._loc_counts
    assert (c['degraded'], c['cap_bites']) == (2, 2) and c['capped_s'] == pytest.approx(0.3)


def test_filter_cap_parameter(make):
    node = autonomous(make(**FILTER, degraded_speed_cap_mps=1.5))
    assert path_tick(node, 'DEGRADED') == pytest.approx(1.5)


def test_filter_cap_after_delay(make):
    node = autonomous(make(**FILTER, degraded_cap_after_s=0.5))
    # DEGRADED from the first: 0, 0.1, .. 0.6 s
    speeds = [path_tick(node, 'DEGRADED') for _ in range(7)]
    assert speeds[:5] == [pytest.approx(3.5)] * 5 and speeds[5:] == [pytest.approx(1.0)] * 2


def test_filter_manual_speed_capped_with_its_sign(make):
    node = make(**FILTER)
    fresh(node)
    assert toggle_manual(node).success
    refs = []
    node.longitudinal_ctrl.compute = lambda vel, ref, dt: refs.append(ref) or 0.0
    for v, level in (
        (-2.0, 'OK'),
        (-2.0, 'DEGRADED'),
        (2.0, 'DEGRADED'),
        (0.5, 'DEGRADED'),
        (2.0, 'OK'),
    ):
        node._manual_v = v
        tick(node, level=level)
    assert refs == [-2.0, -1.0, 1.0, 0.5, 2.0]
    assert node._loc_counts['cap_bites'] == 1 and node._loc_counts['capped_s'] == pytest.approx(
        0.1
    )


# ---- SAFE_STOP steering

def assert_stop_frames(fs, steer):
    assert fs and all(
        f.gear == Gear.NEUTRAL and f.brake == CAN_CMD_MAX and f.pedal == 0 for f in fs
    )
    assert all(f.steering_angle_deg == pytest.approx(steer, abs=0.006) for f in fs)


def test_filter_safe_stop_holds_the_autonomous_steering(make):
    node = autonomous(make(**FILTER))
    node._delta_safe = math.radians(-11.3)
    tick(node)
    assert frames(node)[-1].steering_angle_deg == pytest.approx(-11.3, abs=0.006)
    n = len(node.can.items)
    # watchdog → SAFE_STOP; its on_enter frame + the first hold
    tick(node, level='LOST')
    for _ in range(40):                      # 2 s, through standstill
        tick(node, level='LOST', v=0.0)
    assert node._mode == ControlMode.SAFE_STOP
    assert_stop_frames(frames(node)[n:], -11.3)
    values, _ = diag(node)
    assert values['Safe Stop Steering'] == 'held at -11.30 deg'
    # /resume → IDLE: wheels back to 0 (at standstill)
    node._resume_callback(None, Trigger.Response())
    tick(node)
    assert node._mode == ControlMode.IDLE and frames(node)[-1].steering_angle_deg == 0.0
    assert frames(node)[-2].steering_angle_deg == 0.0


def test_filter_safe_stop_from_manual_holds(make):
    node = make(**FILTER)
    fresh(node)
    assert toggle_manual(node).success
    node._manual_delta = math.radians(7.0)
    tick(node)
    n = len(node.can.items)
    node._request_safe_stop()
    tick(node)
    assert_stop_frames(frames(node)[n:], 7.0)


def test_filter_safe_stop_from_the_path_loop_holds_the_last_frame_sent(make):
    """
    SAFE_STOP raised in the path loop (collision zone) after it computed a new steering.

    The steering of the last frame sent is held, not the newer one that was never sent.
    """
    node = autonomous(make(**FILTER))
    node._delta_safe = math.radians(4.0)
    tick(node)
    n = len(node.can.items)
    node._delta_safe = math.radians(9.0)     # the path loop's new value, not sent yet
    node._request_safe_stop()
    tick(node)
    assert_stop_frames(frames(node)[n:], 4.0)


def test_filter_safe_stop_from_idle_steers_zero(make):
    node = make(**FILTER)
    node._request_safe_stop()
    tick(node)
    assert_stop_frames(frames(node), 0.0)


def test_filter_safe_stop_right_after_start_steers_zero(make):
    """SAFE_STOP before AUTONOMOUS sent a frame, after a held stop: IDLE's 0 is the last one."""
    node = autonomous(make(**FILTER))
    node._delta_safe = math.radians(12.0)
    tick(node)
    tick(node, level='LOST')
    node._resume_callback(None, Trigger.Response())
    fresh(node)
    assert start(node).success
    n = len(node.can.items)
    tick(node, level='LOST')                 # stops before any AUTONOMOUS frame
    assert_stop_frames(frames(node)[n:], 0.0)


def test_safe_stop_hold_is_clipped_and_finite(make):
    node = make(**FILTER)
    fresh(node)
    assert toggle_manual(node).success
    node._manual_delta = math.radians(40.0)
    tick(node)
    node._request_safe_stop()
    assert node._safe_stop_steer_deg == STEER_MAX_DEG
    node._resume_callback(None, Trigger.Response())
    node._last_steer_deg = float('nan')
    node._request_safe_stop()
    assert node._safe_stop_steer_deg == 0.0


@pytest.mark.parametrize('params, expect', [
    ({**FILTER, 'safe_stop_hold_steering': False}, 0.0),
    ({'safe_stop_hold_steering': True}, 6.0),
])
def test_safe_stop_hold_parameter_overrides_the_default(make, params, expect):
    node = make(**params)
    fresh(node)
    assert toggle_manual(node).success
    node._manual_delta = math.radians(6.0)
    tick(node)
    n = len(node.can.items)
    node._request_safe_stop()
    tick(node)
    assert_stop_frames(frames(node)[n:], expect)


# ---- diagnostics / telemetry

def test_filter_diagnostics(make):
    node = autonomous(make(**FILTER))
    advance(node, 0.02)
    status(node, 'DEGRADED', 'bridging', age_s='0.620', dist_m='0.90')
    values, stat = diag(node)
    assert list(values)[:len(GNSS_DIAG_KEYS)] == GNSS_DIAG_KEYS
    assert (
        values['Localization']
        == 'DEGRADED bridging, data 0.620 s old, 0.90 m bridged (status 0.00 s ago)'
    )
    assert values['Localization Stops'] == '0'
    assert values['Safe Stop Steering'] == 'holds the last command'
    assert (
        stat.level == DiagnosticStatus.WARN
        and stat.message == 'Localization DEGRADED: speed capped at 1.00 m/s'
    )
    advance(node, 0.5)
    values, _ = diag(node)
    assert values['Localization'].endswith('(status 0.50 s ago)')


def test_filter_telemetry_line(make):
    node = make(**FILTER)
    node._publish_telemetry('IDLE', 0.0, 0.0, 0.0)
    status(node, 'DEGRADED', 'bridging', age_s='0.620')
    node._publish_telemetry('NAVIGATING', 1.0, 2.0, 0.1)
    status(node, 'OK')
    node._publish_telemetry('MANUAL', 1.0, 0.0, 0.0, manual=True)
    advance(node, 0.4)
    node._publish_telemetry('SAFE_STOP', 0.0, 0.0, 0.0)
    last = [m.data.splitlines()[-1] for m in node.tele.items]
    assert last == [
        'Loc:       no status',
        'Loc:       DEGRADED 0.620 s (cap 1.0)',
        'Loc:       OK',
        'Loc:       no status for 0.4 s',
    ]
    assert node.tele.items[1].data.splitlines()[0] == 'STATUS:    NAVIGATING'
