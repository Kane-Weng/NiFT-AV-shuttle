"""
localization_node.

The localization filter wired to beacon_node's raw feed, /gnss/vel, our own 0x101 commands and
trajectory_controller's /nav_telemetry. Messages go straight into the callbacks on a controlled
clock; the 50 Hz tick is called by hand; publishers and the TF broadcaster are recorded (no DDS
round trips). The ROS context runs on its own domain (LOCALIZATION_TEST_DOMAIN, default 78), never
the default one.
"""
import json
import math
import os

from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TwistStamped
import lanelet2
from nift_common.can_protocol import CAN_ID_COMMAND, compute_steering, DLC_COMMAND, Gear
from nift_common.vehicle import ShuttleConfig
from nift_sensor.localization_node import LocalizationNode, telemetry_state
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

SHUTTLE = ShuttleConfig()
L = SHUTTLE.wheel_base
HALF = SHUTTLE.base_link_x
FULL = SHUTTLE.max_steering_deg
# the antennas, as the node's defaults and the sims
FRONT, REAR = SHUTTLE.gnss_front_x, SHUTTLE.gnss_rear_x
# the kinematic sim's speed point, its ideal feed
SIM = {'speed_point_x_m': -HALF, 'gnss_delay_s': 0.0}
DT = 0.001


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init(domain_id=int(os.environ.get('LOCALIZATION_TEST_DOMAIN', '78')))
    yield
    rclpy.shutdown()


class Recorder:

    def __init__(self):
        self.items = []

    def publish(self, x):
        self.items.append(x)


class Clock:
    ns = 0

    def now(self):
        return Time(nanoseconds=self.ns)


@pytest.fixture
def make():
    nodes = []

    def _make(**params):
        node = LocalizationNode(parameter_overrides=[
            Parameter(k, value=v) for k, v in {**SIM, **params}.items()])
        node.timer.cancel()
        node.odom, node.status, node.tf = Recorder(), Recorder(), []
        node.pub_odom, node.pub_status = node.odom, node.status
        if node.tf_broadcaster is not None:
            node.tf_broadcaster.sendTransform = node.tf.append
        node.clock = Clock()
        node.get_clock = lambda: node.clock
        nodes.append(node)
        return node
    yield _make
    for n in nodes:
        n.destroy_node()


class Truth:
    """
    Ground truth drive along speed and steering segments.

    Kinematic bicycle (rear axle, exact arcs) along (duration, speed, steer_deg) segments, 1 ms
    table.
    """

    def __init__(self, segments, x=10.0, y=20.0, yaw=0.3):
        self.rows = []
        t, v = 0.0, segments[0][1]
        for duration, v, steer in segments:
            kap = math.tan(math.radians(steer)) / L
            for _ in range(round(duration / DT)):
                self.rows.append((t, x, y, yaw, v, steer))
                ds = v * DT
                x += ds * math.cos(yaw + 0.5 * kap * ds)
                y += ds * math.sin(yaw + 0.5 * kap * ds)
                yaw += kap * ds
                t += DT
        self.end = t

    def at(self, t):
        return self.rows[min(len(self.rows) - 1, max(0, round(t / DT)))]

    def point(self, t, d):
        _, x, y, yaw, _, _ = self.at(t)
        return x + d * math.cos(yaw), y + d * math.sin(yaw)


def command(gear, brake, steer_deg):
    """Build a 0x101 drive command as trajectory_controller sends it, pedal 0."""
    frame = Frame(id=CAN_ID_COMMAND, dlc=DLC_COMMAND)
    frame.data[:DLC_COMMAND] = [gear, brake, 0, *compute_steering(steer_deg)]
    return frame


def latlon(node, x, y):
    g = node.projector.reverse(lanelet2.core.BasicPoint3d(x + node.origin[0], y + node.origin[1],
                                                          0.0))
    return g.lat, g.lon


def fix(node, t, x, y, status=NavSatStatus.STATUS_FIX):
    m = NavSatFix()
    m.header.stamp = Time(nanoseconds=round(t * 1e9)).to_msg()
    m.latitude, m.longitude = latlon(node, x, y)
    m.status.status = status
    return m


def run(node, tr, telemetry='NAVIGATING', drop=(), drop_antenna=None, unpaired=False,
        brake_from=None,
        t_end=None):
    """
    Feed a truth drive into the node and record its output.

    Feed the truth's 10 Hz antenna epochs (raw FIX, arriving 25 ms later), /gnss/vel at 50 Hz (each
    delivery's stamp repeated), 0x101 at 20 Hz with the truth's steering, /nav_telemetry at 10 Hz;
    tick at 50 Hz. drop: (start, duration[, antenna]) gaps of that antenna or drop_antenna (None:
    both); unpaired: the deliveries that still come in a gap are NO_FIX raw + no_pair on
    /gnss/hygiene. brake_from: 0x101 brake 255 / NEUTRAL from then on. Returns per tick (t, error
    [m], status level, pose v).
    """
    ev = []
    end = t_end or tr.end
    for i in range(1, int(end / 0.1)):
        te = i * 0.1
        for name, d in (('front', HALF + FRONT), ('rear', HALF + REAR)):
            spans = [g for g in drop if g[0] <= te < g[0] + g[1]]
            if any((g[2] if len(g) > 2 else drop_antenna) in (None, name) for g in spans):
                continue
            x, y = tr.point(te, d)
            if spans and unpaired:
                ev.append((te + 0.025, 1, 'unpaired', (name, te, x, y)))
            else:
                ev.append((te + 0.025, 1, 'raw', (name, te, x, y)))
        if not any(g[0] <= te < g[0] + g[1] and (g[2] if len(g) > 2 else drop_antenna) in (None,
                                                                                           'front')
                   for g in drop):
            # beacon_node's 50 Hz republish
            for k in range(5):
                ev.append((te + 0.03 + 0.02 * k, 2, 'vel', (te, abs(tr.at(te)[4]))))
    for i in range(1, int(end / 0.05)):
        t = i * 0.05
        brake = brake_from is not None and t >= brake_from
        ev.append((t, 0, 'can', (Gear.NEUTRAL if brake else Gear.FORWARD, 255 if brake else 0,
                                 tr.at(t)[5])))
        if i % 2 == 0 and telemetry:
            ev.append((t, 0, 'tele', telemetry))
    for i in range(1, int(end / 0.02)):
        ev.append((i * 0.02, 3, 'tick', None))
    ev.sort(key=lambda e: (e[0], e[1]))
    rows = []
    for t, _, kind, p in ev:
        node.clock.ns = round(t * 1e9)
        if kind == 'raw':
            name, te, x, y = p
            node.raw_callback(name, fix(node, te, x, y))
        elif kind == 'unpaired':
            name, te, x, y = p
            node.raw_callback(name, fix(node, te, x, y, NavSatStatus.STATUS_NO_FIX))
            lat, lon = latlon(node, x, y)
            node.hygiene_callback(String(data=json.dumps({'antenna': name, 'verdict': 'no_pair',
                                                          'lat': lat,
                                                          'lon': lon, 'updated': iso(te)})))
        elif kind == 'vel':
            te, v = p
            m = TwistStamped()
            m.header.stamp = Time(nanoseconds=round(te * 1e9)).to_msg()
            m.twist.linear.x = v
            node.vel_callback(m)
        elif kind == 'can':
            gear, brake, steer = p
            node.can_callback(command(gear, brake, steer))
        elif kind == 'tele':
            node.telemetry_callback(String(data=f'STATUS:    {p}\nTarget V:  1.00 m/s'))
        else:
            node.tick()
            st = node.status.items[-1]
            pose = node.odom.items[-1].pose.pose if node.odom.items else None
            if pose is None:
                rows.append((t, math.inf, st.level, None))
                continue
            _, x, y, yaw, _, _ = tr.at(t)
            bx, by = x + HALF * math.cos(yaw), y + HALF * math.sin(yaw)
            rows.append((t, math.hypot(pose.position.x - bx, pose.position.y - by), st.level,
                         node.odom.items[-1].twist.twist.linear.x))
    return rows


def iso(t):
    from datetime import datetime, timezone
    return datetime.fromtimestamp(t, tz=timezone.utc).isoformat(timespec='milliseconds')


def worst(rows, t0, t1):
    return max(r[1] for r in rows if t0 <= r[0] < t1)


def value(status, key):
    return {kv.key: kv.value for kv in status.values}[key]


# -------------------------------------------------------------------------------------------------------------
# #
def test_clean_drive(make):
    node = make()
    tr = Truth([(4.0, 2.0, 0.0), (6.0, 2.0, FULL)])
    rows = run(node, tr)
    assert worst(rows, 1.5, 10.0) < 0.03
    assert all(r[2] == DiagnosticStatus.OK for r in rows if r[0] > 1.5)
    assert rows[-1][3] == pytest.approx(2.0, abs=0.02)                  # odom twist: the speed
    assert len(node.tf) == len(node.odom.items) and node.tf[-1].child_frame_id == 'base_link'
    assert Time.from_msg(node.tf[-1].header.stamp).nanoseconds == node.clock.ns    # stamped now
    # once per front epoch, not per republish
    assert node.counts['speed'] < 110
    assert value(node.status.items[-1], 'steering_input') == 'on'


def test_shadow_publishes_no_tf(make):
    node = make(mode='shadow')
    rows = run(node, Truth([(4.0, 1.0, 0.0)]))
    assert node.tf_broadcaster is None and node.odom.items and worst(rows, 1.5, 4.0) < 0.03


def test_bad_mode_is_refused():
    with pytest.raises(ValueError, match='direct'):
        LocalizationNode(parameter_overrides=[Parameter('mode', value='direct')])


@pytest.mark.parametrize('telemetry, expect', [('NAVIGATING', 'on'), ('SAFE_STOP', 'on'),
                                               ('MANUAL', 'on'),
                                               ('IDLE', 'off'), (None, 'off')])
def test_steering_only_while_the_stack_drives(make, telemetry, expect):
    node = make()
    run(node, Truth([(3.0, 1.0, 5.0)]), telemetry=telemetry)
    assert value(node.status.items[-1], 'steering_input') == expect
    assert (node._input[0] is not None) == (expect == 'on')


def test_stale_telemetry_turns_the_steering_off(make):
    """
    Stale telemetry turns the steering input off.

    0x101 frames keep coming but /nav_telemetry stops (the controller's state is no longer known).
    """
    node = make()
    run(node, Truth([(3.0, 1.0, 5.0)]))
    # 0.6 s of commands, no telemetry
    for k in range(1, 13):
        node.clock.ns += round(0.05e9)
        node.can_callback(command(Gear.FORWARD, 0, 5.0))
    node.tick()
    assert value(node.status.items[-1], 'steering_input') == 'off'


@pytest.mark.parametrize('telemetry, limit', [('NAVIGATING', (0.0, 0.10)), ('IDLE', (0.15, 9.0))])
def test_turn_entry_in_a_gap_needs_the_steering(make, telemetry, limit):
    """
    The turn starts 0.2 s into a 1.5 s GNSS gap.

    With our 0x101 steering the pose follows it; without (the stack not in control) the filter
    assumes the straight goes on.
    """
    node = make()
    rows = run(node, Truth([(6.25, 1.5, 0.0), (6.0, 1.5, FULL)]), telemetry=telemetry,
               drop=[(6.05, 1.5)])
    assert limit[0] <= worst(rows, 6.05, 7.6) < limit[1]


def test_neutral_keeps_the_last_direction(make):
    """SAFE_STOP commands NEUTRAL + full brake: the shuttle rolls on the way it was going."""
    node = make()
    for i, gear in enumerate([Gear.REVERSE, Gear.NEUTRAL, Gear.NEUTRAL]):
        node.clock.ns = round((1.0 + 0.05 * i) * 1e9)
        node.telemetry_callback(String(data='STATUS:    SAFE_STOP'))
        node.can_callback(command(gear, 255 if gear == Gear.NEUTRAL else 0, 0.0))
        assert node._input[1] == -1


def test_repeat_of_a_dropped_position_is_not_new(make):
    """Bags before repeat_unused: a FIX re-delivery of a position the checks dropped."""
    node = make()
    run(node, Truth([(4.0, 1.0, 0.0)]))
    node.clock.ns = round(4.1e9)
    node.raw_callback('front', fix(node, 4.05, 50.0, 50.0, NavSatStatus.STATUS_NO_FIX))
    node.raw_callback('front', fix(node, 4.08, 50.0, 50.0))
    assert node.counts['repeat_of_dropped'] == 1 and node.filter.counts['rejected'] == 0


def test_at_rest_the_same_position_after_a_dropped_one_is_current(make):
    """
    Standing still, positions stay bit-identical.

    A FIX delivery after a dropped one at the same position is the current position (here it must
    restart a LOST filter), not a stale re-delivery.
    """
    node = make()
    node.vel_callback(TwistStamped())                                   # GNSS speed 0
    for i in range(3):
        node.clock.ns = round((1.0 + 0.1 * i) * 1e9)
        for name, x in (('front', 10.0 + FRONT - REAR), ('rear', 10.0)):
            status = NavSatStatus.STATUS_NO_FIX if i == 0 else NavSatStatus.STATUS_FIX
            node.raw_callback(name, fix(node, 1.0 + 0.1 * i, x, 5.0, status))
    assert node.counts['repeat_of_dropped'] == 0 and node.filter.counts['initialised'] == 1


def test_unpaired_positions_carry_it_through_a_one_antenna_dropout(make):
    """
    Rear out 3 s.

    beacon_node marks the front's deliveries NO_FIX and sends them as no_pair on /gnss/hygiene.
    """
    node = make()
    rows = run(node, Truth([(4.0, 2.0, 0.0), (8.0, 2.0, FULL)]), drop=[(4.0, 3.0)],
               drop_antenna='rear',
               unpaired=True)
    assert all(r[2] == DiagnosticStatus.OK for r in rows if r[0] > 1.5) and worst(rows, 1.5,
                                                                                  12.0) < 0.05
    assert node.counts['unpaired'] >= 29


def test_unpaired_positions_cannot_revive_a_bridging_filter(make):
    """
    Both antennas out 1 s, then only the front, unpaired.

    The filter is already bridging, so they are ignored (one unchecked antenna could be a swapped
    or broken one) and it ends LOST.
    """
    node = make()
    rows = run(node, Truth([(4.0, 2.0, 0.0), (8.0, 2.0, 0.0)]), drop=[(6.05, 1.0, None), (7.05,
                                                                                          3.0,
                                                                                          'rear')],
               unpaired=True)
    assert node.filter.counts['unpaired_ignored'] >= 5
    assert any(r[2] == DiagnosticStatus.ERROR for r in rows if 7.0 <= r[0] < 8.0)


def test_old_hygiene_messages_without_a_position_are_ignored(make):
    node = make()
    node.hygiene_callback(String(data=json.dumps({'antenna': 'front', 'verdict': 'no_pair',
                                                  'updated': iso(1.0)})))
    node.hygiene_callback(String(data='not json'))
    assert node.counts['unpaired'] == 0


def test_lost_while_braking_dead_reckons_the_stop(make):
    """
    GNSS gone for good at 6 s; the controller SAFE_STOPs (brake 255) at 6.5 s.

    LOST at 7.5 s, and the pose keeps following the shuttle as it brakes (0.6 m/s^2 after the 0.47
    s dead time) instead of running on at 2 m/s.
    """
    # (the truth stops at once: not the check)
    tr = Truth([(7.0, 2.0, 0.0), (4.0, 0.0, 0.0)])
    node = make()
    rows = run(node, tr, telemetry='NAVIGATING', drop=[(6.05, 99.0)], brake_from=6.5, t_end=11.5)
    assert rows[-1][2] == DiagnosticStatus.ERROR
    v = {round(r[0], 2): r[3] for r in rows}
    # 0.6 m/s^2 from 6.97 s (the dead time)
    assert v[8.98] == pytest.approx(2.0 - 0.6 * 2.01, abs=0.03)
    # stopped (at 10.3 s), not reversing
    assert v[11.48] == pytest.approx(0.0, abs=1e-9)
    node2 = make(**{'brake.decel_mps2': 0.0})
    rows2 = run(node2, tr, telemetry='NAVIGATING', drop=[(6.05, 99.0)], brake_from=6.5, t_end=11.5)
    assert rows2[-1][3] == pytest.approx(2.0, abs=0.05)


def test_status_message_levels(make):
    node = make()
    node.clock.ns = round(1e9)
    node.tick()
    st = node.status.items[-1]
    assert st.level == DiagnosticStatus.ERROR \
        and st.message == 'LOST uninitialised: no front / rear pair yet'
    assert not node.odom.items
    assert {'age_s', 'dist_m', 'sigma_pos_m', 'sigma_yaw_deg', 'steering_input', 'mode'} \
        <= {kv.key for kv in st.values}


@pytest.mark.parametrize('learn_pairs, start', [(5, True), (0, False)])
def test_lever_arm_off_at_rest(make, learn_pairs, start):
    """
    The antennas really at +-1.44 m (the bench's lever_arm_+0.4m), configured +-1.04.

    The node starts on the measured distance (status values say so), or, with learning off, says
    why it cannot.
    """
    node = make(**{'filter.baseline_learn_pairs': learn_pairs})
    for i in range(20):                                                 # 2 s at rest, 10 Hz pairs
        t = 1.0 + 0.1 * i
        node.clock.ns = round((t + 0.025) * 1e9)
        node.raw_callback('front', fix(node, t, 10.0 + HALF + 1.44, 5.0))
        node.raw_callback('rear', fix(node, t, 10.0 + HALF - 1.44, 5.0))
        node.tick()
    st = node.status.items[-1]
    if start:
        assert st.level == DiagnosticStatus.OK and value(st,
                                                         'antenna_baseline_source') == 'measured'
        assert float(value(st, 'antenna_baseline_m')) == pytest.approx(2.88, abs=0.002)
        assert float(value(st, 'rear_antenna_x_m')) == pytest.approx(1.04 - 2.88, abs=0.002)
        assert node._baseline_logged
    else:
        assert st.message == 'LOST uninitialised: antenna distance 2.88 m vs configured 2.08 m'
        assert value(st, 'antenna_baseline_source') == 'configured'


def test_telemetry_state():
    assert telemetry_state('STATUS:    SAFE_STOP\nTarget V:  0.00 m/s') == 'SAFE_STOP'
    assert telemetry_state('Target V:  0.00 m/s') is None


def test_other_can_frames_are_ignored(make):
    node = make()
    node.can_callback(Frame(id=0x100, dlc=1))
    assert node.counts['can_0x101'] == 0
