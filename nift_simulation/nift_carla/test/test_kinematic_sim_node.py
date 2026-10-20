"""
Tests of the kinematic sim backend node (kinematic_sim_node.py).

The node is driven tick by tick with the CAN frames trajectory_controller sends: the
plant's speed, the shuttle's turning circle, the holds (power-on, E-stop, watchdog) with
the measured braking, teleport, /clock, odom, GNSS and /carla/dbw_state. No tick thread,
no DDS round trips: the node's callbacks and _tick_once() are called directly, publishers
are recorded.
"""
from datetime import datetime
import json
import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
from nift_carla.kinematic_sim_node import KinematicSimNode
from nift_common.can_protocol import Gear
from nift_common.vehicle import ShuttleConfig
from nift_control.dbw_interface.can_interface import create_command_frame, create_estop_frame
import pytest
import rclpy
from rclpy.parameter import Parameter

SHUTTLE = ShuttleConfig()
LOCK = SHUTTLE.max_steering_deg
R_FULL = SHUTTLE.min_turn_radius
SPAWN = (10.0, 20.0, 0.5)


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Recorder:

    def __init__(self):
        self.items = []

    def publish(self, x):
        self.items.append(x)


@pytest.fixture
def make():
    nodes = []

    def _make(**params):
        params = {'octane.port': 0, 'time_factor': 0.0, 'spawn.x': SPAWN[0], 'spawn.y': SPAWN[1],
                  'spawn.yaw': SPAWN[2], **params}
        node = KinematicSimNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()]
        )
        node.rec = SimpleNamespace(
            octane=Recorder(),
            odom=Recorder(),
            clock=Recorder(),
            dbw=Recorder(),
            health=Recorder(),
            fault=Recorder(),
        )
        node.octane.publish = node.rec.octane.publish
        node.pub_odom, node.pub_clock = node.rec.odom, node.rec.clock
        node.pub_dbw_state, node.pub_gps_health = node.rec.dbw, node.rec.health
        node.pub_gnss_fault = node.rec.fault
        nodes.append(node)
        return node
    yield _make
    for n in nodes:
        n.octane.stop()
        n.destroy_node()


def drive(node, secs, pedal=0, brake=0, steer=0, gear=Gear.FORWARD, send=True):
    """
    Run secs of ticks, sending one 0x101 per tick (send=False: CAN goes quiet).

    Returns base_link poses.
    """
    poses = []
    for _ in range(int(round(secs / node.dt))):
        if send:
            node._can_callback(create_command_frame(gear, brake, pedal, steer))
        node._tick_once()
        poses.append(node.car.base_link())
    return poses


def last_dbw(node):
    return json.loads(node.rec.dbw.items[-1].data)


def test_pedal_settles_at_the_plant_speed(make):
    node = make()
    poses = drive(node, 15.0, pedal=100)
    assert node.car.v == pytest.approx(1.035, abs=0.01)
    x, y, yaw = poses[-1]
    assert yaw == pytest.approx(SPAWN[2])                       # straight on, steer 0
    assert math.atan2(y - SPAWN[1], x - SPAWN[0]) == pytest.approx(SPAWN[2])
    assert last_dbw(node)['mode'] == 'active' and last_dbw(node)['plant'] == 'drive'


def test_holds_before_any_command(make):
    node = make()
    drive(node, 3.0, send=False)
    assert node.car.v == 0.0 and node.car.base_link() == pytest.approx(SPAWN)
    state = last_dbw(node)
    assert (
        state['mode'] == 'no_command'
        and state['applied']['hold'] is True
        and state['plant'] == 'hold'
    )


def stop_distance(start, poses):
    return math.hypot(poses[-1][0] - start[0], poses[-1][1] - start[1])


def test_estop_stops_with_the_measured_braking(make):
    node = make()
    drive(node, 12.0, pedal=100)
    v0, start = node.car.v, node.car.base_link()
    node._can_callback(create_estop_frame(True))
    # pedal frames keep coming: ignored, latched
    poses = drive(node, 6.0, pedal=100)
    assert last_dbw(node)['mode'] == 'estop' and node.car.v == 0.0
    # 0.45 s dead time + 0.72 - 0.15 v braking from 1.035 m/s: 0.47 + 0.87 m. CARLA's physics hold:
    # ~0.1 m
    assert v0 == pytest.approx(1.035, abs=0.01)
    assert stop_distance(start, poses) == pytest.approx(1.34, abs=0.1)


def test_watchdog_stops_with_the_measured_braking(make):
    node = make()
    drive(node, 12.0, pedal=100)
    start = node.car.base_link()
    poses = drive(node, 6.0, send=False)
    assert last_dbw(node)['mode'] == 'timeout' and node.car.v == 0.0
    # + 0.5 s watchdog
    assert stop_distance(start, poses) == pytest.approx(0.5 * 1.035 + 1.34, abs=0.1)


@pytest.mark.parametrize('steer, sign', [(LOCK, 1.0), (-LOCK, -1.0)])
def test_full_lock_turns_on_the_shuttles_circle(make, steer, sign):
    node = make()
    drive(node, 20.0, pedal=100, steer=steer)
    assert node.car.steer_deg == pytest.approx(steer)
    # CAN + = left = CCW
    assert node.car.yaw_rate == pytest.approx(sign * node.car.v / R_FULL)
    assert last_dbw(node)['applied']['wheel_deg'] == pytest.approx(steer)


def test_teleport_resets_pose_and_plant(make):
    node = make()
    drive(node, 10.0, pedal=100)
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x, msg.pose.pose.position.y = 50.0, 60.0
    # yaw 1.0
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(0.5), math.cos(0.5)
    node._initialpose_callback(msg)
    node._tick_once()
    assert node.car.base_link() == pytest.approx((50.0, 60.0, 1.0))
    assert node.car.v == 0.0 and node.plant.status()['engaged'] is False


def test_clock_and_odom(make):
    node = make()
    drive(node, 2.5, pedal=100)
    clocks = [c.clock.sec + c.clock.nanosec / 1e9 for c in node.rec.clock.items]
    assert clocks[0] == pytest.approx(1.0)
    assert all(b - a == pytest.approx(0.05) for a, b in zip(clocks, clocks[1:]))
    odom = node.rec.odom.items[-1]
    x, y, yaw = node.car.base_link()
    assert (odom.header.frame_id, odom.child_frame_id) == ('map', 'base_link')
    assert odom.header.stamp == node.rec.clock.items[-1].clock
    assert (odom.pose.pose.position.x, odom.pose.pose.position.y) == pytest.approx((x, y))
    assert 2.0 * math.atan2(
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w
    ) == pytest.approx(yaw)
    assert odom.twist.twist.linear.x == node.car.v


def test_gnss_puts_the_antennas_on_the_shuttle(make):
    node = make()
    drive(node, 0.05, send=False)
    front, rear = node.rec.octane.items[-1]
    assert (front['id'], rear['id']) == ('SIM-FRONT', 'SIM-REAR')
    for p, sign in ((front, 1.0), (rear, -1.0)):
        d = p['state']['dynamics']
        x, y = node.geo.wgs_to_map(d['latitude'], d['longitude'])
        assert x == pytest.approx(
            SPAWN[0] + sign * SHUTTLE.gnss_front_x * math.cos(SPAWN[2]), abs=1e-6
        )
        assert y == pytest.approx(
            SPAWN[1] + sign * SHUTTLE.gnss_front_x * math.sin(SPAWN[2]), abs=1e-6
        )
        assert d['elevation'] == pytest.approx(SHUTTLE.base_link_height + SHUTTLE.gnss_z)


def test_gps_dropout_reaches_gps_health(make):
    node = make(**{'gps.dropout_period_s': 2.0, 'gps.dropout_duration_s': 0.5})
    drive(node, 2.0, send=False)                                # sim time 1.0 .. 2.95
    assert [m.data for m in node.rec.health.items] == [False, True]
    assert len(node.rec.octane.items) == 30                     # 40 ticks, 10 in the dropout


def test_spawn_by_geo_and_no_carla_index(make):
    lat, lon = make().geo.map_to_wgs(*SPAWN[:2])
    node = make(
        **{'spawn.mode': 'geo', 'spawn.lat': lat, 'spawn.lon': lon, 'spawn.heading_deg': 0.0}
    )
    assert node.car.base_link() == pytest.approx((SPAWN[0], SPAWN[1], math.pi / 2))
    with pytest.raises(ValueError, match='carla_index'):
        make(**{'spawn.mode': 'carla_index'})


def test_real_feed_model_from_params(make):
    """
    Check gps.feed.enabled: Octane deliveries carry relay-time stamps.

    Not the tick time, and they repeat positions.
    """
    node = make(
        **{
            'gps.feed.enabled': True,
            'gps.feed.burst_per_min': 0.0,
            'gps.feed.gap_rate_moving': 0.0,
            'gps.feed.gap_rate_still': 0.0,
            'gps.seed': 3,
        }
    )
    assert node.gnss.feed is not None and node.gnss.feed.burst_per_min == 0.0
    drive(node, 10.0, pedal=60)
    front = [p for batch in node.rec.octane.items for p in batch if p['id'] == 'SIM-FRONT']
    ticks_ms = {round(k * node.dt * 1000) for k in range(1, 400)}
    stamps_ms = [
        round(datetime.fromisoformat(p['state']['dynamics']['updated']).timestamp() * 1000) - 1000
        for p in front
    ]
    # relay times, mostly between ticks
    assert sum(s not in ticks_ms for s in stamps_ms) > len(stamps_ms) / 2
    positions = [
        (p['state']['dynamics']['latitude'], p['state']['dynamics']['longitude']) for p in front
    ]
    assert len(set(positions)) < len(positions)                              # repeats


def test_dead_zone_from_params(make):
    lat, lon = make().geo.map_to_wgs(*SPAWN[:2])
    node = make(**{'gps.dead_zones': [lat - 1e-4, lat + 1e-4, lon - 1e-4, lon + 1e-4]})
    drive(node, 1.0, send=False)
    assert node.rec.octane.items == [] and [m.data for m in node.rec.health.items] == [False]


def test_fault_scenario_from_params_reaches_gnss_fault(make):
    """
    Run gps.faults (a YAML list, as the launch arg resolves it) through the node.

    The dropout window empties the Octane feed and its start / end go out on
    /carla/gnss_fault with the scenario name.
    """
    node = make(
        **{
            'gps.faults': '[{kind: dropout, t_s: 1.5, duration_s: 0.5}]',
            'gps.faults_name': 'test_dropout',
        }
    )
    assert node.gnss.exact and len(node.gnss.faults) == 1
    drive(node, 2.0, send=False)                                # sim time 1.05 .. 3.0
    evs = [json.loads(m.data) for m in node.rec.fault.items]
    assert [(e['edge'], e['t'], e['scenario']) for e in evs] == [
        ('start', 1.5, 'test_dropout'),
        ('end', 2.0, 'test_dropout'),
    ]
    stamps = [datetime.fromisoformat(p['state']['dynamics']['updated']).timestamp()
              for batch in node.rec.octane.items for p in batch]
    assert not any(1.5 <= t < 2.0 - 1e-6 for t in stamps) and any(t >= 2.0 for t in stamps)
