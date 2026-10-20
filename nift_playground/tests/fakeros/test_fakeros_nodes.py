"""
The six repo node classes construct unchanged on fakeros with their real parameter files (ARCH 2, 6.1).

The sources follow ARCH 6.1 for sim_kinematic (ekf mode, so both TF providers are built). The
OctaneEmulator swap is ARCH 2.1's, with the sim-gnss agent's BrowserOctane. These tests need the
other stand-ins (fakelanelet2, fakesocketio, stubs, playground.octane_bridge); they skip if one is
missing.
"""
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
START_NS = 1_000_000_000
DT_NS = 50_000_000


def repo(*rel):
    return os.path.join(REPO, *rel)


def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else k
        if isinstance(v, dict):
            out.update(flatten(v, key))
        else:
            out[key] = v
    return out


@pytest.fixture
def stack(world, monkeypatch):
    for mod in ('lanelet2', 'socketio', 'dotenv', 'aiohttp', 'pyproj', 'playground.octane_bridge', 'numpy',
                'yaml'):
        pytest.importorskip(mod)
    import socketio
    import yaml
    from fakeros import node_scoped
    from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
    from nift_common.vehicle import ShuttleConfig
    from playground.octane_bridge import BrowserOctane

    if hasattr(socketio, 'hub') and hasattr(socketio.hub, 'reset'):
        socketio.hub.reset()
    shuttle = ShuttleConfig()
    origin = {'map_origin_lat': MAP_ORIGIN_LAT, 'map_origin_lon': MAP_ORIGIN_LON}
    sim_time = {'use_sim_time': True}
    with open(repo('nift_simulation/nift_carla/config/carla_params.yaml')) as f:
        carla = yaml.safe_load(f)['carla_bridge']['ros__parameters']
    octane = carla['octane']
    for k, v in (('MCITY_OCTANE_SERVER', f"http://127.0.0.1:{octane['port']}"),
                 ('MCITY_OCTANE_KEY', octane['api_key']), ('MCITY_OCTANE_NAMESPACE', octane['namespace']),
                 ('MCITY_FRONT_BEACON_ID', octane['front_id']), ('MCITY_REAR_BEACON_ID', octane['rear_id'])):
        monkeypatch.setenv(k, str(v))

    import nift_carla.kinematic_sim_node as kin_mod
    monkeypatch.setattr(kin_mod, 'OctaneEmulator', BrowserOctane)
    world.configure_node('kinematic_sim', parameters=[
        flatten(carla),
        {**origin, 'spawn.mode': 'map', 'spawn.x': 101.52, 'spawn.y': 162.66, 'spawn.yaw': -1.612,
         'time_factor': 0.0, 'lockstep': False, 'gps.seed': 1}])
    world.configure_node('nift_beacon_node', name='beacon_node', parameters=[
        repo('nift_sensor/config/beacon_node.yaml'), sim_time, {'hygiene.enabled': True}])
    world.configure_node('shuttle_dynamic_broadcaster', parameters=[
        repo('nift_sensor/config/shuttle_tf.yaml'), sim_time, origin, {'gnss.offset_x': shuttle.gnss_front_x}])
    world.configure_node('localization_node', parameters=[
        repo('nift_sensor/config/localization.yaml'), sim_time,
        node_scoped('localization_node', {
            'mode': 'ekf', 'antenna.front_x_m': shuttle.gnss_front_x, 'antenna.rear_x_m': shuttle.gnss_rear_x,
            **origin, 'gnss_delay_s': 0.0, 'speed_point_x_m': -shuttle.base_link_x})])
    world.configure_node('waypoint_node', parameters=[
        repo('nift_waypoint/config/waypoint_params.yaml'), sim_time, origin,
        {'map_file': repo('maps/lanelet2_odr2024.osm')}])
    world.configure_node('trajectory_controller', parameters=[
        sim_time, repo('nift_control/config/planning.yaml'), repo('nift_control/config/adrc_params.yaml'),
        {'lateral_enabled': True},
        node_scoped('trajectory_controller', {'localization_source': 'filter', 'filter_type': 'none'})])
    return world


def build(world):
    from nift_carla.kinematic_sim_node import KinematicSimNode
    from nift_control.trajectory_controller import TrajectoryController
    from nift_sensor.beacon_node import BeaconNode
    from nift_sensor.localization_node import LocalizationNode
    from nift_sensor.shuttle_tf_node import ShuttleDynamicBroadcaster
    from nift_waypoint.waypoint_node import WaypointNode
    nodes = {}
    nodes['kinematic_sim'] = KinematicSimNode()
    nodes['beacon_node'] = BeaconNode()
    nodes['shuttle_dynamic_broadcaster'] = ShuttleDynamicBroadcaster()
    nodes['localization_node'] = LocalizationNode()
    world.drain()                                  # auth_ok before planning and control (ARCH 6.1)
    nodes['waypoint_node'] = WaypointNode()
    nodes['trajectory_controller'] = TrajectoryController()
    world.drain()
    return nodes


def test_all_nodes_construct_with_their_real_parameters(stack):
    w = stack
    nodes = build(w)
    assert sorted(n.get_name() for n in w.nodes) == sorted(nodes)
    kin, bcn = nodes['kinematic_sim'], nodes['beacon_node']
    tf, loc = nodes['shuttle_dynamic_broadcaster'], nodes['localization_node']
    wp, ctl = nodes['waypoint_node'], nodes['trajectory_controller']
    g = lambda n, name: n.get_parameter(name).value      # noqa: E731

    # kinematic_sim: carla_params as a launch dict, then the launch overrides
    assert g(kin, 'time_factor') == 0.0 and g(kin, 'lockstep') is False and g(kin, 'gps.seed') == 1
    assert g(kin, 'plant.pedal_map') == [0.0, 0.46, 40.0, 0.635, 70.0, 0.83, 100.0, 1.035, 140.0, 1.275]
    assert g(kin, 'gps.dead_zones') == [] and g(kin, 'gps.faults') == ''
    assert g(kin, 'spawn.x') == 101.52 and kin.dt == 0.05
    assert kin._sim_ns == START_NS - DT_NS == w.now_ns
    # beacon_node: own name remapped, /** file, dynamic hygiene parameters
    assert bcn.get_name() == 'beacon_node' and bcn.get_logger().name == 'beacon_node'
    assert g(bcn, 'hygiene.baseline_learn_pairs') == 20 and g(bcn, 'use_sim_time') is True
    assert any('auth_ok' in text and name == 'beacon_node' for _, name, text in w.logs)
    # shuttle_tf: node-scoped file plus dicts
    assert g(tf, 'gnss.offset_y') == 0.0 and g(tf, 'gnss.offset_x') == pytest.approx(tf.gnss_offset_x)
    # localization: node-scoped override after its own file
    assert g(loc, 'mode') == 'ekf' and g(loc, 'filter.gate_pos') == 13.8 and g(loc, 'gnss_delay_s') == 0.0
    assert g(loc, 'steer_input_states') == ['NAVIGATING', 'MANUAL', 'SAFE_STOP']
    # waypoint_node: the dicts' /** key comes after the file's node key in the table, so map_file wins
    assert g(wp, 'map_file').endswith('lanelet2_odr2024.osm') and g(wp, 'path_downsample_stride') == 5
    # trajectory_controller: ARCH 6.2's precedence case
    assert g(ctl, 'filter_type') == 'none' and g(ctl, 'goal_tolerance') == 2.0 and g(ctl, 'adrc.w_cl') == 2.0
    assert g(ctl, 'lateral_enabled') is True and g(ctl, 'localization_source') == 'filter'
    assert g(ctl, 'safe_stop_hold_steering') is True
    # services resolve to absolute names
    assert {'/start_navigation', '/trigger_estop', '/resume', '/toggle_manual_mode',
            '/carla/dbw_power_cycle'} <= set(w.services)
    # every Updater: period 1 s, "<launch name>: <task>"
    assert g(bcn, 'diagnostic_updater.period') == 1.0


def test_stack_runs_on_the_clock_grid(stack):
    from diagnostic_msgs.msg import DiagnosticArray
    w = stack
    nodes = build(w)
    kin = nodes['kinematic_sim']
    diags = []
    from rclpy.node import Node
    obs = Node('observer')
    obs.create_subscription(DiagnosticArray, '/diagnostics', diags.append, 10)
    for k in range(1, 41):                         # 2 s of sim
        t = START_NS - DT_NS + k * DT_NS
        w.step(t, kin._tick_once)
        assert kin._sim_ns == t
    names = {s.name for a in diags for s in a.status}
    assert {'kinematic_sim: Kinematic sim', 'beacon_node: GNSS Interface', 'shuttle_dynamic_broadcaster: TF Bridge',
            'localization_node: Localization', 'waypoint_node: Lanelet2 Router',
            'trajectory_controller: Trajectory Controller'} <= names
    errors = [(name, text) for level, name, text in w.logs if level >= 40]
    assert errors == []
    # the TF map -> base_link is there for the planner and the controller
    tfc = nodes['trajectory_controller']
    import rclpy.time
    tr = tfc.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    assert tr.transform.translation.x == pytest.approx(101.52, abs=1.0)


def test_wrong_typed_override_fails_construction_like_ros(stack):
    from rclpy.exceptions import InvalidParameterTypeException
    from nift_control.trajectory_controller import TrajectoryController
    from fakeros import node_scoped
    stack.node_sources['trajectory_controller'].append(node_scoped('trajectory_controller', {'base_speed': 3}))
    with pytest.raises(InvalidParameterTypeException, match="'base_speed'"):
        TrajectoryController()
