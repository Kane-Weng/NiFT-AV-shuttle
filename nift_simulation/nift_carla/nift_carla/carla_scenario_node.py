#!/usr/bin/env python3
"""
Spawn the obstacles of a scenario YAML as CARLA actors.

File: carla_scenario_node.py
Author: Kane Weng

Description:
  CARLA replacement for the Gazebo obstacle_spawner (Gazebo is gone). Reads the same
  scenario YAML (`obstacles:` list, map-frame metres) and spawns CARLA actors:

    shape: cylinder, length >= 1.5   -> walker.pedestrian.*      (pedestrian)
    shape: cylinder, otherwise       -> static.prop.trafficcone01 (bollard / cone)
    shape: box,      size[0] >= 1.5  -> vehicle.*  parked, physics frozen
    shape: box,      otherwise       -> static.prop.box01

  Optional per-obstacle keys (all additive to the Gazebo schema):
    blueprint: <carla id>      explicit blueprint, overrides the mapping
    lat/lon: <deg>             position in WGS84 instead of x/y
    yaw: <rad>                 heading in the map frame (default 0)
    physics: true              let a vehicle simulate physics (hand brake on)
    walk_to: [x, y]            pedestrian walks to this map-frame point ...
    speed: <m/s>               ... at this speed (default 1.0)

  `color` is accepted and ignored.  Positions go through CarlaGeo so the
  obstacle appears where the same YAML puts it in RViz.

  Only the bridge ticks the world (synchronous mode).  This node waits for
  /clock to advance before connecting and never calls world.tick().

Usage:
  ros2 run nift_carla carla_scenario_node --ros-args -p scenario_file:=/abs/path.yaml
      -p carla_host:=<ip>
"""

import signal
import threading
from typing import Dict, List, Optional

import carla
from nift_carla.carla_geo import CarlaGeo
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import yaml

DEFAULT_PEDESTRIAN = 'walker.pedestrian.0001'
DEFAULT_CONE = 'static.prop.trafficcone01'
DEFAULT_BOX = 'static.prop.box01'
DEFAULT_VEHICLE = 'vehicle.tesla.model3'


def resolve_blueprint(obs: Dict) -> str:
    """Map a Gazebo-schema obstacle to a CARLA blueprint id."""
    if obs.get('blueprint'):
        return str(obs['blueprint'])
    shape = str(obs.get('shape', 'cylinder')).lower()
    if shape == 'cylinder':
        return DEFAULT_PEDESTRIAN if float(obs.get('length', 0.0)) >= 1.5 else DEFAULT_CONE
    size = obs.get('size', [1.0, 1.0, 1.0])
    return DEFAULT_VEHICLE if float(size[0]) >= 1.5 else DEFAULT_BOX


def actor_kind(bp_id: str) -> str:
    if bp_id.startswith('walker.'):
        return 'walker'
    if bp_id.startswith('vehicle.'):
        return 'vehicle'
    return 'prop'


class CarlaScenarioNode(Node):

    def __init__(self):
        super().__init__('carla_scenario')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('carla_timeout_s', 60.0)
        self.declare_parameter('map_origin_lat', MAP_ORIGIN_LAT)
        self.declare_parameter('map_origin_lon', MAP_ORIGIN_LON)
        self.declare_parameter('wait_for_clock', True)
        self.declare_parameter('walker_start_delay_s', 1.0)
        # like Gazebo's rigid cylinder: survives being hit
        self.declare_parameter('walkers_invincible', True)

        path = self.get_parameter('scenario_file').value
        if not path:
            raise RuntimeError('scenario_file parameter is required')
        with open(path, 'r') as f:
            scenario = yaml.safe_load(f) or {}
        self.obstacles: List[Dict] = scenario.get('obstacles', []) or []
        self.get_logger().info(f'Loaded {len(self.obstacles)} obstacle(s) from {path}')

        self._clock_stamps = set()
        self._spawned = False
        self._actors: List = []
        self._walkers: List = []      # (controller, target carla.Location, speed)
        self.world = None
        self.geo: Optional[CarlaGeo] = None

        if bool(self.get_parameter('wait_for_clock').value):
            self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self._timer = self.create_timer(0.5, self._try_spawn)

    # ------------------------------------------------------------------ #
    def _clock_cb(self, msg: Clock):
        if len(self._clock_stamps) < 3:
            self._clock_stamps.add((msg.clock.sec, msg.clock.nanosec))

    def _ready(self) -> bool:
        return (not bool(self.get_parameter('wait_for_clock').value)) or len(
            self._clock_stamps
        ) >= 2

    def _try_spawn(self):
        if self._spawned or not self._ready():
            return
        try:
            self._connect()
            self._spawn_all()
            self._spawned = True
            self._timer.cancel()
            delay = float(self.get_parameter('walker_start_delay_s').value)
            if self._walkers:
                self._walker_timer = self.create_timer(delay, self._start_walkers)
        except RuntimeError as ex:
            self.get_logger().error(f'CARLA error while spawning scenario: {ex} — retrying')

    def _connect(self):
        client = carla.Client(
            self.get_parameter('carla_host').value, int(self.get_parameter('carla_port').value)
        )
        client.set_timeout(float(self.get_parameter('carla_timeout_s').value))
        self.world = client.get_world()
        self.carla_map = self.world.get_map()
        self.geo = CarlaGeo.from_opendrive(
            self.carla_map.to_opendrive(),
            float(self.get_parameter('map_origin_lat').value),
            float(self.get_parameter('map_origin_lon').value))
        self.get_logger().info(f'Connected; map {self.carla_map.name}')

    # ------------------------------------------------------------------ #
    def _ground_z(self, cx: float, cy: float) -> float:
        try:
            # roads at ~276 m
            hit = self.world.ground_projection(carla.Location(x=cx, y=cy, z=1000.0), 2000.0)
            if hit is not None:
                return hit.location.z
        except Exception:
            pass
        wp = self.carla_map.get_waypoint(carla.Location(x=cx, y=cy, z=0.0), project_to_road=True)
        return wp.transform.location.z if wp is not None else 0.0

    def _map_xy(self, obs: Dict):
        if 'lat' in obs and 'lon' in obs:
            return self.geo.wgs_to_map(float(obs['lat']), float(obs['lon']))
        return float(obs.get('x', 0.0)), float(obs.get('y', 0.0))

    def _spawn_all(self):
        bp_lib = self.world.get_blueprint_library()
        for obs in self.obstacles:
            name = obs.get('name', '?')
            bp_id = resolve_blueprint(obs)
            found = bp_lib.filter(bp_id)
            if not found:
                self.get_logger().error(f'{name}: blueprint "{bp_id}" not found — skipped')
                continue
            bp = found[0]
            kind = actor_kind(bp.id)
            x, y = self._map_xy(obs)
            yaw = float(obs.get('yaw', 0.0))
            cx, cy, _ = self.geo.map_to_carla(x, y)
            cyaw = self.geo.map_yaw_to_carla_yaw_deg(x, y, yaw)
            gz = self._ground_z(cx, cy)
            z = (
                gz
                + {'walker': 1.0, 'vehicle': 0.5, 'prop': 0.1}[kind]
                + float(obs.get('z_offset', 0.0))
            )
            if bp.has_attribute('role_name'):
                bp.set_attribute('role_name', f'scenario_{name}')
            if kind == 'walker' and bp.has_attribute('is_invincible'):
                inv = bool(obs.get('invincible', self.get_parameter('walkers_invincible').value))
                bp.set_attribute('is_invincible', 'true' if inv else 'false')
            tf = carla.Transform(carla.Location(x=cx, y=cy, z=z), carla.Rotation(yaw=cyaw))
            actor = self.world.try_spawn_actor(bp, tf)
            if actor is None:
                self.get_logger().error(
                    f'{name}: spawn failed at CARLA ({cx:.2f}, {cy:.2f}, {z:.2f})'
                )
                continue
            self._actors.append(actor)
            self.get_logger().info(
                f'Spawned {name} as {bp.id} at map ({x:.2f}, {y:.2f}) -> CARLA ({cx:.2f}, '
                f'{cy:.2f}, {z:.2f})'
            )

            if kind == 'vehicle':
                if bool(obs.get('physics', False)):
                    actor.apply_control(carla.VehicleControl(hand_brake=True))
                else:
                    actor.set_simulate_physics(False)
            elif kind == 'walker' and obs.get('walk_to'):
                tx, ty = float(obs['walk_to'][0]), float(obs['walk_to'][1])
                tcx, tcy, _ = self.geo.map_to_carla(tx, ty)
                target = carla.Location(x=tcx, y=tcy, z=self._ground_z(tcx, tcy) + 1.0)
                ctrl_bp = bp_lib.find('controller.ai.walker')
                ctrl = self.world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=actor)
                self._actors.append(ctrl)
                self._walkers.append((ctrl, target, float(obs.get('speed', 1.0))))

    def _start_walkers(self):
        self._walker_timer.cancel()
        for ctrl, target, speed in self._walkers:
            try:
                ctrl.start()
                ctrl.go_to_location(target)
                ctrl.set_max_speed(speed)
            except RuntimeError as ex:
                self.get_logger().error(f'walker start failed: {ex}')
        self.get_logger().info(f'Started {len(self._walkers)} walker(s)')

    # ------------------------------------------------------------------ #
    def destroy_actors(self):
        """
        Destroy spawned actors.

        In synchronous mode a plain destroy() is only applied at the next tick (and is lost
        if the bridge leaves first), so use apply_batch_sync with a tick cue.
        """
        if not self._actors:
            return
        for a in self._actors:
            try:
                if hasattr(a, 'stop'):
                    a.stop()
            except Exception:
                pass
        try:
            client = carla.Client(
                self.get_parameter('carla_host').value, int(self.get_parameter('carla_port').value)
            )
            client.set_timeout(5.0)
            sync = client.get_world().get_settings().synchronous_mode
            client.apply_batch_sync(
                [carla.command.DestroyActor(a.id) for a in reversed(self._actors)], sync
            )
        except Exception as ex:
            self.get_logger().warn(
                f'batch destroy failed ({ex}); falling back to per-actor destroy'
            )
            for a in reversed(self._actors):
                try:
                    a.destroy()
                except Exception:
                    pass
        self._actors.clear()


def main(args=None):
    # Own SIGINT/SIGTERM: rclpy's default handler shuts the context down AND lets Python raise
    # KeyboardInterrupt asynchronously, which used to land inside the CARLA cleanup RPCs.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    node = CarlaScenarioNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        # a stray KeyboardInterrupt must not skip the cleanup
        for _ in range(2):
            try:
                node.destroy_actors()
                break
            except KeyboardInterrupt:
                continue
        executor.remove_node(node)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
