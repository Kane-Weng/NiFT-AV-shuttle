"""
Test harness for the GNSS chain: kinematic_sim -> BrowserOctane -> fake socket.io -> beacon_node -> shuttle_tf_node / localization_node.

It builds the four repo nodes the way ARCH section 6.1 does for `sim_kinematic` (same YAML files and launch overrides), a probe node that records the topics, and a driver that plays the controller's part (a 0x101 frame and a NAVIGATING /nav_telemetry line every tick). It steps the world in the order of ARCH section 7.

fakeros is being finished in parallel. Where the wave B API is not there yet, this file falls back on the wave A world. Each fallback is marked "FAKEROS-FALLBACK" and can go once fakeros has `world.call_soon`, `world.fire_timers` (or `world.step`) and launch-name parameters.
"""

import heapq
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
PY = os.path.join(REPO, 'nift_playground', 'py')
if PY not in sys.path:
    sys.path.insert(0, PY)

from playground import paths  # noqa: E402

paths.setup(REPO)

import yaml  # noqa: E402

START_NS = 1_000_000_000
OCTANE_ENV = ('MCITY_OCTANE_SERVER', 'MCITY_OCTANE_KEY', 'MCITY_OCTANE_NAMESPACE',
              'MCITY_FRONT_BEACON_ID', 'MCITY_REAR_BEACON_ID')
ROUTE01_SPAWN = (101.52, 166.85, -1.6123)       # tests/reference/runs/gnss_outage30s_01


def flatten(d, prefix=''):
    out = {}
    for k, v in (d or {}).items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, key + '.'))
        else:
            out[key] = v
    return out


def load_yaml(*rel):
    with open(os.path.join(REPO, *rel)) as f:
        return yaml.safe_load(f)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def world():
    from fakeros._core import world as w
    return w


def fire_timers(w, t):
    """ARCH 7 step 4: fire every timer due at or before t, rcl's catch-up rule, now() = t."""
    if hasattr(w, 'fire_timers'):
        w.fire_timers(t)
        return
    # FAKEROS-FALLBACK: the wave A heap of (due, seq, timer)
    while w._timers and w._timers[0][0] <= t:
        due, _, tm = heapq.heappop(w._timers)
        if tm.cancelled:
            continue
        w.now_ns = t
        tm.callback()
        w.drain()
        if not tm.cancelled:
            nxt = due + tm.period_ns
            if nxt < t:
                nxt += -(-(t - nxt) // tm.period_ns) * tm.period_ns
            w.schedule(tm, nxt)
    w.now_ns = t


def set_params(own_name, launch_name, params):
    """Parameters of one node, as the launch set would give them."""
    # FAKEROS-FALLBACK: the wave A world keys parameters by the node's own name and has no remap table
    w = world()
    w.params[own_name] = dict(params)
    if launch_name != own_name:
        w.params[launch_name] = dict(params)


class Chain:
    """The GNSS part of the sim_kinematic launch set, plus a probe and a driver."""

    def __init__(self, localization='direct', gnss_feed='ideal', gnss_faults='', gnss_seed=1, spawn=ROUTE01_SPAWN,
                 gnss_checks=True, pedal=70, steer_deg=None, drive=True, sim_params=None):
        from nift_carla import sim_gnss
        from nift_common.vehicle import ShuttleConfig
        import socketio
        from playground.sim_backend import SimBackend

        w = world()
        w.reset()
        w.now_ns = START_NS - 50_000_000            # ARCH 7: nodes are built at START_NS - dt
        self.logs = []
        w.log_sink = self._log_sink
        shuttle = ShuttleConfig()
        carla = load_yaml('nift_simulation', 'nift_carla', 'config', 'carla_params.yaml')
        bridge = flatten(carla['carla_bridge']['ros__parameters'])
        octane = carla['carla_bridge']['ros__parameters']['octane']
        faults_catalogue = load_yaml('nift_simulation', 'nift_carla', 'config', 'gnss_faults.yaml')
        zone_sets = load_yaml('nift_simulation', 'nift_carla', 'config', 'gnss_dead_zones.yaml')
        zone_sets = zone_sets.get('sets', zone_sets) if isinstance(zone_sets, dict) else zone_sets

        # 1. kinematic_sim (kinematic_sim.launch.py _build())
        kin = dict(bridge)
        kin.update({'spawn.mode': 'map', 'spawn.x': spawn[0], 'spawn.y': spawn[1], 'spawn.yaw': spawn[2],
                    'time_factor': 0.0, 'lockstep': False, 'use_sim_time': True})
        faults_arg = gnss_faults
        if isinstance(gnss_faults, list):
            faults_arg = ''
            kin['gps.faults'] = yaml.safe_dump(gnss_faults, default_flow_style=True, width=1000).strip()
            kin['gps.faults_name'] = 'test'
        kin.update(sim_gnss.launch_overrides(gnss_feed, '', str(gnss_seed), zone_sets, faults_arg, faults_catalogue))
        kin.update(sim_params or {})
        set_params('kinematic_sim', 'kinematic_sim', kin)
        self.sim = SimBackend(**{})
        self.hub = socketio.hub

        # 2. beacon_node, with the five MCITY_OCTANE_* as components/gnss.py sets them
        self._env = {k: os.environ.get(k) for k in OCTANE_ENV}
        os.environ.update({
            'MCITY_OCTANE_SERVER': f"http://127.0.0.1:{octane['port']}",
            'MCITY_OCTANE_KEY': str(octane['api_key']),
            'MCITY_OCTANE_NAMESPACE': str(octane['namespace']),
            'MCITY_FRONT_BEACON_ID': str(octane['front_id']),
            'MCITY_REAR_BEACON_ID': str(octane['rear_id']),
        })
        bcn = flatten(load_yaml('nift_sensor', 'config', 'beacon_node.yaml')['/**']['ros__parameters'])
        bcn.update({'use_sim_time': True, 'hygiene.enabled': bool(gnss_checks)})
        set_params('nift_beacon_node', 'beacon_node', bcn)
        from nift_sensor.beacon_node import BeaconNode
        self.beacon = BeaconNode()

        # 3. / 4. the TF providers
        self.tf_node = self.loc = None
        if localization in ('direct', 'shadow'):
            tfp = flatten(load_yaml('nift_sensor', 'config', 'shuttle_tf.yaml')['shuttle_dynamic_broadcaster']
                          ['ros__parameters'])
            tfp.update({'use_sim_time': True, 'gnss.offset_x': shuttle.gnss_front_x})
            set_params('shuttle_dynamic_broadcaster', 'shuttle_dynamic_broadcaster', tfp)
            from nift_sensor.shuttle_tf_node import ShuttleDynamicBroadcaster
            self.tf_node = ShuttleDynamicBroadcaster()
        if localization in ('ekf', 'shadow'):
            lp = flatten(load_yaml('nift_sensor', 'config', 'localization.yaml')['localization_node']['ros__parameters'])
            lp.update({'use_sim_time': True, 'mode': localization, 'antenna.front_x_m': shuttle.gnss_front_x,
                       'antenna.rear_x_m': shuttle.gnss_rear_x,
                       'gnss_delay_s': 0.061 if gnss_feed == 'real' else 0.0,
                       'speed_point_x_m': -shuttle.base_link_x})
            set_params('localization_node', 'localization_node', lp)
            from nift_sensor.localization_node import LocalizationNode
            self.loc = LocalizationNode()

        # the probe and the driver (the controller's part)
        self._build_probe()
        self.drive = drive
        self.pedal = pedal
        self.steer_deg = steer_deg or (lambda t: 0.0)
        self._build_driver()
        # ARCH 6.1: drain until beacon_node has logged auth_ok (zero sim time passes)
        w.drain()

    # ------------------------------------------------------------------ #
    def _log_sink(self, *args):
        level, name, text = args[0], args[1], args[2]
        self.logs.append((world().now_ns, level, name, str(text)))

    def logs_matching(self, node, needle):
        return [(t, text) for t, _lv, name, text in self.logs if node in name and needle in text]

    def _build_probe(self):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from tf2_msgs.msg import TFMessage
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Bool, Float64, String
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticStatus

        self.probe = Node('gnss_probe')
        self.truth = {}                 # sim ns -> (x, y, yaw, v)
        self.tf = []                    # (now ns, stamp ns, x, y, yaw)
        self.fix = []                   # (now ns, stamp ns, lat, lon)
        self.heading = []               # (now ns, deg)
        self.vel = []                   # (now ns, v)
        self.loc_status = []            # (now ns, level, message)
        self.gnss_fault = []            # (now ns, dict)
        self.gps_health = []            # (now ns, bool)
        now = lambda: world().now_ns  # noqa: E731
        ns = lambda st: st.sec * 1_000_000_000 + st.nanosec  # noqa: E731

        def on_tf(m):
            for tr in m.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'base_link':
                    self.tf.append((now(), ns(tr.header.stamp), tr.transform.translation.x,
                                    tr.transform.translation.y, yaw_of(tr.transform.rotation)))

        def on_odom(m):
            p = m.pose.pose
            self.truth[ns(m.header.stamp)] = (p.position.x, p.position.y, yaw_of(p.orientation),
                                              m.twist.twist.linear.x)

        p = self.probe
        p.create_subscription(TFMessage, '/tf', on_tf, 100)
        p.create_subscription(Odometry, '/ackermann_like_controller/odom', on_odom, 10)
        p.create_subscription(NavSatFix, '/gnss/fix', lambda m: self.fix.append(
            (now(), ns(m.header.stamp), m.latitude, m.longitude)), qos_profile_sensor_data)
        p.create_subscription(Float64, '/gnss/heading_deg', lambda m: self.heading.append((now(), m.data)),
                              qos_profile_sensor_data)
        p.create_subscription(TwistStamped, '/gnss/vel', lambda m: self.vel.append((now(), m.twist.linear.x)),
                              qos_profile_sensor_data)
        p.create_subscription(DiagnosticStatus, '/localization/status', lambda m: self.loc_status.append(
            (now(), m.level, m.message)), 10)
        p.create_subscription(String, '/carla/gnss_fault', lambda m: self.gnss_fault.append(
            (now(), json.loads(m.data))), 10)
        p.create_subscription(Bool, '/carla/gps_health', lambda m: self.gps_health.append((now(), m.data)), 10)

    def _build_driver(self):
        from rclpy.node import Node
        from can_msgs.msg import Frame
        from std_msgs.msg import String
        from nift_common.can_protocol import CAN_ID_COMMAND, compute_steering, Gear

        self.driver = Node('test_driver')
        pub_can = self.driver.create_publisher(Frame, '/to_can_bus', 10)
        pub_tele = self.driver.create_publisher(String, '/nav_telemetry', 10)

        def tick():
            if not self.drive:
                return
            t = world().now_ns
            lo, hi = compute_steering(self.steer_deg((t - START_NS) / 1e9))
            f = Frame(id=CAN_ID_COMMAND, dlc=5, data=[int(Gear.FORWARD), 0, int(self.pedal), lo, hi, 0, 0, 0])
            f.header.stamp.sec, f.header.stamp.nanosec = divmod(t, 1_000_000_000)
            pub_can.publish(f)
            pub_tele.publish(String(data='STATUS:    NAVIGATING'))

        self.driver.create_timer(0.05, tick)

    # ------------------------------------------------------------------ #
    def step(self, n=1):
        w = world()
        for _ in range(n):
            t = self.sim.sim_ns + self.sim.dt_ns
            if hasattr(w, 'step'):
                w.step(t, pre=lambda *_a, _t=t: self.sim.tick(_t))
            else:
                w.now_ns = t                # 1. the clock
                self.sim.tick(t)            # 2. the sim
                w.drain()                   # 3. messages and socket.io deliveries
                fire_timers(w, t)           # 4. timers on the clock grid

    def run_until(self, t_rel_s):
        """Step until sim time - 1 s reaches t_rel_s."""
        while (self.sim.sim_ns - START_NS) / 1e9 < t_rel_s - 1e-9:
            self.step()

    @property
    def t_rel(self):
        return (self.sim.sim_ns - START_NS) / 1e9

    def close(self):
        self.sim.close()
        world().drain()
        for k, v in self._env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        world().log_sink = None

    # ------------------------------------------------------------------ #
    def truth_at(self, stamp_ns):
        """The true pose at stamp_ns, linear between ticks (the real feed stamps off the tick grid)."""
        exact = self.truth.get(stamp_ns)
        if exact is not None:
            return exact
        dt = self.sim.dt_ns
        t0 = START_NS + ((stamp_ns - START_NS) // dt) * dt
        a, b = self.truth.get(t0), self.truth.get(t0 + dt)
        if a is None or b is None:
            return None
        s = (stamp_ns - t0) / dt
        yaw = a[2] + s * wrap(b[2] - a[2])
        return (a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), yaw, a[3] + s * (b[3] - a[3]))

    def tf_errors(self, since_ns=0, use='stamp'):
        """(pos error m, yaw error rad) of each TF against the truth at its stamp (or at the time it was sent)."""
        out = []
        for now_ns, stamp_ns, x, y, yaw in self.tf:
            if now_ns < since_ns:
                continue
            tr = self.truth_at(stamp_ns if use == 'stamp' else now_ns)
            if tr is None:
                continue
            out.append((math.hypot(x - tr[0], y - tr[1]), abs(wrap(yaw - tr[2]))))
        return out
