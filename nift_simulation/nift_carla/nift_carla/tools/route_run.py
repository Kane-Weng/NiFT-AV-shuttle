#!/usr/bin/env python3
"""
Run one test route in the sim, headless; record it and measure it.

File: route_run.py

Description:
  One route of config/test_routes.yaml, end to end:

    1. launch   ros2 launch nift_bringup system.launch.py profile:=sim_kinematic rviz:=false
                spawn_x:=.. spawn_y:=.. spawn_yaw:=.. time_factor:=.. in its own process
                group, output to launch.log (--backend carla: profile:=sim perception:=false).
                Refused while a sim stack runs on this machine or is visible on this
                ROS_DOMAIN_ID. --attach: use the stack that runs already (e.g. the launcher,
                started with the route's spawn pose) and leave it running afterwards.
    2. ready    /start_navigation offered, odom and /gnss/fix flowing, waypoint_node
                listening on /goal_pose (all within --ready-timeout).
    3. record   ros2 bag record of BAG_TOPICS into <run dir>/bag, own process group.
    4. drive    /goal_pose until /planned_path (transient local) brings a path to this goal,
                /start_navigation (retried), then wait, in sim time (odom stamps), for
                  arrived    /nav_telemetry left NAVIGATING for IDLE, shuttle still STILL_S
                  safe_stop  SAFE_STOP / ESTOP
                  stalled    still for --stall-s while NAVIGATING (route 11 today)
                  timeout    the route's timeout_s (or --timeout) of sim time, or a
                             wall-clock cap
    5. stop     the recorder (SIGINT, so the bag is closed: never read a bag that is still
                being written, that crashes the Humble recorder), then the launch (SIGINT to
                its group, then TERM, KILL); checks that no process of its session is left.
    6. measure  route_metrics on the bag: metrics.json + metrics.txt; --plot: plot.png.

  Run dir (default <repo>/nift_bags/routes/<name>_<YYYYmmdd_HHMMSS>, git-ignored; --out):
  bag/, launch.log, bag_record.log, run.json (route, command, git commit, outcome),
  params/ (planning.yaml and waypoint_params.yaml as installed), metrics.json,
  metrics.txt, plot.png.

  Exit code: 0 the route completed, 1 it did not (stalled, safe stop, timeout), 2 the run
  could not be set up (stack already running or not coming up, no path, bag not closed),
  130 Ctrl-C.

  time_factor: up to 10 gives the same results on the kinematic backend; 0 (as fast as
  possible) starves beacon_node's GNSS freshness gate and /start_navigation is refused, so
  it is refused here. The tool sets no ROS_DOMAIN_ID: it uses the environment's.

Usage:
  ros2 run nift_carla sim_route_run --list
  ros2 run nift_carla sim_route_run 04 [--time-factor 5] [--plot]
  ros2 run nift_carla sim_route_run 11 --stall-s 10
  ros2 run nift_carla sim_route_run 03 --attach        # stack already up at route 03's spawn
  ros2 run nift_carla sim_route_run 04 --rviz --extra 'lockstep:=false' --out ~/runs/r04
  ros2 run nift_carla sim_route_run 04 --backend carla --time-factor 1
"""

import argparse
from datetime import datetime
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nift_bringup.components.bag import bag_dir
from nift_carla.tools import route_metrics
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

BAG_TOPICS = ['/planned_path', '/nav_telemetry', '/ackermann_like_controller/odom',
              '/to_can_bus', '/gnss/fix', '/gnss/vel', '/gnss/heading_deg', '/carla/dbw_state',
              '/clock', '/goal_pose', '/rosout', '/diagnostics',
              # the stack's believed pose; GNSS fault starts / ends (sim_gnss_bench)
              '/tf', '/carla/gnss_fault',
              # beacon_node's raw feed and dropped deliveries
              '/gnss/raw/front', '/gnss/raw/rear', '/gnss/hygiene',
              # localization_node (localization:=ekf | shadow)
              '/localization/odom', '/localization/status']
# a running sim stack: processes (any ROS domain: a second stack would also clash on the
# Octane emulator's port) and nodes (this ROS domain)
STACK_PROC = re.compile(
    r'kinematic_sim_node|carla_bridge_node|trajectory_controller|'
    r'waypoint_node|beacon_node|shuttle_tf_node|localization_node|ros2 launch nift_bringup'
)
STACK_NODES = ('trajectory_controller', 'kinematic_sim', 'carla_bridge', 'waypoint_node',
               'nift_beacon_node', 'localization_node')
PARAM_FILES = [('nift_control', 'config/planning.yaml'),
               ('nift_waypoint', 'config/waypoint_params.yaml')]

STILL_V = route_metrics.STILL_V
STILL_S = route_metrics.STILL_S
GOAL_MATCH_M = 3.0      # a /planned_path ending this close to the goal is the path to it
SPAWN_MATCH_M = 2.0     # the shuttle must start this close to the route's spawn
DEFAULT_TIME_FACTOR = {'kinematic': 5.0, 'carla': 1.0}


class SetupError(Exception):
    """The run could not be set up (exit code 2)."""


# ---------------------------------------------------------------- routes

def routes_file():
    return os.path.join(get_package_share_directory('nift_carla'), 'config', 'test_routes.yaml')


def load_routes(path):
    """Read {id: route} from a test_routes.yaml; every route gets its 'id'."""
    with open(path) as f:
        routes = (yaml.safe_load(f) or {}).get('routes') or {}
    out = {}
    for rid, r in routes.items():
        rid = str(rid)
        for key in ('name', 'spawn', 'goal'):
            if key not in r:
                raise SetupError(f'{path}: route {rid} has no {key}')
        out[rid] = dict(r, id=rid, goal=list(r['goal']) + [0.0] * (3 - len(r['goal'])))
    return out


def find_route(routes, key):
    """Find a route by id ('04', '4') or name ('04_right_turn')."""
    if key in routes:
        return routes[key]
    for r in routes.values():
        if key == r['name'] or (key.isdigit() and r['id'].isdigit() and int(key) == int(r['id'])):
            return r
    raise SetupError(f'unknown route "{key}"; known: {", ".join(routes)} (--list)')


def listing(routes):
    rows = []
    for r in routes.values():
        sx, sy, syaw = r['spawn']
        gx, gy = r['goal'][:2]
        rows.append(f'  {r["id"]:>3}  {r["name"]:<18} spawn {sx:7.2f} {sy:7.2f} {syaw:7.4f}  '
                    f'goal {gx:7.2f} {gy:7.2f}  {r.get("description", "")}')
    return '\n'.join(rows)


# ---------------------------------------------------------------- processes

def launch_command(route, args, time_factor):
    """Build the ros2 launch command line of this run."""
    sx, sy, syaw = route['spawn']
    cmd = ['ros2', 'launch', 'nift_bringup', 'system.launch.py']
    if args.backend == 'kinematic':
        cmd.append('profile:=sim_kinematic')
    else:
        cmd += ['profile:=sim', 'perception:=false']
    cmd += [f'rviz:={"true" if args.rviz else "false"}', f'spawn_x:={sx}', f'spawn_y:={sy}',
            f'spawn_yaw:={syaw}', f'time_factor:={time_factor:g}']
    if args.map_file:
        cmd.append(f'map_file:={args.map_file}')
    extra = shlex.split(args.extra or '')
    bad = [e for e in extra if ':=' not in e]
    if bad:
        raise SetupError(f'--extra takes launch arguments name:=value, not {bad}')
    return cmd + extra


def processes():
    """
    List [(pid, session id, command line)] of every live process.

    Zombies are left out: a defunct node is no running stack (in a container whose init
    does not reap, e.g. `sleep infinity`, they stay for good).
    """
    out = subprocess.run(
        ['ps', '-eo', 'pid=,sid=,stat=,args='], capture_output=True, text=True
    ).stdout
    procs = []
    for line in out.splitlines():
        parts = line.split(None, 3)
        if len(parts) == 4 and not parts[2].startswith('Z'):
            procs.append((int(parts[0]), int(parts[1]), parts[3]))
    return procs


def stack_processes():
    """List the sim stack processes (STACK_PROC) other than this one."""
    me = os.getpid()
    return [(p, s, a) for p, s, a in processes() if p != me and STACK_PROC.search(a)]


def session_processes(sid):
    """List the processes left in session sid (start_new_session: sid = the child's pid)."""
    return [(p, s, a) for p, s, a in processes() if s == sid and p != os.getpid()]


def start(cmd, log_path):
    """Start cmd in its own session / process group, output to log_path."""
    log = open(log_path, 'w')
    log.write('$ ' + ' '.join(shlex.quote(c) for c in cmd) + '\n')
    log.flush()
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True)


def stop_group(proc, int_wait, term_wait=5.0):
    """SIGINT the process group of proc, then TERM, then KILL; True if SIGINT was enough."""
    if proc is None or proc.poll() is not None:
        return True
    for sig, wait in ((signal.SIGINT, int_wait), (signal.SIGTERM, term_wait),
                      (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return True
        try:
            proc.wait(timeout=wait)
            return sig == signal.SIGINT
        except subprocess.TimeoutExpired:
            continue
    return False


def log_tail(path, n=25):
    try:
        with open(path, errors='replace') as f:
            return ''.join(f.readlines()[-n:])
    except OSError:
        return ''


def git_state(repo):
    """Return {'commit': short hash, 'dirty_files': n} of the checkout ({} if none)."""
    try:
        def git(*a):
            return subprocess.run(['git', '-C', str(repo), *a], capture_output=True, text=True,
                                  timeout=10).stdout
        commit = git('rev-parse', '--short', 'HEAD').strip()
        dirty = git('status', '--porcelain', '--untracked-files=no').splitlines()
        return {'commit': commit, 'dirty_files': len(dirty)} if commit else {}
    except (OSError, subprocess.SubprocessError):
        return {}


def copy_params(run_dir):
    """Copy the parameter files the stack reads at launch into params/, to compare runs."""
    os.makedirs(os.path.join(run_dir, 'params'), exist_ok=True)
    for pkg, rel in PARAM_FILES:
        try:
            shutil.copy(os.path.join(get_package_share_directory(pkg), rel),
                        os.path.join(run_dir, 'params', os.path.basename(rel)))
        except Exception as e:      # a missing copy must not stop the run
            print(f'  (could not copy {pkg}/{rel}: {e})')


# ---------------------------------------------------------------- ROS side

class RouteRunner(Node):
    """Watches the stack (odom, telemetry, path, GNSS), sends the goal, starts navigation."""

    def __init__(self):
        super().__init__('sim_route_run', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = None            # (sim t, x, y, v) of the last odom
        self.odom_count = 0
        self.odom_wall = None
        self.fix_wall = None
        self.status = []            # [(sim t, STATUS)]
        self.telemetry = {}
        self.paths = []             # [(wall t, Path)] received
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planned_path',
                                 lambda m: self.paths.append((time.time(), m)), latched)
        self.create_subscription(Odometry, route_metrics.ODOM, self._odom, 50)
        self.create_subscription(String, '/nav_telemetry', self._telemetry, 50)
        self.create_subscription(NavSatFix, '/gnss/fix', self._fix, qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.start_client = self.create_client(Trigger, '/start_navigation')

    def _odom(self, m):
        p = m.pose.pose.position
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.odom = (route_metrics.stamp_s(m.header.stamp), p.x, p.y, v)
        self.odom_count += 1
        self.odom_wall = time.time()

    def _fix(self, _m):
        self.fix_wall = time.time()

    def _telemetry(self, m):
        f = route_metrics.parse_telemetry(m.data)
        if 'STATUS' in f:
            self.status.append((self.sim_now(), f['STATUS']))
            self.telemetry = f

    def sim_now(self):
        return self.odom[0] if self.odom else 0.0

    def spin_for(self, wall_s, until=None):
        """Spin up to wall_s (wall clock) or until until() is true; return until()'s value."""
        end = time.time() + wall_s
        while time.time() < end:
            if until is not None and until():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(until()) if until is not None else False

    def visible_stack_nodes(self):
        return sorted({n for n, _ in self.get_node_names_and_namespaces() if n in STACK_NODES})


def _interrupt(*_):
    raise KeyboardInterrupt


def say(t0, msg):
    print(f'{time.time() - t0:6.1f}s  {msg}', flush=True)


def wait_ready(node, launch, timeout, t0):
    """Block until the stack can take a goal; SetupError with the reason after timeout."""
    end = time.time() + timeout
    checks = [
        ('/start_navigation offered', node.start_client.service_is_ready),
        ('odom flowing', lambda: node.odom_count >= 10 and time.time() - node.odom_wall < 1.0),
        ('/gnss/fix fresh', lambda: node.fix_wall and time.time() - node.fix_wall < 1.0),
        ('waypoint_node on /goal_pose', lambda: node.count_subscribers('/goal_pose') >= 1),
    ]
    for what, ok in checks:
        while not ok():
            if launch is not None and launch.poll() is not None:
                raise SetupError(f'the launch exited (code {launch.returncode}) before: {what}')
            if time.time() > end:
                raise SetupError(f'stack not ready after {timeout:.0f} s: no {what}')
            node.spin_for(0.2)
        say(t0, f'ready: {what}')


def send_goal(node, route, timeout=30.0):
    """Publish the goal until /planned_path brings a path ending at it; return its length."""
    gx, gy, gyaw = route['goal']
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x, goal.pose.position.y = float(gx), float(gy)
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    n_seen, sent_at, first_stamp = len(node.paths), time.time(), None
    end = time.time() + timeout
    while time.time() < end:
        goal.header.stamp = node.get_clock().now().to_msg()
        if first_stamp is None:
            first_stamp = route_metrics.stamp_s(goal.header.stamp)
        node.goal_pub.publish(goal)
        node.spin_for(2.0, lambda: len(node.paths) > n_seen)
        for wall, p in node.paths[n_seen:]:
            st = route_metrics.stamp_s(p.header.stamp)
            if wall < sent_at or (st > 0 and first_stamp > 0 and st < first_stamp):
                continue                        # the latched path of an earlier goal
            if len(p.poses) < 2:
                raise SetupError('waypoint_node published an empty /planned_path: no route to '
                                 'the goal (see launch.log)')
            last = p.poses[-1].pose.position
            if math.hypot(last.x - gx, last.y - gy) <= GOAL_MATCH_M:
                xy = [(q.pose.position.x, q.pose.position.y) for q in p.poses]
                return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(xy, xy[1:]))
        n_seen = len(node.paths)
    raise SetupError(f'no /planned_path to the goal ({gx}, {gy}) within {timeout:.0f} s')


def start_navigation(node, t0, attempts=10):
    """Call /start_navigation until it accepts; SetupError with the last reply if never."""
    reply = None
    for _ in range(attempts):
        fut = node.start_client.call_async(Trigger.Request())
        node.spin_for(5.0, fut.done)
        reply = fut.result() if fut.done() else None
        if reply is not None and reply.success:
            say(t0, f'/start_navigation: {reply.message}')
            return
        node.spin_for(1.0)
    raise SetupError(f'/start_navigation refused: {reply.message if reply else "no reply"}')


def drive(node, launch, timeout_s, stall_s, time_factor, t0):
    """Wait for the end of the route; return (outcome, detail)."""
    sim0, wall0 = node.sim_now(), time.time()
    wall_cap = timeout_s / max(time_factor or 1.0, 0.1) * 2.0 + 60.0
    n_seen = len(node.status)
    seen_nav = left = still_since = None
    moved, last_print = False, 0.0
    while True:
        node.spin_for(0.1)
        now, wall, v = node.sim_now(), time.time(), node.odom[3]
        moved = moved or v > route_metrics.MOVING_V
        if v >= STILL_V:
            still_since = None
        elif still_since is None:
            still_since = now
        still = now - still_since if still_since is not None else 0.0
        for t, s in node.status[n_seen:]:
            if seen_nav is None and s == 'NAVIGATING':
                seen_nav = t
            elif seen_nav is not None and left is None and s != 'NAVIGATING':
                left = (t, s)
        n_seen = len(node.status)
        if wall - last_print > 5.0:
            last_print = wall
            tel = node.telemetry
            say(t0, f'sim {now - sim0:6.1f} s  v {v:4.2f} m/s  {tel.get("STATUS", "-"):<10} '
                    f'dist end {tel.get("Dist End", "-"):>9}  CTE {tel.get("CTE", "-")}')
        if left is not None:
            after = f'after {left[0] - sim0:.1f} s (sim)'
            if left[1] == 'IDLE' and still >= STILL_S:
                return 'arrived', f'NAVIGATING -> IDLE {after}, at rest'
            if left[1] in ('SAFE_STOP', 'ESTOP') and (still >= STILL_S or now - left[0] > 15):
                return left[1].lower(), f'{left[1]} {after}'
            if left[1] not in ('IDLE', 'SAFE_STOP', 'ESTOP'):
                return left[1].lower(), f'left NAVIGATING for {left[1]} {after}'
        elif still >= stall_s:
            where = 'in NAVIGATING' if moved else 'without moving off'
            return 'stalled', (f'still for {still:.1f} s (sim) {where}, '
                               f'{now - sim0:.1f} s after the start')
        if now - sim0 > timeout_s:
            return 'timeout', f'not done after {timeout_s:.0f} s (sim)'
        if wall - wall0 > wall_cap:
            return 'timeout', (f'not done after {wall_cap:.0f} s wall clock '
                               f'(sim clock at {now - sim0:.1f} s)')
        if wall - node.odom_wall > 10.0:
            return 'no_odom', 'no odom for 10 s: the sim stopped?'
        if launch is not None and launch.poll() is not None:
            return 'stack_exited', f'the launch exited (code {launch.returncode})'


# ---------------------------------------------------------------- one run

class RouteRun:
    """One run: its processes, its run dir and run.json."""

    def __init__(self, args, route, time_factor, timeout_s, run_dir):
        self.args, self.route, self.run_dir = args, route, run_dir
        self.time_factor, self.timeout_s = time_factor, timeout_s
        self.t0 = time.time()
        self.node = self.launch = self.recorder = None
        self.bag_closed = False
        self.info = {
            'route': route['id'], 'name': route['name'],
            'description': route.get('description', ''), 'spawn': route['spawn'],
            'goal': route['goal'][:2], 'backend': args.backend, 'attach': args.attach,
            'time_factor': None if args.attach else time_factor, 'map_file': args.map_file,
            'timeout_s': timeout_s, 'stall_s': args.stall_s,
            'started': datetime.now().isoformat(timespec='seconds'),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
            'git': git_state(bag_dir().parent)}

    def say(self, msg):
        say(self.t0, msg)

    def path(self, *rel):
        return os.path.join(self.run_dir, *rel)

    def write_info(self, **more):
        self.info.update(more)
        with open(self.path('run.json'), 'w') as f:
            json.dump(self.info, f, indent=2)

    def bring_up(self):
        """Launch the stack (or find it with --attach) and wait until it takes a goal."""
        node, args, route = self.node, self.args, self.route
        node.spin_for(2.0)                     # discovery
        if args.attach:
            if not node.spin_for(5.0, lambda: 'trajectory_controller' in
                                 node.visible_stack_nodes()):
                raise SetupError('--attach: no trajectory_controller on ROS_DOMAIN_ID '
                                 f'{os.environ.get("ROS_DOMAIN_ID", "0")}: is the stack running?')
            self.info['command'] = '(attached to a running stack)'
        else:
            visible = node.visible_stack_nodes()
            if visible:
                raise SetupError(f'stack nodes already visible on this ROS_DOMAIN_ID: '
                                 f'{", ".join(visible)}; stop them first, or use --attach')
            cmd = launch_command(route, args, self.time_factor)
            self.info['command'] = ' '.join(cmd)
            self.say(f'launch: {self.info["command"]}  (log: launch.log)')
            self.launch = start(cmd, self.path('launch.log'))
        copy_params(self.run_dir)
        self.write_info()
        wait_ready(node, self.launch, args.ready_timeout, self.t0)

        sx, sy = route['spawn'][:2]
        node.spin_for(10.0, lambda: node.odom[3] < STILL_V)
        x, y = node.odom[1:3]
        off = math.hypot(x - sx, y - sy)
        if off > SPAWN_MATCH_M:
            msg = (f'the shuttle is at ({x:.2f}, {y:.2f}), {off:.1f} m from route '
                   f'{route["id"]}\'s spawn ({sx}, {sy})')
            if not (args.attach and args.any_start):
                if args.attach:
                    msg += ('; start the stack with spawn_x:={} spawn_y:={} spawn_yaw:={}, or '
                            'pass --any-start'.format(*route['spawn']))
                raise SetupError(msg)
            self.say(f'WARNING: {msg} (--any-start)')

    def record(self):
        """Start ros2 bag record and wait until it subscribed."""
        node = self.node
        before = node.count_subscribers(route_metrics.ODOM)
        self.recorder = start(['ros2', 'bag', 'record', '-o', self.path('bag')] + BAG_TOPICS,
                              self.path('bag_record.log'))
        # the goal publisher exists already, so the recorder subscribes to /goal_pose at once
        if not node.spin_for(20.0, lambda: node.count_subscribers(route_metrics.ODOM) > before
                             and node.count_subscribers('/goal_pose') >= 2):
            raise SetupError('ros2 bag record did not subscribe within 20 s (bag_record.log):\n'
                             + log_tail(self.path('bag_record.log'), 10))
        node.spin_for(1.0)
        self.say(f'recording {len(BAG_TOPICS)} topics into bag/')

    def stop(self):
        """Stop the recorder, then the launch; report what is left."""
        if self.recorder is not None:
            clean = stop_group(self.recorder, int_wait=20.0)
            self.bag_closed = os.path.isfile(self.path('bag', 'metadata.yaml'))
            if self.bag_closed:
                self.say('bag closed')
            else:
                self.say(f'BAG NOT CLOSED (recorder {"stopped" if clean else "killed"}; try '
                         f'ros2 bag reindex {self.path("bag")})')
        if self.launch is not None:
            clean = stop_group(self.launch, int_wait=30.0, term_wait=10.0)
            left = session_processes(self.launch.pid)
            for p, _, _ in left:
                try:
                    os.kill(p, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            note = '' if clean else ' (needed SIGTERM/SIGKILL)'
            if left:
                note += f'; killed {len(left)} stray process(es): ' + ', '.join(
                    a[:60] for _, _, a in left)
            else:
                note += ', no process of it left'
            self.say('stack stopped' + note)
            others = stack_processes()
            if others:
                self.say('note: other sim stack processes run on this machine (not ours, left '
                         'alone): ' + ', '.join(str(p) for p, _, _ in others))
        elif self.args.attach:
            self.say('attached: the stack keeps running')

    def run(self):
        """Run the route; return the exit code."""
        outcome, detail, code = 'setup_failed', '', 2
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        prev_term = signal.signal(signal.SIGTERM, _interrupt)
        created = not os.path.exists(self.run_dir)
        try:
            os.makedirs(self.run_dir, exist_ok=True)
            self.say(f'route {self.route["id"]} {self.route["name"]}: '
                     f'{self.route.get("description", "")}')
            self.say(f'run dir {self.run_dir}')
            self.node = RouteRunner()
            self.bring_up()
            self.record()
            length = send_goal(self.node, self.route)
            self.say(f'path: {length:.1f} m to the goal {tuple(self.route["goal"][:2])}')
            start_navigation(self.node, self.t0)
            outcome, detail = drive(self.node, self.launch, self.timeout_s, self.args.stall_s,
                                    self.info['time_factor'], self.t0)
            self.say(f'{outcome}: {detail}')
            code = 0 if outcome == 'arrived' else 1
        except SetupError as e:
            detail = str(e)
            self.say(f'SETUP FAILED: {e}')
            if self.launch is not None:
                print(log_tail(self.path('launch.log')))
        except KeyboardInterrupt:
            outcome, detail, code = 'interrupted', 'Ctrl-C', 130
            self.say('interrupted: stopping the bag and the stack')
        finally:
            signal.signal(signal.SIGINT, lambda *_: print('  (stopping, please wait)'))
            if self.node is not None:
                self.node.destroy_node()
            rclpy.try_shutdown()
            self.stop()
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, prev_term)
        if created and self.launch is None and self.recorder is None:
            shutil.rmtree(self.run_dir, ignore_errors=True)     # nothing ran: leave no dir
            return code
        self.write_info(outcome=outcome, detail=detail, wall_s=round(time.time() - self.t0, 1))
        if self.bag_closed:
            code = self.measure(code)
        self.say(f'run dir {self.run_dir}')
        return code

    def measure(self, code):
        """Metrics (and plot) of the closed bag; a route that did not complete gives 1."""
        try:
            m = route_metrics.compute(route_metrics.load_run(self.run_dir))
        except (OSError, ValueError) as e:
            self.say(f'metrics: {e}')
            return code or 1
        route_metrics.write_reports(m, self.run_dir)
        print('\n' + route_metrics.summary(m) + '\n\n' + route_metrics.TABLE_HEAD + '\n'
              + route_metrics.table_row(m))
        if self.args.plot:
            from nift_carla.tools import route_plot
            self.say('plot: ' + route_plot.plot_runs([self.run_dir], self.path('plot.png'),
                                                     map_file=self.args.map_file))
        return 1 if code == 0 and not m['completed'] else code


# ---------------------------------------------------------------- main

def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('route', nargs='?', help='route id (04) or name (04_right_turn)')
    ap.add_argument('--list', action='store_true', help='print the routes and exit')
    ap.add_argument('--routes', help='routes file (default: the installed test_routes.yaml)')
    ap.add_argument('--attach', action='store_true',
                    help="use the stack that runs already (started at the route's spawn) and "
                         'leave it running')
    ap.add_argument('--backend', choices=('kinematic', 'carla'), default='kinematic')
    ap.add_argument('--time-factor', type=float,
                    help='sim speed-up: default 5 (kinematic) / 1 (carla); not 0. Not used '
                         'with --attach')
    ap.add_argument('--rviz', action=argparse.BooleanOptionalAction, default=False,
                    help='start RViz (default off)')
    ap.add_argument('--map-file', help="map_file:= (a file in nift_waypoint's maps/ or a path)")
    ap.add_argument('--extra', help="more launch arguments, e.g. 'lockstep:=false'")
    ap.add_argument('--out', help='run dir (default <repo>/nift_bags/routes/<name>_<date>)')
    ap.add_argument('--timeout', type=float,
                    help="sim seconds after the start (default: the route's timeout_s)")
    ap.add_argument('--stall-s', type=float, default=10.0,
                    help='sim seconds still in NAVIGATING that count as a stall (default 10)')
    ap.add_argument('--ready-timeout', type=float, default=120.0,
                    help='wall seconds for the stack to come up (default 120)')
    ap.add_argument('--any-start', action='store_true',
                    help="--attach: run even if the shuttle is not at the route's spawn")
    ap.add_argument('--plot', action='store_true', help='also write plot.png (sim_route_plot)')
    return ap.parse_args(argv)


def main(argv=None):
    sys.stdout.reconfigure(line_buffering=True)
    # Ctrl-C always stops the run cleanly, also when started with SIGINT ignored (from `&`)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    args = parse_args(argv)
    try:
        routes = load_routes(args.routes or routes_file())
        if args.list or not args.route:
            print(f'Routes ({args.routes or routes_file()}):\n{listing(routes)}')
            return 0
        route = find_route(routes, args.route)
    except (OSError, SetupError) as e:
        print(f'sim_route_run: {e}', file=sys.stderr)
        return 2

    time_factor = args.time_factor
    if time_factor is None:
        time_factor = DEFAULT_TIME_FACTOR[args.backend]
    if time_factor <= 0:
        print('sim_route_run: time_factor must be > 0: at 0 the free-running sim clock outruns '
              "the Octane socket, beacon_node's GNSS freshness gate trips and "
              '/start_navigation is refused', file=sys.stderr)
        return 2
    if time_factor > 10:
        print(f'WARNING: time_factor {time_factor:g}: only up to 10 is known to give the same '
              'results')
    timeout_s = args.timeout or float(route.get('timeout_s', 240))
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.abspath(os.path.expanduser(
        args.out or os.path.join(bag_dir(), 'routes', f'{route["name"]}_{stamp}')))
    if os.path.exists(os.path.join(run_dir, 'bag')):
        print(f'sim_route_run: {run_dir}/bag exists already; pick another --out',
              file=sys.stderr)
        return 2
    if not args.attach:
        busy = stack_processes()
        if busy:
            print('sim_route_run: a sim stack is already running on this machine; stop it '
                  'first, or use --attach to drive it:\n'
                  + '\n'.join(f'  {p}  {a[:100]}' for p, _, a in busy), file=sys.stderr)
            return 2
    return RouteRun(args, route, time_factor, timeout_s, run_dir).run()


if __name__ == '__main__':
    sys.exit(main())
