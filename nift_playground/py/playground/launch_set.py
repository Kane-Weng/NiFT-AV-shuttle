"""
The playground's launch set: the nodes of `profile:=sim_kinematic`, with the same parameter sources, remaps and environment as nift_bringup (ARCH section 6).

The component set comes from the repo's own `nift_bringup.profiles.resolve()`, which has no launch imports and so runs in the browser too. Each node gets the list of parameter sources bringup gives it, in the same order. `effective_params()` merges them the way rcl and rclpy do (ARCH 6.2). `tests/test_launch_set.py` runs bringup's components with stub launch modules and checks that both sides give the same sources.

Nothing here constructs a node. The runner does that, in the order of `LaunchSet.nodes`.
"""

import copy
from dataclasses import dataclass, field
import os
import tempfile

import yaml

from . import paths

# ARCH 6.4: the page's launch options and their defaults
DEFAULT_OPTIONS = {
    'localization': 'direct',
    'gnss_feed': 'ideal',
    'gnss_faults': '',
    'gnss_dead_zones': '',
    'gnss_seed': 1,
    'gnss_checks': 'true',
    'perception': False,
    'filter_type': '',
    'spawn': None,
    'params': {},
}
LOCALIZATION_MODES = ('direct', 'ekf', 'shadow')
FILTER_TYPES = ('', 'distance_stop', 'cbf', 'potential_field', 'none')
PROFILE = 'sim_kinematic'
MAP_REL = 'maps/lanelet2_odr2024.osm'

# components/localization.py SIM_REAL_FEED_DELAY_S (bringup imports launch_ros, so it is copied; the launch set test catches drift)
SIM_REAL_FEED_DELAY_S = 0.061

# ARCH 6.3: waypoint_node's map_publish_period_s, the only override of a repo node's parameter. None keeps waypoint_params.yaml's value.
# Measured natively on route 03 (2026-10-20): the 1 s republish of /lanelet_map_markers took 85 % of the run time, for no subscriber,
# so the playground publishes the markers once at start-up and then every hour.
WAYPOINT_MAP_PUBLISH_PERIOD_S = 3600.0

# where a list of GNSS faults goes as a file (a fixed name: it is echoed on /carla/gnss_fault, and a random name would break replay)
FAULTS_FILE = 'playground_gnss_faults.yaml'
# label of the playground's own override source (ARCH 6.3), left out of the comparison with bringup
PLAYGROUND_OVERRIDE = 'playground override (ARCH 6.3)'

# repo-relative config files
CARLA_PARAMS = 'nift_simulation/nift_carla/config/carla_params.yaml'
TEST_ROUTES = 'nift_simulation/nift_carla/config/test_routes.yaml'
GNSS_FAULTS = 'nift_simulation/nift_carla/config/gnss_faults.yaml'
GNSS_DEAD_ZONES = 'nift_simulation/nift_carla/config/gnss_dead_zones.yaml'
PROFILES = 'nift_bringup/config/profiles.yaml'


# ARCH 8.1: the parameters the page's set_param may stage, with their range (web/src/panels/presets.ts PARAMS).
# All are doubles.
PAGE_PARAMS = {
    ('trajectory_controller', 'base_speed'): (0.5, 6.0),
    ('trajectory_controller', 'pure_pursuit_ld_min'): (1.0, 15.0),
    ('trajectory_controller', 'pure_pursuit_ld_max'): (1.0, 20.0),
    ('trajectory_controller', 'goal_tolerance'): (0.2, 5.0),
    ('trajectory_controller', 'decel_distance'): (1.0, 30.0),
    ('trajectory_controller', 'cte_tolerance'): (0.2, 5.0),
    ('trajectory_controller', 'min_safe_dist'): (0.5, 10.0),
    ('trajectory_controller', 'collision_zone_dist'): (0.2, 5.0),
}


def doubles_as_floats(node: str, values: dict, known: dict) -> dict:
    """
    Page, scenario and CLI parameter values with each whole number made a float where the parameter is a double.

    JSON in JavaScript writes 2.0 as 2, and rclpy refuses an integer for a double parameter. A parameter is a double when it is one of PAGE_PARAMS or when its value from the YAML files (`known`) is a float.
    """
    out = {}
    for name, v in values.items():
        if (isinstance(v, int) and not isinstance(v, bool)
                and ((node, name) in PAGE_PARAMS or isinstance(known.get(name), float))):
            v = float(v)
        out[name] = v
    return out


class LaunchError(ValueError):
    """Bad launch options (the page gets the message)."""


# ================================================================ parameter sources

@dataclass
class Source:
    """
    One entry of a Node's `parameters=[...]`.

    `table` is what rcl reads from it: {node key: {flat name: value}}. A YAML file keeps its own keys. A dict is a `/**` file, as launch_ros writes it.
    """

    kind: str               # 'file' | 'dict'
    label: str              # repo-relative path, or a short name of the dict
    table: dict
    raw: object = None      # what a launch file passes: the file's path, the dict, or ('scoped', node, params)

    def describe(self) -> dict:
        return {'kind': self.kind, 'label': self.label, 'table': self.table}


def flatten(d: dict, prefix: str = '') -> dict:
    """Nested maps to dotted names, as rcl reads a YAML file and launch_ros a dict."""
    out = {}
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict):
            out.update(flatten(v, key))
        else:
            out[key] = v
    return out


def file_source(rel: str) -> Source:
    with open(paths.repo_path(rel)) as f:
        doc = yaml.safe_load(f) or {}
    table = {}
    for key, body in doc.items():
        if isinstance(body, dict) and isinstance(body.get('ros__parameters'), dict):
            table[str(key)] = flatten(body['ros__parameters'])
    return Source('file', rel, table, paths.repo_path(rel))


def dict_source(label: str, params: dict) -> Source:
    return Source('dict', label, {'/**': flatten(params)}, copy.deepcopy(params))


def scoped_source(label: str, node: str, params: dict) -> Source:
    """A node-scoped file, as components/common.py node_params() writes it."""
    return Source('file', label, {node: flatten(params)}, ('scoped', node, dict(params)))


def _key_matches(key: str, node: str) -> bool:
    return key in ('/**', '**', '/*', node, '/' + node)


def effective_params(sources, node: str) -> dict:
    """
    The parameter values node `node` gets from `sources` (ARCH 6.2).

    rcl merges every file into one table of node keys, in the order each key first appears, a later file winning within a key. rclpy then applies the keys that match the node in table order, so a later key wins over an earlier one.
    """
    table = {}
    for src in sources:
        for key, params in src.table.items():
            table.setdefault(key, {}).update(params)
    out = {}
    for key, params in table.items():
        if _key_matches(key, node):
            out.update(params)
    return out


# ================================================================ options

def load_yaml(rel: str):
    with open(paths.repo_path(rel)) as f:
        return yaml.safe_load(f)


def routes() -> dict:
    """test_routes.yaml routes: {'01': {name, description, spawn, goal, timeout_s}, ...}."""
    return {str(k): v for k, v in (load_yaml(TEST_ROUTES).get('routes') or {}).items()}


def carla_bridge_params() -> dict:
    return load_yaml(CARLA_PARAMS)['carla_bridge']['ros__parameters']


def normalize_options(launch=None, base=None) -> dict:
    """Merge `launch` (partial, ARCH 6.4) over `base` (default: DEFAULT_OPTIONS), check it, return a full copy."""
    opts = copy.deepcopy(base if base is not None else DEFAULT_OPTIONS)
    for k, v in (launch or {}).items():
        if k not in DEFAULT_OPTIONS:
            raise LaunchError(f'unknown launch option {k!r}; known: {sorted(DEFAULT_OPTIONS)}')
        opts[k] = copy.deepcopy(v)
    if opts['localization'] not in LOCALIZATION_MODES:
        raise LaunchError(f"localization {opts['localization']!r}: expected one of {LOCALIZATION_MODES}")
    if opts['gnss_feed'] not in ('ideal', 'real'):
        raise LaunchError(f"gnss_feed {opts['gnss_feed']!r}: expected ideal or real")
    if str(opts['gnss_checks']).lower() not in ('true', 'false'):
        raise LaunchError(f"gnss_checks {opts['gnss_checks']!r}: expected true or false")
    opts['gnss_checks'] = str(opts['gnss_checks']).lower()
    if isinstance(opts['perception'], str):
        opts['perception'] = opts['perception'].lower() in ('true', '1', 'yes', 'on')
    opts['perception'] = bool(opts['perception'])
    try:
        opts['gnss_seed'] = int(opts['gnss_seed'])
    except (TypeError, ValueError):
        raise LaunchError(f"gnss_seed {opts['gnss_seed']!r}: expected an integer") from None
    if opts['filter_type'] not in FILTER_TYPES:
        raise LaunchError(f"filter_type {opts['filter_type']!r}: expected one of {FILTER_TYPES}")
    if opts['filter_type'] and not opts['perception']:
        if opts['filter_type'] != 'none':
            raise LaunchError(f"filter_type {opts['filter_type']} needs perception: without /obstacles it sees an empty road")
    if isinstance(opts['gnss_dead_zones'], (list, tuple)):
        opts['gnss_dead_zones'] = ','.join(opts['gnss_dead_zones'])
    if not isinstance(opts['gnss_faults'], (str, list)):
        raise LaunchError('gnss_faults: expected a scenario name or a list of faults')
    spawn = opts['spawn']
    if spawn is not None and not isinstance(spawn, (str, dict)):
        raise LaunchError('spawn: expected {x, y, yaw} or a route id')
    if isinstance(spawn, str):
        if spawn not in routes():
            raise LaunchError(f'spawn: unknown route {spawn!r}; known: {sorted(routes())}')
    elif isinstance(spawn, dict):
        try:
            opts['spawn'] = {'x': float(spawn['x']), 'y': float(spawn['y']), 'yaw': float(spawn.get('yaw', 0.0))}
        except (KeyError, TypeError, ValueError):
            raise LaunchError('spawn: expected {x, y, yaw} numbers') from None
    if not isinstance(opts['params'], dict):
        raise LaunchError('params: expected {node: {name: value}}')
    return opts


def spawn_pose(opts):
    """(x, y, yaw) of the spawn option, None for carla_params.yaml's."""
    spawn = opts['spawn']
    if spawn is None:
        return None
    if isinstance(spawn, str):
        x, y, yaw = routes()[spawn]['spawn']
        return float(x), float(y), float(yaw)
    return spawn['x'], spawn['y'], spawn['yaw']


def _num(v: float) -> str:
    """A launch argument string for a number (what `ros2 launch ... spawn_x:=1.53` would carry)."""
    return repr(float(v))


_PROCESS_WORKDIR = []


def default_workdir() -> str:
    """A temp folder of this process, for launch sets built outside a Session."""
    if not _PROCESS_WORKDIR:
        _PROCESS_WORKDIR.append(tempfile.mkdtemp(prefix='nift_playground_'))
    return _PROCESS_WORKDIR[0]


def faults_arg(opts, workdir=None) -> str:
    """The gnss_faults:= argument: a scenario name, or a list written to a YAML file in this session's folder."""
    faults = opts['gnss_faults']
    if isinstance(faults, str):
        return faults
    if not faults:
        return ''
    path = os.path.join(workdir or default_workdir(), FAULTS_FILE)
    with open(path, 'w') as f:
        yaml.safe_dump({'faults': [dict(d) for d in faults]}, f)
    return path


def cli_args(opts, workdir=None) -> dict:
    """The launch arguments of `ros2 launch nift_bringup system.launch.py profile:=sim_kinematic ...` for these options."""
    cli = {
        'localization': opts['localization'],
        'rviz': 'false',
        'time_factor': '0',
        'lockstep': 'false',
        'gnss_feed': opts['gnss_feed'],
        'gnss_dead_zones': opts['gnss_dead_zones'],
        'gnss_seed': str(opts['gnss_seed']),
        'gnss_faults': faults_arg(opts, workdir),
        'gnss_checks': opts['gnss_checks'],
    }
    pose = spawn_pose(opts)
    if pose is not None:
        cli['spawn_x'], cli['spawn_y'], cli['spawn_yaw'] = (_num(v) for v in pose)
    # perception is a playground extra (ARCH 6.3): bringup resolves without it, and the
    # filter_type then has to be none or ''
    if not opts['perception'] and opts['filter_type']:
        cli['filter_type'] = opts['filter_type']
    return cli


def resolve_plan(opts, workdir=None):
    from nift_bringup import profiles
    try:
        return profiles.resolve(profiles.load(paths.repo_path(PROFILES)), PROFILE, cli_args(opts, workdir))
    except profiles.ProfileError as e:
        raise LaunchError(str(e)) from None


# ================================================================ nodes

@dataclass
class NodeSpec:
    name: str                       # launch name (parameters, logging, diagnostics)
    own_name: str                   # the name the node passes to Node.__init__
    module: str
    cls: str
    sources: list
    env: dict = field(default_factory=dict)
    after_auth: bool = False        # bringup starts it once beacon_node logs auth_ok
    repo: bool = True               # a repo node (False: the playground's own)

    @property
    def params(self) -> dict:
        return effective_params(self.sources, self.name)

    def describe(self) -> dict:
        return {'name': self.name, 'own_name': self.own_name, 'module': self.module, 'class': self.cls,
                'repo': self.repo, 'after_auth': self.after_auth,
                'sources': [s.label for s in self.sources], 'env': sorted(self.env)}


def _sim_time(plan) -> Source:
    return dict_source('{use_sim_time}', {'use_sim_time': plan.sim})


def _map_origin(plan) -> dict:
    return {'map_origin_lat': float(plan.args['map_origin_lat']),
            'map_origin_lon': float(plan.args['map_origin_lon'])}


def sim_feed_real(plan) -> bool:
    """components/localization.py sim_feed_real()."""
    if plan.args['gnss_feed']:
        return plan.args['gnss_feed'] == 'real'
    return bool(carla_bridge_params().get('gps', {}).get('feed', {}).get('enabled', False))


def filter_params(plan, mode) -> dict:
    """components/localization.py filter_params()."""
    from nift_common.vehicle import ShuttleConfig
    shuttle = ShuttleConfig()
    params = {'mode': mode, 'antenna.front_x_m': shuttle.gnss_front_x,
              'antenna.rear_x_m': shuttle.gnss_rear_x, **_map_origin(plan)}
    if plan.sim:
        params['gnss_delay_s'] = SIM_REAL_FEED_DELAY_S if sim_feed_real(plan) else 0.0
        params['speed_point_x_m'] = -shuttle.base_link_x if plan.kinematic else 0.0
    return params


def kinematic_overrides(plan) -> dict:
    """kinematic_sim.launch.py _build(): the dict after carla_params.yaml's section."""
    from nift_carla.sim_gnss import launch_overrides
    a = plan.args
    overrides = {'map_origin_lat': float(a['map_origin_lat']), 'map_origin_lon': float(a['map_origin_lon'])}
    if a['spawn_x'] != '' and a['spawn_y'] != '':
        overrides['spawn.mode'] = 'map'
        overrides['spawn.x'] = float(a['spawn_x'])
        overrides['spawn.y'] = float(a['spawn_y'])
        overrides['spawn.yaw'] = float(a['spawn_yaw'] or 0.0)
    if a['time_factor']:
        overrides['time_factor'] = float(a['time_factor'])
    if a['lockstep']:
        overrides['lockstep'] = a['lockstep'].lower() in ('true', '1')
    sets, catalogue = {}, {}
    if a['gnss_dead_zones'] or a['gnss_faults']:
        sets = load_yaml(GNSS_DEAD_ZONES)['sets']
    if a['gnss_faults']:
        catalogue = load_yaml(GNSS_FAULTS)
    try:
        overrides.update(launch_overrides(a['gnss_feed'], a['gnss_dead_zones'], a['gnss_seed'], sets,
                                          a['gnss_faults'], catalogue))
    except ValueError as e:
        raise LaunchError(str(e)) from None
    if os.path.basename(a['gnss_faults']) == FAULTS_FILE and overrides.get('gps.faults_name') == a['gnss_faults']:
        # a list of faults (ours): echo the file name, not the session's temp folder, so traces replay exactly
        overrides['gps.faults_name'] = FAULTS_FILE
    return overrides


def octane_env() -> dict:
    """components/gnss.py: the five MCITY_OCTANE_* of a sim run."""
    octane = carla_bridge_params()['octane']
    return {
        'MCITY_OCTANE_SERVER': f"http://127.0.0.1:{octane['port']}",
        'MCITY_OCTANE_KEY': str(octane['api_key']),
        'MCITY_OCTANE_NAMESPACE': str(octane['namespace']),
        'MCITY_FRONT_BEACON_ID': str(octane['front_id']),
        'MCITY_REAR_BEACON_ID': str(octane['rear_id']),
    }


def repo_nodes(plan) -> list:
    """The repo nodes bringup starts for this plan, in its order of readiness (ARCH 6.1), without the page's `params`."""
    from nift_common.vehicle import ShuttleConfig
    nodes = []
    # 1. the vehicle (components/can.py -> kinematic_sim.launch.py)
    shared = carla_bridge_params()
    nodes.append(NodeSpec('kinematic_sim', 'kinematic_sim', 'nift_carla.kinematic_sim_node', 'KinematicSimNode',
                          [dict_source(CARLA_PARAMS + ' carla_bridge', shared),
                           dict_source('kinematic_sim.launch.py overrides', kinematic_overrides(plan))]))
    # 2. GNSS (components/gnss.py)
    nodes.append(NodeSpec('beacon_node', 'nift_beacon_node', 'nift_sensor.beacon_node', 'BeaconNode',
                          [file_source('nift_sensor/config/beacon_node.yaml'), _sim_time(plan),
                           dict_source('{hygiene.enabled}', {'hygiene.enabled': plan.args['gnss_checks'] == 'true'})],
                          env=octane_env()))
    # 3. map -> base_link (components/localization.py)
    if 'localization' in plan.on:
        nodes.append(NodeSpec('shuttle_dynamic_broadcaster', 'shuttle_dynamic_broadcaster',
                              'nift_sensor.shuttle_tf_node', 'ShuttleDynamicBroadcaster',
                              [file_source('nift_sensor/config/shuttle_tf.yaml'), _sim_time(plan),
                               dict_source('{map_origin}', _map_origin(plan)),
                               dict_source('{gnss.offset_x}', {'gnss.offset_x': ShuttleConfig().gnss_front_x})]))
    if 'filter' in plan.on:
        nodes.append(NodeSpec('localization_node', 'localization_node', 'nift_sensor.localization_node',
                              'LocalizationNode',
                              [file_source('nift_sensor/config/localization.yaml'), _sim_time(plan),
                               scoped_source('localization_node node_params', 'localization_node',
                                             filter_params(plan, plan.localization_mode))]))
    # 4. after auth_ok: planning, then control (components/planning.py, control.py)
    if 'planning' in plan.on:
        map_params = {'map_file': paths.repo_path(MAP_REL)}
        nodes.append(NodeSpec('waypoint_node', 'waypoint_node', 'nift_waypoint.waypoint_node', 'WaypointNode',
                              [file_source('nift_waypoint/config/waypoint_params.yaml'), _sim_time(plan),
                               dict_source('{map_origin}', _map_origin(plan)),
                               dict_source('{map_file}', map_params)], after_auth=True))
    if 'longitudinal' in plan.on:
        overrides = {'localization_source': 'filter' if plan.localization_mode == 'ekf' else 'gnss'}
        if plan.args['filter_type']:
            overrides['filter_type'] = plan.args['filter_type']
        nodes.append(NodeSpec('trajectory_controller', 'trajectory_controller',
                              'nift_control.trajectory_controller', 'TrajectoryController',
                              [_sim_time(plan), file_source('nift_control/config/planning.yaml'),
                               file_source('nift_control/config/adrc_params.yaml'),
                               dict_source('{lateral_enabled}', {'lateral_enabled': 'lateral' in plan.on}),
                               scoped_source('trajectory_controller node_params', 'trajectory_controller',
                                             overrides)], after_auth=True))
    return nodes


PLAYGROUND_NODES = (
    ('playground_operator', 'playground.operator', 'PlaygroundOperator'),
    ('playground_observer', 'playground.observer', 'PlaygroundObserver'),
)


@dataclass
class LaunchSet:
    options: dict
    plan: object
    nodes: list
    warnings: list
    route: object = None            # the test route of the spawn option, if any

    def node(self, name):
        return next((n for n in self.nodes if n.name == name), None)

    @property
    def remaps(self) -> dict:
        """{own name: launch name} for fakeros (ARCH 6.2)."""
        return {n.own_name: n.name for n in self.nodes if n.own_name != n.name}

    @property
    def env(self) -> dict:
        out = {}
        for n in self.nodes:
            out.update(n.env)
        return out

    def describe(self) -> dict:
        from nift_bringup.profiles import summary
        return {'options': self.options, 'profile': PROFILE, 'localization_mode': self.plan.localization_mode,
                'components': sorted(self.plan.on), 'filter_type': self.filter_type,
                'nodes': [n.describe() for n in self.nodes], 'warnings': list(self.warnings),
                'summary': summary(self.plan), 'route': self.route}

    @property
    def filter_type(self) -> str:
        ctl = self.node('trajectory_controller')
        return ctl.params.get('filter_type', '') if ctl else ''


def build(launch=None, workdir=None) -> LaunchSet:
    """Resolve the launch options into the node list (ARCH 6)."""
    opts = normalize_options(launch)
    plan = resolve_plan(opts, workdir)
    nodes = repo_nodes(plan)
    warnings = list(plan.warnings)

    if WAYPOINT_MAP_PUBLISH_PERIOD_S is not None:
        wp = next((n for n in nodes if n.name == 'waypoint_node'), None)
        if wp is not None:
            wp.sources.append(scoped_source(PLAYGROUND_OVERRIDE, 'waypoint_node',
                                            {'map_publish_period_s': float(WAYPOINT_MAP_PUBLISH_PERIOD_S)}))
            warnings.append(f'waypoint_node map_publish_period_s {WAYPOINT_MAP_PUBLISH_PERIOD_S:g} s (playground '
                            'override: /lanelet_map_markers has no subscriber here and cost most of the run time)')

    # perception: a playground extra on top of the resolved plan (ARCH 6.3)
    if opts['perception']:
        ctl = next((n for n in nodes if n.name == 'trajectory_controller'), None)
        if ctl is not None:
            src = ctl.sources[-1]
            params = dict(src.raw[2])
            params.pop('filter_type', None)
            if opts['filter_type']:
                params['filter_type'] = opts['filter_type']
            ctl.sources[-1] = scoped_source(src.label, 'trajectory_controller', params)
        warnings = [w for w in warnings if not w.startswith('perception is off')]
        warnings.append('perception: playground_perception publishes /obstacles from the page\'s obstacles '
                        '(ground truth, not lidar_node)')

    # the page's parameter overrides: a node-scoped file appended last for that node
    by_name = {n.name: n for n in nodes}
    for node_name, values in (opts['params'] or {}).items():
        if node_name not in by_name:
            raise LaunchError(f'params: no node {node_name!r} in this launch; nodes: {sorted(by_name)}')
        if not isinstance(values, dict):
            raise LaunchError(f'params.{node_name}: expected {{name: value}}')
        values = doubles_as_floats(node_name, values, by_name[node_name].params)
        opts['params'][node_name] = values
        by_name[node_name].sources.append(scoped_source('page params', node_name, dict(values)))

    for name, module, cls in PLAYGROUND_NODES:
        nodes.append(NodeSpec(name, name, module, cls, [], repo=False))
    if opts['perception']:
        nodes.append(NodeSpec('playground_perception', 'playground_perception', 'playground.perception',
                              'PlaygroundPerception', [], repo=False))

    route = None
    if isinstance(opts['spawn'], str):
        route = dict(routes()[opts['spawn']], id=opts['spawn'])
    return LaunchSet(opts, plan, nodes, warnings, route)


def presets() -> dict:
    """ready.presets without the GNSS faults (the session adds the sim backend's list): test routes and dead-zone sets."""
    out_routes = []
    for rid, r in routes().items():
        out_routes.append({'id': rid, 'name': r.get('name'), 'description': r.get('description'),
                           'spawn': r.get('spawn'), 'goal': r.get('goal'), 'timeout_s': r.get('timeout_s')})
    zones = sorted((load_yaml(GNSS_DEAD_ZONES) or {}).get('sets') or {})
    return {'routes': out_routes, 'gnss_dead_zones': zones}
