"""
Launch profiles and the rules between their components.

File: profiles.py

Description:
  Launch profiles for system.launch.py: a named set of components from config/profiles.yaml, plus
  per-component and argument overrides from the command line, checked against the rules between
  components before anything starts. No launch imports, so it is tested on its own
  (test/test_profiles.py).

  Rules (all broken ones are reported together)
    localization and filter need gnss; planning needs localization or filter; longitudinal needs
      can + gnss; lateral needs longitudinal + (localization or filter) + (planning or
      static_path).
    planning and static_path both publish /planned_path: not both. static_path is sim only.
    perception with longitudinal needs localization or filter: trajectory_controller moves the
      obstacles into the map frame and drops them all without map -> base_link.
    perception off (controller running) -> filter_type none and a warning; asking for any other
      filter is an error, since it would see an empty road.
    In sim, can is the sim backend's virtual ECU: always on. sim_backend picks the backend: carla
      (CARLA server + bridge) or kinematic (kinematic_sim_node: no GPU, no LiDAR, so perception
      off).
    map -> base_link has two providers, `localization` (shuttle_tf_node) and `filter`
      (localization_node); which are on is the mode (localization_mode(), LOCALIZATION_MODES):
      localization alone = direct (the default), filter alone = ekf, both = shadow. localization:=
      also takes a mode name, which sets both switches (localization:=ekf = filter:=true
      localization:=false); a filter:= that contradicts it is an error.

Usage:
  from nift_bringup.profiles import load, resolve, summary, ProfileError
  plan = resolve(load(path), 'shuttle', {'perception': 'false'})
"""

from dataclasses import dataclass, field
import os

from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
import yaml

MODES = ('real', 'sim')
SIM_BACKENDS = ('carla', 'kinematic')
RVIZ_LAYOUTS = ('single', 'split')
# map -> base_link mode -> the (localization, filter) switches that give it
# (components/localization.py)
LOCALIZATION_MODES = {'direct': (True, False), 'ekf': (False, True), 'shadow': (True, True)}

# name -> description, in summary order
COMPONENTS = {
    'can': "CAN bridge (real: socketcan on can_interface; sim: the sim backend's virtual ECU)",
    'gnss': "beacon_node (real: Mcity Octane; sim: the sim backend's Octane emulator)",
    'localization': 'shuttle_tf_node: map -> base_link from GNSS',
    'filter': 'localization_node: GNSS filter -> map -> base_link + /localization/status',
    'perception': 'lidar_node -> /obstacles',
    'planning': 'waypoint_node: Lanelet2 routes on /planned_path',
    'static_path': 'static_waypoint_publisher: scenario_file waypoints on /planned_path',
    'longitudinal': 'trajectory_controller: ADRC speed control (MANUAL, /teleop_cmd_vel)',
    'lateral': 'Pure Pursuit route following (AUTONOMOUS, /start_navigation)',
    'bag': 'ros2 bag record -> nift_bags/ (repo root, git-ignored)',
    'rviz': 'RViz',
    'web': 'rosbridge :9090 for nift_web_frontend',
}

# summary line of a mode with the filter (direct, the default: no line)
LOCALIZATION_NOTES = {
    'ekf': 'localization_node publishes map -> base_link (predicted to now), shuttle_tf_node off; '
           'trajectory_controller localization_source filter',
    'shadow': 'shuttle_tf_node publishes map -> base_link, localization_node only '
              '/localization/odom + /localization/status; trajectory_controller on GNSS',
}
# command-line help of a component switch that takes more than true | false
SWITCH_HELP = {
    'localization': 'Also a mode, which sets the filter switch too: direct (localization only) | '
                    'ekf (filter:=true localization:=false) | shadow (both)'}

MAP_TF = ('localization', 'filter')     # the providers of map -> base_link: any one of them
# component -> requirements; a tuple is "any one of"
REQUIRES = {
    'localization': ['gnss'],
    'filter': ['gnss'],
    'planning': [MAP_TF],
    'longitudinal': ['can', 'gnss'],
    'lateral': ['longitudinal', MAP_TF, ('planning', 'static_path')],
}
SIM_ONLY = {'static_path'}
AFTER_AUTH = {'planning', 'longitudinal', 'lateral'}    # started once beacon_node logs auth_ok

FILTER_TYPES = ('distance_stop', 'cbf', 'potential_field', 'none')

# launch argument -> (default, description). '' on the command line = not given.
ARGS = {
    'can_interface': (
        'can0', 'real: CAN interface for the socketcan bridge (vxcan0 for a dry run)'),
    'filter_type': (
        '', "safety filter: distance_stop | cbf | potential_field | none; '' = planning.yaml's. "
        'Forced to none when perception is off'),
    'perception_yaml': (
        '', "lidar_node parameters; '' = nift_sensor perception.yaml (real) / "
        'nift_carla perception_carla.yaml (sim)'),
    'map_origin_lat': (
        str(MAP_ORIGIN_LAT), 'map datum latitude (default: Mcity, nift_common site.py)'),
    'map_origin_lon': (
        str(MAP_ORIGIN_LON), 'map datum longitude (default: Mcity, nift_common site.py)'),
    'map_file': (
        '', "Lanelet2 map: a file in nift_waypoint's maps/ (repo maps/) or a path; '' = "
        'lanelet2_odr2024.osm on the shuttle and in sim'),
    'sim_backend': (
        'carla', 'sim: carla (CARLA server + bridge, needs an NVIDIA GPU) | kinematic '
        '(kinematic_sim_node: no GPU, no LiDAR)'),
    'carla_host': ('localhost', 'sim: CARLA server host'),
    'ego_blueprint': (
        '', 'sim: CARLA blueprint of the ego vehicle, e.g. vehicle.nift.shuttle or '
        "vehicle.nissan.micra; '' = carla_params.yaml"),
    'carla_port': ('2000', 'sim: CARLA RPC port'),
    'scenario_file': (
        '', 'sim: scenario YAML (obstacles; spawn_pose; waypoints for static_path); '
        "'' = none. package://<pkg>/<path> works"),
    'spawn_x': (
        '', "sim: spawn pose, map frame; '' = scenario spawn_pose, else carla_params.yaml"),
    'spawn_y': ('', 'sim: see spawn_x'),
    'spawn_yaw': ('', 'sim: see spawn_x [rad]'),
    'time_factor': ('', "sim: '' = carla_params.yaml (1.0), 0 = as fast as possible"),
    'rviz_layout': (
        'single', 'rviz: single | split (a second, render-only window with the top-down view, '
        'navigation_topdown.rviz)'),
    'lockstep': ('', "sim: true | false; '' = carla_params.yaml (true)"),
    'gnss_feed': (
        '', 'sim: real (the real Octane feed: 10 Hz behind a jittered relay, gaps) | ideal; '
        "'' = carla_params.yaml (ideal)"),
    'gnss_dead_zones': (
        '', 'sim: GNSS dead-zone sets from nift_carla gnss_dead_zones.yaml, comma-separated, '
        "e.g. north_end; '' = none"),
    'gnss_seed': ('', "sim: GNSS random seed; '' = carla_params.yaml (0 = new each run)"),
    'gnss_faults': (
        '', 'sim: a GNSS fault scenario from nift_carla gnss_faults.yaml '
        "(e.g. dropout_1.5s_turn) or a .yaml file; '' = none"),
    'gnss_checks': (
        'true', 'beacon_node feed checks (late, stuck and inconsistent GNSS is dropped, so it '
        'ends in a stop): true | false (the old rule: every new stamp is published)'),
}
PATH_ARGS = ('scenario_file', 'perception_yaml')

# map_file:='' -> per mode. Both use the map generated from the digital twin's 2024 OpenDRIVE,
# which is also the world a stock CARLA server with the twin loads.
DEFAULT_MAPS = {'real': 'lanelet2_odr2024.osm', 'sim': 'lanelet2_odr2024.osm'}

_TRUE, _FALSE = ('true', '1', 'yes', 'on'), ('false', '0', 'no', 'off')
_MODE_SWITCHES = {'direct': 'localization on, filter off; both on is shadow',
                  'ekf': 'filter on, localization off', 'shadow': 'both on'}


class ProfileError(Exception):
    pass


def localization_mode(on) -> str:
    """
    Return the map -> base_link mode of a component set.

    direct | ekf | shadow; '' = neither provider is on.
    """
    switches = ('localization' in on, 'filter' in on)
    return next((mode for mode, s in LOCALIZATION_MODES.items() if s == switches), '')


@dataclass
class Plan:
    """Launch configuration of the chosen profile & command-line overrides."""

    profile: str
    description: str
    mode: str                                       # 'real' | 'sim'
    on: set                                         # components to start
    source: dict = field(default_factory=dict)      # component -> 'command line' | 'sim'
    args: dict = field(default_factory=dict)        # every ARGS key, resolved (filter_type final)
    warnings: list = field(default_factory=list)

    @property
    def sim(self) -> bool:
        return self.mode == 'sim'

    @property
    def kinematic(self) -> bool:
        """Sim on the kinematic backend (no CARLA, no /scan)."""
        return self.sim and self.args['sim_backend'] == 'kinematic'

    @property
    def localization_mode(self) -> str:
        """Return direct | ekf | shadow | '': from the components on, the one source of truth."""
        return localization_mode(self.on)


def load(path: str) -> dict:
    """Read and check profiles.yaml: {name: {description, mode, components, args}}."""
    with open(path) as f:
        profiles = yaml.safe_load(f) or {}

    for name, p in profiles.items():
        where = f'{os.path.basename(path)}, profile {name}'
        if not isinstance(p, dict):
            raise ProfileError(f'{where}: not a mapping')

        # Unknown key
        extra = set(p) - {'description', 'mode', 'components', 'args'}
        if extra:
            raise ProfileError(f'{where}: unknown key(s) {sorted(extra)}')

        # Unknown mode
        if p.get('mode') not in MODES:
            raise ProfileError(f'{where}: mode must be one of {MODES}')

        # Unknown component
        bad = [c for c in p.get('components') or [] if c not in COMPONENTS]
        if bad:
            raise ProfileError(f'{where}: unknown component(s) {bad}; known: {list(COMPONENTS)}')

        # Unknown arg
        bad = sorted(set(p.get('args') or {}) - set(ARGS))
        if bad:
            hint = ('; localization / filter are components: list them'
                    if set(bad) & set(MAP_TF) else '')
            raise ProfileError(f'{where}: unknown arg(s) {bad}{hint}')

    return profiles


def listing(profiles: dict) -> str:
    """Build the table of profile names, modes, and descriptions."""
    w = max((len(n) for n in profiles), default=0)
    rows = [f'  {n:<{w}}  {p["mode"]:<4}  {p.get("description", "")}' for n, p in profiles.items()]
    return 'Profiles (nift_bringup/config/profiles.yaml):\n' + '\n'.join(rows)


def package_path(value: str) -> str:
    """'package://<pkg>/<rel>' -> that file in <pkg>'s share directory; anything else unchanged."""
    if not value.startswith('package://'):
        return value
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    pkg, _, rel = value[len('package://'):].partition('/')

    try:
        return os.path.join(get_package_share_directory(pkg), rel)
    except PackageNotFoundError:
        raise ProfileError(f'{value}: package {pkg} not found (built and sourced?)') from None


def maps_dir() -> str:
    """
    Return nift_waypoint's installed maps/.

    setup.py installs the repo's maps/lanelet2*.osm there.
    """
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('nift_waypoint'), 'maps')


def installed_maps() -> list:
    """File names of the installed maps, [] if nift_waypoint is not built."""
    try:
        return sorted(f for f in os.listdir(maps_dir()) if f.endswith('.osm'))
    except Exception:
        return []


def map_path(value: str) -> str:
    """map_file value -> file: a bare name is one of the installed maps, anything else a path."""
    if os.sep in value or value.startswith('~'):
        return os.path.expanduser(value)
    return os.path.join(maps_dir(), value)


def _bool(name: str, value: str):
    """Parse a command-line true/false string."""
    v = (value or '').strip().lower()
    if v == '':     # Empty string returns None -> means not given, use the profile
        return None
    if v in _TRUE:
        return True
    if v in _FALSE:
        return False
    raise ProfileError(f'{name}:={value}: expected true or false')


def resolve(profiles: dict, name: str, cli: dict) -> Plan:
    """Tke a profile and command-line overrides and returns a Plan."""
    # 1. Pick the profile and mode
    if not name:
        raise ProfileError('no profile given: profile:=<name>\n' + listing(profiles))
    if name not in profiles:
        raise ProfileError(f'unknown profile "{name}"\n' + listing(profiles))
    p = profiles[name]
    mode = cli.get('mode') or p['mode']
    if mode not in MODES:
        raise ProfileError(f'mode:={mode}: expected one of {MODES}')

    # localization:= also takes a mode: it sets both map -> base_link switches (filter:= may only
    # agree)
    cli = dict(cli)
    given = (cli.get('localization') or '').strip()
    if given.lower() in LOCALIZATION_MODES:
        mode_name, (loc, filt) = given.lower(), LOCALIZATION_MODES[given.lower()]
        if _bool('filter', cli.get('filter', '')) not in (None, filt):
            raise ProfileError(f'localization:={given} is filter:={str(filt).lower()} '
                               f'({mode_name}: {_MODE_SWITCHES[mode_name]}), '
                               f'but filter:={cli["filter"].strip()} was given: '
                               'drop one of them')
        cli['localization'], cli['filter'] = str(loc).lower(), str(filt).lower()
    elif given and given.lower() not in _TRUE + _FALSE:
        raise ProfileError(f'localization:={given}: expected true, false or a mode: '
                           f'{" | ".join(LOCALIZATION_MODES)}')

    # 2. Set the component switches
    on, source = set(p.get('components') or []), {}
    for c in COMPONENTS:                # apply each component:=true/false override
        v = _bool(c, cli.get(c, ''))
        if v is not None:
            (on.add if v else on.discard)(c)
            source[c] = 'command line'

    # 3. Check the rules
    errors = []
    if mode == 'sim':
        if 'can' in source and 'can' not in on:
            errors.append('can cannot be off in sim: the sim backend is the vehicle')
        on.add('can')
        source.setdefault('can', 'sim')
    else:
        errors += [f'{c} is sim only' for c in sorted(on & SIM_ONLY)]
    for c, needs in REQUIRES.items():
        if c in on:
            for need in needs:
                alts = need if isinstance(need, tuple) else (need,)
                if not on & set(alts):
                    errors.append(f'{c} needs {" or ".join(alts)}')
    if {'planning', 'static_path'} <= on:
        errors.append('planning and static_path both publish /planned_path: pick one')
    if {'perception', 'longitudinal'} <= on and not on & set(MAP_TF):
        errors.append('perception with longitudinal needs localization or filter: '
                      'without map -> base_link '
                      'trajectory_controller drops every obstacle')

    # 4. Resolve the args: command-line > profile > default
    prof_args, args = p.get('args') or {}, {}
    for k, (default, _) in ARGS.items():
        v = cli.get(k, '')
        if v == '' and k in prof_args:
            v = str(prof_args[k])
        args[k] = v if v != '' else default
    for k in PATH_ARGS:
        if args[k]:
            args[k] = package_path(args[k])
    args['map_file'] = args['map_file'] or DEFAULT_MAPS[mode]
    if 'static_path' in on and not args['scenario_file']:
        errors.append('static_path needs scenario_file:=<yaml> with `waypoints`')
    if args['rviz_layout'] not in RVIZ_LAYOUTS:
        errors.append(f'rviz_layout:={args["rviz_layout"]}: expected one of {RVIZ_LAYOUTS}')
    if args['gnss_feed'] not in ('', 'real', 'ideal'):
        errors.append(f'gnss_feed:={args["gnss_feed"]}: expected real or ideal')
    if args['gnss_seed'] and not args['gnss_seed'].lstrip('-').isdigit():
        errors.append(f'gnss_seed:={args["gnss_seed"]}: expected an integer')
    if args['gnss_checks'] not in ('true', 'false'):
        errors.append(f'gnss_checks:={args["gnss_checks"]}: expected true or false')
    if args['sim_backend'] not in SIM_BACKENDS:
        errors.append(f'sim_backend:={args["sim_backend"]}: expected one of {SIM_BACKENDS}')
    elif mode == 'sim' and args['sim_backend'] == 'kinematic' and 'perception' in on:
        errors.append('perception needs /scan, and the kinematic backend has no LiDAR: '
                      'perception:=false (safety filter off) or sim_backend:=carla')

    # 5. Settle `filter_type`
    warnings, ft = [], args['filter_type']
    if ft and ft not in FILTER_TYPES:
        errors.append(f'filter_type:={ft}: expected one of {FILTER_TYPES}')
    elif 'longitudinal' in on and 'perception' not in on:
        if ft not in ('', 'none'):
            errors.append(f'filter_type:={ft} needs perception: '
                          'without /obstacles it sees an empty road')
        args['filter_type'] = 'none'
        warnings.append('perception is off: safety filter none and no collision-zone stop; '
                        'nothing slows or stops the vehicle for obstacles')
    elif 'longitudinal' in on and ft == 'none':
        warnings.append('filter_type none: perception runs but nothing slows or stops the vehicle '
                        'for obstacles')
    if 'filter' in on and args['gnss_checks'] == 'false':
        warnings.append(f'filter ({localization_mode(on)}) with gnss_checks false: '
                        'the filter takes every GNSS '
                        'delivery unchecked (stuck, swapped or offset antennas reach it)')

    if errors:
        raise ProfileError(f'profile {name} ({mode}):\n  ' + '\n  '.join(errors))
    return Plan(name, p.get('description', ''), mode, on, source, args, warnings)


def summary(plan: Plan) -> str:
    """Build the dry-run and startup log."""
    where = (('KINEMATIC SIM, no CARLA' if plan.kinematic else 'CARLA') if plan.sim
             else 'REAL VEHICLE')
    lines = [f'profile {plan.profile} ({where}): {plan.description}']
    for c, desc in COMPONENTS.items():
        notes = [plan.source[c]] if c in plan.source else []
        if c in plan.on:
            if c in AFTER_AUTH:
                notes.append('after beacon auth_ok')
            lines.append(f'  ON   {c:<13} {desc}' + ''.join(f' [{n}]' for n in notes))
        else:
            lines.append(f'  off  {c}' + ''.join(f' [{n}]' for n in notes))
    if 'filter' in plan.on:      # ekf | shadow (direct: as before, no line)
        lines.append(f'  localization: {plan.localization_mode}: '
                     f'{LOCALIZATION_NOTES[plan.localization_mode]}')
    if 'longitudinal' in plan.on:
        lines.append(f'  safety filter: {plan.args["filter_type"] or "planning.yaml filter_type"}')
    if 'planning' in plan.on:
        lines.append(f'  map: {plan.args["map_file"]}')
    lines += [f'  WARNING: {w}' for w in plan.warnings]
    return '\n'.join(lines)
