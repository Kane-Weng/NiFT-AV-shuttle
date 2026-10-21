"""
The playground's launch set equals bringup's (ARCH 6 and 10).

The test runs the repo's own `nift_bringup/launch/system.launch.py` with `profile:=sim_kinematic` on stub `launch`, `launch_ros` and `ament_index_python` modules. The stubs record every `Node(...)`, follow `IncludeLaunchDescription` into `kinematic_sim.launch.py`, and fire the `OnProcessIO` handlers with the log lines they wait for. Then, for each repo node, the captured parameter sources (files read, dicts as `/**`), the merged parameters, the environment and the executable must equal the playground's `launch_set.build()`.

No ROS install is needed. The stubs live in `sys.modules` only while this module's tests run.
"""

import importlib.util
import os
import re
import sys
import types

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))

SHARE = {   # package -> its source dir, which has the installed share layout (config/, launch/)
    'nift_bringup': 'nift_bringup', 'nift_sensor': 'nift_sensor', 'nift_waypoint': 'nift_waypoint',
    'nift_control': 'nift_control', 'nift_carla': 'nift_simulation/nift_carla', 'nift_common': 'nift_common',
    'shuttle_description': 'shuttle_description',
}
MARKERS = ('Octane emulator listening', 'auth_ok')     # what system.launch.py's OnProcessIO handlers wait for


# ================================================================ stub launch modules

class Stub:
    """Any launch action or substitution: keeps its arguments."""

    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        for k, v in kwargs.items():
            setattr(self, k, v)


class PackageNotFoundError(KeyError):
    pass


def get_package_share_directory(pkg):
    if pkg not in SHARE:
        raise PackageNotFoundError(pkg)
    return os.path.join(REPO, SHARE[pkg])


class LaunchConfiguration(Stub):
    def perform(self, context):
        return context.launch_configurations.get(self.args[0], '')


class PathJoinSubstitution(Stub):
    def perform(self, context):
        return os.path.join(*[p.perform(context) if hasattr(p, 'perform') else p for p in self.args[0]])


class FindPackageShare(Stub):
    def perform(self, context):
        return get_package_share_directory(self.args[0])


class Node(Stub):
    @property
    def node_executable(self):
        return self.kwargs.get('executable')


def _module(name, **attrs):
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    mod.__getattr__ = lambda attr: type(attr, (Stub,), {})     # anything else: a recording stub class
    return mod


def stub_modules():
    acts = {n: type(n, (Stub,), {}) for n in ('DeclareLaunchArgument', 'LogInfo', 'OpaqueFunction',
                                              'RegisterEventHandler', 'EmitEvent', 'IncludeLaunchDescription',
                                              'ExecuteProcess')}
    mods = {
        'launch': _module('launch', LaunchDescription=type('LaunchDescription', (Stub,), {})),
        'launch.actions': _module('launch.actions', **acts),
        'launch.event_handlers': _module('launch.event_handlers', OnProcessIO=type('OnProcessIO', (Stub,), {})),
        'launch.events': _module('launch.events'),
        'launch.launch_description_sources': _module(
            'launch.launch_description_sources',
            PythonLaunchDescriptionSource=type('PythonLaunchDescriptionSource', (Stub,), {}),
            AnyLaunchDescriptionSource=type('AnyLaunchDescriptionSource', (Stub,), {})),
        'launch.substitutions': _module('launch.substitutions', LaunchConfiguration=LaunchConfiguration,
                                        PathJoinSubstitution=PathJoinSubstitution),
        'launch_ros': _module('launch_ros'),
        'launch_ros.actions': _module('launch_ros.actions', Node=Node),
        'launch_ros.substitutions': _module('launch_ros.substitutions', FindPackageShare=FindPackageShare),
        'launch_ros.parameter_descriptions': _module('launch_ros.parameter_descriptions'),
        'ament_index_python': _module('ament_index_python'),
        'ament_index_python.packages': _module('ament_index_python.packages',
                                               get_package_share_directory=get_package_share_directory,
                                               get_package_prefix=get_package_share_directory,
                                               PackageNotFoundError=PackageNotFoundError),
    }
    return mods


class Context:
    def __init__(self, configs):
        self.launch_configurations = dict(configs)


class Event:
    def __init__(self, text):
        self.text = text.encode()


def _load_file(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run_launch_file(path, arguments, acts):
    """generate_launch_description() of a launch file with these arguments; expand into Nodes."""
    ld = _load_file(path, 'launchfile_' + re.sub(r'\W', '_', os.path.basename(path)))
    desc = ld.generate_launch_description()
    entities = desc.args[0]
    configs = {e.args[0]: e.kwargs.get('default_value', '') for e in entities
               if type(e).__name__ == 'DeclareLaunchArgument'}
    configs = {k: (v.perform(Context({})) if hasattr(v, 'perform') else v) for k, v in configs.items()}
    configs.update(arguments)
    ctx = Context(configs)
    out = []
    for e in entities:
        if type(e).__name__ == 'OpaqueFunction':
            expand(e.kwargs['function'](ctx), out, acts)
    return out


def expand(actions, out, acts):
    """Walk launch actions: collect Nodes, fire event handlers, follow includes."""
    for a in actions or []:
        kind = type(a).__name__
        if kind == 'Node':
            out.append(a)
        elif kind == 'RegisterEventHandler':
            handler = a.args[0]
            cb = handler.kwargs.get('on_stdout')
            for marker in MARKERS:
                res = cb(Event(f'[INFO] [x]: {marker} ...')) if cb else None
                if res:
                    expand(res, out, acts)
        elif kind == 'IncludeLaunchDescription':
            src = a.args[0]
            args = dict(a.kwargs.get('launch_arguments') or {})
            expand(run_launch_file(src.args[0], args, acts), out, acts)


@pytest.fixture(scope='module')
def bringup(playground):
    """Run system.launch.py with stub launch modules: cli dict -> {node name: Node stub}."""
    saved = {k: sys.modules.get(k) for k in stub_modules()}
    sys.modules.update(stub_modules())
    for k in [m for m in sys.modules if m.startswith('nift_bringup.components')]:
        del sys.modules[k]
    from nift_bringup import profiles
    old_maps_dir = profiles.maps_dir
    profiles.maps_dir = lambda: os.path.join(REPO, 'maps')        # setup.py installs maps/ there

    def run(cli):
        args = {'profile': 'sim_kinematic', 'checks': 'false', 'dry_run': 'false', **cli}
        nodes = run_launch_file(os.path.join(REPO, 'nift_bringup', 'launch', 'system.launch.py'), args, None)
        return {n.kwargs.get('name') or n.kwargs['executable']: n for n in nodes}

    yield run
    profiles.maps_dir = old_maps_dir
    for k, v in saved.items():
        if v is None:
            sys.modules.pop(k, None)
        else:
            sys.modules[k] = v
    for k in [m for m in sys.modules if m.startswith('nift_bringup.components')]:
        del sys.modules[k]


# ================================================================ comparison

def entry_points():
    """executable -> module, from the packages' setup.py console_scripts."""
    out = {}
    for pkg in ('nift_sensor', 'nift_waypoint', 'nift_control', 'nift_simulation/nift_carla'):
        with open(os.path.join(REPO, pkg, 'setup.py')) as f:
            for exe, mod in re.findall(r"'(\w+)\s*=\s*([\w.]+):main'", f.read()):
                out[exe] = mod
    return out


def bringup_sources(node):
    """A Node's parameters=[...] as {key: flat params} tables (files read, dicts as /**)."""
    from playground.launch_set import flatten
    tables = []
    for p in node.kwargs.get('parameters') or []:
        if isinstance(p, str):
            with open(p) as f:
                doc = yaml.safe_load(f) or {}
            tables.append({str(k): flatten(v['ros__parameters']) for k, v in doc.items()
                           if isinstance(v, dict) and isinstance(v.get('ros__parameters'), dict)})
        else:
            tables.append({'/**': flatten(p)})
    return tables


def compare(bringup_nodes, ls):
    from playground.launch_set import effective_params, FAULTS_FILE, PLAYGROUND_OVERRIDE, Source
    eps = entry_points()
    repo_nodes = [n for n in ls.nodes if n.repo]
    expected = {k: v for k, v in bringup_nodes.items() if k != 'robot_state_publisher'}
    assert sorted(n.name for n in repo_nodes) == sorted(expected), 'node set differs from bringup'
    for spec in repo_nodes:
        b = expected[spec.name]
        assert eps[b.kwargs['executable']] == spec.module, spec.name
        mine = [s.table for s in spec.sources if s.label != PLAYGROUND_OVERRIDE]
        theirs = bringup_sources(b)
        for t in theirs:      # a list of faults: the playground echoes the file name, not its temp folder
            d = t.get('/**', {})
            if os.path.basename(str(d.get('gps.faults_name', ''))) == FAULTS_FILE:
                d['gps.faults_name'] = FAULTS_FILE
        assert mine == theirs, f'{spec.name}: parameter sources differ'
        theirs_eff = effective_params([Source('x', 'x', t) for t in theirs], spec.name)
        mine_eff = effective_params([s for s in spec.sources if s.label != PLAYGROUND_OVERRIDE], spec.name)
        assert mine_eff == theirs_eff
        assert (b.kwargs.get('additional_env') or {}) == spec.env, f'{spec.name}: environment differs'
        from nift_bringup.profiles import AFTER_AUTH
        assert spec.after_auth == (spec.name in ('waypoint_node', 'trajectory_controller')), spec.name
        assert AFTER_AUTH >= {'planning', 'longitudinal'}


CASES = [
    {},
    {'localization': 'ekf'},
    {'localization': 'shadow'},
    {'spawn': '03'},
    {'spawn': {'x': 10.0, 'y': -5.5, 'yaw': 0.3}},
    {'gnss_feed': 'real', 'gnss_seed': 7, 'localization': 'ekf'},
    {'gnss_checks': 'false'},
    {'gnss_faults': 'dropout_1.5s_turn', 'spawn': '04'},
    {'gnss_dead_zones': 'route04_turn', 'spawn': '04'},
    {'gnss_faults': [{'kind': 'dropout', 't_s': 20.0, 'duration_s': 2.0}]},
    # the launcher view's switches
    {'components': {'lateral': False}},
    {'components': {'lateral': False, 'longitudinal': False}},
    {'localization': 'off', 'components': {'planning': False, 'lateral': False}},
    {'localization': 'off', 'components': {'gnss': False, 'planning': False, 'longitudinal': False, 'lateral': False}},
]


@pytest.mark.parametrize('opts', CASES, ids=[str(c) for c in CASES])
def test_launch_set_equals_bringup(bringup, opts):
    from playground import launch_set
    ls = launch_set.build(opts)
    compare(bringup(launch_set.cli_args(ls.options)), ls)


def test_playground_defaults(playground):
    """ARCH 6.3: time_factor 0, lockstep false, gnss_seed 1, filter none, the odr2024 map."""
    from playground import launch_set
    ls = launch_set.build()
    sim = ls.node('kinematic_sim').params
    assert sim['time_factor'] == 0.0 and sim['lockstep'] is False and sim['gps.seed'] == 1
    assert ls.node('trajectory_controller').params['filter_type'] == 'none'
    assert ls.node('trajectory_controller').params['localization_source'] == 'gnss'
    assert ls.node('waypoint_node').params['map_file'].endswith('maps/lanelet2_odr2024.osm')
    assert ls.remaps == {'nift_beacon_node': 'beacon_node'}
    assert ls.env['MCITY_OCTANE_SERVER'] == 'http://127.0.0.1:8765'
    assert [n.name for n in ls.nodes][:3] == ['kinematic_sim', 'beacon_node', 'shuttle_dynamic_broadcaster']
    ekf = launch_set.build({'localization': 'ekf'})
    assert ekf.node('trajectory_controller').params['localization_source'] == 'filter'
    assert ekf.node('localization_node').params['mode'] == 'ekf'
    assert ekf.node('shuttle_dynamic_broadcaster') is None


def test_params_precedence(playground):
    """ARCH 6.2: a /** dict after a node-scoped file loses to it when the /** key came first."""
    from playground.launch_set import dict_source, effective_params, scoped_source
    a = dict_source('a', {'use_sim_time': True})
    planning = scoped_source('planning', 'trajectory_controller', {'filter_type': 'cbf', 'x': 1})
    late_dict = dict_source('late', {'filter_type': 'none'})
    late_scoped = scoped_source('scoped', 'trajectory_controller', {'filter_type': 'none'})
    assert effective_params([a, planning, late_dict], 'trajectory_controller')['filter_type'] == 'cbf'
    assert effective_params([a, planning, late_scoped], 'trajectory_controller')['filter_type'] == 'none'
    # the node key first, the /** key later: the wildcard wins (shuttle_tf.yaml then {gnss.offset_x})
    assert effective_params([planning, late_dict], 'trajectory_controller')['filter_type'] == 'none'
    assert 'x' not in effective_params([planning], 'other_node')


def test_page_params_and_perception(playground):
    from playground import launch_set
    ls = launch_set.build({'params': {'trajectory_controller': {'filter_type': 'cbf', 'max_speed': 1.5}},
                           'perception': True})
    p = ls.node('trajectory_controller').params
    assert p['max_speed'] == 1.5
    assert ls.node('playground_perception') is not None
    ls = launch_set.build({'perception': True})
    assert ls.filter_type == 'cbf'                      # planning.yaml's
    with pytest.raises(launch_set.LaunchError):
        launch_set.build({'filter_type': 'cbf'})        # needs perception
    with pytest.raises(launch_set.LaunchError):
        launch_set.build({'params': {'nobody': {'a': 1}}})
    with pytest.raises(launch_set.LaunchError):
        launch_set.build({'spawn': '99'})


def test_map_markers_override_is_the_only_one(playground):
    """ARCH 6.3: the playground overrides one repo parameter, and says so."""
    from playground import launch_set
    ls = launch_set.build()
    extra = [(n.name, s.table) for n in ls.nodes for s in n.sources if s.label == launch_set.PLAYGROUND_OVERRIDE]
    if launch_set.WAYPOINT_MAP_PUBLISH_PERIOD_S is None:
        assert extra == []
    else:
        assert extra == [('waypoint_node', {'waypoint_node': {'map_publish_period_s': 3600.0}})]
        assert any('map_publish_period_s' in w for w in ls.warnings)


def test_filter_params_constant(bringup):
    """launch_set copies SIM_REAL_FEED_DELAY_S from components/localization.py (it imports launch_ros)."""
    from nift_bringup.components import localization
    from playground import launch_set
    assert launch_set.SIM_REAL_FEED_DELAY_S == localization.SIM_REAL_FEED_DELAY_S


@pytest.mark.parametrize('opts', CASES[:6], ids=[str(c) for c in CASES[:6]])
def test_fakeros_merges_the_same(playground, opts):
    """fakeros merges the raw sources with rcl's YAML rules; the result must equal the launch set's description."""
    import fakeros
    from playground import launch_set
    from playground.runner import fakeros_source
    if not hasattr(fakeros, 'effective_params'):
        pytest.skip('fakeros without effective_params (spike core)')
    ls = launch_set.build(opts)
    for spec in ls.nodes:
        if not spec.repo:
            continue
        theirs = fakeros.effective_params([fakeros_source(s) for s in spec.sources], spec.name)
        mine = spec.params
        assert sorted(theirs) == sorted(mine), spec.name
        for k, v in mine.items():
            t = theirs[k]
            if isinstance(v, list) and not v:
                assert t in (None, [], ()), (spec.name, k, t)      # rcl: an empty sequence is no value
            elif isinstance(v, list):
                assert list(t) == v, (spec.name, k)
            else:
                assert t == v and type(t) is type(v), (spec.name, k, t, v)


def test_components_option(playground):
    """The launcher view's switches: localization and filter become the mode, the rest go to bringup."""
    from playground import launch_set
    o = launch_set.normalize_options({'components': {'filter': True}})
    assert o['localization'] == 'shadow' and o['components'] == {}
    o = launch_set.normalize_options({'localization': 'ekf', 'components': {'filter': False, 'lateral': 'false'}})
    assert o['localization'] == 'off' and o['components'] == {'lateral': False}
    with pytest.raises(launch_set.LaunchError, match='cannot be switched'):
        launch_set.normalize_options({'components': {'rviz': True}})
    with pytest.raises(launch_set.LaunchError, match='lateral needs'):
        launch_set.build({'components': {'planning': False}})
    ls = launch_set.build({'localization': 'off', 'components': {'gnss': False, 'planning': False, 'longitudinal': False, 'lateral': False}})
    assert [n.name for n in ls.nodes if n.repo] == ['kinematic_sim']
