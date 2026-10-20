"""Rules of nift_bringup.profiles, and every shipped profile in config/profiles.yaml resolving."""

import os

from nift_bringup.profiles import (
    DEFAULT_MAPS,
    load,
    localization_mode,
    map_path,
    ProfileError,
    resolve,
    summary,
)
import pytest

PROFILES_YAML = os.path.join(os.path.dirname(__file__), '..', 'config', 'profiles.yaml')

FULL = ['can', 'gnss', 'localization', 'perception', 'planning', 'longitudinal', 'lateral']
PROFILES = {
    'full': {'description': 'd', 'mode': 'real', 'components': FULL},
    'speed': {'description': 'd', 'mode': 'real', 'components': ['can', 'gnss', 'longitudinal']},
    'sim': {'description': 'd', 'mode': 'sim', 'components': [c for c in FULL if c != 'can']},
    'bench': {'description': 'd', 'mode': 'sim', 'args': {'scenario_file': '/tmp/s.yaml'},
              'components': ['gnss', 'localization', 'perception', 'static_path', 'longitudinal',
                             'lateral']},
    'kin': {'description': 'd', 'mode': 'sim', 'args': {'sim_backend': 'kinematic'},
            'components': ['gnss', 'localization', 'planning', 'longitudinal', 'lateral']},
    'kin_ekf': {'description': 'd', 'mode': 'sim', 'args': {'sim_backend': 'kinematic'},
                'components': ['gnss', 'filter', 'planning', 'longitudinal', 'lateral']},
}


def errors_of(name, **cli):
    with pytest.raises(ProfileError) as e:
        resolve(PROFILES, name, cli)
    return str(e.value)


def test_shipped_profiles_all_resolve():
    profiles = load(PROFILES_YAML)
    assert {'shuttle', 'sim', 'longitudinal_test'} <= set(profiles)
    for name in profiles:
        plan = resolve(profiles, name, {})
        assert plan.mode == profiles[name]['mode']
        assert summary(plan).startswith(f'profile {name} ')
        # today's: no profile has it
        assert 'filter' not in plan.on and plan.localization_mode in ('direct', '')


def test_full_profile_keeps_planning_yaml_filter():
    plan = resolve(PROFILES, 'full', {})
    assert plan.on == set(FULL)
    assert plan.args['filter_type'] == '' and not plan.warnings


def test_perception_off_turns_filter_off_and_warns():
    plan = resolve(PROFILES, 'full', {'perception': 'false'})
    assert 'perception' not in plan.on and plan.source['perception'] == 'command line'
    assert plan.args['filter_type'] == 'none'
    assert any('perception is off' in w for w in plan.warnings)


def test_speed_control_has_no_filter_and_no_lateral():
    plan = resolve(PROFILES, 'speed', {})
    assert 'lateral' not in plan.on and plan.args['filter_type'] == 'none'


def test_filter_asked_for_without_perception_is_an_error():
    assert 'needs perception' in errors_of('full', perception='false', filter_type='cbf')


def test_unknown_filter_type_is_an_error():
    assert 'expected one of' in errors_of('full', filter_type='cfb')


def test_missing_requirements_are_all_reported():
    msg = errors_of('full', gnss='false')
    assert 'localization needs gnss' in msg and 'longitudinal needs gnss' in msg


def test_lateral_needs_a_path_source():
    assert 'lateral needs planning or static_path' in errors_of('full', planning='false')


def test_perception_with_controller_needs_localization():
    msg = errors_of('full', localization='false', planning='false', lateral='false')
    assert 'perception with longitudinal needs localization or filter' in msg


def test_can_is_always_on_in_sim():
    plan = resolve(PROFILES, 'sim', {})
    assert 'can' in plan.on and plan.source['can'] == 'sim'
    assert 'cannot be off in sim' in errors_of('sim', can='false')


def test_static_path_is_sim_only_and_excludes_planning():
    assert 'static_path is sim only' in errors_of('bench', mode='real')
    assert 'pick one' in errors_of('bench', planning='true')


def test_static_path_needs_a_scenario_file():
    profiles = {'b': dict(PROFILES['bench'], args={})}
    with pytest.raises(ProfileError, match='static_path needs scenario_file'):
        resolve(profiles, 'b', {})


def test_command_line_beats_profile_args_beats_default():
    assert resolve(PROFILES, 'bench', {}).args['scenario_file'] == '/tmp/s.yaml'
    assert resolve(PROFILES, 'bench',
                   {'scenario_file': '/x.yaml'}).args['scenario_file'] == '/x.yaml'
    assert resolve(PROFILES, 'bench', {}).args['carla_host'] == 'localhost'


def test_mode_override():
    plan = resolve(PROFILES, 'speed', {'mode': 'sim'})
    assert plan.sim and 'can' in plan.on


def test_no_or_unknown_profile_lists_the_profiles():
    assert 'full' in errors_of('') and 'bench' in errors_of('nope')


def test_bad_bool_is_an_error():
    assert 'expected true or false' in errors_of('full', rviz='maybe')


def test_sim_backend_defaults_to_carla():
    plan = resolve(PROFILES, 'sim', {})
    assert plan.args['sim_backend'] == 'carla' and not plan.kinematic
    assert '(CARLA)' in summary(plan)


def test_kinematic_backend_runs_without_perception():
    plan = resolve(PROFILES, 'kin', {})
    assert plan.kinematic and 'can' in plan.on
    assert (plan.args['filter_type'] == 'none'
            and any('perception is off' in w for w in plan.warnings))
    assert '(KINEMATIC SIM, no CARLA)' in summary(plan)


def test_kinematic_backend_has_no_lidar_for_perception():
    assert 'kinematic backend has no LiDAR' in errors_of('kin', perception='true')
    assert 'kinematic backend has no LiDAR' in errors_of('sim', sim_backend='kinematic')


def test_unknown_sim_backend_is_an_error():
    assert 'expected one of' in errors_of('sim', sim_backend='gazebo')


def test_gnss_args_pass_through_and_are_checked():
    plan = resolve(PROFILES, 'kin', {'gnss_feed': 'real', 'gnss_dead_zones': 'north_end',
                                     'gnss_seed': '3'})
    assert (plan.args['gnss_feed'], plan.args['gnss_dead_zones'],
            plan.args['gnss_seed']) == ('real', 'north_end', '3')
    assert resolve(PROFILES, 'kin', {}).args['gnss_feed'] == ''          # '' = carla_params.yaml
    assert 'expected real or ideal' in errors_of('kin', gnss_feed='octane')
    assert 'expected an integer' in errors_of('kin', gnss_seed='seven')


def test_gnss_checks_reach_beacon_node():
    from nift_bringup.components.gnss import Gnss
    for given, want in (({}, True), ({'gnss_checks': 'false'}, False)):
        plan = resolve(PROFILES, 'speed', given)
        node = Gnss().actions(plan)[0]
        params = {''.join(t.text for t in k): v for d in node._Node__parameters if isinstance(d,
                                                                                              dict)
                  for k, v in d.items()}
        assert params['hygiene.enabled'] is want
        files = [''.join(t.text for t in f.param_file) for f in node._Node__parameters
                 if not isinstance(f, dict)]
        assert any(f.endswith('beacon_node.yaml') for f in files)
    assert 'expected true or false' in errors_of('kin', gnss_checks='maybe')


def test_sim_backend_does_nothing_on_the_real_vehicle():
    plan = resolve(PROFILES, 'kin', {'mode': 'real', 'can': 'true'})
    assert not plan.sim and not plan.kinematic


def test_rviz_layout():
    assert resolve(PROFILES, 'sim', {}).args['rviz_layout'] == 'single'
    assert resolve(PROFILES, 'sim', {'rviz_layout': 'split'}).args['rviz_layout'] == 'split'
    assert 'expected one of' in errors_of('sim', rviz_layout='quad')


def test_map_file_default_is_per_mode():
    # both modes use the map generated from the twin's OpenDRIVE
    assert resolve(PROFILES, 'full',
                   {}).args['map_file'] == 'lanelet2_odr2024.osm' == DEFAULT_MAPS['real']
    assert resolve(PROFILES, 'sim',
                   {}).args['map_file'] == 'lanelet2_odr2024.osm' == DEFAULT_MAPS['sim']
    assert resolve(PROFILES, 'kin', {}).args['map_file'] == DEFAULT_MAPS['sim']
    assert resolve(PROFILES, 'speed', {'mode': 'sim'}).args['map_file'] == DEFAULT_MAPS['sim']
    assert resolve(PROFILES, 'full', {'map_file': 'x.osm'}).args['map_file'] == 'x.osm'
    assert '  map: lanelet2_odr2024.osm' in summary(resolve(PROFILES, 'full', {}))
    assert 'map:' not in summary(resolve(PROFILES, 'speed', {}))      # no planning, no map


def test_map_path_name_or_path(monkeypatch):
    monkeypatch.setattr('nift_bringup.profiles.maps_dir', lambda: '/share/nift_waypoint/maps')
    assert map_path('lanelet2_odr2024.osm') == '/share/nift_waypoint/maps/lanelet2_odr2024.osm'
    assert map_path('/tmp/m.osm') == '/tmp/m.osm'
    assert map_path('maps/m.osm') == 'maps/m.osm'
    assert map_path('~/m.osm') == os.path.expanduser('~/m.osm')


def test_load_rejects_unknown_components(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('x: {mode: real, components: [can, lidar]}\n')
    with pytest.raises(ProfileError, match='unknown component'):
        load(str(f))


def launched_params(node):
    """
    Return a launch Node's parameters by name, in the order rcl applies them (later wins).

    Dicts and params files (node-scoped yaml, nested keys joined with '.').
    """
    import yaml

    def flat(d, prefix=''):
        for k, v in d.items():
            if isinstance(v, dict):
                yield from flat(v, f'{prefix}{k}.')
            else:
                yield f'{prefix}{k}', v
    out = {}
    for entry in node._Node__parameters:
        if isinstance(entry, dict):
            out.update({''.join(t.text for t in k): v for k, v in entry.items()})
        else:
            with open(''.join(t.text for t in entry.param_file)) as f:
                for scope in yaml.safe_load(f).values():
                    out.update(flat(scope.get('ros__parameters') or {}))
    return out


def param_files(node):
    return [''.join(t.text for t in e.param_file) for e in node._Node__parameters
            if not isinstance(e, dict)]


def nodes_of(plan):
    """Return the executables the map -> base_link components start for `plan`."""
    from nift_bringup.components import REGISTRY
    return [n.node_executable for c in ('localization', 'filter') if c in plan.on
            for n in REGISTRY[c].actions(plan)]


def test_localization_mode_from_the_switches():
    assert (localization_mode({'localization'}) == 'direct'
            and localization_mode({'filter'}) == 'ekf')
    assert localization_mode({'localization',
                              'filter'}) == 'shadow' and localization_mode({'gnss'}) == ''
    for name, given, want in (('kin', {}, 'direct'), ('kin', {'filter': 'true'}, 'shadow'),
                              ('kin', {'filter': 'true', 'localization': 'false'},
                               'ekf'), ('kin_ekf', {}, 'ekf'),
                              ('kin_ekf', {'localization': 'true'}, 'shadow'), ('speed', {}, '')):
        assert resolve(PROFILES, name, given).localization_mode == want, (name, given)


def test_localization_mode_names_set_both_switches():
    for given, on in (('ekf', {'filter'}), ('shadow', {'localization', 'filter'}),
                      ('direct', {'localization'}),
                      ('EKF', {'filter'}), (' shadow ', {'localization', 'filter'}),
                      ('true', {'localization'})):
        plan = resolve(PROFILES, 'kin', {'localization': given})
        assert plan.on & {'localization', 'filter'} == on, given
    plan = resolve(PROFILES, 'kin', {'localization': 'ekf'})
    assert plan.source['localization'] == plan.source['filter'] == 'command line'
    # speed has neither: the mode switches it on
    plan = resolve(PROFILES, 'speed', {'localization': 'ekf'})
    assert plan.localization_mode == 'ekf' and plan.source['filter'] == 'command line'
    # with the filter in the profile: a mode name pins both, true / false only the localization
    # switch
    for given, want in (('direct', 'direct'), ('true', 'shadow'), ('false', 'ekf'), ('shadow',
                                                                                     'shadow')):
        assert resolve(PROFILES, 'kin_ekf',
                       {'localization': given}).localization_mode == want, given
    plan = resolve(PROFILES, 'kin', {'localization': 'false', 'planning': 'false',
                                     'lateral': 'false'})
    assert plan.localization_mode == '' and not plan.on & {'localization', 'filter'}
    assert 'expected true, false or a mode: direct | ekf | shadow' in errors_of(
        'kin', localization='kalman')


def test_localization_mode_name_and_a_contradicting_filter_switch_is_an_error():
    for given, filt in (('ekf', 'false'), ('shadow', 'false'), ('direct', 'true'), ('ekf', 'off')):
        msg = errors_of('kin', localization=given, filter=filt)
        assert (f'localization:={given} is filter:=' in msg
                and f'but filter:={filt} was given' in msg)
    assert 'both on is shadow' in errors_of('kin', localization='direct', filter='true')
    for given, filt, want in (('ekf', 'true', 'ekf'), ('shadow', 'true', 'shadow'),
                              ('direct', 'false', 'direct'),
                              ('false', 'true', 'ekf'), ('true', 'false', 'direct')):
        assert resolve(PROFILES, 'kin', {'localization': given,
                                         'filter': filt}).localization_mode == want
    assert 'filter:=maybe: expected true or false' in errors_of('kin', localization='ekf',
                                                                filter='maybe')


def test_filter_stands_in_for_localization_in_every_rule():
    # planning, lateral, perception
    plan = resolve(PROFILES, 'full', {'localization': 'false', 'filter': 'true'})
    assert {'filter', 'planning', 'lateral', 'perception',
            'longitudinal'} <= plan.on and not plan.warnings
    # lateral with static_path
    plan = resolve(PROFILES, 'bench', {'localization': 'ekf'})
    assert 'static_path' in plan.on and plan.localization_mode == 'ekf'
    msg = errors_of('full', localization='false')
    for rule in ('planning needs localization or filter', 'lateral needs localization or filter',
                 'perception with longitudinal needs localization or filter'):
        assert rule in msg
    assert 'filter needs gnss' in errors_of('speed', filter='true', gnss='false',
                                            longitudinal='false')


def test_profile_lists_the_filter_as_a_component(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('ekf: {mode: sim, components: [gnss, filter]}\n')
    assert resolve(load(str(f)), 'ekf', {}).localization_mode == 'ekf'
    f.write_text('ekf: {mode: sim, components: [gnss, localization], args: {localization: ekf}}\n')
    with pytest.raises(ProfileError, match='localization / filter are components'):
        load(str(f))


def test_localization_direct_is_shuttle_tf_node_as_before():
    from nift_bringup.components.localization import Localization
    for name in ('full', 'kin'):
        plan = resolve(PROFILES, name, {})
        assert plan.localization_mode == 'direct' and 'filter' not in plan.on
        assert nodes_of(plan) == ['shuttle_tf_node']
        nodes = Localization().actions(plan)
        assert param_files(nodes[0])[0].endswith('nift_sensor/config/shuttle_tf.yaml')
        params = launched_params(nodes[0])
        assert params['use_sim_time'] is plan.sim and params['gnss.offset_x'] == 1.041
        assert (params['map_origin_lat'], params['map_origin_lon']) == (42.3005, -83.6987)
        assert 'localization:' not in summary(plan) and '  off  filter' in summary(plan)


def test_localization_ekf_runs_the_filter_instead_of_shuttle_tf_node():
    from nift_bringup.components.localization import Filter
    for given in ({'localization': 'ekf'}, {'localization': 'false', 'filter': 'true'}):
        plan = resolve(PROFILES, 'kin', {**given, 'map_origin_lat': '42.0'})
        assert nodes_of(plan) == ['localization_node']
        node = Filter().actions(plan)[0]
        assert param_files(node)[0].endswith('nift_sensor/config/localization.yaml')
        p = launched_params(node)
        assert p['mode'] == 'ekf' and p['use_sim_time'] is True
        assert (p['antenna.front_x_m'], p['antenna.rear_x_m']) == (1.041, -1.041)
        assert (p['map_origin_lat'], p['map_origin_lon']) == (42.0, -83.6987)
        # ideal feed; kinematic: rear axle
        assert p['gnss_delay_s'] == 0.0 and p['speed_point_x_m'] == -1.041
        assert '  localization: ekf: localization_node publishes map -> base_link' in summary(plan)
        assert '  off  localization [command line]' in summary(plan)


def test_localization_shadow_runs_both():
    from nift_bringup.components.localization import Filter, Localization
    for given in ({'localization': 'shadow'}, {'filter': 'true'}):
        plan = resolve(PROFILES, 'full', given)
        assert nodes_of(plan) == ['shuttle_tf_node', 'localization_node']
        assert launched_params(Localization().actions(plan)[0])['gnss.offset_x'] == 1.041
        p = launched_params(Filter().actions(plan)[0])
        assert p['mode'] == 'shadow' and p['use_sim_time'] is False
        assert ('  localization: shadow: shuttle_tf_node publishes map -> base_link'
                in summary(plan))
        assert '  ON   filter        localization_node' in summary(plan)


def test_filter_output_is_its_status():
    from nift_bringup.components import REGISTRY
    assert REGISTRY['filter'].outputs(resolve(PROFILES, 'kin_ekf', {})) == ['/localization/status']


def test_localization_filter_delay_and_speed_point_per_feed_and_backend():
    from nift_bringup.components.localization import Filter

    def filter_params(name, **cli):
        return launched_params(Filter().actions(
            resolve(PROFILES, name, dict(cli, localization='ekf')))[0])
    real_feed = filter_params('kin', gnss_feed='real')
    assert real_feed['gnss_delay_s'] == 0.061 and real_feed['speed_point_x_m'] == -1.041
    assert filter_params('kin', gnss_feed='ideal')['gnss_delay_s'] == 0.0
    carla = filter_params('sim', perception='false')
    # CARLA: the actor's velocity
    assert carla['gnss_delay_s'] == 0.0 and carla['speed_point_x_m'] == 0.0
    # the shuttle: localization.yaml's delay, the default speed point
    shuttle = filter_params('full')
    assert shuttle['gnss_delay_s'] == 0.061 and 'speed_point_x_m' not in shuttle


def test_controller_gets_the_localization_source():
    from nift_bringup.components.control import Control
    for name, given, want in (('full', {}, 'gnss'), ('full', {'localization': 'direct'}, 'gnss'),
                              ('full', {'localization': 'shadow'}, 'gnss'),
                              ('full', {'localization': 'ekf'}, 'filter'),
                              ('full', {'filter': 'true'}, 'gnss'),
                              ('full', {'filter': 'true', 'localization': 'false'}, 'filter'),
                              ('kin', {'localization': 'ekf'}, 'filter'), ('kin_ekf', {},
                                                                           'filter'),
                              ('kin_ekf', {'localization': 'true'}, 'gnss'), ('speed', {},
                                                                              'gnss')):
        params = launched_params(Control().actions(resolve(PROFILES, name, given))[0])
        assert params['localization_source'] == want, (name, given)
    params = launched_params(Control().actions(resolve(PROFILES, 'kin',
                                                       {'localization': 'ekf'}))[0])
    # still set as before
    assert params['filter_type'] == 'none' and params['lateral_enabled'] is True


def test_localization_filter_without_feed_checks_warns():
    plan = resolve(PROFILES, 'kin', {'localization': 'ekf', 'gnss_checks': 'false'})
    assert any(w.startswith('filter (ekf) with gnss_checks false') for w in plan.warnings)
    plan = resolve(PROFILES, 'kin', {'filter': 'true', 'gnss_checks': 'false'})
    assert any(w.startswith('filter (shadow) with gnss_checks false') for w in plan.warnings)
    assert not any('unchecked' in w for w in resolve(PROFILES, 'kin',
                                                     {'gnss_checks': 'false'}).warnings)
