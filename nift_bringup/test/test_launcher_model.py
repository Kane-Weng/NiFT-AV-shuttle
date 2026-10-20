"""
Test the launcher's logic (launcher_model.py).

Node states from profiles.resolve(), clicks and what goes off with them, hints, the command and
its round trip through resolve(); and that the map places every component.
"""

import os

from nift_bringup.launcher_model import (
    LAUNCH,
    LauncherModel,
    localization,
    NodeState,
    safety_filter,
)
from nift_bringup.profiles import COMPONENTS, load, ProfileError, REQUIRES, resolve
import pytest

PROFILES_YAML = os.path.join(os.path.dirname(__file__), '..', 'config', 'profiles.yaml')
PROFILES = load(PROFILES_YAML)
ON, LOCKED, AVAILABLE, BLOCKED = (NodeState.ON, NodeState.LOCKED, NodeState.AVAILABLE,
                                  NodeState.BLOCKED)


def model(preset='none', mode=None, **args):
    m = LauncherModel(PROFILES, preset)
    if mode:
        m.set_mode(mode)
    for k, v in args.items():
        m.set_arg(k, v)
    return m


def round_trip(m):
    """Feed the command's arguments back through resolve() as the command line."""
    cmd = m.command()
    assert cmd[:len(LAUNCH)] == LAUNCH
    cli = dict(arg.split(':=', 1) for arg in cmd[len(LAUNCH):])
    return resolve(PROFILES, cli.pop('profile'), cli)


# ---------------------------------------------------------------- map layout
def test_every_component_has_a_place_on_the_map():
    pytest.importorskip('python_qt_binding')
    from nift_bringup import launcher
    assert set(launcher.POSITIONS) == set(COMPONENTS)
    assert len(set(launcher.POSITIONS.values())) == len(COMPONENTS)        # no two on one spot
    assert set(launcher.COLOR) == set(COMPONENTS)
    assert set(launcher.LABEL_SIDE) | set(launcher.SUBTITLE) <= set(COMPONENTS)
    edges = {(src, dst) for dst, needs in REQUIRES.items() for need in needs
             for src in (need if isinstance(need, tuple) else (need,))}
    assert set(launcher.BENDS) <= edges
    assert all(need in REQUIRES[dst] for dst, need in launcher.JUNCTIONS)
    assert set(launcher.JUNCTION_LABEL) <= set(launcher.JUNCTIONS.values())
    assert not any(t in REQUIRES or t in {s for s, _ in edges} for t in launcher.TOOLS)


# ---------------------------------------------------------------- presets and the command
def test_a_preset_alone_is_just_profile():
    for name, p in PROFILES.items():
        m = model(name)
        assert m.command() == LAUNCH + [f'profile:={name}']
        plan = m.plan()
        assert plan.on == m.on and plan.mode == p['mode'] and m.errors() == []


def test_command_holds_only_what_differs_from_the_preset():
    m = model('shuttle')
    m.click('rviz')
    m.click('web')
    assert m.command() == LAUNCH + ['profile:=shuttle', 'rviz:=false', 'web:=true']
    m.set_arg('carla_host', ' carla-server ')
    m.set_mode('sim')
    assert m.command() == LAUNCH + ['profile:=shuttle', 'mode:=sim', 'rviz:=false', 'web:=true',
                                    'carla_host:=carla-server']
    assert m.command(checks=False, dry_run=True)[-2:] == ['checks:=false', 'dry_run:=true']


def test_sim_never_switches_can():
    assert not any(a.startswith('can:=') for a in model('sim').command())
    assert not any(a.startswith('can:=') for a in model('none', mode='sim').command())
    # the real CAN bridge is a switch again
    assert 'can:=true' in model('sim', mode='real').command()


def variants(name):
    """Yield the preset, each single click on it, the other mode, and lateral with its needs."""
    yield model(name)
    for c in COMPONENTS:
        m = model(name)
        m.click(c)
        yield m
    yield model(name, mode='real' if PROFILES[name]['mode'] == 'sim' else 'sim')
    m = model(name)
    m.turn_on_with_prereqs('lateral')
    yield m


def test_command_round_trips_through_resolve():
    changed = 0
    for name in PROFILES:
        for m in variants(name):
            # clicks never leave a broken selection
            assert m.errors() == [], (name, m.command())
            plan = round_trip(m)
            assert (plan.on, plan.mode) == (m.on, m.mode), m.command()
            changed += len(m.command()) > len(LAUNCH) + 1
    assert changed > 50         # most variants really differ from their preset


# ---------------------------------------------------------------- node states
def test_states_from_nothing():
    states = model('none').states()
    available = {c for c, s in states.items() if s == AVAILABLE}
    blocked = {c for c, s in states.items() if s == BLOCKED}
    assert available == {'can', 'gnss', 'perception', 'bag', 'rviz', 'web'}
    assert blocked == {'localization', 'filter', 'planning', 'static_path', 'longitudinal',
                       'lateral'}


def test_activatable_is_what_resolve_accepts():
    for name in PROFILES:
        for m in variants(name):
            for c in COMPONENTS:
                if c in m.on:
                    continue
                try:
                    resolve(PROFILES, m.preset, {**m.cli(), c: 'true'})
                    accepted = True
                except ProfileError:
                    accepted = False
                assert m.activatable(c) == accepted, (m.command(), c)


def test_can_is_locked_in_sim():
    m = model('sim')
    assert m.state('can') == LOCKED
    change = m.click('can')
    assert 'can' in m.on and not change.turned_off and 'CARLA bridge' in change.message


def test_kinematic_button_from_the_carla_sim_switches_perception_off():
    m = model('sim')
    change = m.set_backend('kinematic')
    assert (m.kinematic and change.turned_off == ['perception']
            and 'kinematic sim' in change.message)
    assert (m.errors() == [] and m.state('perception') == BLOCKED
            and 'LiDAR' in m.hint('perception'))
    assert m.command() == LAUNCH + ['profile:=sim', 'perception:=false', 'sim_backend:=kinematic']
    assert round_trip(m).kinematic
    assert 'kinematic sim' in m.click('can').message


def test_kinematic_button_from_the_real_vehicle():
    m = model('shuttle')
    change = m.set_backend('kinematic')
    assert m.sim and m.kinematic and 'can' in m.on and change.turned_off == ['perception']
    assert {'mode:=sim', 'sim_backend:=kinematic', 'perception:=false'} <= set(m.command())
    assert round_trip(m).kinematic
    m.set_mode('real')
    assert not any(a.startswith('sim_backend') for a in m.command())


def test_kinematic_preset_and_back_to_carla():
    m = model('sim_kinematic')
    assert (m.kinematic and m.vehicle == 'kinematic sim'
            and m.command() == LAUNCH + ['profile:=sim_kinematic'])
    m.set_backend('carla')
    assert not m.kinematic and m.vehicle == 'CARLA bridge' and 'sim_backend:=carla' in m.command()
    assert m.state('perception') == AVAILABLE
    m.set_backend('kinematic')                      # the preset's own backend: nothing to add
    assert m.command() == LAUNCH + ['profile:=sim_kinematic']


def test_a_preset_brings_its_own_backend():
    m = model('sim')
    m.set_backend('kinematic')
    m.select('sim')
    assert not m.kinematic and m.args['sim_backend'] == '' and m.errors() == []


def test_static_path_is_sim_only_needs_a_scenario_and_excludes_planning():
    assert model('none').state('static_path') == BLOCKED
    assert 'sim only' in model('none').hint('static_path')
    assert 'scenario_file' in model('none', mode='sim').hint('static_path')
    assert model('none', mode='sim', scenario_file='/tmp/s.yaml').activatable('static_path')
    bench = model('sim_filter_bench')
    assert bench.state('planning') == BLOCKED and 'pick one' in bench.hint('planning')


# ---------------------------------------------------------------- clicks
def test_switching_off_takes_what_needs_it():
    m = model('shuttle')
    change = m.click('gnss')
    assert change.turned_off == ['gnss', 'localization', 'planning', 'longitudinal', 'lateral']
    assert m.on == {'can', 'perception', 'bag', 'rviz'}
    assert change.message == 'gnss off, and with it localization, planning, longitudinal, lateral'


def test_switching_off_is_refused_when_resolve_still_fails():
    m = model('shuttle')
    before = set(m.on)
    change = m.click('localization')        # perception + longitudinal still need it
    assert m.on == before and not change.turned_off
    assert 'stays on' in change.message
    assert 'perception with longitudinal needs localization or filter' in change.message


def test_blocked_click_changes_nothing_and_says_what_is_needed():
    m = model('none')
    change = m.click('lateral')
    assert m.on == set() and not change.turned_on
    assert change.message.startswith(
        'lateral needs: longitudinal, localization or filter, planning;')
    assert change.highlight == ['can', 'gnss', 'localization', 'filter', 'planning',
                                'longitudinal']
    sim = model('none', mode='sim')
    assert sim.hint('lateral').startswith('lateral needs: longitudinal, localization or filter, '
                                          'planning or static_path')
    assert 'static_path' in sim.click('lateral').highlight


def test_hint_names_rules_beyond_requires():
    hint = model('speed_control').hint('perception')
    assert 'perception with longitudinal needs localization' in hint


def test_turn_on_with_prereqs():
    m = model('none')
    change = m.turn_on_with_prereqs('lateral')
    assert m.on == {'can', 'gnss', 'localization', 'planning', 'longitudinal', 'lateral'}
    assert change.message == 'lateral on, with can, gnss, localization, planning, longitudinal'

    bench = model('sim_filter_bench')
    bench.click('lateral')
    bench.turn_on_with_prereqs('lateral')
    # keeps the path source it has
    assert 'static_path' in bench.on and 'planning' not in bench.on

    bench = model('sim_filter_bench')
    bench.click('static_path')              # lateral goes with it
    bench.turn_on_with_prereqs('lateral')
    assert {'planning', 'lateral'} <= bench.on and 'static_path' not in bench.on


def test_turn_on_with_prereqs_refuses_what_they_cannot_fix():
    m = model('sim')                        # planning on, no scenario_file
    before = set(m.on)
    change = m.turn_on_with_prereqs('static_path')
    assert m.on == before and not change.turned_on and 'pick one' in change.message


def test_switching_to_real_drops_sim_only():
    m = model('sim_filter_bench')
    change = m.set_mode('real')
    assert change.turned_off == ['static_path', 'lateral']
    assert 'can' in m.on and m.mode == 'real' and m.errors() == []
    assert model('none').set_mode('sim').turned_on == ['can']


# ---------------------------------------------------------------- typed-in arguments
def test_a_bad_argument_breaks_the_plan_but_not_the_map():
    m = model('sim', filter_type='cfb')
    assert any('filter_type:=cfb' in e for e in m.errors())
    with pytest.raises(ProfileError):
        m.plan()
    assert m.click('perception').turned_off == ['perception']       # nothing new breaks
    assert m.activatable('perception')


def test_a_half_typed_package_path_is_an_error_not_a_crash():
    m = model('sim', scenario_file='package://')
    assert m.errors() and m.errors()[0].startswith('package path:')


def test_safety_filter():
    assert safety_filter(model('speed_control').plan()) == 'none (perception off)'
    assert safety_filter(model('shuttle',
                               filter_type='potential_field').plan()) == 'potential_field'
    assert 'planning.yaml' in safety_filter(model('shuttle').plan())
    assert 'longitudinal off' in safety_filter(model('can_only').plan())


# ---------------------------------------------------------------- the filter station
def test_filter_station_switches_the_localization_mode():
    from nift_bringup.components.control import localization_source
    m = model('sim_kinematic')
    assert m.state('filter') == AVAILABLE and localization(m.plan()).startswith('direct: ')
    change = m.click('filter')                                  # beside localization: shadow
    assert (change.turned_on == ['filter']
            and change.message.startswith('filter on. Localization shadow: '))
    assert m.command() == LAUNCH + ['profile:=sim_kinematic', 'filter:=true']
    plan = round_trip(m)
    assert plan.localization_mode == 'shadow' and localization_source(plan) == 'gnss'
    # the filter alone: ekf, nothing else goes off
    change = m.click('localization')
    assert (change.turned_off == ['localization']
            and change.message.startswith('localization off. Localization ekf: '))
    assert m.command() == LAUNCH + ['profile:=sim_kinematic', 'localization:=false',
                                    'filter:=true']
    plan = round_trip(m)
    assert plan.localization_mode == 'ekf' and localization_source(plan) == 'filter'
    assert localization(plan).startswith('ekf: the filter publishes map')
    assert m.click('localization').message.startswith('localization on. Localization shadow: ')
    assert m.click('filter').message.startswith('filter off. Localization direct: ')
    assert m.command() == LAUNCH + ['profile:=sim_kinematic']


def test_filter_alone_takes_what_needs_map_to_base_link_when_it_goes_off():
    m = model('sim_kinematic')
    m.click('filter')
    m.click('localization')
    change = m.click('filter')
    assert change.turned_off == ['filter', 'planning',
                                 'lateral'] and 'Localization' not in change.message
    assert localization(m.plan()) == ''


def test_shuttle_with_the_filter_beside_it_is_shadow():
    m = model('shuttle')
    m.click('filter')
    assert m.command() == LAUNCH + ['profile:=shuttle', 'filter:=true']
    assert round_trip(m).localization_mode == 'shadow'
    # ekf: the filter serves perception too
    assert m.click('localization').turned_off == ['localization']
    assert m.errors() == [] and round_trip(m).localization_mode == 'ekf'


def test_double_click_takes_localization_before_the_filter():
    m = model('none')
    change = m.turn_on_with_prereqs('planning')
    assert change.turned_on == ['gnss', 'localization',
                                'planning'] and 'Localization' not in change.message
    m = model('none')
    m.turn_on_with_prereqs('filter')
    assert m.on == {'gnss', 'filter'}
    # the filter is a map -> base_link already
    change = m.turn_on_with_prereqs('lateral')
    assert 'localization' not in m.on and change.turned_on == ['can', 'planning', 'longitudinal',
                                                               'lateral']
