"""
The session (ARCH 8): the worker protocol both ways, GNSS loss to SAFE_STOP, and exact replay of a scenario.

Each test drives the real stack natively (route 03's spawn, a few hundred ticks), so the file takes some seconds.
"""

import json
import math

import pytest

GOAL_03 = (47.38, -16.92, -0.0452)


def new_session(playground, launch=None, record=False):
    from playground.session import Session
    s = Session(playground)
    s.record = record
    out = s.handle_obj({'type': 'init', 'launch': dict({'spawn': '03'}, **(launch or {})), 'id': 'i'})
    assert not [m for m in out if m['type'] == 'error'], out
    return s, out


def of(msgs, kind):
    return [m for m in msgs if m['type'] == kind]


def drive_03(s, ticks_after_start=200):
    """Ready, goal, start, then `ticks_after_start` ticks. Returns every message."""
    out = s.advance_obj(20)
    out += s.handle_obj({'type': 'goal', 'x': GOAL_03[0], 'y': GOAL_03[1], 'yaw': GOAL_03[2]})
    out += s.advance_obj(10)
    out += s.handle_obj({'type': 'cmd', 'name': 'start', 'id': 'start'})
    assert of(out, 'reply')[-1]['ok'], of(out, 'reply')
    out += s.advance_obj(ticks_after_start)
    return out


# ================================================================ protocol

def test_init_ready_and_presets(playground):
    s, out = new_session(playground)
    assert [m['type'] for m in out][:2] == ['progress', 'ready']
    ready = of(out, 'ready')[0]
    assert ready['python'] and ready['dt'] == 0.05
    assert ready['launch']['localization_mode'] == 'direct' and ready['launch']['filter_type'] == 'none'
    assert {r['id'] for r in ready['presets']['routes']} >= {'01', '03', '12'}
    assert ready['presets']['stack']['requires']['lateral'] == [['longitudinal'], ['localization', 'filter'], ['planning', 'static_path']]
    gf = ready['presets']['gnss_faults']                 # ARCH 12: one flat list
    kinds = {p['name']: p['kind'] for p in gf}
    assert kinds['dropout_1.5s_turn'] == 'scheduled' and kinds['outage'] == 'link' and kinds['dropout'] == 'sim'
    assert ready['launch']['options']['spawn'] == '03' and ready['launch']['options']['localization'] == 'direct'
    st = of(out, 'state')[-1]
    assert st['tick'] == 0 and st['t'] == pytest.approx(-0.05)
    json.dumps(out, allow_nan=False)


def test_json_text_roundtrip_and_errors(playground):
    s, _ = new_session(playground)
    out = json.loads(s.handle('{"type": "nope", "id": "x"}'))
    assert out == [{'type': 'reply', 'id': 'x', 'ok': False, 'message': "unknown message type 'nope'"}]
    out = json.loads(s.handle('{"type": "cmd", "name": "fly"}'))
    assert out[0]['type'] == 'error' and not out[0]['fatal']
    out = json.loads(s.advance(4))
    assert of(out, 'state')[-1]['tick'] == 4          # stateHz 10 at dt 0.05: every 2nd tick
    assert len(of(out, 'state')) == 2


def test_bad_launch_options_keep_the_stack(playground):
    s, _ = new_session(playground)
    s.advance_obj(4)
    out = s.handle_obj({'type': 'reset', 'launch': {'localization': 'gps'}, 'id': 'r'})
    assert out == [{'type': 'reply', 'id': 'r', 'ok': False,
                    'message': "localization 'gps': expected one of ('direct', 'ekf', 'shadow', 'off')"}]
    out = s.handle_obj({'type': 'scenario_load', 'scenario': {'v': 2}, 'id': 'l'})
    assert not out[-1]['ok']
    assert s.runner.tick == 4                           # still the same stack


def test_pacing_messages(playground):
    s, _ = new_session(playground)
    out = s.handle_obj({'type': 'pause', 'id': 'p'})
    assert of(out, 'state')[-1]['paused'] is True
    out = s.handle_obj({'type': 'step', 'n': 3, 'id': 's'})
    assert of(out, 'state')[-1]['tick'] == 3 and of(out, 'reply')[-1]['ok']
    out = s.handle_obj({'type': 'speed', 'factor': 'max', 'id': 'f'})
    assert of(out, 'state')[-1]['factor'] == 'max'
    out = s.handle_obj({'type': 'speed', 'factor': 1000, 'id': 'f'})
    assert not of(out, 'reply')[-1]['ok']
    out = s.handle_obj({'type': 'resume_sim'})
    assert of(out, 'state')[-1]['paused'] is False
    assert s.events == []                               # pacing is not logged


def test_set_param_takes_effect_on_reset(playground):
    s, _ = new_session(playground)
    out = s.handle_obj({'type': 'set_param', 'node': 'trajectory_controller', 'name': 'base_speed', 'value': 2,
                        'id': 'sp'})
    assert of(out, 'reply')[-1]['ok'] and 'next reset' in of(out, 'reply')[-1]['message']
    out = s.handle_obj({'type': 'reset', 'launch': {'localization': 'ekf'}, 'id': 'r'})
    ready = of(out, 'ready')[0]
    # the int 2 is staged as 2.0: base_speed is a double, and an int would be the wrong type on ROS
    assert ready['launch']['options']['params'] == {'trajectory_controller': {'base_speed': 2.0}}
    assert ready['launch']['options']['spawn'] == '03'          # a partial reset keeps the other options
    assert 'localization_node' in [n['name'] for n in ready['launch']['nodes']]
    assert s.runner.nodes['trajectory_controller'].get_parameter('base_speed').value == 2.0
    assert s.staged == {}
    out = s.advance_obj(30)
    assert of(out, 'state')[-1]['locStatus'] in ('OK', 'DEGRADED')


@pytest.mark.parametrize('msg, text', [
    ({'node': 'trajectory_controller', 'name': 'max_speed', 'value': 1.0}, 'not offered'),
    ({'node': 'waypoint_node', 'name': 'map_publish_period_s', 'value': 'fast'}, 'not offered'),
    ({'node': 'trajectory_controller', 'name': 'base_speed', 'value': 'fast'}, 'takes a number'),
    ({'node': 'trajectory_controller', 'name': 'base_speed', 'value': True}, 'takes a number'),
    ({'node': 'trajectory_controller', 'name': 'base_speed', 'value': 60.0}, 'within'),
])
def test_set_param_refuses_bad_values(playground, msg, text):
    s, _ = new_session(playground)
    out = s.handle_obj(dict(msg, type='set_param', id='sp'))
    assert out == [{'type': 'reply', 'id': 'sp', 'ok': False, 'message': out[0]['message']}]
    assert text in out[0]['message'] and s.staged == {}


def test_failed_reset_restores_the_previous_launch(playground):
    """H1: a node that raises at start-up (a parameter of the wrong type) does not end the session."""
    s, _ = new_session(playground)
    s.handle_obj({'type': 'obstacle_add', 'oid': 'keep', 'poly': [[0, 0], [1, 0], [1, 1]]})
    s.handle_obj({'type': 'set_param', 'node': 'trajectory_controller', 'name': 'base_speed', 'value': 2.5})
    out = s.handle_obj({'type': 'reset', 'id': 'r',
                        'launch': {'params': {'waypoint_node': {'map_publish_period_s': 'fast'}}}})
    err = of(out, 'error')
    assert len(err) == 1 and not err[0]['fatal'] and 'InvalidParameterType' in err[0]['message']
    reply = of(out, 'reply')[-1]
    assert not reply['ok'] and 'previous launch was restored' in reply['message']
    assert of(out, 'ready')[-1]['launch']['options']['params'] == {}
    assert s.runner is not None and s.staged == {}           # the bad launch's staged values are gone
    assert list(s.runner.obstacles) == ['keep']
    out = s.advance_obj(20)
    assert not of(out, 'error') and of(out, 'state')[-1]['poseEst'] is not None
    out = s.handle_obj({'type': 'reset', 'id': 'r2'})       # and a later reset works
    assert of(out, 'reply')[-1]['ok']
    # without a reply id: one error, not two
    out = s.handle_obj({'type': 'reset', 'launch': {'params': {'waypoint_node': {'map_publish_period_s': 'x'}}}})
    assert len(of(out, 'error')) == 1 and s.runner is not None


def test_broken_scenario_is_refused_not_fatal(playground):
    """M8: a bad URL hash refuses the scenario; the stack starts (init) or stays (scenario_load)."""
    from playground.session import Session
    from playground.scenario import encode
    s = Session(playground)
    out = s.handle_obj({'type': 'init', 'scenario': 'not-a-scenario!!', 'launch': {'spawn': '03'}, 'id': 'i'})
    assert of(out, 'ready') and not of(out, 'error')
    assert not of(out, 'reply')[-1]['ok'] and 'scenario refused' in of(out, 'reply')[-1]['message']
    import base64
    import zlib
    c = zlib.compressobj(9, zlib.DEFLATED, -15)
    bad_json = base64.urlsafe_b64encode(c.compress(b'{"v": 1, "events": [') + c.flush()).decode().rstrip('=')
    for sc in (bad_json, '#' + encode([1, 2]), {'v': 1, 'events': [{'tick': -1, 'msg': {'type': 'goal'}}]},
               {'v': 1, 'events': 'x'}, '{"v": 1'):
        out = s.handle_obj({'type': 'scenario_load', 'scenario': sc, 'id': 'l'})
        assert out[-1]['type'] == 'reply' and not out[-1]['ok'], (sc, out)
        assert not of(out, 'error')
    assert s.runner is not None


def test_gnss_fault_bad_params_refused(playground):
    s, _ = new_session(playground)
    out = s.handle_obj({'type': 'gnss_fault', 'preset': 'dropout', 'params': {'durration_s': 1.0}, 'id': 'f'})
    assert not out[-1]['ok'] and 'unknown parameter' in out[-1]['message']
    out = s.handle_obj({'type': 'gnss_fault', 'preset': 'outage', 'params': {'x': 1}, 'id': 'f'})
    assert not out[-1]['ok']
    out = s.handle_obj({'type': 'gnss_fault', 'preset': 'teleport', 'id': 'f'})
    assert not out[-1]['ok'] and not of(out, 'error')
    assert s.events == []


def test_switched_off_components_run(playground):
    """The launcher view can leave the stack without control or GNSS: the sim still runs and Start is refused."""
    s, out = new_session(playground, {'components': {'lateral': False, 'longitudinal': False}})
    assert 'trajectory_controller' not in [n['name'] for n in of(out, 'ready')[0]['launch']['nodes']]
    out = s.advance_obj(20)
    out += s.handle_obj({'type': 'cmd', 'name': 'start', 'id': 'start'})
    assert not of(out, 'reply')[-1]['ok']
    out = s.handle_obj({'type': 'reset', 'id': 'r', 'launch': {'localization': 'off', 'components': {'gnss': False, 'planning': False, 'longitudinal': False, 'lateral': False}}})
    assert of(out, 'reply')[-1]['ok'], of(out, 'reply')
    assert of(s.advance_obj(40), 'state')[-1]['tick'] > 0
    json.dumps(out, allow_nan=False)


def test_services_and_dbw_estop(playground):
    s, _ = new_session(playground)
    s.advance_obj(20)
    out = s.handle_obj({'type': 'cmd', 'name': 'start', 'id': 'a'})
    assert not of(out, 'reply')[-1]['ok']                        # no path yet: refused, with the reason
    out = s.handle_obj({'type': 'cmd', 'name': 'manual', 'id': 'm'})
    assert of(out, 'reply')[-1]['ok']
    out = s.handle_obj({'type': 'teleop', 'v': 1.0, 'steer': 0.0, 'id': 't'})
    out += s.advance_obj(60)
    assert of(out, 'state')[-1]['status'] == 'MANUAL' and of(out, 'state')[-1]['speed'] > 0.3
    out = s.handle_obj({'type': 'cmd', 'name': 'dbw_estop', 'id': 'e'})
    out += s.advance_obj(10)
    assert of(out, 'state')[-1]['ecu'] == 'estop'
    out = s.handle_obj({'type': 'cmd', 'name': 'power_cycle', 'id': 'pc'})
    assert of(out, 'reply')[-1]['ok']


def test_initialpose_teleports(playground):
    s, _ = new_session(playground)
    s.advance_obj(10)
    out = s.handle_obj({'type': 'initialpose', 'x': -12.81, 'y': 44.70, 'yaw': -0.0169, 'id': 'ip'})
    out += s.advance_obj(10)
    st = of(out, 'state')[-1]
    assert math.hypot(st['pose']['x'] + 12.81, st['pose']['y'] - 44.70) < 0.01
    # beacon_node's feed checks drop the 20 m jump (baseline, heading), pause /gnss/*, then re-anchor
    out += s.advance_obj(60)
    texts = [m['text'] for m in of(out, 'log') if m['node'] == 'beacon_node']
    assert any(t.startswith('GNSS stale') for t in texts) and any('re-anchored' in t for t in texts)
    st = of(out, 'state')[-1]
    assert math.hypot(st['poseEst']['x'] + 12.81, st['poseEst']['y'] - 44.70) < 0.1 and not st['gnss']['stale']


def test_obstacles_and_perception(playground):
    s, _ = new_session(playground, {'perception': True})
    out = s.handle_obj({'type': 'obstacle_add', 'oid': 'b1', 'id': 'r1',
                        'box': {'x': 1.2, 'y': 22.0, 'yaw': 0.0, 'length': 1.0, 'width': 1.0}})
    assert of(out, 'reply')[-1] == {'type': 'reply', 'id': 'r1', 'ok': True, 'message': ''}
    ob = of(out, 'obstacles')[-1]
    assert list(ob['polygons'][0]) == ['oid', 'poly'] and ob['polygons'][0]['oid'] == 'b1'
    assert len(ob['polygons'][0]['poly']) == 4
    out = s.advance_obj(10)
    perceived = of(out, 'obstacles')[-1]['perceived']
    assert perceived and len(perceived) == 1              # 8 m ahead of the shuttle, in the ROI
    out = s.handle_obj({'type': 'obstacle_move', 'oid': 'b1', 'x': 40.0, 'y': 22.0, 'id': 'r2'})
    assert of(out, 'reply')[-1]['ok']
    out += s.advance_obj(10)
    assert of(out, 'obstacles')[-1]['perceived'] == []    # behind: cleared with an empty polygon
    out = s.handle_obj({'type': 'obstacle_remove', 'oid': 'b1', 'id': 'r3'})
    assert of(out, 'obstacles')[-1]['polygons'] == [] and of(out, 'reply')[-1]['ok']
    out = s.handle_obj({'type': 'obstacle_remove', 'oid': 'b1', 'id': 'r4'})
    assert not of(out, 'reply')[-1]['ok']                 # gone already
    out = s.handle_obj({'type': 'obstacle_add', 'id': 'r5', 'poly': [[0, 0], [1, 0], [1, 1]]})
    assert not of(out, 'reply')[-1]['ok'] and 'oid' in of(out, 'reply')[-1]['message']
    out = s.handle_obj({'type': 'set_obstacles', 'obstacles': [{'oid': 'a', 'poly': [[0, 0], [1, 0], [1, 1]]}]})
    assert [p['oid'] for p in of(out, 'obstacles')[-1]['polygons']] == ['a']
    s2, _ = new_session(playground)
    out = s2.handle_obj({'type': 'obstacle_add', 'oid': 'x', 'poly': [[0, 0], [1, 0], [1, 1]]})
    assert of(out, 'obstacles')[-1]['perceived'] is None  # no perception: the page only draws it


# ================================================================ GNSS loss

def test_gnss_outage_ends_in_safe_stop(playground):
    s, _ = new_session(playground)
    out = drive_03(s, 100)
    assert of(out, 'state')[-1]['status'] == 'NAVIGATING'
    out = s.handle_obj({'type': 'gnss_fault', 'preset': 'outage', 'params': {'duration_s': 30.0}, 'id': 'f'})
    assert of(out, 'reply')[-1]['ok'], of(out, 'reply')
    t_fault = of(out, 'state')[-1]['t']
    out = s.advance_obj(200)
    logs = [(m['node'], m['text']) for m in of(out, 'log')]
    assert any(n == 'beacon_node' and t.startswith('GNSS stale') for n, t in logs)
    assert any(n == 'trajectory_controller' and 'GNSS lost' in t for n, t in logs)
    stop = next(e for e in of(out, 'event') if e['kind'] == 'mode' and e['data']['status'] == 'SAFE_STOP')
    # as on ROS: beacon_node pauses /gnss/* 0.55 s after the last fix, the controller stops 0.3 s later
    assert 0.6 < stop['t'] - t_fault < 1.2, stop['t'] - t_fault
    st = of(out, 'state')[-1]
    assert st['status'] == 'SAFE_STOP' and abs(st['speed']) < 0.05 and st['gnss']['stale'] is True


def test_scheduled_fault_launch_option(playground):
    """gnss_faults as a list (a temporary YAML file, as gnss_faults:= takes one)."""
    s, out = new_session(playground, {'gnss_faults': [{'kind': 'dropout', 't_s': 8.0, 'duration_s': 30.0}]})
    out = drive_03(s, 200)
    faults = [e for e in of(out, 'event') if e['kind'] == 'gnss_fault']
    assert faults and faults[0]['data']['scenario'].endswith('playground_gnss_faults.yaml')
    assert any(e['kind'] == 'mode' and e['data']['status'] == 'SAFE_STOP' for e in of(out, 'event'))


# ================================================================ scenarios

def trace_rows(s):
    t = s.runner.observer.trace
    return {k: v for k, v in t.rows.items()}


def test_replay_is_exact(playground):
    """The same launch options and command log give the same trace (ARCH 7, 8.5)."""
    s, _ = new_session(playground, {'localization': 'ekf'}, record=True)
    drive_03(s, 120)
    s.handle_obj({'type': 'gnss_fault', 'preset': 'dropout', 'params': {'duration_s': 0.8}})
    s.handle_obj({'type': 'obstacle_add', 'oid': 'o', 'id': 'r', 'poly': [[0, 0], [1, 0], [1, 1]]})
    s.handle_obj({'type': 'obstacle_add', 'oid': 'b', 'box': {'x': 5, 'y': 5, 'yaw': 0.3, 'length': 2, 'width': 1}})
    s.advance_obj(80)
    s.handle_obj({'type': 'obstacle_move', 'oid': 'b', 'x': 6.0, 'y': 4.0, 'yaw': 1.1, 'id': 'm'})
    s.handle_obj({'type': 'cmd', 'name': 'stop', 'id': 'stop'})
    s.advance_obj(60)
    out = s.handle_obj({'type': 'scenario_save', 'id': 'save'})
    sc = of(out, 'scenario')[0]['scenario']
    assert [e['msg']['type'] for e in sc['events']] == ['goal', 'cmd', 'gnss_fault', 'obstacle_add', 'obstacle_add',
                                                        'obstacle_move', 'cmd']
    assert all('id' not in e['msg'] for e in sc['events'])     # reply ids are not part of the log
    polygons = s.runner.obstacles_message()['polygons']
    ticks = s.runner.tick
    first = trace_rows(s)

    from playground.scenario import decode, encode
    assert decode(encode(sc)) == sc
    s2, _ = new_session(playground, record=True)
    out = s2.handle_obj({'type': 'scenario_load', 'scenario': encode(sc), 'id': 'load'})
    assert of(out, 'reply')[-1]['ok'] and of(out, 'ready')[0]['launch']['localization_mode'] == 'ekf'
    s2.advance_obj(ticks)
    second = trace_rows(s2)
    for name in first:
        assert first[name] == second[name], name
    assert s2.scenario()['events'] == sc['events']
    assert s2.runner.obstacles_message()['polygons'] == polygons


def test_obstacles_survive_a_reset_and_replay(playground):
    s, _ = new_session(playground)
    s.handle_obj({'type': 'obstacle_add', 'oid': 'b', 'box': {'x': 5, 'y': 5, 'yaw': 0.3, 'length': 2, 'width': 1}})
    s.advance_obj(5)
    out = s.handle_obj({'type': 'reset', 'id': 'r'})
    assert [p['oid'] for p in of(out, 'obstacles')[-1]['polygons']] == ['b']
    sc = s.scenario()
    assert sc['events'][0] == {'tick': 0, 'msg': {'type': 'set_obstacles', 'obstacles': [
        {'oid': 'b', 'poly': s.runner.obstacles_message()['polygons'][0]['poly']}]}}
    s2, _ = new_session(playground)
    s2.handle_obj({'type': 'scenario_load', 'scenario': sc})
    assert s2.runner.obstacles_message()['polygons'] == s.runner.obstacles_message()['polygons']


def test_reset_equals_a_fresh_session(playground):
    """Reset keeps the interpreter and the modules; the run after it equals one in a new Session."""
    a, _ = new_session(playground, record=True)
    drive_03(a, 100)
    a.handle_obj({'type': 'reset'})
    drive_03(a, 100)
    b, _ = new_session(playground, record=True)
    drive_03(b, 100)
    assert trace_rows(a) == trace_rows(b)


def test_scenario_codec():
    import sys
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py'))
    from playground import scenario
    sc = scenario.make({'spawn': '04'}, [{'tick': 3, 'msg': {'type': 'goal', 'x': 1.0, 'y': 2.0}}], 'n')
    text = scenario.encode(sc)
    assert '=' not in text and '+' not in text and '/' not in text
    assert scenario.decode(text) == sc and scenario.decode('#' + text) == sc
    import base64
    import zlib
    zlib_text = base64.b64encode(zlib.compress(json.dumps(sc).encode())).decode()
    assert scenario.decode(zlib_text) == sc
    with pytest.raises(scenario.ScenarioError):
        scenario.check({'v': 1, 'events': [{'tick': 1, 'msg': {'type': 'pause'}}]})
    with pytest.raises(scenario.ScenarioError):
        scenario.check({'v': 1, 'events': [{'tick': 5, 'msg': {'type': 'goal'}}, {'tick': 2, 'msg': {'type': 'goal'}}]})


def test_environment_is_left_as_it_was(playground):
    """L5: the MCITY_OCTANE_* keys are set only while beacon_node is built (that is when it reads them)."""
    import os
    before = {k: v for k, v in os.environ.items() if k.startswith('MCITY_')}
    s, _ = new_session(playground)
    s.advance_obj(10)
    assert {k: v for k, v in os.environ.items() if k.startswith('MCITY_')} == before
    assert s.runner.nodes['beacon_node'].server == 'http://127.0.0.1:8765'


@pytest.mark.parametrize('arg', ['trajectory_controller.base_speed', 'base_speed=2', 'a.b='])
def test_cli_rejects_param_without_node_name_value(playground, arg, capsys):
    from playground import cli
    with pytest.raises(SystemExit) as e:
        cli.main(['--route', '03', '--param', arg])
    assert e.value.code == 2 and 'NODE.NAME=VALUE' in capsys.readouterr().err


def test_faults_file_is_per_session(playground):
    """L3: a list of faults goes to the session's own folder; the echoed name is the file name only."""
    faults = [{'kind': 'dropout', 't_s': 8.0, 'duration_s': 1.0}]
    a, _ = new_session(playground, {'gnss_faults': faults})
    b, _ = new_session(playground, {'gnss_faults': faults})
    assert a.workdir != b.workdir
    p = a.launch.node('kinematic_sim').params
    assert p['gps.faults_name'] == 'playground_gnss_faults.yaml'


def test_whole_number_double_from_json_replays(playground):
    """A double the page staged as 2.0 comes back from JavaScript's JSON as 2; the launch makes it 2.0 again."""
    from playground.session import Session
    text = ('{"v": 1, "launch": {"spawn": "03", "params": {"trajectory_controller": {"base_speed": 2, '
            '"goal_tolerance": 1}}}, "events": []}')
    s = Session(playground)
    out = s.handle_obj({'type': 'scenario_load', 'scenario': json.loads(text), 'id': 'l'})
    assert of(out, 'reply')[-1]['ok'] and not of(out, 'error'), out
    ctl = s.runner.nodes['trajectory_controller']
    assert ctl.get_parameter('base_speed').value == 2.0 and isinstance(ctl.get_parameter('base_speed').value, float)
    assert s.options['params']['trajectory_controller'] == {'base_speed': 2.0, 'goal_tolerance': 1.0}
    out = json.loads(s.handle(json.dumps({'type': 'init', 'launch': {
        'spawn': '03', 'params': {'waypoint_node': {'map_publish_period_s': 5}}}, 'id': 'i'})))
    assert of(out, 'reply')[-1]['ok']                    # a double of the YAML files: 5 becomes 5.0


def test_public_collect_and_state(playground):
    """What web/src/worker/core.ts calls between ticks."""
    s, _ = new_session(playground)
    s.handle_obj({'type': 'obstacle_add', 'oid': 'a', 'poly': [[0, 0], [1, 0], [1, 1]]})
    s.advance_obj(1)
    msgs = s.collect() + [s.state()]
    assert msgs[-1]['type'] == 'state' and msgs[-1]['tick'] == 1
    json.dumps(msgs, allow_nan=False)
