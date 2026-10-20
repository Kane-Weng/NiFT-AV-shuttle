"""
End to end, natively, through the worker protocol: route 03 of test_routes.yaml arrives.

Only JSON text goes in and out (`Session.handle` and `Session.advance`), as in the Web Worker. The stack is the real one: kinematic_sim_node with BrowserOctane, beacon_node over the socket.io stand-in, shuttle_tf_node, waypoint_node on the Lanelet2 stand-in and trajectory_controller, all unchanged on fakeros.

The ROS reference (tests/reference/runs/direct_03) arrives 48.1 s after the start and rests 2.81 m past the path end, 2.89 m from the goal.
"""

import json
import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
ROUTES = os.path.join(REPO, 'nift_simulation', 'nift_carla', 'config', 'test_routes.yaml')
REFERENCE = os.path.join(REPO, 'nift_playground', 'tests', 'reference', 'runs', 'direct_03', 'summary.json')


def test_route03_goal_start_arrive(playground):
    from playground.session import Session
    with open(ROUTES) as f:
        route = yaml.safe_load(f)['routes']['03']
    gx, gy, gyaw = route['goal']
    s = Session(playground)

    def send(msg):
        return json.loads(s.handle(json.dumps(msg)))

    def advance(n):
        return json.loads(s.advance(n))

    out = send({'type': 'init', 'baseUrl': '/', 'mapUrl': 'maps/lanelet2_odr2024.osm', 'launch': {'spawn': '03'},
                'id': 'init'})
    assert not [m for m in out if m['type'] == 'error'], out
    ready = next(m for m in out if m['type'] == 'ready')
    assert [n['name'] for n in ready['launch']['nodes'] if n['repo']] == [
        'kinematic_sim', 'beacon_node', 'shuttle_dynamic_broadcaster', 'waypoint_node', 'trajectory_controller']
    assert {'id': 'init', 'ok': True, 'message': 'ready', 'type': 'reply'} in out

    # GNSS and TF flowing within a second
    msgs = advance(20)
    state = [m for m in msgs if m['type'] == 'state'][-1]
    assert state['poseEst'] is not None and state['gnss']['fixAge'] is not None
    assert math.hypot(state['poseEst']['x'] - route['spawn'][0], state['poseEst']['y'] - route['spawn'][1]) < 0.05

    out = send({'type': 'goal', 'x': gx, 'y': gy, 'yaw': gyaw, 'id': 'g'})
    out += advance(10)
    ack = next(m for m in out if m['type'] == 'goal_ack')
    assert ack['ok'] and (ack['x'], ack['y']) == (gx, gy)
    path = [m for m in out if m['type'] == 'path'][-1]['points']
    assert len(path) >= 2 and math.hypot(path[-1][0] - gx, path[-1][1] - gy) < 3.0

    out = send({'type': 'cmd', 'name': 'start', 'id': 's'})
    reply = next(m for m in out if m['type'] == 'reply')
    assert reply['ok'], reply
    t_start = [m for m in out if m['type'] == 'state'][-1]['t']

    arrived, events, last = None, [], None
    for _ in range(route['timeout_s'] * 20 // 100):
        msgs = advance(100)
        assert not [m for m in msgs if m['type'] == 'error'], [m for m in msgs if m['type'] == 'error']
        events += [m for m in msgs if m['type'] == 'event']
        last = [m for m in msgs if m['type'] == 'state'][-1]
        arrived = next((e for e in events if e['kind'] == 'arrived'), None)
        if arrived is not None and abs(last['speed']) < 0.05:
            break
    assert arrived is not None, f'no arrival; last state {last}'
    modes = [e['data']['status'] for e in events if e['kind'] == 'mode']
    assert modes[-2:] == ['NAVIGATING', 'IDLE'] and 'SAFE_STOP' not in modes

    with open(REFERENCE) as f:
        ref = json.load(f)
    nav_s = arrived['t'] - t_start
    assert abs(nav_s - ref['nav_duration_s']) < 0.5, nav_s
    dist = math.hypot(last['pose']['x'] - gx, last['pose']['y'] - gy)
    assert abs(dist - ref['final_dist_to_goal']) < 0.5, dist
    assert last['status'] == 'IDLE' and last['mode'] == 'IDLE' and last['gear'] == 'N'
