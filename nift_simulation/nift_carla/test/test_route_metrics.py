"""
Unit tests of the route metrics (nift_carla/tools/route_metrics.py) on synthetic runs.

CTE sign and size, path matching on a route that passes near itself, rest measured along the
lane when the path ends in a sideways hook, wall -> sim time, 0x101 decode, steering rate, turn
windows and radius, and the outcome of whole synthetic runs (completed, stalled).
"""

import json
import math
import os

from can_msgs.msg import Frame
from nift_carla.tools import route_metrics as rm
from nift_carla.tools.route_run import load_routes
from nift_common.can_protocol import (
    CAN_CMD_MAX,
    CAN_ID_COMMAND,
    CAN_ID_ESTOP,
    DLC_COMMAND,
    DLC_ESTOP,
    Gear,
    STEER_FACTOR,
    STEER_OFFSET,
)
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES_YAML = os.path.join(HERE, '..', 'config', 'test_routes.yaml')


def straight(length=60.0, step=2.5):
    xs = np.arange(0.0, length + 1e-9, step)
    return np.c_[xs, np.zeros_like(xs)]


def odom_along(xs, y=0.0, v=2.0, t0=0.0):
    """Odom rows (t, x, y, yaw, v) driving east at v, one per x."""
    t = t0 + (np.asarray(xs) - xs[0]) / v
    return np.c_[t, xs, np.full_like(t, y), np.zeros_like(t), np.full_like(t, v)]


# ---------------------------------------------------------------- CTE and matching

@pytest.mark.parametrize('offset', [0.2, -0.35])
def test_cte_sign_is_plus_left(offset):
    path, s_cum = rm.clean_path(straight())
    pts = np.c_[np.linspace(1.0, 59.0, 50), np.full(50, offset)]
    cte, s, _ = rm.project(path, s_cum, pts)
    assert cte == pytest.approx(np.full(50, offset))
    assert s == pytest.approx(pts[:, 0])


def test_project_follows_progress_on_a_path_that_passes_near_itself():
    # out east along y = 0, a U-turn, back west along y = 2: the legs are 2 m apart
    out = np.c_[np.linspace(0.0, 40.0, 17), np.zeros(17)]
    arc = np.array([(40.0 + math.sin(a), 1.0 - math.cos(a)) for a in np.linspace(0, math.pi, 9)])
    back = np.c_[np.linspace(40.0, 0.0, 17), np.full(17, 2.0)]
    path, s_cum = rm.clean_path(np.vstack([out, arc[1:], back[1:]]))
    # on the path out and round the U-turn, then 1.2 m left of the return leg (y = 0.8):
    # 0.8 m from the out leg, so the globally nearest segment is on the wrong leg
    there = np.c_[np.linspace(0.5, 39.5, 40), np.zeros(40)]
    home = np.c_[np.linspace(38.0, 5.0, 34), np.full(34, 0.8)]
    pts = np.vstack([there, arc[1:], home])
    cte, s, _ = rm.project(path, s_cum, pts, relock=3.0)
    assert np.all(np.diff(s) > 0)
    back_leg = slice(len(there) + len(arc) - 1, None)
    assert np.all(s[back_leg] > 40.0 + math.pi)     # matched to the return leg ...
    assert cte[back_leg] == pytest.approx(np.full(34, 1.2))     # ... 1.2 m left of it
    assert np.abs(cte[:len(there)]) == pytest.approx(np.zeros(40), abs=1e-9)


def test_heading_error_and_wrap():
    assert rm.wrap_deg([190.0, -190.0, 360.0, 0.0]) == pytest.approx([-170.0, 170.0, 0.0, 0.0])
    h = rm.segment_headings(np.array([[0, 0], [1, 0], [1, 1]], float))
    assert h == pytest.approx([0.0, math.pi / 2])


# ---------------------------------------------------------------- goal stop geometry

def hooked_path():
    """Return 50 m east, then a 0.78 m hook to the right (exit leg to a goal beside the lane)."""
    return np.vstack([straight(50.0, 10.0), [[50.5, -0.6]]])


def test_rest_is_measured_along_the_lane_not_the_hook():
    path, s_cum = rm.clean_path(hooked_path())
    rest_xy = np.array([53.0, 0.0])                 # 2.5 m past the end in the lane direction
    rest = rm.along_end(path, s_cum, rest_xy)[0]
    assert rest == pytest.approx(2.46, abs=0.05)
    hook = (path[-1] - path[-2]) / np.linalg.norm(path[-1] - path[-2])
    assert np.dot(rest_xy - path[-1], hook) == pytest.approx(1.14, abs=0.05)   # the wrong one
    assert rm.end_direction(path, s_cum) == pytest.approx([0.998, -0.057], abs=1e-3)


def test_distance_to_go_is_continuous_and_negative_past_the_end():
    path, s_cum = rm.clean_path(straight(60.0))
    xs = np.linspace(40.0, 63.0, 231)
    pts = np.c_[xs, np.zeros_like(xs)]
    _, s, _ = rm.project(path, s_cum, pts)
    dtg = rm.distance_to_go(path, s_cum, s, pts)
    assert dtg == pytest.approx(60.0 - xs)
    assert np.max(np.abs(np.diff(dtg))) < 0.2


def test_rest_index_and_still_for():
    v = np.array([0.0, 0.0, 0.5, 1.0, 2.0, 1.0, 0.2, 0.04, 0.0, 0.0])
    t = np.arange(len(v)) * 0.5
    assert rm.rest_index(v) == 7
    assert rm.still_for(t, v) == pytest.approx(1.0)
    assert rm.rest_index(np.zeros(5)) is None


# ---------------------------------------------------------------- time, CAN

def test_wall_to_sim_with_time_factor_5():
    wall = 100.0 + np.arange(101) * 0.01            # 1 s of wall clock ...
    sim = 5.0 * (wall - 100.0)                      # ... is 5 s of sim time
    assert rm.wall_to_sim([100.5], wall, sim) == pytest.approx([2.5])
    assert rm.wall_to_sim([99.9, 101.1], wall, sim) == pytest.approx([-0.5, 5.5])   # extrapolated
    assert rm.wall_to_sim([3.0], [], []) == pytest.approx([3.0])


def can_frame(data, can_id=CAN_ID_COMMAND, dlc=DLC_COMMAND, sec=0):
    f = Frame(id=can_id, dlc=dlc, is_extended=False, is_rtr=False, is_error=False)
    f.data[:len(data)] = data
    f.header.stamp.sec = sec
    return f


def test_command_decode_matches_the_dbc_example_and_skips_bad_frames():
    zero = [STEER_OFFSET & 0xFF, STEER_OFFSET >> 8]
    frames = [(1.0, can_frame([Gear.FORWARD, 0, 40, 0xE2, 0x07], sec=7)),     # 2018 = 0x07E2
              (1.1, can_frame([1], can_id=CAN_ID_ESTOP, dlc=DLC_ESTOP)),
              (1.2, can_frame([Gear.FORWARD, 0, 40, *zero], dlc=DLC_COMMAND - 1)),
              (1.3, can_frame([7, 0, 0, *zero])),                              # no such gear
              (1.4, can_frame([Gear.NEUTRAL, CAN_CMD_MAX, 0, *zero])),
              (1.5, can_frame([Gear.REVERSE, 0, 0, 0, 0]))]
    cmds = rm.decode_commands(frames)
    assert [t for t, _, _ in cmds] == [1.0, 1.4, 1.5]
    (_, stamp, fwd), (_, _, stop), (_, _, rev) = cmds
    assert stamp == 7.0 and (fwd.gear, fwd.brake, fwd.pedal) == (Gear.FORWARD, 0, 40)
    assert fwd.steering_angle_deg == pytest.approx(5.18)
    assert stop.brake == CAN_CMD_MAX and stop.steering_angle_deg == pytest.approx(0.0)
    assert rev.steering_angle_deg == pytest.approx(-STEER_OFFSET / STEER_FACTOR)


def test_steer_rates_skip_frames_with_the_same_stamp():
    t = np.array([0.0, 0.05, 0.10, 0.15, 0.15, 0.20])
    steer = np.array([0.0, 0.5, 0.5, 1.0, 0.0, 0.0])
    t_r, rate = rm.steer_rates(t, steer)
    assert t_r == pytest.approx([0.05, 0.10, 0.15, 0.20])
    assert rate == pytest.approx([10.0, 0.0, 10.0, 0.0])


def test_telemetry_parsing():
    text = ('STATUS:    NAVIGATING\nTarget V:  1.50 m/s\nCurr V:    1.40 m/s\nDist End:  --- m\n'
            'CTE:       0.10 m')
    f = rm.parse_telemetry(text)
    assert f['STATUS'] == 'NAVIGATING'
    assert rm.first_number(f['Target V']) == 1.5
    assert rm.first_number(f['Dist End']) is None
    assert rm.first_number(None) is None


def test_nav_window_and_sequence():
    status = [(1.0, 'NAVIGATING'), (1.1, 'NAVIGATING'), (9.0, 'IDLE')]
    assert rm.nav_window(status) == (1.0, 9.0, 'IDLE')
    assert rm.nav_window(status[:2]) == (1.0, None, None)
    assert rm.nav_window([]) == (None, None, None)
    assert rm.status_sequence(s for _, s in status) == ['NAVIGATING', 'IDLE']


# ---------------------------------------------------------------- path shape

def l_turn(radius=10.0):
    """40 m east, a right turn of `radius` m, 40 m south."""
    a = np.linspace(0.0, math.pi / 2, 200)          # dense: no kinks at the 0.25 m resampling
    arc = np.c_[40.0 + radius * np.sin(a), -radius + radius * np.cos(a)]
    south = np.c_[np.full(17, 40.0 + radius), np.linspace(-radius, -radius - 40.0, 17)]
    return np.vstack([straight(40.0), arc[1:], south[1:]])


def test_radius_and_turn_window_of_a_right_turn():
    rs, srs = rm.resample(l_turn(10.0))
    r3 = rm.radius3(rs)
    assert np.nanmin(r3) == pytest.approx(10.0, rel=0.02)
    wins = rm.turn_windows(rs, srs)
    assert len(wins) == 1
    s0, s1, dh = wins[0]
    assert dh == pytest.approx(-90.0, abs=3.0)
    arc_s0, arc_s1 = 40.0, 40.0 + math.pi * 10.0 / 2
    assert s0 < arc_s0 - 5.0 and s1 > arc_s1 + 5.0     # padded around the arc


def test_radius_is_inf_on_a_straight():
    rs, _ = rm.resample(straight())
    assert np.all(np.isinf(rm.radius3(rs)))


# ---------------------------------------------------------------- whole runs

def synthetic_run(stall=False):
    """Build a 60 m straight route at 2 m/s, 0.1 m left of the path, in sim time."""
    path = straight(60.0)
    xs = np.arange(0.0, 59.31, 0.1)
    od = odom_along(xs, y=0.1)
    status = [(t, 'NAVIGATING') for t in np.arange(0.0, od[-1, 0], 0.1)]
    if stall:
        # stops 40 m in and waits, still NAVIGATING
        od = od[od[:, 1] <= 40.0]
        t_end = od[-1, 0]
        still = np.c_[t_end + np.arange(1, 101) * 0.05, np.full(100, 40.0), np.full(100, 0.1),
                      np.zeros(100), np.zeros(100)]
        od = np.vstack([od, still])
        status = [(t, 'NAVIGATING') for t in np.arange(0.0, od[-1, 0], 0.1)]
    else:
        # arrival 0.7 m before the end at 2 m/s, then 1 m/s² to rest 2 m past the end
        t_arr = od[-1, 0]
        tt = np.arange(0.05, 2.0 + 1e-9, 0.05)
        vv = 2.0 - tt
        xx = 59.3 + 2.0 * tt - 0.5 * tt ** 2
        stop = np.c_[t_arr + tt, xx, np.full_like(tt, 0.1), np.zeros_like(tt), vv]
        rest = np.c_[t_arr + 2.0 + np.arange(1, 41) * 0.05, np.full(40, xx[-1]), np.full(40, 0.1),
                     np.zeros(40), np.zeros(40)]
        od = np.vstack([od, stop, rest])
        status.append((t_arr, 'IDLE'))
    # 20 Hz FORWARD frames, steering steps of 0.5° every 0.1 s (10 Hz updates) -> 10 °/s
    tc = np.arange(0.0, od[-1, 0], 0.05)
    steer = 0.5 * (np.floor(tc / 0.1) % 2)
    can = np.c_[tc, np.full_like(tc, Gear.FORWARD), np.zeros_like(tc),
                np.full_like(tc, CAN_CMD_MAX), steer]
    tel = np.array([(t, 3.5, 0.0) for t, _ in status])
    return rm.Run(path=path, od=od, status=status, tel=tel, can=can, bag='/x/01_synthetic_bag')


def test_completed_run():
    m = rm.compute(synthetic_run())
    assert m['outcome'] == 'completed' and m['completed']
    assert m['route'] == '01'
    assert m['cte_rms'] == pytest.approx(0.1) and m['cte_mean'] == pytest.approx(0.1)
    assert m['heading_err_max_deg'] == pytest.approx(0.0)
    assert m['steer_rate_max_deg_s'] == pytest.approx(10.0)
    assert m['arrival_speed'] == pytest.approx(2.0)
    assert m['arrival_vs_end'] == pytest.approx(-0.7, abs=0.01)
    assert m['rest_vs_end'] == pytest.approx(1.3, abs=0.06)
    assert m['rest_drift'] < 0.01
    assert m['cruise_speed'] == pytest.approx(2.0)
    assert m['path_r_min'] is None                  # straight
    assert rm.table_row(m) == ('| 01 | yes | 0.10 / 0.10 m | 0° | 10 °/s | 2.00 m/s | '
                               f'{m["rest_vs_end"]:+.2f} m |')
    json.dumps(m)                                    # serialisable as metrics.json


def test_stalled_run():
    m = rm.compute(synthetic_run(stall=True))
    assert m['outcome'] == 'stalled' and not m['completed']
    assert m['still_at_end_s'] == pytest.approx(5.0, abs=0.06)
    assert m['arrival_speed'] is None
    assert m['stop_s'] == pytest.approx(40.0, abs=0.1)
    row = rm.table_row(m)
    assert '**no**: stalled 40 m in' in row and row.endswith('| – | – |')


def test_find_bag_accepts_a_run_dir(tmp_path):
    run = tmp_path / '04_right_turn_20261011_120000'
    (run / 'bag').mkdir(parents=True)
    (run / 'bag' / 'metadata.yaml').write_text('')
    (run / 'run.json').write_text('{"route": "04"}')
    assert rm.find_bag(str(run)) == (str(run / 'bag'), str(run))
    assert rm.find_bag(str(run / 'bag')) == (str(run / 'bag'), str(run))
    assert rm.bag_name(str(run / 'bag')) == run.name
    (run / 'bag' / 'metadata.yaml').unlink()
    with pytest.raises(FileNotFoundError, match='reindex'):
        rm.find_bag(str(run))


def test_routes_file_has_the_doc_routes():
    routes = load_routes(ROUTES_YAML)
    assert list(routes) == ['01', '02', '03', '04', '05', '11', '12', '13', '14']
    assert routes['04']['spawn'] == [-12.81, 44.70, -0.0169]
    assert routes['04']['goal'][:2] == [60.00, 12.00]
    assert routes['11']['goal'][:2] == [58.35, -70.04]
    assert routes['13']['name'] == '13_ne_roundabout_from_straight'
    assert routes['14']['spawn'][:2] == [135.3, 116.5]
    for r in routes.values():
        assert r['name'].startswith(r['id'] + '_') and r['timeout_s'] > 0
