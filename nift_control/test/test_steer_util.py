import math

from nift_common.vehicle import ShuttleConfig
from nift_control.utils.steer_util import (
    ack_steps,
    ack_summary,
    parse_seq,
    seq_angle,
    seq_duration,
    turn_after_stop_deg,
    wrap_deg,
)
import pytest

SHUTTLE = ShuttleConfig()


def test_parse_seq():
    assert parse_seq('10:2,-10:2,0:3') == [(10.0, 2.0), (-10.0, 2.0), (0.0, 3.0)]
    lock = SHUTTLE.max_steering_deg
    assert parse_seq(f' {lock:g}:4 ') == [(lock, 4.0)]
    for bad in ('', f'{lock + 1:g}:2', '5', '5:0', 'a:1', '5:-1'):
        with pytest.raises(ValueError):
            parse_seq(bad)


def test_seq_angle():
    seq = [(10.0, 2.0), (-10.0, 2.0), (0.0, 3.0)]
    assert seq_duration(seq) == 7.0
    assert seq_angle(seq, 0.0) == (0, 10.0)
    assert seq_angle(seq, 1.99) == (0, 10.0)
    assert seq_angle(seq, 2.0) == (1, -10.0)
    assert seq_angle(seq, 6.9) == (2, 0.0)
    assert seq_angle(seq, 100.0) == (2, 0.0)        # after the end: the last step


def test_wrap_deg():
    assert wrap_deg(190.0) == pytest.approx(-170.0)
    assert wrap_deg(-190.0) == pytest.approx(170.0)
    assert wrap_deg(180.0) == pytest.approx(180.0)
    assert wrap_deg(-180.0) == pytest.approx(180.0)
    assert wrap_deg(5.0) == pytest.approx(5.0)


def test_turn_after_stop():
    # 1 m/s, full lock: 1.3 m blind + 3.33 m braking at 0.15 m/s² on R = wheel_base / tan(full
    # lock) = 7.77 m
    lock, L = SHUTTLE.max_steering_deg, SHUTTLE.wheel_base
    d = turn_after_stop_deg(1.0, lock, L, 1.3, 0.15)
    assert d == pytest.approx(math.degrees((1.3 + 1.0 / 0.3) / SHUTTLE.min_turn_radius), rel=1e-9)
    assert turn_after_stop_deg(1.0, -lock, L, 1.3, 0.15) == pytest.approx(d)
    assert turn_after_stop_deg(1.0, 0.0, L, 1.3, 0.15) == 0.0
    assert turn_after_stop_deg(0.0, lock, L, 1.3, 0.15) == 0.0


def test_ack_echo_vs_moves():
    events = [(10.0, 15.0, 0.0), (15.0, 0.0, 15.0)]
    # echo: the ack jumps to the command in the frame after it (frames every 0.05 s)
    acks = [(10.0 + 0.05 * i, 0.0 if i < 1 else 15.0) for i in range(100)]
    acks += [(15.0 + 0.05 * i, 0.0 if i >= 1 else 15.0) for i in range(100)]
    acks.sort()
    steps = ack_steps(events, acks)
    assert [s['kind'] for s in steps] == ['echo', 'echo']
    assert steps[0]['start'] == pytest.approx(0.05)
    assert ack_summary(steps) == {'kind': 'echo'}
    # moves: 0.2 s dead time, then 20 deg/s
    acks = []
    for i in range(200):
        t = 10.0 + 0.05 * i
        acks.append((t, min(15.0, max(0.0, 20.0 * (t - 10.2)))))
    steps = ack_steps(events[:1], acks)
    assert steps[0]['kind'] == 'moves'
    assert steps[0]['start'] == pytest.approx(0.25)
    assert steps[0]['rate_dps'] == pytest.approx(20.0, rel=0.15)
    assert ack_summary(steps)['kind'] == 'moves'


def test_ack_none():
    steps = ack_steps([(1.0, 5.0, 0.0)], [])
    assert steps[0]['kind'] == 'none'
    assert ack_summary(steps) == {'kind': 'none'}
