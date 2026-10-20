"""
Unit tests of the shuttle's longitudinal plant model (shuttle_plant.py).

The fitted behaviour of the garage test 2026-10-08 is reproduced by the model that the
CARLA bridge drives the ego with.
"""
from nift_carla.shuttle_plant import PlantConfig, ShuttlePlant
from nift_common.vehicle import ShuttleConfig
import pytest

DT = 0.05
CFG = PlantConfig()


def run(plant, segments, t0=0.0, v0=0.0):
    """
    Drive the plant through [(secs, pedal, brake, neutral, reverse)].

    The ideal loop: v_meas = the previous v_set. Returns [(t, v_set)] with t = sim time of
    the tick that set it.
    """
    out, t, v = [], t0, v0
    for secs, pedal, brake, neutral, reverse in segments:
        for _ in range(int(round(secs / DT))):
            v_new = plant.step(t, DT, v, pedal, brake, neutral, reverse, True)
            out.append((t, v_new))
            v = 0.0 if v_new is None else v_new
            t += DT
    return out


def hold(secs=1.0):
    return (secs, 0, 255, True, False)         # NEUTRAL + full brake, as the controller's IDLE


def forward(secs, pedal=0, brake=0):
    return (secs, pedal, brake, False, False)


def v_at(trace, t):
    return next(v for tt, v in trace if tt >= t - 1e-9)


def dist(trace):
    return sum(abs(v) * DT for _, v in trace)


# ---------------------------------------------------------------- config maps
@pytest.mark.parametrize('pedal, v', [
    (0, 0.46),          # creep
    (40, 0.635),
    (55, 0.7325),       # linear between the measured points
    (140, 1.275),
    (240, 1.875),       # extrapolated with the last segment's slope
])
def test_pedal_map(pedal, v):
    assert CFG.steady_speed(pedal) == pytest.approx(v)


def test_pedal_map_governor():
    assert CFG.steady_speed(10000) == CFG.max_speed


@pytest.mark.parametrize('brake, speed, a', [
    (80, 0.0, 0.72),
    (160, 0.0, 0.72),       # no count dependence above brake_full
    (40, 0.0, 0.72),
    (20, 0.0, 0.36),        # proportional below it
    (80, 1.0, 0.57),        # mildly weaker at speed
    (80, 5.0, 0.30),        # floor
    (0, 1.0, 0.0),
])
def test_brake_map(brake, speed, a):
    assert CFG.brake_decel(brake, speed) == pytest.approx(a)


# ---------------------------------------------------------------- engagement
def test_engagement_delay_from_standstill():
    plant = ShuttlePlant()
    tr = run(plant, [hold(), forward(8.0, pedal=40)])
    t_rel = 1.0                                        # the release command
    assert all(v == 0.0 for t, v in tr if t < t_rel + CFG.t_eng - 1e-6)
    assert v_at(tr, t_rel + CFG.t_eng + 0.15) > 0.05
    assert plant.status()['engaged'] is True
    assert v_at(tr, t_rel + CFG.t_eng + 5.0) == pytest.approx(0.635, abs=0.01)


def test_engagement_does_not_depend_on_pedal():
    t140 = run(ShuttlePlant(), [hold(), forward(8.0, pedal=140)])
    t0 = run(ShuttlePlant(), [hold(), forward(8.0, pedal=0)])
    first = lambda tr: next(t for t, v in tr if v > 0.05)  # noqa: E731
    assert first(t140) == pytest.approx(first(t0), abs=DT)
    # pedal 0 with the brake off = creep
    assert v_at(t0, 7.0) == pytest.approx(0.46, abs=0.01)


def test_stop_under_brake_disengages_again():
    plant = ShuttlePlant()
    tr = run(plant, [hold(), forward(6.0, pedal=40), forward(5.0, brake=80)])
    assert tr[-1][1] == 0.0 and plant.status()['plant'] == 'hold'
    tr2 = run(plant, [forward(4.0, pedal=40)], t0=tr[-1][0] + DT)
    t_rel = tr2[0][0]
    assert all(v == 0.0 for t, v in tr2 if t < t_rel + CFG.t_eng - 1e-6)
    assert v_at(tr2, t_rel + CFG.t_eng + 0.3) > 0.05


# ---------------------------------------------------------------- motor loop
def test_step_is_rate_limited_then_levels():
    tr = run(ShuttlePlant(), [hold(), forward(9.0, pedal=140)])
    slopes = [(v1 - v0) / DT for (_, v0), (_, v1) in zip(tr, tr[1:])]
    assert max(slopes) == pytest.approx(CFG.a_lim, abs=1e-6)
    # the ramp runs at the limit until the lag takes over near the plateau
    t_eng = 1.0 + CFG.t_eng
    assert v_at(tr, t_eng + 1.0) == pytest.approx(CFG.a_lim * 1.0, abs=0.05)
    assert v_at(tr, t_eng + 5.0) == pytest.approx(1.275, abs=0.01)
    assert v_at(tr, 9.9) == pytest.approx(1.275, abs=0.001)          # no drift on the plateau


def test_pedal_off_falls_at_the_rate_limit_to_the_creep():
    plant = ShuttlePlant()
    tr = run(plant, [hold(), forward(8.0, pedal=100)])
    v_plateau = tr[-1][1]
    assert v_plateau == pytest.approx(1.035, abs=0.01)
    t_off = tr[-1][0] + DT
    tr2 = run(plant, [forward(5.0, pedal=0)], t0=t_off, v0=v_plateau)
    assert all(
        v == pytest.approx(v_plateau, abs=1e-3) for t, v in tr2 if t < t_off + CFG.cmd_delay - 1e-6
    )  # dead time
    slopes = [(v1 - v0) / DT for (_, v0), (_, v1) in zip(tr2, tr2[1:])]
    assert min(slopes) == pytest.approx(-CFG.a_lim, abs=1e-6)
    assert tr2[-1][1] == pytest.approx(0.46, abs=0.01)
    assert plant.status()['plant'] == 'creep'


def test_neutral_runs_down_to_zero_without_creep():
    plant = ShuttlePlant()
    tr = run(plant, [hold(), forward(8.0, pedal=70)])
    tr2 = run(plant, [(6.0, 0, 0, True, False)], t0=tr[-1][0] + DT, v0=tr[-1][1])
    assert tr2[-1][1] == 0.0
    assert plant.status()['plant'] == 'neutral'


def test_reverse():
    tr = run(ShuttlePlant(), [hold(), (8.0, 40, 0, False, True)])
    assert tr[-1][1] == pytest.approx(-0.635, abs=0.01)


def test_governor():
    cfg = PlantConfig(pedal_map=((0.0, 0.46), (255.0, 9.0)))
    tr = run(ShuttlePlant(cfg), [hold(), forward(20.0, pedal=255)])
    assert max(v for _, v in tr) == pytest.approx(ShuttleConfig().max_speed)


# ---------------------------------------------------------------- brake
def test_brake_dead_time_and_stop_distance():
    plant = ShuttlePlant()
    tr = run(plant, [hold(), forward(8.0, pedal=70)])
    v0 = tr[-1][1]
    assert v0 == pytest.approx(0.83, abs=0.01)
    t_b = tr[-1][0] + DT
    tr2 = run(plant, [forward(5.0, brake=80)], t0=t_b, v0=v0)
    assert all(v == pytest.approx(v0, abs=1e-3) for t, v in tr2 if t < t_b + CFG.cmd_delay - 1e-6)
    t_stop = next(t for t, v in tr2 if v == 0.0) - t_b
    # garage P3 / P3 repeat (brake 80 from 0.84): stop 1.9-2.0 s after the command, 0.78-0.97 m by
    # position
    assert 1.6 < t_stop < 2.3
    assert 0.7 < dist(tr2) < 1.1
    assert plant.status()['plant'] == 'hold'


def test_brake_count_does_not_matter_above_40():
    def stop_time(brake):
        plant = ShuttlePlant()
        tr = run(plant, [hold(), forward(8.0, pedal=100), forward(6.0, brake=brake)])
        return next(t for t, v in tr if t > 9.0 and v == 0.0)
    assert stop_time(40) == stop_time(160)
    assert stop_time(20) > stop_time(40) + 0.5


def test_brake_from_speed_is_weaker():
    # mean decel of a stop: from 0.46 m/s ~0.65-0.7, from 1.27 m/s ~0.6 (pooled planned brakes of
    # both bags)
    def mean_decel(pedal):
        plant = ShuttlePlant()
        tr = run(plant, [hold(), forward(8.0, pedal=pedal)])
        v0, t_b = tr[-1][1], tr[-1][0] + DT
        tr2 = run(plant, [forward(6.0, brake=120)], t0=t_b, v0=v0)
        t_stop = next(t for t, v in tr2 if v == 0.0) - t_b
        return v0, v0 / (t_stop - CFG.cmd_delay)
    v_hi, a_hi = mean_decel(140)
    v_lo, a_lo = mean_decel(0)
    assert (v_hi, v_lo) == (pytest.approx(1.275, abs=0.01), pytest.approx(0.46, abs=0.01))
    assert a_hi == pytest.approx(0.63, abs=0.06)
    # the 0.05 m/s snap at the end shortens the short stop
    assert a_lo == pytest.approx(0.75, abs=0.08)
    assert a_lo > a_hi


# ---------------------------------------------------------------- ECU modes / time
def test_inactive_is_physics_hold_and_resets_engagement():
    plant = ShuttlePlant()
    run(plant, [hold(), forward(6.0, pedal=70)])
    assert plant.step(7.0, DT, 0.83, 0, 255, True, False, False) is None     # watchdog / E-stop
    assert plant.status() == {'plant': 'off', 'v_cmd': 0.0, 'v_set': None, 'engaged': False,
                              'eff_pedal': 0, 'eff_brake': 255, 'eff_gear': 'NEUTRAL'}
    tr = run(plant, [forward(3.0, pedal=70)], t0=7.05)
    assert all(v == 0.0 for t, v in tr if t < 7.05 + CFG.t_eng - 1e-6)         # engages anew


def test_rolling_with_brake_off_is_engaged():
    # e.g. a teleport while moving, or power-on while rolling: no engagement wait while the wheels
    # turn
    plant = ShuttlePlant()
    assert plant.step(0.0, DT, 0.0, 70, 0, False, False, True) == 0.0            # at rest: waits
    plant.reset()
    for i in range(12):
        v = plant.step(i * DT, DT, 0.5, 70, 0, False, False, True)
    assert v > 0.5 and plant.status()['engaged'] is True


def test_one_tick_physics_losses_do_not_accumulate():
    # CARLA's drag takes a little off the velocity that was set, every tick: the model must not see
    # it
    plant, ideal = ShuttlePlant(), ShuttlePlant()
    t, v, v_ideal = 0.0, 0.0, 0.0
    for _ in range(20):
        plant.step(t, DT, v, 0, 255, True, False, True)
        ideal.step(t, DT, v_ideal, 0, 255, True, False, True)
        t += DT
    for _ in range(160):
        v = plant.step(t, DT, v - 0.012 if v > 0.012 else 0.0, 100, 0, False, False, True)
        v_ideal = ideal.step(t, DT, v_ideal, 100, 0, False, False, True)
        t += DT
    assert v == pytest.approx(v_ideal) and v == pytest.approx(1.035, abs=0.01)


def test_blocked_by_an_obstacle_resyncs():
    plant = ShuttlePlant()
    run(plant, [hold(), forward(6.0, pedal=100)])
    t = 7.0
    # a wall: the measured speed stays 0 while the pedal is on
    for _ in range(40):
        v = plant.step(t, DT, 0.0, 100, 0, False, False, True)
        t += DT
    # it keeps trying from ~0, not from the plateau
    assert 0.0 < v <= CFG.v_sync_tol + CFG.a_lim * DT + 1e-9
    # the wall is gone, the car moves again
    v = plant.step(t, DT, 0.6, 100, 0, False, False, True)
    assert v == pytest.approx(0.6 + CFG.a_lim * DT)                 # from 0.6, at the rate limit


def test_time_jump_back_resets():
    plant = ShuttlePlant()
    run(plant, [hold(), forward(6.0, pedal=70)])
    assert plant.step(2.0, DT, 0.83, 70, 0, False, False, True) == 0.0
    assert plant.status()['engaged'] is False


def test_status_is_json_ready():
    import json
    plant = ShuttlePlant()
    run(plant, [hold(), forward(3.0, pedal=40)])
    s = json.loads(json.dumps(plant.status()))
    assert s['plant'] == 'drive' and s['eff_gear'] == 'FORWARD' and s['eff_pedal'] == 40
    assert s['v_cmd'] == pytest.approx(0.635)
