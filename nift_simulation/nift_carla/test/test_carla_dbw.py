import json
import math

from can_msgs.msg import Frame
from nift_carla.carla_dbw import (
    EcuConfig,
    EcuMode,
    EcuOutput,
    inner_wheel_deg,
    SHUTTLE_WHEELBASE,
    VirtualEcu,
)
from nift_common.can_protocol import CAN_ID_COMMAND, CAN_ID_ESTOP, Gear
from nift_common.can_protocol import STEER_MAX_DEG as LOCK
from nift_control.dbw_interface.can_interface import create_command_frame, create_estop_frame
import pytest


def cmd(gear=Gear.FORWARD, brake=0, pedal=0, deg=0.0):
    return create_command_frame(gear, brake, pedal, deg)


def raw(can_id, data):
    # Frames the encoder refuses to build (foreign sender, bad bytes)
    f = Frame(id=can_id, dlc=len(data))
    f.data[:len(data)] = data
    return f


def fed(*frames, t=0.0, config=EcuConfig()):
    ecu = VirtualEcu(config)
    for f in frames:
        ecu.on_frame(f, t)
    return ecu


def assert_hold(out: EcuOutput):
    assert (out.throttle, out.brake, out.hand_brake) == (0.0, 1.0, True)


def test_power_on_holds_until_first_command():
    ecu = VirtualEcu()
    assert ecu.mode(0.0) == EcuMode.NO_COMMAND
    out = ecu.control(0.0)
    assert_hold(out)
    assert (out.steer, out.reverse) == (0.0, False)


def test_forward_pedal():
    out = fed(cmd(Gear.FORWARD, pedal=255)).control(0.05)
    assert out == EcuOutput(1.0, 0.0, 0.0, False, False)


def test_brake_scaling():
    out = fed(cmd(Gear.FORWARD, brake=51)).control(0.05)
    assert (out.throttle, out.brake, out.hand_brake) == (0.0, pytest.approx(0.2), False)


def test_reverse_gear():
    out = fed(cmd(Gear.REVERSE, pedal=100)).control(0.05)
    assert (out.throttle, out.reverse) == (pytest.approx(100 / 255), True)


def test_neutral_ignores_pedal():
    out = fed(cmd(Gear.NEUTRAL, pedal=100)).control(0.05)
    assert (out.throttle, out.brake, out.reverse, out.neutral) == (0.0, 0.0, False, True)
    assert fed(cmd(Gear.FORWARD, pedal=100)).control(0.05).neutral is False


def test_neutral_full_brake_does_not_use_hand_brake():
    # SAFE_STOP / IDLE send NEUTRAL + 255 while still rolling; a CARLA hand brake
    # would lock the rear wheels mid-stop, so only the service brake is applied.
    out = fed(cmd(Gear.NEUTRAL, brake=255)).control(0.05)
    assert (out.throttle, out.brake, out.hand_brake) == (0.0, 1.0, False)


def test_brake_priority_over_pedal():
    val = 1500
    ecu = fed(raw(CAN_ID_COMMAND, [1, 90, 60, val & 0xFF, val >> 8]))
    out = ecu.control(0.05)
    assert (out.throttle, out.brake) == (0.0, pytest.approx(90 / 255))
    assert ecu.counters['brake_priority'] == 1


@pytest.mark.parametrize('deg, steer', [
    (0.0, 0.0),
    (LOCK, -1.0),       # full left on the bus = full left in CARLA (negative)
    (-LOCK, 1.0),
    (-LOCK / 2, 0.5),
])
def test_steer_sign_and_scale(deg, steer):
    assert fed(cmd(deg=deg)).control(0.05).steer == pytest.approx(steer)


def test_steer_saturates_foreign_sender():
    val = 3500   # +20 deg, beyond the bus limit
    out = fed(raw(CAN_ID_COMMAND, [1, 0, 0, val & 0xFF, val >> 8])).control(0.05)
    assert out.steer == -1.0


def test_steer_config():
    def steer(deg, **cfg):
        return fed(cmd(deg=deg), config=EcuConfig(**cfg)).control(0.05).steer

    assert steer(LOCK, steer_sign=1.0) == 1.0
    # Wheel limit below the bus limit: saturate at the wheel
    assert steer(15.0, wheel_max_steer_deg=10.0) == -1.0
    assert steer(5.0, wheel_max_steer_deg=10.0) == -0.5


def test_watchdog_trips_holds_steer_and_recovers():
    ecu = fed(cmd(Gear.REVERSE, pedal=80, deg=-LOCK / 2), t=10.0)
    assert ecu.control(10.5).hand_brake is False            # age == timeout is still fresh
    out = ecu.control(10.51)
    assert ecu.mode(10.51) == EcuMode.TIMEOUT
    assert_hold(out)
    assert (out.steer, out.reverse) == (0.5, True)          # wheel and gear stay where they were
    ecu.control(11.0)
    assert ecu.counters['timeouts'] == 1                    # counted once per trip
    ecu.on_frame(cmd(Gear.FORWARD, pedal=40), 11.0)
    assert ecu.control(11.05).throttle == pytest.approx(40 / 255)
    assert ecu.mode(11.05) == EcuMode.ACTIVE


def test_watchdog_custom_timeout():
    ecu = fed(cmd(pedal=10), config=EcuConfig(cmd_timeout_s=0.2))
    assert ecu.mode(0.2) == EcuMode.ACTIVE
    assert ecu.mode(0.25) == EcuMode.TIMEOUT


def test_command_from_the_future_is_stale():
    # Sim time went backwards (reconnect / world reload): fail safe, not "fresh forever"
    ecu = fed(cmd(pedal=100), t=10.0)
    assert ecu.mode(9.0) == EcuMode.TIMEOUT
    assert_hold(ecu.control(9.0))


def test_estop_latches_until_power_cycle():
    ecu = fed(cmd(pedal=100, deg=3.0))
    ecu.on_frame(create_estop_frame(True), 0.05)
    assert ecu.mode(0.1) == EcuMode.ESTOP
    assert_hold(ecu.control(0.1))

    ecu.on_frame(cmd(pedal=255), 0.1)                        # commands keep coming: ignored
    ecu.on_frame(create_estop_frame(False), 0.1)             # software release: ignored
    assert_hold(ecu.control(0.15))
    assert ecu.mode(0.15) == EcuMode.ESTOP
    assert ecu.counters['estop_release_ignored'] == 1

    ecu.power_cycle()
    assert ecu.mode(0.2) == EcuMode.NO_COMMAND
    assert ecu.counters['estop_frames'] == 0
    ecu.on_frame(cmd(pedal=255), 0.2)
    assert ecu.control(0.25).throttle == 1.0


def test_estop_before_any_command():
    ecu = fed(create_estop_frame(True))
    assert ecu.mode(0.0) == EcuMode.ESTOP
    assert_hold(ecu.control(0.0))


def test_bad_frames_do_not_refresh_command():
    ecu = fed(raw(CAN_ID_COMMAND, [3, 0, 0, 0xDC, 0x05]))   # gear 3 = Err_State
    assert ecu.mode(0.0) == EcuMode.NO_COMMAND
    ecu.on_frame(cmd(pedal=50), 1.0)
    ecu.on_frame(raw(CAN_ID_COMMAND, [1, 0, 0, 0xDC]), 1.4)  # DLC 4
    ecu.on_frame(raw(CAN_ID_ESTOP, []), 1.4)                  # DLC 0
    assert ecu.control(1.45).throttle == pytest.approx(50 / 255)
    assert ecu.mode(1.55) == EcuMode.TIMEOUT                  # watchdog runs from the good frame
    assert ecu.counters['bad_frames'] == 3
    assert ecu.counters['frames'] == 1


def test_other_ids_ignored():
    ecu = fed(raw(0x102, [7]), raw(0x301, [1, 0, 0, 0xDC, 0x05]))
    assert ecu.mode(0.0) == EcuMode.NO_COMMAND
    assert ecu.counters['ignored_frames'] == 2


def test_status_is_json_ready():
    ecu = VirtualEcu()
    s = ecu.status(0.0)
    assert (s['mode'], s['cmd_age'], s['gear']) == ('no_command', None, None)
    ecu.on_frame(cmd(Gear.REVERSE, brake=20, deg=-2.5), 1.0)
    s = json.loads(json.dumps(ecu.status(1.2)))
    assert s['mode'] == 'active'
    assert s['cmd_age'] == pytest.approx(0.2)
    assert (s['gear'], s['brake'], s['pedal'], s['steer_deg']) == ('REVERSE', 20, 0, -2.5)
    assert s['frames'] == 1


# ---- steering map: the ego steered onto the shuttle's turning circle ----

MICRA_WB, MICRA_TRACK = 2.490, 1.40     # vehicle survey 2026-10-09


def rear_axle_radius(inner_deg, wheelbase, track):
    """Return the radius CARLA gives for an inner front-wheel angle (survey: within 0.2 %)."""
    return wheelbase / math.tan(math.radians(inner_deg)) + track / 2.0


@pytest.mark.parametrize('cmd_deg', [1.0, 5.0, 10.0, 15.0])
def test_inner_wheel_angle_puts_the_ego_on_the_shuttle_circle(cmd_deg):
    inner = inner_wheel_deg(cmd_deg, SHUTTLE_WHEELBASE, MICRA_WB, MICRA_TRACK)
    shuttle_r = SHUTTLE_WHEELBASE / math.tan(math.radians(cmd_deg))
    assert rear_axle_radius(inner, MICRA_WB, MICRA_TRACK) == pytest.approx(shuttle_r, rel=1e-9)


def test_inner_wheel_angle_shape():
    assert inner_wheel_deg(0.0, SHUTTLE_WHEELBASE, MICRA_WB, MICRA_TRACK) == 0.0
    assert inner_wheel_deg(-10.0, SHUTTLE_WHEELBASE, MICRA_WB, MICRA_TRACK) == \
        -inner_wheel_deg(10.0, SHUTTLE_WHEELBASE, MICRA_WB, MICRA_TRACK)
    # the survey measured 7.80 m at full lock with the Micra's limit set to 19.40 deg
    assert inner_wheel_deg(15.0, SHUTTLE_WHEELBASE, MICRA_WB, MICRA_TRACK) == pytest.approx(
        19.40, abs=0.01
    )
    # same geometry and no track: the identity
    assert inner_wheel_deg(12.0, 2.0, 2.0, 0.0) == pytest.approx(12.0)


def test_ecu_steers_through_the_map_and_full_lock_is_full_scale():
    limit = inner_wheel_deg(LOCK, SHUTTLE_WHEELBASE, MICRA_WB, MICRA_TRACK)
    config = EcuConfig(wheel_max_steer_deg=limit, map_wheelbase=MICRA_WB, map_track=MICRA_TRACK)
    for deg in (LOCK, -LOCK, LOCK + 5.0):               # beyond full lock: clipped to it first
        steer = fed(cmd(deg=deg), config=config).control(0.05).steer
        # steer_sign -1: CAN left = CARLA negative
        assert steer == pytest.approx(-math.copysign(1.0, deg))
    steer5 = fed(cmd(deg=5.0), config=config).control(0.05).steer
    assert steer5 == pytest.approx(
        -inner_wheel_deg(5.0, SHUTTLE_WHEELBASE, MICRA_WB, MICRA_TRACK) / limit
    )
