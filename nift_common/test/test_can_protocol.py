from types import SimpleNamespace

from nift_common.can_protocol import (
    CAN_ID_ACK,
    CAN_ID_COMMAND,
    CAN_ID_ESTOP,
    compute_steering,
    decode_command_frame,
    decode_estop_frame,
    decode_steering,
    Gear,
    STEER_MAX_DEG,
)
import pytest


def frame(can_id, data, dlc=None):
    return SimpleNamespace(id=can_id, dlc=len(data) if dlc is None else dlc,
                           data=list(data) + [0] * (8 - len(data)),
                           is_extended=False, is_rtr=False, is_error=False)


def test_dbc_worked_example():
    assert compute_steering(5.18) == (0xE2, 0x07)       # 2018 = 0x07E2, low byte first
    assert decode_steering(0xE2, 0x07) == pytest.approx(5.18)


def test_full_lock_is_exact():
    assert STEER_MAX_DEG == 15.0
    assert compute_steering(15.0) == (3000 & 0xFF, 3000 >> 8)
    # degrees(radians(15))
    assert compute_steering(14.999999999999998) == (3000 & 0xFF, 3000 >> 8)
    assert compute_steering(-15.0) == (0, 0)
    assert compute_steering(40.0) == compute_steering(15.0)                    # clamped


@pytest.mark.parametrize('deg', [-14.99, -7.456, -0.004, 0.0, 0.004, 3.333, 7.456, 12.345])
def test_round_trip_within_half_a_count(deg):
    assert abs(decode_steering(*compute_steering(deg)) - deg) <= 0.005 + 1e-9


def test_decode_command_and_ack():
    lo, hi = compute_steering(-3.21)
    cmd = decode_command_frame(frame(CAN_ID_COMMAND, [Gear.FORWARD, 40, 0, lo, hi]))
    assert (cmd.gear, cmd.brake, cmd.pedal) == (Gear.FORWARD, 40, 0)
    assert cmd.steering_angle_deg == pytest.approx(-3.21)
    ack = decode_command_frame(frame(CAN_ID_ACK, [Gear.REVERSE, 0, 9, lo, hi]), can_id=CAN_ID_ACK)
    assert ack.gear == Gear.REVERSE and ack.steering_angle_deg == pytest.approx(-3.21)


def test_decode_rejects():
    with pytest.raises(ValueError):
        decode_command_frame(frame(CAN_ID_ESTOP, [1, 0, 0, 0, 0]))
    with pytest.raises(ValueError):
        decode_command_frame(frame(CAN_ID_COMMAND, [1, 0, 0, 0], dlc=4))
    with pytest.raises(ValueError):
        decode_command_frame(frame(CAN_ID_COMMAND, [3, 0, 0, 0, 0]))     # gear 3


def test_estop():
    assert decode_estop_frame(frame(CAN_ID_ESTOP, [1])) is True
    assert decode_estop_frame(frame(CAN_ID_ESTOP, [0])) is False
