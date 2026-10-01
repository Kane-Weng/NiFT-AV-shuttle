import random

from can_msgs.msg import Frame
from nift_control.dbw_interface.can_interface import (
    CAN_ID_COMMAND,
    CAN_ID_ESTOP,
    CommandFrame,
    create_command_frame,
    create_estop_frame,
    decode_command_frame,
    decode_estop_frame,
    Gear,
)
import pytest

STEER_LSB = 0.01   # deg


def _frame(can_id, data, dlc=None, **flags):
    f = Frame(id=can_id, dlc=len(data) if dlc is None else dlc, **flags)
    f.data[:len(data)] = data
    return f


def _steer_round_trip(deg):
    return decode_command_frame(create_command_frame(Gear.FORWARD, 0, 0, deg)).steering_angle_deg


# Bytes captured from create_command_frame before the decoders were added; they pin
# the wire format so an encoder + decoder change in lockstep cannot pass unnoticed.
@pytest.mark.parametrize('deg, lo, hi', [
    (0.0, 0xDC, 0x05),      # 1500
    (15.0, 0xB8, 0x0B),     # 3000
    (-15.0, 0x00, 0x00),    # 0
    (5.678, 0x13, 0x08),    # 2067
    (-5.678, 0xA4, 0x03),   # 932
    (12.345, 0xAE, 0x0A),   # 2734
])
def test_command_frame_golden_bytes(deg, lo, hi):
    f = create_command_frame(Gear.FORWARD, 0, 10, deg)
    assert (f.id, f.dlc) == (CAN_ID_COMMAND, 5)
    assert list(f.data) == [1, 0, 10, lo, hi, 0, 0, 0]


def test_estop_frame_golden_bytes():
    on, off = create_estop_frame(True), create_estop_frame(False)
    assert (on.id, on.dlc, list(on.data)) == (CAN_ID_ESTOP, 1, [1, 0, 0, 0, 0, 0, 0, 0])
    assert (off.id, off.dlc, list(off.data)) == (CAN_ID_ESTOP, 1, [0] * 8)


@pytest.mark.parametrize('gear', list(Gear))
@pytest.mark.parametrize('brake, pedal', [
    (0, 0), (1, 0), (50, 0), (254, 0), (255, 0),
    (0, 1), (0, 128), (0, 254), (0, 255),
])
def test_command_round_trip_gear_brake_pedal(gear, brake, pedal):
    got = decode_command_frame(create_command_frame(gear, brake, pedal, 0.0))
    assert got == CommandFrame(gear, brake, pedal, 0.0)
    assert isinstance(got.gear, Gear)


def test_steer_round_trip_on_grid():
    # int() in compute_steering truncates, so the decoded angle is never above the
    # commanded one and at most one LSB below it. It is exact on most grid points;
    # float representation (e.g. -10.22 * 100 = -1022.0000000000001) costs one LSB on some.
    exact = 0
    for k in range(-1500, 1501):
        deg = k / 100
        got = _steer_round_trip(deg)
        assert 0.0 <= deg - got <= STEER_LSB + 1e-9, (deg, got)
        exact += got == deg
    assert exact > 2900


def test_steer_round_trip_random():
    rng = random.Random(0)
    for _ in range(5000):
        deg = rng.uniform(-15.0, 15.0)
        got = _steer_round_trip(deg)
        assert 0.0 <= deg - got <= STEER_LSB + 1e-9, (deg, got)


def test_steer_truncates_toward_negative():
    assert _steer_round_trip(0.004) == 0.0
    assert _steer_round_trip(-0.004) == -0.01


def test_steer_saturates_in_encoder():
    assert _steer_round_trip(20.0) == 15.0
    assert _steer_round_trip(-20.0) == -15.0


def test_brake_priority_survives_round_trip():
    got = decode_command_frame(create_command_frame(Gear.REVERSE, 80, 120, 0.0))
    assert (got.gear, got.brake, got.pedal) == (Gear.REVERSE, 80, 0)


def test_decoder_reports_wire_values_unfiltered():
    # A foreign sender may break the encoder's guarantees; the decoder reports what
    # is on the wire and leaves clipping and brake priority to the receiver.
    val = 3500   # +20 deg
    got = decode_command_frame(_frame(CAN_ID_COMMAND, [1, 90, 60, val & 0xFF, val >> 8]))
    assert got == CommandFrame(Gear.FORWARD, 90, 60, 20.0)


def test_decoder_accepts_longer_dlc():
    got = decode_command_frame(_frame(CAN_ID_COMMAND, [2, 0, 7, 0xDC, 0x05, 0, 0, 0]))
    assert got == CommandFrame(Gear.REVERSE, 0, 7, 0.0)


@pytest.mark.parametrize('frame', [
    _frame(CAN_ID_ESTOP, [1, 0, 0, 0xDC, 0x05]),                        # wrong id
    _frame(CAN_ID_COMMAND, [1, 0, 0, 0xDC]),                            # DLC 4
    _frame(CAN_ID_COMMAND, [1, 0, 0, 0xDC, 0x05], is_extended=True),    # 29-bit 0x101
    _frame(CAN_ID_COMMAND, [1, 0, 0, 0xDC, 0x05], is_rtr=True),
    _frame(CAN_ID_COMMAND, [1, 0, 0, 0xDC, 0x05], is_error=True),
    _frame(CAN_ID_COMMAND, [3, 0, 0, 0xDC, 0x05]),                      # unknown gear
])
def test_decode_command_rejects_malformed(frame):
    with pytest.raises(ValueError):
        decode_command_frame(frame)


def test_estop_round_trip():
    assert decode_estop_frame(create_estop_frame(True)) is True
    assert decode_estop_frame(create_estop_frame(False)) is False
    assert decode_estop_frame(_frame(CAN_ID_ESTOP, [0xFF])) is True


@pytest.mark.parametrize('frame', [
    _frame(CAN_ID_COMMAND, [1]),                    # wrong id
    _frame(CAN_ID_ESTOP, [1], dlc=0),
    _frame(CAN_ID_ESTOP, [1], is_extended=True),
])
def test_decode_estop_rejects_malformed(frame):
    with pytest.raises(ValueError):
        decode_estop_frame(frame)
