import math
import random

from can_msgs.msg import Frame
from nift_common.vehicle import ShuttleConfig
from nift_control.dbw_interface import can_interface
from nift_control.dbw_interface.can_interface import (
    CAN_CMD_MAX,
    CAN_ID_COMMAND,
    CAN_ID_ESTOP,
    CommandFrame,
    create_command_frame,
    create_estop_frame,
    decode_command_frame,
    decode_estop_frame,
    Gear,
    STEER_FACTOR,
    STEER_MAX_DEG,
)
import pytest

STEER_LSB = 0.01   # deg


def _frame(can_id, data, dlc=None, **flags):
    f = Frame(id=can_id, dlc=len(data) if dlc is None else dlc, **flags)
    f.data[:len(data)] = data
    return f


def _steer_round_trip(deg):
    return decode_command_frame(create_command_frame(Gear.FORWARD, 0, 0, deg)).steering_angle_deg


# Golden bytes: they pin the wire format so an encoder + decoder change in lockstep cannot
# pass unnoticed. The encoder rounds to the 0.01° grid.
@pytest.mark.parametrize('deg, lo, hi', [
    (0.0, 0xDC, 0x05),      # 1500
    (15.0, 0xB8, 0x0B),     # 3000
    (-15.0, 0x00, 0x00),    # 0
    (5.18, 0xE2, 0x07),     # 2018, the .dbc's worked example
    (5.678, 0x14, 0x08),    # 2068 (2067.8)
    (-5.678, 0xA4, 0x03),   # 932 (932.2)
    (12.345, 0xAE, 0x0A),   # 2734 (2734.5: half to even)
    (math.degrees(ShuttleConfig().max_steering_angle), 0xB8, 0x0B),   # 14.999999999999998 -> 3000
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
    (0, 0), (1, 0), (50, 0), (CAN_CMD_MAX - 1, 0), (CAN_CMD_MAX, 0),
    (0, 1), (0, 128), (0, CAN_CMD_MAX - 1), (0, CAN_CMD_MAX),
])
def test_command_round_trip_gear_brake_pedal(gear, brake, pedal):
    got = decode_command_frame(create_command_frame(gear, brake, pedal, 0.0))
    assert got == CommandFrame(gear, brake, pedal, 0.0)
    assert isinstance(got.gear, Gear)


def test_steer_round_trip_on_grid():
    # compute_steering rounds, so every grid point comes back exactly (with int() truncation,
    # e.g. -10.22 * 100 = -1022.0000000000001 lost one LSB)
    n = round(STEER_MAX_DEG * STEER_FACTOR)
    for k in range(-n, n + 1):
        deg = k / STEER_FACTOR
        assert _steer_round_trip(deg) == deg, deg


def test_steer_round_trip_random():
    rng = random.Random(0)
    for _ in range(5000):
        deg = rng.uniform(-STEER_MAX_DEG, STEER_MAX_DEG)
        got = _steer_round_trip(deg)
        assert abs(got - deg) <= STEER_LSB / 2 + 1e-9, (deg, got)


def test_steer_rounds_to_nearest():
    assert _steer_round_trip(0.004) == 0.0
    assert _steer_round_trip(-0.004) == 0.0
    assert _steer_round_trip(0.006) == 0.01
    assert _steer_round_trip(-0.006) == -0.01


def test_steer_saturates_in_encoder():
    assert _steer_round_trip(20.0) == STEER_MAX_DEG
    assert _steer_round_trip(-20.0) == -STEER_MAX_DEG


class _Log:

    def __init__(self):
        self.warnings = []

    def warning(self, text):
        self.warnings.append(text)

    def error(self, text):
        raise AssertionError(text)


def test_full_lock_in_radians_is_in_bounds(monkeypatch):
    # The controller clips to ±max_steering_angle [rad]; its degrees must pass the ±STEER_MAX_DEG
    # check without a warning (0.2618 rad = 15.000035° used to warn on every full-lock frame).
    log = _Log()
    monkeypatch.setattr(can_interface, 'logger', log)
    full = math.degrees(ShuttleConfig().max_steering_angle)
    for deg in (full, -full):
        assert abs(_steer_round_trip(deg)) == STEER_MAX_DEG
    assert log.warnings == []


def test_steer_out_of_bounds_warns_and_clips(monkeypatch):
    log = _Log()
    monkeypatch.setattr(can_interface, 'logger', log)
    assert _steer_round_trip(STEER_MAX_DEG + 0.5) == STEER_MAX_DEG
    assert _steer_round_trip(-STEER_MAX_DEG - 0.5) == -STEER_MAX_DEG
    assert len(log.warnings) == 2


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
