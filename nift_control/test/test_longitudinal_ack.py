"""longitudinal_test reads the DBW 0x301 acknowledgement with the shared 0x101 steering decode."""

from types import SimpleNamespace

from can_msgs.msg import Frame
from nift_common.can_protocol import CAN_ID_ACK, CAN_ID_COMMAND, compute_steering, STEER_MAX_DEG
from nift_control.utils.longitudinal_test import LongitudinalTest
import pytest


def _node():
    return SimpleNamespace(steer_ack=None, acks=[], _now=lambda: 1.0)


def _ack(data, can_id=CAN_ID_ACK, dlc=None, **flags):
    f = Frame(id=can_id, dlc=len(data) if dlc is None else dlc, **flags)
    f.data[:len(data)] = data
    return f


@pytest.mark.parametrize('deg', [0.0, 5.18, -7.25, STEER_MAX_DEG, -STEER_MAX_DEG])
def test_ack_steering(deg):
    node = _node()
    LongitudinalTest._on_can(node, _ack([1, 0, 0, *compute_steering(deg)]))
    assert node.steer_ack == pytest.approx(deg, abs=1e-9)
    assert node.acks == [(1.0, node.steer_ack)]


def test_ack_same_as_the_old_hand_decode():
    for val in range(0, 3001, 7):
        node = _node()
        LongitudinalTest._on_can(node, _ack([0, 0, 0, val & 0xFF, val >> 8]))
        assert node.steer_ack == pytest.approx((val - 1500) * 0.01, abs=1e-12)


def test_ack_with_an_unknown_gear_byte_still_counts():
    node = _node()
    LongitudinalTest._on_can(node, _ack([7, 0, 0, *compute_steering(3.0)]))
    assert node.steer_ack == pytest.approx(3.0)


@pytest.mark.parametrize('frame', [
    _ack([1, 0, 0, 0xDC, 0x05], can_id=CAN_ID_COMMAND),    # our own 0x101
    _ack([1, 0, 0, 0xDC]),                                  # DLC 4
])
def test_other_frames_ignored(frame):
    node = _node()
    LongitudinalTest._on_can(node, frame)
    assert node.steer_ack is None and node.acks == []
