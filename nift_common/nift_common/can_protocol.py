"""
The shuttle's drive-by-wire CAN protocol, with pure encode and decode functions.

File: can_protocol.py

Description:
  The shuttle's drive-by-wire CAN protocol (the vendor .dbc): IDs, byte layout, steering
  encoding, gear codes, and the pure encode / decode functions. ROS-free: a frame is anything
  with id, dlc, data, is_extended, is_rtr and is_error (can_msgs/Frame). nift_control's
  can_interface builds the ROS frames on top of it.

  Wire layout:
    0x101, DLC 5: [gear, brake, pedal, steer_lo, steer_hi]
                  steer_lo | steer_hi << 8 = round(deg * 100 + 1500). Low byte first, as in
                  the .dbc comment's worked example (+5.18° -> 2018 = 0x07E2 -> E2 07); the
                  .dbc signal definition itself (31|16@0+, Motorola) would put the high byte
                  first.
    0x100, DLC 1: [enable]
    0x301:        the DBW's acknowledgement of 0x101 (DBW_Shuttle_Command_Ack), the same layout

Usage:
  from nift_common.can_protocol import CAN_ID_COMMAND, Gear, compute_steering, decode_command_frame
"""

from dataclasses import dataclass
from enum import IntEnum

from .vehicle import ShuttleConfig

CAN_ID_ESTOP = 0x100
CAN_ID_COMMAND = 0x101
CAN_ID_ACK = 0x301
DLC_ESTOP = 1
DLC_COMMAND = 5
CAN_CMD_MAX = 255                                   # brake and pedal: 0 ~ 255
STEER_MAX_DEG = ShuttleConfig().max_steering_deg    # the shuttle's full lock
STEER_FACTOR = 100
STEER_OFFSET = 1500


class Gear(IntEnum):
    """ReqGearSelection."""

    NEUTRAL = 0
    FORWARD = 1
    REVERSE = 2


@dataclass(frozen=True)
class CommandFrame:
    """Decoded 0x101 drive command, as the DBW ECU receives it."""

    gear: Gear
    brake: int                  # 0 ~ 255
    pedal: int                  # 0 ~ 255
    steering_angle_deg: float   # 0.01° grid, not clipped


def compute_steering(steering_angle_deg: float):
    """
    Steering angle [deg] -> (low_byte, high_byte).

    Clamped to ±STEER_MAX_DEG, rounded to the 0.01° grid.
    """
    deg = min(max(float(steering_angle_deg), -STEER_MAX_DEG), STEER_MAX_DEG)
    val = int(round(deg * STEER_FACTOR + STEER_OFFSET))
    return val & 0xFF, val >> 8


def decode_steering(low_byte: int, high_byte: int) -> float:
    """Inverse of compute_steering: the two data bytes -> steering angle [deg]."""
    val = (int(high_byte) << 8) | int(low_byte)
    return (val - STEER_OFFSET) / STEER_FACTOR


def _check_frame(frame, can_id: int, min_dlc: int):
    if frame.is_extended or frame.is_rtr or frame.is_error:
        raise ValueError(f'CAN 0x{frame.id:X}: not a standard data frame')
    if frame.id != can_id:
        raise ValueError(f'CAN 0x{frame.id:X}: expected 0x{can_id:X}')
    if frame.dlc < min_dlc:
        raise ValueError(f'CAN 0x{can_id:X}: DLC {frame.dlc} < {min_dlc}')


def decode_command_frame(frame, can_id: int = CAN_ID_COMMAND) -> CommandFrame:
    """
    Decode a drive command frame (0x101, or its acknowledgement 0x301 with can_id=CAN_ID_ACK).

    Returns the values on the wire: not clipped, brake priority not re-applied (the receiver's
    job). The encoder rounds, so the steering is within 0.005° of the commanded angle.
    :raises ValueError: wrong ID or frame type, DLC < 5, or an unknown gear byte.
    """
    _check_frame(frame, can_id, DLC_COMMAND)
    data = [int(b) for b in frame.data[:DLC_COMMAND]]
    try:
        gear = Gear(data[0])
    except ValueError:
        raise ValueError(f'CAN 0x{can_id:X}: invalid gear byte {data[0]}') from None
    return CommandFrame(gear, data[1], data[2], decode_steering(data[3], data[4]))


def decode_estop_frame(frame) -> bool:
    """
    Decode an E-stop frame (0x100): True if it requests the E-stop (any non-zero byte).

    :raises ValueError: wrong ID or frame type, or DLC < 1.
    """
    _check_frame(frame, CAN_ID_ESTOP, DLC_ESTOP)
    return int(frame.data[0]) != 0
