#!/usr/bin/env python3
"""
CAN frame encoders and decoders for the NiFT shuttle's drive-by-wire commands.

File: can_interface.py
Author: Kane Weng
Date: Feb 11, 2026

Description:
  Low-level CAN frame builders and the Gear enum for the NiFT shuttle.
  Encodes all vehicle commands into the byte layout defined by the .dbc file
  so that no other module needs to know the wire format.

  Gear
  ────
  IntEnum mapping NEUTRAL / FORWARD / REVERSE to the .dbc ReqGearSelection
  values (0 / 1 / 2).

  compute_steering(steering_angle_deg) → (low_byte, high_byte)
  ─────────────────────────────────────────────────────────────
  Converts a steering angle in degrees [-15, 15] to the two data bytes the
  CAN receiver expects:
    val = steering_angle_deg * 100 + 1500
  Clamped to the hardware limit before encoding. The low byte goes in
  data[3] and the high byte in data[4], as in the .dbc comment's worked
  example (+5.18° → 2018 = 0x07E2 → E2 07). The .dbc signal definition
  itself (31|16@0+, Motorola) would put the high byte first; the worked
  example is what this encoder follows.

  create_command_frame(gear, brake, pedal, steering_angle_deg, ...)
  ─────────────────────────────────────────────────────────────────
  Builds a 5-byte CAN frame (ID 0x101) for normal driving commands.
  Validates all inputs and enforces brake-priority (simultaneous pedal +
  brake is illegal — pedal is zeroed with an ERROR log).
  Steering input must be in DEGREES, not radians.

  create_estop_frame(enable, ...)
  ────────────────────────────────
  Builds a 1-byte CAN frame (ID 0x100) that requests the hardware
  emergency stop. Per the .dbc an engaged E-stop is cleared only by
  power-cycling the shuttle; enable=False sends "Not Estopped" and does
  not release it.

  decode_command_frame(frame) → CommandFrame
  ──────────────────────────────────────────
  Inverse of create_command_frame, for receivers of the frame (the virtual
  ECU in nift_carla). Returns the wire values as sent: no clipping and no
  brake-priority override, that is the receiver's job. Steering comes back
  on the 0.01° grid; the encoder truncates, so decoded ≤ commanded by less
  than 0.01°.

  decode_estop_frame(frame) → bool
  ─────────────────────────────────
  Inverse of create_estop_frame: True = E-stop requested (any non-zero byte).

  Both decoders raise ValueError for a frame that is not the expected
  standard-ID data frame or is too short.

  Wire layout, as the encoders write it
  ─────────────────────────────────────
    0x101, DLC 5: [gear, brake, pedal, steer_lo, steer_hi]
                  steer_lo | steer_hi << 8 = int(deg * 100 + 1500)
    0x100, DLC 1: [enable]

Usage:
  from .can_interface import create_command_frame, create_estop_frame, Gear
  from .can_interface import decode_command_frame, decode_estop_frame
"""

from dataclasses import dataclass
from enum import IntEnum

from builtin_interfaces.msg import Time
from can_msgs.msg import Frame   # from the can_msgs package
import numpy as np
from rclpy.logging import get_logger

# Constants from .dbc file
CAN_ID_ESTOP = 0X100
CAN_ID_COMMAND = 0X101
DLC_ESTOP = 1
DLC_COMMAND = 5
STEER_MAX_DEG = 15
STEER_FACTOR = 100
STEER_OFFSET = 1500


# ENUM for ReqGearSelection
class Gear(IntEnum):
    NEUTRAL = 0
    FORWARD = 1
    REVERSE = 2


logger = get_logger('vehicle_command_util')


def compute_steering(steering_angle: int):
    """Generate the data bytes for steering angle, low byte first (see module docstring)."""
    steering_angle = np.clip(steering_angle, -STEER_MAX_DEG, STEER_MAX_DEG)
    val = int((steering_angle * STEER_FACTOR) + STEER_OFFSET)       # Defined in .dbc file
    high_byte = val >> 8
    low_byte = val & 0xFF
    return (low_byte, high_byte)


def create_estop_frame(enable: bool = True, time_msg: Time = None):
    """
    Create a CAN frame message for Emergency Stop.

    :param enable: Boolean indicating whether to engage the E-Stop.
    :param time_msg: builtin_interfaces/msg/Time object for the header.
    :return: can_msgs/msg/Frame
    """
    msg = Frame(is_rtr=False, is_error=False, is_extended=False)
    msg.id = CAN_ID_ESTOP
    msg.data[0] = int(enable)
    msg.dlc = DLC_ESTOP

    msg.header.frame_id = ''
    if time_msg is not None:
        msg.header.stamp = time_msg

    return msg


def create_command_frame(gear: Gear, brake: int, pedal: int,
                         steering_angle: int, state_label: str = '', time_msg: Time = None):
    """
    Create a CAN frame message with validated vehicle movement commands.

    :param gear: Gear enum (e.g., Gear.NEUTRAL, Gear.FORWARD, Gear.REVERSE).
    :param brake: Brake percentage (integer 0 ~ 255).
    :param pedal: Pedal percentage (integer 0 ~ 255).
    :param steering_angle: Degree of steering angle (integer -15 ~ 15).
    :param state_label: The string label of the current FSM state (used ONLY for logging).
    :param time_msg: builtin_interfaces/msg/Time object for the header.
    :return: can_msgs/msg/Frame
    """
    # Validation
    if gear not in [Gear.NEUTRAL, Gear.FORWARD, Gear.REVERSE]:
        logger.error(
            f"[{state_label}] CRITICAL: Invalid gear command '{gear}'. Defaulting to NEUTRAL.")
        gear = Gear.NEUTRAL

    if not (0 <= brake <= 255):
        logger.warning(f'[{state_label}] Brake command {brake} out of bounds (0-255). Clipping.')
        brake = int(np.clip(brake, 0, 255))

    if not (0 <= pedal <= 255):
        logger.warning(f'[{state_label}] Pedal command {pedal} out of bounds (0-255). Clipping.')
        pedal = int(np.clip(pedal, 0, 255))

    if not (-15 <= steering_angle <= 15):
        logger.warning(f'[{state_label}] Steering angle {steering_angle} '
                       'out of bounds (-15 to 15). Clipping.')
        steering_angle = int(np.clip(steering_angle, -15, 15))

    if pedal > 0 and brake > 0:
        logger.error(f'[{state_label}] Simultaneous pedal ({pedal}) and brake ({brake}) '
                     'requested! Overriding pedal to 0.')
        pedal = 0  # Brake priority

    # CAN frame construction
    msg = Frame(is_rtr=False, is_error=False, is_extended=False)
    msg.id = CAN_ID_COMMAND
    msg.data[0:5] = [gear, brake, pedal, *compute_steering(steering_angle)]
    msg.dlc = DLC_COMMAND

    msg.header.frame_id = ''
    if time_msg is not None:
        msg.header.stamp = time_msg

    return msg


@dataclass(frozen=True)
class CommandFrame:
    """Decoded 0x101 drive command, as the DBW ECU receives it."""

    gear: Gear
    brake: int                  # 0 ~ 255
    pedal: int                  # 0 ~ 255
    steering_angle_deg: float   # 0.01° grid, not clipped


def _check_frame(frame: Frame, can_id: int, min_dlc: int):
    if frame.is_extended or frame.is_rtr or frame.is_error:
        raise ValueError(f'CAN 0x{frame.id:X}: not a standard data frame')
    if frame.id != can_id:
        raise ValueError(f'CAN 0x{frame.id:X}: expected 0x{can_id:X}')
    if frame.dlc < min_dlc:
        raise ValueError(f'CAN 0x{can_id:X}: DLC {frame.dlc} < {min_dlc}')


def decode_steering(low_byte: int, high_byte: int) -> float:
    """Inverse of compute_steering: the two data bytes → steering angle in degrees."""
    val = (int(high_byte) << 8) | int(low_byte)
    return (val - STEER_OFFSET) / STEER_FACTOR


def decode_command_frame(frame: Frame) -> CommandFrame:
    """
    Decode a drive command frame (ID 0x101) built by create_command_frame.

    :param frame: can_msgs/msg/Frame
    :return: CommandFrame with the values on the wire (not clipped, brake priority not re-applied).
    :raises ValueError: wrong ID or frame type, DLC < 5, or an unknown gear byte.
    """
    _check_frame(frame, CAN_ID_COMMAND, DLC_COMMAND)
    data = [int(b) for b in frame.data[:DLC_COMMAND]]
    try:
        gear = Gear(data[0])
    except ValueError:
        raise ValueError(f'CAN 0x{CAN_ID_COMMAND:X}: invalid gear byte {data[0]}') from None
    return CommandFrame(gear, data[1], data[2], decode_steering(data[3], data[4]))


def decode_estop_frame(frame: Frame) -> bool:
    """
    Decode an E-stop frame (ID 0x100) built by create_estop_frame.

    :param frame: can_msgs/msg/Frame
    :return: True if the frame requests the E-stop (any non-zero byte), False for "Not Estopped".
    :raises ValueError: wrong ID or frame type, or DLC < 1.
    """
    _check_frame(frame, CAN_ID_ESTOP, DLC_ESTOP)
    return int(frame.data[0]) != 0
