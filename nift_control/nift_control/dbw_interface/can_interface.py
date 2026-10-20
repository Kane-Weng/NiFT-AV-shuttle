#!/usr/bin/env python3
"""
CAN frame builders for the NiFT shuttle's drive-by-wire commands.

File: can_interface.py
Author: Kane Weng
Date: Feb 11, 2026

Description:
  ROS CAN frame builders for the NiFT shuttle's drive-by-wire. The wire format (IDs,
  byte layout, steering encoding, Gear, the decoders) lives in nift_common.can_protocol;
  this module re-exports it, so
  `from nift_control.dbw_interface.can_interface import X` keeps working.

  create_command_frame(gear, brake, pedal, steering_angle_deg, ...)
    5-byte 0x101 frame for normal driving. Validates the inputs (out-of-range values are
    clipped with a WARN) and enforces brake priority (pedal + brake together is illegal:
    the pedal is zeroed with an ERROR log). Steering is in DEGREES, not radians; it is
    rounded to the 0.01° grid.

  create_estop_frame(enable, ...)
    1-byte 0x100 frame that requests the hardware E-stop. Per the .dbc an engaged E-stop
    is cleared only by power-cycling the shuttle; enable=False sends "Not Estopped" and
    does not release it.

Usage:
  from .can_interface import create_command_frame, create_estop_frame, Gear
  from .can_interface import decode_command_frame, decode_estop_frame
"""

from builtin_interfaces.msg import Time
from can_msgs.msg import Frame  # from the can_msgs package
from nift_common.can_protocol import (  # noqa: F401  (re-exported)
    CAN_CMD_MAX,
    CAN_ID_ACK,
    CAN_ID_COMMAND,
    CAN_ID_ESTOP,
    CommandFrame,
    compute_steering,
    decode_command_frame,
    decode_estop_frame,
    decode_steering,
    DLC_COMMAND,
    DLC_ESTOP,
    Gear,
    STEER_FACTOR,
    STEER_MAX_DEG,
    STEER_OFFSET,
)
import numpy as np
from rclpy.logging import get_logger

# float slack on the ±STEER_MAX_DEG check: degrees(radians(15)) is 14.999999999999998
STEER_TOL_DEG = 1e-6

logger = get_logger('vehicle_command_util')


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
                         steering_angle: float, state_label: str = '', time_msg: Time = None):
    """
    Create a CAN frame message with validated vehicle movement commands.

    :param gear: Gear enum (e.g., Gear.NEUTRAL, Gear.FORWARD, Gear.REVERSE).
    :param brake: Brake count (integer 0 ~ CAN_CMD_MAX).
    :param pedal: Pedal count (integer 0 ~ CAN_CMD_MAX).
    :param steering_angle: Steering angle [deg], ±STEER_MAX_DEG.
    :param state_label: The string label of the current FSM state (used ONLY for logging).
    :param time_msg: builtin_interfaces/msg/Time object for the header.
    :return: can_msgs/msg/Frame
    """
    # Validation
    if gear not in [Gear.NEUTRAL, Gear.FORWARD, Gear.REVERSE]:
        logger.error(
            f"[{state_label}] CRITICAL: Invalid gear command '{gear}'. Defaulting to NEUTRAL.")
        gear = Gear.NEUTRAL

    if not (0 <= brake <= CAN_CMD_MAX):
        logger.warning(
            f'[{state_label}] Brake command {brake} out of bounds (0-{CAN_CMD_MAX}). Clipping.')
        brake = int(np.clip(brake, 0, CAN_CMD_MAX))

    if not (0 <= pedal <= CAN_CMD_MAX):
        logger.warning(
            f'[{state_label}] Pedal command {pedal} out of bounds (0-{CAN_CMD_MAX}). Clipping.')
        pedal = int(np.clip(pedal, 0, CAN_CMD_MAX))

    if not (-STEER_MAX_DEG - STEER_TOL_DEG <= steering_angle <= STEER_MAX_DEG + STEER_TOL_DEG):
        logger.warning(f'[{state_label}] Steering angle {steering_angle} out of bounds '
                       f'(±{STEER_MAX_DEG:g}). Clipping.')
        steering_angle = float(np.clip(steering_angle, -STEER_MAX_DEG, STEER_MAX_DEG))

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
