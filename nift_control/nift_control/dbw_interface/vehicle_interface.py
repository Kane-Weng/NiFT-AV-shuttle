"""
Bridge between the speed controller output and the shuttle's gear, brake and throttle.

File: vehicle_interface.py
Author: Kane Weng
Date: Apr 16, 2026

Description:
  Finite State Machine (FSM) bridge between the ADRC scalar control effort
  and the physical gear / brake / throttle signals sent over CAN.

  VehicleMode FSM
  ───────────────
  States: STANDBY → FORWARD / REVERSE → BRAKING_TO_REVERSE / BRAKING_TO_FORWARD

  The FSM prevents illegal hardware transitions (e.g. shifting into reverse
  while still rolling forward) by requiring the vehicle to fully stop before
  changing direction. Transitions:
    STANDBY           → FORWARD / REVERSE when target speed crosses threshold
    FORWARD           → BRAKING_TO_REVERSE when target speed goes negative
    FORWARD           → STANDBY when target speed ≈ 0 and vehicle is stopped
    BRAKING_TO_REVERSE→ REVERSE once stopped (or back to FORWARD if planner reverses)
    (mirror logic for REVERSE / BRAKING_TO_FORWARD)

  NiftVehicleBridge.map_signals(u, target_speed, current_speed)
  ─────────────────────────────────────────────────────────────
  Inputs:
    u             — ADRC output ∈ [-255, 255]. Positive = accelerate in current
                    direction; negative = decelerate / brake.
    target_speed  — Planner's desired speed [m/s] (used for state transitions).
    current_speed — Measured speed [m/s] (used to detect "stopped").
  Outputs:
    (gear, brake, throttle) where brake and throttle ∈ [0, 255] and are
    mutually exclusive (CAN rejects simultaneous pedal + brake).

  During BRAKING_* states the bridge enforces a minimum brake of 50 to
  ensure the vehicle decelerates even when u is small.

Usage:
  from .vehicle_interface import NiftVehicleBridge, VehicleMode
"""
from enum import Enum

import numpy as np

from .can_interface import Gear


class VehicleMode(Enum):
    STANDBY = 0
    FORWARD = 1
    REVERSE = 2
    BRAKING_TO_REVERSE = 3
    BRAKING_TO_FORWARD = 4


# Constants
CAN_CMD_LIMIT = 255           # Maximum CAN bus actuator command (0-255)


class NiftVehicleBridge():

    def __init__(self, zero_speed_tolerance=0.1):
        self.state = VehicleMode.STANDBY
        # m/s (speed at which we consider the vehicle "stopped")
        self.zero_speed_tolerance = zero_speed_tolerance

    def map_signals(self, u, target_speed, current_speed):
        """
        Map the ADRC control effort (u) to physical gear, brake, and throttle.

        A finite state machine guards the gear changes.
        """
        # ==========================================
        # 1. STATE MACHINE TRANSITIONS
        # ==========================================
        is_stopped = abs(current_speed) <= self.zero_speed_tolerance

        if self.state == VehicleMode.STANDBY:
            if target_speed > self.zero_speed_tolerance:
                self.state = VehicleMode.FORWARD
            elif target_speed < -self.zero_speed_tolerance:
                self.state = VehicleMode.REVERSE

        elif self.state == VehicleMode.FORWARD:
            if target_speed < -self.zero_speed_tolerance:
                self.state = VehicleMode.BRAKING_TO_REVERSE
            elif abs(target_speed) <= self.zero_speed_tolerance and is_stopped:
                self.state = VehicleMode.STANDBY

        elif self.state == VehicleMode.REVERSE:
            if target_speed > self.zero_speed_tolerance:
                self.state = VehicleMode.BRAKING_TO_FORWARD
            elif abs(target_speed) <= self.zero_speed_tolerance and is_stopped:
                self.state = VehicleMode.STANDBY

        elif self.state == VehicleMode.BRAKING_TO_REVERSE:
            # 1. PRIORITY: Did the planner change its mind back to forward?
            if target_speed > self.zero_speed_tolerance:
                self.state = VehicleMode.FORWARD

            # 2. If intent hasn't changed, check if we have successfully stopped
            elif is_stopped:
                if abs(target_speed) <= self.zero_speed_tolerance:
                    self.state = VehicleMode.STANDBY  # Planner decided to just park it
                else:
                    self.state = VehicleMode.REVERSE  # Safe to shift into reverse

        elif self.state == VehicleMode.BRAKING_TO_FORWARD:
            # 1. PRIORITY: Did the planner change its mind back to reverse?
            if target_speed < -self.zero_speed_tolerance:
                self.state = VehicleMode.REVERSE

            # 2. If intent hasn't changed, check if we have successfully stopped
            elif is_stopped:
                if abs(target_speed) <= self.zero_speed_tolerance:
                    self.state = VehicleMode.STANDBY  # Planner decided to just park it
                else:
                    self.state = VehicleMode.FORWARD  # Safe to shift into forward

        # ==========================================
        # 2. MAP 'u' TO HARDWARE BASED ON STATE
        # ==========================================
        throttle = 0
        brake = 0
        gear = Gear.NEUTRAL
        actuation_mag = int(np.clip(abs(u), 0, CAN_CMD_LIMIT))

        if self.state in [VehicleMode.FORWARD, VehicleMode.BRAKING_TO_REVERSE]:
            gear = Gear.FORWARD
            if u > 0 and self.state == VehicleMode.FORWARD:
                throttle = actuation_mag
            else:
                brake = actuation_mag
                if self.state == VehicleMode.BRAKING_TO_REVERSE:
                    brake = max(brake, 50)

        elif self.state in [VehicleMode.REVERSE, VehicleMode.BRAKING_TO_FORWARD]:
            gear = Gear.REVERSE
            if u < 0 and self.state == VehicleMode.REVERSE:
                throttle = actuation_mag
            else:
                brake = actuation_mag
                if self.state == VehicleMode.BRAKING_TO_FORWARD:
                    brake = max(brake, 50)

        elif self.state == VehicleMode.STANDBY:
            gear = Gear.NEUTRAL
            brake = max(actuation_mag, 50)

        return gear, brake, throttle

    def reset(self):
        self.state = VehicleMode.STANDBY
