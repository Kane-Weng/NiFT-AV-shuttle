"""
Virtual drive-by-wire ECU that turns the shuttle CAN frames into pedal, brake and steer.

File: carla_dbw.py
Author: Kane Weng

Description:
  Virtual drive-by-wire ECU: plays the shuttle's DBW unit in simulation.
  trajectory_controller sends the same CAN frames it sends to the shuttle
  (0x101 command, 0x100 E-stop); this module decodes them with
  nift_common.can_protocol and turns them into pedal / wheel values the
  bridge applies as carla.VehicleControl. No carla or ROS node code in here,
  so the behaviour is unit-tested without a server.

  Modes (evaluated each tick at sim time t)
  ─────────────────────────────────────────
    NO_COMMAND  no 0x101 since power-on      → hold
    ACTIVE      last 0x101 age ≤ cmd_timeout_s:
                  throttle = pedal / 255, brake = brake / 255
                  pedal ignored when brake > 0 (brake priority) or in NEUTRAL
                  reverse = gear REVERSE
                  steer = steer_sign * wheel_deg / wheel_max_steer_deg, where
                    wheel_deg = clip(deg, ±STEER_MAX_DEG) (map_wheelbase 0), or the inner
                    front-wheel angle that turns the ego on the shuttle's circle
                    (inner_wheel_deg(): steering map, map_wheelbase > 0)
    TIMEOUT     last 0x101 older than cmd_timeout_s (or from the future)
                                             → hold, last steer kept
    ESTOP       0x100 with enable received   → hold, latched

  hold = throttle 0, brake 1, hand brake. The E-stop latch survives "Not
  Estopped" frames; only power_cycle() clears it, as on the shuttle (vendor
  .dbc: "Estop can only be disabled by cycling the power").

  Not thread-safe: the bridge calls on_frame() from its CAN subscription and
  control() from its tick loop under one lock. Times are sim seconds.

Usage:
  ecu = VirtualEcu(EcuConfig(cmd_timeout_s=0.5))
  ecu.on_frame(frame, sim_t)    # every can_msgs/Frame from /to_can_bus
  out = ecu.control(sim_t)      # once per tick → carla.VehicleControl
  ecu.status(sim_t)             # dict for /carla/dbw_state and diagnostics
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional

from nift_common.can_protocol import (
    CAN_CMD_MAX,
    CAN_ID_COMMAND,
    CAN_ID_ESTOP,
    CommandFrame,
    decode_command_frame,
    decode_estop_frame,
    Gear,
    STEER_MAX_DEG,
)
from nift_common.vehicle import ShuttleConfig

CAN_FULL_SCALE = float(CAN_CMD_MAX)                 # brake / pedal count at full scale
# m: the vehicle the steering commands are computed for
SHUTTLE_WHEELBASE = ShuttleConfig().wheel_base


def inner_wheel_deg(cmd_deg: float, cmd_wheelbase: float, wheelbase: float, track: float) -> float:
    """
    Return the inner front-wheel angle [deg] that keeps a vehicle on the commanded circle.

    The vehicle of `wheelbase` and front `track` [m] ends up on the same circle (rear-axle
    centre) as a bicycle-model vehicle of `cmd_wheelbase` steered at `cmd_deg`; same sign,
    0 at 0.

    CARLA turns the inner front wheel to steer * max_steer_angle and the outer one less
    (Ackermann), so the rear-axle radius is wheelbase / tan(inner) + track / 2 (vehicle
    survey 2026-10-09: within 0.2 % on four blueprints). Setting that equal to
    cmd_wheelbase / tan(cmd) gives the angle below. Without it the Micra (2.49 m, track
    1.40 m) turned 9.98 m at 15 deg where the shuttle turns 7.77 m.
    """
    t = math.tan(math.radians(abs(cmd_deg)))
    inner = math.degrees(math.atan2(wheelbase * t, cmd_wheelbase - 0.5 * track * t))
    return math.copysign(inner, cmd_deg)


class EcuMode(Enum):
    NO_COMMAND = 'no_command'       # no 0x101 frame since power-on
    ACTIVE = 'active'               # last 0x101 frame is recent enough
    TIMEOUT = 'timeout'             # last 0x101 frame is older than 0.5 s
    ESTOP = 'estop'                 # an E-stop frame was received


@dataclass(frozen=True)
class EcuConfig:
    cmd_timeout_s: float = 0.5          # no 0x101 for this long → hold
    wheel_max_steer_deg: float = STEER_MAX_DEG  # CARLA front-wheel limit that steer = ±1 reaches
    steer_sign: float = -1.0            # CAN + = left (ROS), CARLA + = right
    # ego wheelbase [m] for the steering map; 0 = no map (wheel = command)
    map_wheelbase: float = 0.0
    map_track: float = 0.0              # ego front track [m] for the steering map
    cmd_wheelbase: float = SHUTTLE_WHEELBASE   # wheelbase the commands assume (the shuttle's)


@dataclass(frozen=True)
class EcuOutput:
    throttle: float     # 0 .. 1
    brake: float        # 0 .. 1
    steer: float        # -1 .. 1, CARLA sign
    reverse: bool
    hand_brake: bool
    neutral: bool = False   # gear NEUTRAL (no drive; the shuttle plant must not creep)


class VirtualEcu:

    def __init__(self, config: EcuConfig = EcuConfig()):
        self.cfg = config
        self.power_cycle()

    def power_cycle(self) -> None:
        """Key off / on: clears the E-stop latch, the last command and the counters."""
        self._cmd: Optional[CommandFrame] = None
        self._cmd_t = 0.0
        self._estop = False
        self._mode = EcuMode.NO_COMMAND
        self.counters = {
            'frames': 0,                    # valid 0x101
            'estop_frames': 0,              # valid 0x100
            'bad_frames': 0,                # 0x101 / 0x100 that failed to decode
            'ignored_frames': 0,            # other IDs
            'brake_priority': 0,            # 0x101 with brake and pedal both > 0
            'estop_release_ignored': 0,     # "Not Estopped" while latched
            'timeouts': 0,                  # ACTIVE → TIMEOUT transitions
        }

    def on_frame(self, frame, sim_t: float) -> None:
        """Feed one can_msgs/Frame received at sim time sim_t."""
        if frame.id == CAN_ID_COMMAND:
            try:
                cmd = decode_command_frame(frame)
            except ValueError:
                self.counters['bad_frames'] += 1
                return
            self._cmd, self._cmd_t = cmd, sim_t
            self.counters['frames'] += 1
            if cmd.brake > 0 and cmd.pedal > 0:
                self.counters['brake_priority'] += 1
        elif frame.id == CAN_ID_ESTOP:
            try:
                engage = decode_estop_frame(frame)
            except ValueError:
                self.counters['bad_frames'] += 1
                return
            self.counters['estop_frames'] += 1
            if engage:
                self._estop = True
            elif self._estop:
                self.counters['estop_release_ignored'] += 1
        else:
            self.counters['ignored_frames'] += 1

    def mode(self, sim_t: float) -> EcuMode:
        if self._estop:
            return EcuMode.ESTOP
        if self._cmd is None:
            return EcuMode.NO_COMMAND
        if not 0.0 <= sim_t - self._cmd_t <= self.cfg.cmd_timeout_s:
            return EcuMode.TIMEOUT
        return EcuMode.ACTIVE

    def control(self, sim_t: float) -> EcuOutput:
        """Pedals and wheel for the tick at sim_t. Call once per tick."""
        mode = self.mode(sim_t)
        if mode == EcuMode.TIMEOUT and self._mode == EcuMode.ACTIVE:
            self.counters['timeouts'] += 1
        self._mode = mode

        cmd = self._cmd
        steer = self._steer(cmd.steering_angle_deg) if cmd else 0.0
        reverse = cmd is not None and cmd.gear == Gear.REVERSE
        if mode != EcuMode.ACTIVE:
            return EcuOutput(0.0, 1.0, steer, reverse, True)

        pedal = 0 if cmd.brake > 0 or cmd.gear == Gear.NEUTRAL else cmd.pedal
        return EcuOutput(pedal / CAN_FULL_SCALE, cmd.brake / CAN_FULL_SCALE, steer, reverse, False,
                         neutral=cmd.gear == Gear.NEUTRAL)

    def status(self, sim_t: float) -> dict:
        """JSON-ready snapshot: mode, last command as received, its age, counters."""
        cmd = self._cmd
        return {
            'mode': self.mode(sim_t).value,
            'cmd_age': None if cmd is None else round(sim_t - self._cmd_t, 3),
            'gear': None if cmd is None else cmd.gear.name,
            'brake': None if cmd is None else cmd.brake,
            'pedal': None if cmd is None else cmd.pedal,
            'steer_deg': None if cmd is None else cmd.steering_angle_deg,
            **self.counters,
        }

    def _steer(self, deg: float) -> float:
        deg = min(max(deg, -STEER_MAX_DEG), STEER_MAX_DEG)
        if self.cfg.map_wheelbase > 0.0:
            deg = inner_wheel_deg(
                deg, self.cfg.cmd_wheelbase, self.cfg.map_wheelbase, self.cfg.map_track
            )
        steer = self.cfg.steer_sign * deg / self.cfg.wheel_max_steer_deg
        return min(max(steer, -1.0), 1.0)
