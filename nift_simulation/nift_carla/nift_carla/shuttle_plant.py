"""
Longitudinal plant of the NiFT shuttle: pedal and brake counts to speed.

File: shuttle_plant.py
Author: Kane Weng

Description:
  Longitudinal plant of the NiFT shuttle for the CARLA bridge: turns the virtual
  ECU's pedal / brake / gear into the speed the shuttle would have, so the CAN
  frames produce the shuttle's speed response in simulation instead of the
  Micra's torque response. Fitted to the garage test of 2026-10-08
  (longitudinal_test bag longtest_20261008_172401):

    pedal      a SPEED command. Steady speed = pedal_map(count): 0.46 m/s at
               pedal 0 (creep), 0.64 at 40, 0.83 at 70, 1.04 at 100, 1.28 at
               140, extrapolated above (~0.006 m/s per count). The motor loop
               follows it as a rate-limited first-order lag,
                   dv/dt = clip((v_cmd - v) / tau, -a_lim, +a_lim),
               tau 0.6 s, a_lim 0.6 m/s^2, in both directions: a pedal-off is
               not a coast, the speed falls at a_lim and settles at the creep.
    brake      a deceleration: independent of the count from 40 up to 255 (it
               saturates early) and only mildly weaker at speed, a = a0 + k_v * v
               (0.72 - 0.15 v, floor a_min: 0.65 at 0.45 m/s, 0.57 at 1.0, 0.44 at
               1.86 — pooled from both bags of 2026-10-08: stops from 0.45 m/s
               averaged 0.65, from 1.27 m/s 0.47, brake 255 from 1.06 m/s
               0.57-0.62, brake 80 from 1.81 m/s 0.59), scaled by count /
               brake_full below 40 (the shuttle's brake_full_count; not
               measured). The ECU already zeroes the pedal while braking (brake
               priority).
    dead time  cmd_delay 0.45 s from any 0x101 change to the speed (pedal and
               brake alike).
    engagement from standstill the drive engages t_eng 1.7 s after the brake
               release, whatever the pedal; stopped under the brake it
               disengages again. (Measured from the command, so the plant waits
               t_eng - cmd_delay after the delayed release.)
    NEUTRAL    no drive: the motor loop runs toward 0 (assumed, not measured).
    governor   max_speed, the shuttle's (nift_common vehicle.yaml, 11 mph).
  Not modelled (see the fit): the load droop (+18 % steady speed downhill on
  the pad), the 0.012 m/s^2 upward drift of the plateau, and the stronger
  braking of two abort stops (1.5 m/s^2 at count 150).

  The plant integrates its own speed and returns the speed for the next tick,
  which the bridge sets with set_target_velocity. The measured longitudinal
  speed is only used to re-sync when the two disagree by more than v_sync_tol
  (the ego is held back by an obstacle or a collision, was teleported, or the
  physics brake had it): the one-tick losses of CARLA's own drag on the
  velocity it was set must not accumulate (a 0.03 m/s nudge from rest is gone
  within a tick). Pure Python, unit-tested without a server
  (test/test_shuttle_plant.py).

Usage:
  plant = ShuttlePlant(PlantConfig())
  # m/s, or None = physics hold
  v = plant.step(sim_t, dt, v_measured, pedal, brake, neutral, reverse, active)
  plant.status()   # dict for /carla/dbw_state
"""

from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

from nift_common.vehicle import ShuttleConfig

SHUTTLE = ShuttleConfig()

PEDAL_MAP_2026_10_08 = ((0.0, 0.46), (40.0, 0.635), (70.0, 0.83), (100.0, 1.035), (140.0, 1.275))


@dataclass(frozen=True)
class PlantConfig:
    # (pedal count, steady m/s), linear between
    pedal_map: Tuple[Tuple[float, float], ...] = PEDAL_MAP_2026_10_08
    max_speed: float = SHUTTLE.max_speed    # governor
    a_lim: float = 0.6              # motor loop rate limit [m/s^2]
    tau: float = 0.6                # motor loop lag [s]
    # brake release -> drive engaged, from standstill [s, from the command]
    t_eng: float = 1.7
    cmd_delay: float = 0.45         # command -> speed dead time [s]
    brake_a0: float = 0.72          # brake decel at v = 0 [m/s^2] (count >= brake_full)
    brake_k_v: float = -0.15        # d(decel)/dv [1/s]
    brake_a_min: float = 0.3        # decel floor at speed [m/s^2]
    # brake decel all there from this count; proportional below
    brake_full: float = float(SHUTTLE.brake_full_count)
    v_stop: float = 0.05            # below this the plant snaps to 0 and the brake holds
    # measured speed this far from the model's -> the model takes the measurement
    v_sync_tol: float = 0.2

    def steady_speed(self, pedal: float) -> float:
        """
        Return the steady speed the motor loop holds for a pedal count.

        Piecewise linear, extrapolated above the map.
        """
        pts = self.pedal_map

        # Below map: creep speed
        if pedal <= pts[0][0]:
            return pts[0][1]

        # Inside the map: ordinary linear interpolation
        for (p0, v0), (p1, v1) in zip(pts, pts[1:]):
            if pedal <= p1:
                return v0 + (v1 - v0) * (pedal - p0) / (p1 - p0)

        # Above the map: extend last segment
        (p0, v0), (p1, v1) = pts[-2], pts[-1]
        return min(v1 + (v1 - v0) * (pedal - p1) / (p1 - p0), self.max_speed)

    def brake_decel(self, brake: float, speed: float) -> float:
        """Deceleration [m/s^2] for a brake count at a (positive) speed."""
        if brake <= 0.0:
            return 0.0

        # Speed-dependent strength (braking is weaker at higher speed)
        a = max(self.brake_a0 + self.brake_k_v * speed, self.brake_a_min)
        return a * min(brake / self.brake_full, 1.0)


class ShuttlePlant:

    def __init__(self, config: PlantConfig = PlantConfig()):
        self.cfg = config
        self.reset()

    def reset(self) -> None:
        """
        Forget command history and engagement (key off / teleport).

        Same as the ECU's power_cycle().
        """
        # (t, pedal, brake, neutral, reverse) not yet effective
        self._q = deque()
        self._eff = (0.0, 255.0, True, False)   # effective command: pedal, brake, neutral, reverse
        self._engaged = False
        self._t_release: Optional[float] = None
        self._t_last: Optional[float] = None
        self._v = 0.0                           # the model's speed (signed)
        self.v_cmd = 0.0
        self.v_set: Optional[float] = None
        self.state = 'off'

    # ------------------------------------------------------------------ #
    def step(self, t: float, dt: float, v_measured: float, pedal: float, brake: float,
             neutral: bool, reverse: bool, active: bool) -> Optional[float]:
        """
        Run one tick at sim time t and return the speed to set for the next tick.

        Inputs are the ECU's command for this tick (pedal / brake counts, gear flags, ACTIVE
        or not) and the measured longitudinal speed. Returns None when the ECU is not ACTIVE
        (power-on, watchdog, E-stop: the physics hold applies, and the drive disengages).
        """
        # sim time went backwards (reconnect / world reload)
        if self._t_last is not None and t < self._t_last:
            self.reset()

        self._t_last = t
        v_measured = float(v_measured)

        # Not active: returns early and resets the plant's working state
        if not active:
            self._q.clear()
            self._eff = (0.0, 255.0, True, False)
            self._engaged, self._t_release = False, None
            self._v = v_measured                    # the physics has the car
            self.v_cmd, self.v_set, self.state = 0.0, None, 'off'
            return None

        # Re-sync: model taking v_measured if it differs from the model's by more than tol
        if abs(v_measured - self._v) > self.cfg.v_sync_tol:
            # held back / pushed by something the model does not know
            self._v = v_measured

        # Dead time: a command becomes effective cmd_delay after it was received
        self._q.append((t, float(pedal), float(brake), bool(neutral), bool(reverse)))
        while self._q and self._q[0][0] <= t - self.cfg.cmd_delay + 1e-9:
            self._eff = self._q.popleft()[1:]
        p_eff, b_eff, neutral_eff, reverse_eff = self._eff

        v = self._v
        speed = abs(v)
        stopped = speed < self.cfg.v_stop

        # Braking and stopped: drive disengaged
        if b_eff > 0.0 and stopped:
            self._engaged, self._t_release = False, None

        # Brake off, not yet engaged
        elif b_eff <= 0.0 and not self._engaged:
            # Rolling
            if not stopped:
                self._engaged = True
            # Start the release timer
            elif self._t_release is None:
                self._t_release = t
            # Wait till t_eng
            elif t - self._t_release >= self.cfg.t_eng - self.cfg.cmd_delay - 1e-9:
                self._engaged = True

        sign = -1.0 if reverse_eff else 1.0
        # Not engaged
        if not self._engaged:
            self.v_cmd, self.state = 0.0, ('hold' if b_eff > 0.0 else 'engaging')
            v_next = 0.0
        # Braking
        elif b_eff > 0.0:
            self.v_cmd, self.state = 0.0, 'brake'
            new_speed = speed - self.cfg.brake_decel(b_eff, speed) * dt
            v_next = 0.0 if new_speed < self.cfg.v_stop else (v / speed) * new_speed
        # Drive, creep, or neutral
        else:
            if neutral_eff:
                self.v_cmd, self.state = 0.0, 'neutral'
            else:
                self.v_cmd = sign * self.cfg.steady_speed(p_eff)
                self.state = 'creep' if p_eff <= 0.0 else 'drive'
            dv = max(-self.cfg.a_lim, min(self.cfg.a_lim, (self.v_cmd - v) / self.cfg.tau))
            v_next = v + dv * dt
            if neutral_eff and abs(v_next) < self.cfg.v_stop:
                v_next = 0.0
        v_next = max(-self.cfg.max_speed, min(self.cfg.max_speed, v_next))
        self._v = self.v_set = v_next
        return v_next

    def status(self) -> dict:
        """JSON-ready: what the plant is doing, for /carla/dbw_state."""
        p_eff, b_eff, neutral_eff, reverse_eff = self._eff
        return {
            'plant': self.state,
            'v_cmd': round(self.v_cmd, 3),
            'v_set': None if self.v_set is None else round(self.v_set, 3),
            'engaged': self._engaged,
            'eff_pedal': int(p_eff),
            'eff_brake': int(b_eff),
            'eff_gear': 'NEUTRAL' if neutral_eff else ('REVERSE' if reverse_eff else 'FORWARD'),
        }
