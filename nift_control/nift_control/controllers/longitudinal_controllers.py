"""
Longitudinal (speed) controllers: PID and ADRC.

File: longitudinal_controllers.py
Author: Kane Weng
Date: Mar 14, 2026

Description:
  Concrete longitudinal (speed) controllers for the NiFT shuttle.
  All classes subclass LongitudinalController and return a scalar control
  effort u ∈ [-255, 255], where positive = throttle and negative = brake.

  PIDController
  ─────────────
  Classic discrete-time PID with integral anti-windup (clipped to ±50).

  ADRCController
  ──────────────
  Active Disturbance Rejection Controller built around a discrete-time
  Linear Extended State Observer (LESO). The LESO jointly estimates vehicle
  speed and the "total disturbance" (friction, slope, wind) in one state
  vector, allowing the control law to cancel disturbances feed-forward
  rather than waiting for them to appear as tracking error.

  Tuning knobs:
    b0    — control gain (how powerfully throttle affects speed)
    w_cl  — closed-loop bandwidth [rad/s]; higher = faster settling.
            The gain is Kp = 2 * w_cl, twice the textbook value (see __init__)
    k_eso — observer-to-controller bandwidth ratio; higher = observer
            responds faster, but amplifies sensor noise

  The controller assumes a fixed sampling time (self.delta) matching the
  actuation loop period. The optional dt argument to compute() exists for
  API compatibility but is not used internally.

Usage:
  from .longitudinal_controllers import ADRCController
"""

from nift_common.can_protocol import (
    CAN_CMD_MAX,  # |u| limit: the CAN brake / pedal full scale (0-255)
)
import numpy as np

from .control_base import LongitudinalController


class PIDController(LongitudinalController):

    def __init__(self, kp=40.0, ki=0.5, kd=0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.first_run = True

    def compute(self, current_speed: float, desired_speed: float, dt: float) -> float:
        """
        Compute the PID control effort.

        Parameters
        ----------
        current_speed : float
            The current measured speed of the system.
        desired_speed : float
            The target speed the system aims to reach.
        dt : float
            The time interval since the last update (delta time).

        Returns
        -------
        float
            The calculated control signal to be applied to the system.

        """
        error = desired_speed - current_speed
        if self.first_run:
            self.previous_error = error
            self.first_run = False

        self.integral_error = np.clip(self.integral_error + (error * dt), -50, 50)
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        u = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative)
        u_lim = np.clip(u, -CAN_CMD_MAX, CAN_CMD_MAX)

        return u_lim

    def reset(self):
        self.integral_error = 0.0
        self.first_run = True


class ADRCController(LongitudinalController):

    def __init__(self, delta=0.05, b0=1, w_cl=2.0, k_eso=5.0):
        """
        Initialize the Active Disturbance Rejection Controller (ADRC).

        This implementation uses a discrete-time Linear Extended State Observer (LESO)
        to estimate system states and "Total Disturbance" (f) in real-time.

        Parameters
        ----------
        delta : float
            The fixed sampling time in seconds (e.g., 0.05 for 20Hz).
        b0 : float
            Critical gain. How powerfully the throttle actually affects the speed.
        w_cl : float
            Bandwidth. How aggressively the controller chases the target.
        k_eso : float
            Observer ratio. How much faster the Observer thinks compared to the controller.

        """
        self.delta = delta  # 1/20Hz = 0.05
        self.b0 = b0
        self.u_prev = 0.0

        # --- 1. TUNING THE CONTROLLER ---
        # Convert the desired bandwidth (w_cl) into a Proportional Gain (Kp).
        # sCL = -w_cl, and Kp = -2 * sCL = 2 * w_cl. That is twice the textbook first-order
        # gain (Herbst 2013 takes Kp = -sCL = w_cl). With the disturbance cancelled, the loop
        # is first order with its pole at -Kp, so it settles (2 %) in about 4 / Kp = 2 / w_cl
        # seconds, not 4 / w_cl. The shuttle's own delays make the real response slower.
        # The gains in adrc_params.yaml were tuned on the shuttle with this factor, so it stays.
        t_settle = 4 / w_cl
        sCL = -4 / t_settle
        self.Kp = -2 * sCL

        # --- 2. TUNING THE OBSERVER ---
        # The observer needs to be faster than the controller so it can detect
        # disturbances (like a hill) before the controller tries to react.
        # Its poles sit at k_eso * sCL = -k_eso * w_cl, which is k_eso / 2 times the loop pole -Kp.
        sESO = k_eso * sCL
        zESO = np.exp(sESO * delta)  # Moving from continuous time to discrete steps

        # self.L is the "Correction factor". It tells the observer how much to change its guess
        # when its prediction doesn't match the actual sensor reading.
        self.L = np.array([
            1 - (zESO)**2,                 # How fast to correct the speed estimate
            (1 / delta) * (1 - zESO)**2    # How fast to correct the disturbance estimate
        ]).reshape(-1, 1)

        # --- 3. THE INTERNAL PHYSICS ENGINE ---
        # These matrices represent the ideal, frictionless physics of the shuttle.
        self.Ad = np.array([[1, delta], [0, 1]])   # State transition (Speed + Disturbance)
        self.Bd = np.array([[b0 * delta], [0]])    # How the throttle input affects the state
        self.Cd = np.array([[1, 0]])               # We only have a sensor for state 1 (Speed)

        # Pre-compute the Observer's "Update Matrix".
        # This combines the ideal physics (Ad, Bd) with the correction factor (L).
        self.oA = self.Ad - self.L @ self.Cd @ self.Ad
        self.oB = self.Bd - self.L @ self.Cd @ self.Bd

        # --- 4. MEMORY ---
        # self.xhat is the observer's notebook:
        # xhat[0] = Estimated Speed
        # xhat[1] = Estimated Ghost Force (Total Disturbance)
        self.xhat = np.zeros((2, 1))

    def compute(self, y, r, dt=None):
        """
        Calculate the next control action.

        y: Current measured speed
        r: Target speed
        dt: Actual time since last call (Optional: ADRC usually assumes fixed self.delta)
        """
        # --- STEP 1: THE OBSERVER UPDATES ITS NOTEBOOK ---
        # xhat[k] = oA*xhat[k-1] + oB*u[k-1] + L*y[k]
        # New guess = (Internal Physics * Old Guess) + (Physics * Last Throttle)
        #             + (Correction * Actual Sensor)
        self.xhat = self.oA @ self.xhat + self.oB * self.u_prev + self.L * y
        est_speed = self.xhat[0, 0]
        est_disturbance = self.xhat[1, 0]

        # --- STEP 2: THE CONTROL LAW ---
        # u = (Proportional Control) - (Disturbance Cancellation)
        # 1. Kp * (r - est_speed): Standard P-control acting on the *clean* estimated speed,
        #    not the noisy sensor.
        # 2. - est_disturbance: We perfectly subtract the unknown friction/hill forces.
        # 3. / b0: We scale the whole thing by how powerful our motor is.
        u = (self.Kp * (r - est_speed) - est_disturbance) / self.b0

        u_lim = np.clip(u, -CAN_CMD_MAX, CAN_CMD_MAX)
        self.u_prev = u_lim
        return u_lim

    def reset(self):
        self.xhat = np.zeros((2, 1))
        self.u_prev = 0.0
