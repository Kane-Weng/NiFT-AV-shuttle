#!/usr/bin/env python3
"""
Interactive Tkinter tuner for ADRC speed control and manual CAN commands.

File: longitudinal_tuner.py
Author: Kane Weng
Date: Apr 15, 2026

Description:
  Developer tool for testing and tuning the NiFT longitudinal control stack
  in isolation (no path planner). Contains the ADRC controller, CAN output,
  a Tkinter dashboard, and a live speed graph — all in one self-contained node.

  The GUI feeds speed/steer commands directly into the controller in-process.
  Live ADRC re-tuning (b0, w_cl, k_eso) takes effect immediately by
  reinitializing the controller object in-place.

  A watchdog silences CAN output if the GUI has been idle for > 0.5 s,
  allowing the vehicle to be taken back to manual mode safely.

  Started by `longitudinal_test tune`, which passes the ADRC gains: trajectory_controller's
  adrc_params.yaml values, overridden by --b0 / --w_cl / --k_eso. None of longitudinal_test's
  limits or guards apply here.

Usage:
  ros2 run nift_control longitudinal_test tune [--b0 0.5 --w_cl 2.0 --k_eso 5.0]
"""

from collections import deque
import math
from tkinter import ttk
import tkinter as tk

from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
from nift_common.can_protocol import CAN_CMD_MAX, STEER_MAX_DEG
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from ..controllers import ADRCController
from ..dbw_interface import (
    create_command_frame,
    create_estop_frame,
    Gear,
    NiftVehicleBridge,
    VehicleMode,
)

CONTROL_HZ = 20
WATCHDOG_TIMEOUT = 0.5
PLOT_MAX_PTS = 200
STEER_LIM = int(STEER_MAX_DEG)   # [deg] steering controls: whole degrees within full lock


class LongitudinalTuner(Node):

    def __init__(self, gains: dict, zero_speed_tol: float):
        super().__init__('longitudinal_tuner')
        self._gains = gains     # initial values of the tuning panel

        # ------------------------------------------------------------------ #
        #  Controllers & bridge                                                #
        # ------------------------------------------------------------------ #
        self.controller = ADRCController(
            delta=1.0 / CONTROL_HZ, b0=gains['b0'], w_cl=gains['w_cl'], k_eso=gains['k_eso']
        )
        self.bridge = NiftVehicleBridge(zero_speed_tolerance=zero_speed_tol)

        # ------------------------------------------------------------------ #
        #  State                                                               #
        # ------------------------------------------------------------------ #
        self.current_speed = 0.0
        self.desired_speed = 0.0
        self.desired_steer = 0.0
        self._is_active = False
        self._last_cmd_time = self.get_clock().now()
        self._last_loop_time = self.get_clock().now()
        self._plot_start = self.get_clock().now()

        self._origin_lat: float | None = None
        self._origin_lon: float | None = None

        # ------------------------------------------------------------------ #
        #  Pubs / Subs                                                         #
        # ------------------------------------------------------------------ #
        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)

        self.create_subscription(TwistStamped, '/gnss/vel', self._on_vel, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gnss/fix', self._on_fix, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/gnss/heading_deg', self._on_heading, qos_profile_sensor_data
        )

        # ------------------------------------------------------------------ #
        #  Timers                                                              #
        # ------------------------------------------------------------------ #
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)
        self.create_timer(0.1, self._update_plot)

        # ------------------------------------------------------------------ #
        #  GUI + graph                                                         #
        # ------------------------------------------------------------------ #
        self._build_gui()
        self._build_graph()

        self.get_logger().info('Longitudinal Tuner ready.')

    # ====================================================================== #
    #  ROS Callbacks                                                           #
    # ====================================================================== #

    def _on_vel(self, msg: TwistStamped):
        speed = msg.twist.linear.x
        self.current_speed = (
            -abs(speed) if self.bridge.state == VehicleMode.REVERSE else abs(speed)
        )
        self._speed_str.set(f"{speed:.2f}")
        self._draw_speed_needle(speed)

    def _on_fix(self, msg: NavSatFix):
        if self._origin_lat is None:
            self._origin_lat, self._origin_lon = msg.latitude, msg.longitude
        y = (msg.latitude - self._origin_lat) * 111111.0
        x = (
            (msg.longitude - self._origin_lon)
            * 111111.0
            * math.cos(math.radians(self._origin_lat))
        )
        self._lat_str.set(f"{msg.latitude:.6f}")
        self._lon_str.set(f"{msg.longitude:.6f}")
        self._local_x_str.set(f"{x:.2f}")
        self._local_y_str.set(f"{y:.2f}")

    def _on_heading(self, msg: Float64):
        self._heading_str.set(f"{msg.data:.1f}")
        self._draw_heading_needle(msg.data)

    # ====================================================================== #
    #  Control loop — 20 Hz                                                   #
    # ====================================================================== #

    def _control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_loop_time).nanoseconds / 1e9
        self._last_loop_time = now

        dt_watchdog = (now - self._last_cmd_time).nanoseconds / 1e9
        if dt_watchdog > WATCHDOG_TIMEOUT:
            if self._is_active:
                self.get_logger().warn('Watchdog timeout — silencing CAN output')
                cmd = create_command_frame(Gear.NEUTRAL, CAN_CMD_MAX, 0, self.desired_steer,
                                           state_label='watchdog', time_msg=now.to_msg())
                self.can_pub.publish(cmd)
                self._is_active = False
                self.controller.reset()
                self.bridge.reset()
            return

        u = self.controller.compute(self.current_speed, self.desired_speed, dt)
        gear, brake, throttle = self.bridge.map_signals(u, self.desired_speed, self.current_speed)
        cmd = create_command_frame(gear, brake, throttle, self.desired_steer,
                                   state_label='tuning', time_msg=now.to_msg())
        self.can_pub.publish(cmd)

    # ====================================================================== #
    #  Speed graph — 10 Hz                                                    #
    # ====================================================================== #

    def _build_graph(self):
        self._plot_times = deque(maxlen=PLOT_MAX_PTS)
        self._plot_desired = deque(maxlen=PLOT_MAX_PTS)
        self._plot_actual = deque(maxlen=PLOT_MAX_PTS)

        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(10, 4))
        self._fig.canvas.manager.set_window_title('NiFT Longitudinal Control — Speed Trace')
        self._line_desired, = self._ax.plot([], [], 'r--', label='Desired (m/s)', linewidth=2)
        self._line_actual, = self._ax.plot([], [], 'b-', label='Actual (m/s)', linewidth=2)
        self._ax.set_ylim(-1, 6)
        self._ax.set_ylabel('Speed (m/s)')
        self._ax.set_xlabel('Time (s)')
        self._ax.set_title('Longitudinal Control Performance')
        self._ax.grid(True, linestyle='--', alpha=0.5)
        self._ax.legend(loc='upper right')

    def _update_plot(self):
        t = (self.get_clock().now() - self._plot_start).nanoseconds / 1e9
        self._plot_times.append(t)
        self._plot_desired.append(self.desired_speed)
        self._plot_actual.append(self.current_speed)

        self._line_desired.set_data(self._plot_times, self._plot_desired)
        self._line_actual.set_data(self._plot_times, self._plot_actual)
        if len(self._plot_times) > 1:
            self._ax.set_xlim(min(self._plot_times), max(self._plot_times))

        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

    # ====================================================================== #
    #  Controller interface (GUI → controller, no topic)                      #
    # ====================================================================== #

    def _set_cmd(self, speed: float, steer: float):
        """Set the active command in-process. Replaces the old /control topic."""
        self.desired_speed = speed
        self.desired_steer = steer
        self._last_cmd_time = self.get_clock().now()
        self._is_active = True

    def _retune_adrc(self, b0: float, w_cl: float, k_eso: float):
        """Reinitializes ADRC with new gains. Replaces the old /tune_adrc topic."""
        self.controller = ADRCController(delta=1.0 / CONTROL_HZ, b0=b0, w_cl=w_cl, k_eso=k_eso)
        self.get_logger().info(f"ADRC retuned: b0={b0:.3f}  w_cl={w_cl:.3f}  k_eso={k_eso:.3f}")

    def trigger_estop(self):
        self._is_active = False
        self.desired_speed = 0.0
        self.desired_steer = 0.0
        self.can_pub.publish(create_estop_frame(time_msg=self.get_clock().now().to_msg()))
        self.controller.reset()
        self.bridge.reset()

    # ====================================================================== #
    #  GUI — build                                                             #
    # ====================================================================== #

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title('NiFT Longitudinal Tuner')
        self.root.geometry('900x750')

        self._speed_str = tk.StringVar(value='0.00')
        self._heading_str = tk.StringVar(value='0.0')
        self._lat_str = tk.StringVar(value='0.000000')
        self._lon_str = tk.StringVar(value='0.000000')
        self._local_x_str = tk.StringVar(value='0.0')
        self._local_y_str = tk.StringVar(value='0.0')

        self._build_telemetry_panel()
        self._build_mode_panel()
        self._build_cmd_panel()
        self._build_can_panel()
        self._build_tuning_panel()

        tk.Button(self.root, text='EMERGENCY STOP', bg='#cc0000', fg='white',
                  font=('Arial', 14, 'bold'), command=self._on_estop_btn, height=2
                  ).pack(side=tk.BOTTOM, fill=tk.X, padx=20, pady=20)

        # 10 Hz timer: reads sliders and pushes to controller
        self.create_timer(0.1, self._cmd_tick)

    def _build_telemetry_panel(self):
        frame = ttk.LabelFrame(self.root, text=' Live Telemetry ', padding=10)
        frame.pack(fill=tk.X, padx=20, pady=10)

        text = ttk.Frame(frame)
        text.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 40))
        for i, (label, var) in enumerate([
            ('Speed (m/s):', self._speed_str),
            ('Heading (deg):', self._heading_str),
            ('Lat:', self._lat_str),
            ('Lon:', self._lon_str),
            ('Local E:', self._local_x_str),
            ('Local N:', self._local_y_str),
        ]):
            ttk.Label(text, text=label, width=15).grid(row=i, column=0, sticky='w', pady=4)
            ttk.Label(text, textvariable=var, font=('Courier', 12, 'bold')).grid(
                row=i, column=1, sticky='w', pady=4
            )

        self._canvas = tk.Canvas(frame, width=400, height=200, bg='#1e1e1e', highlightthickness=0)
        self._canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Speedometer arc
        self._canvas.create_arc(20, 20, 180, 180, start=150, extent=-240,
                                style=tk.ARC, outline='#444444', width=5)
        self._speed_dial_text = self._canvas.create_text(
            100, 140, text='0.0 m/s', fill='#00ffcc', font=('Courier', 14, 'bold')
        )
        self._speed_needle = self._canvas.create_line(100, 100, 40, 100, fill='#00ffcc', width=3)

        # Compass
        self._canvas.create_oval(220, 20, 380, 180, outline='#444444', width=5)
        for txt, x, y in [('N', 300, 10), ('S', 300, 190), ('W', 210, 100), ('E', 390, 100)]:
            self._canvas.create_text(x, y, text=txt, fill='white', font=('Arial', 10, 'bold'))
        self._heading_dial_text = self._canvas.create_text(
            300, 140, text='0.0°', fill='#ff3366', font=('Courier', 14, 'bold')
        )
        self._heading_needle = self._canvas.create_line(300, 100, 300, 30,
                                                        fill='#ff3366', width=3, arrow=tk.LAST)

    def _draw_speed_needle(self, speed: float):
        clamped = max(0.0, min(speed, 10.0))
        angle_rad = math.radians(150 - (clamped / 10.0) * 240)
        nx = 100 + 70 * math.cos(angle_rad)
        ny = 100 - 70 * math.sin(angle_rad)
        self._canvas.coords(self._speed_needle, 100, 100, nx, ny)
        self._canvas.itemconfig(self._speed_dial_text, text=f"{speed:.1f} m/s")

    def _draw_heading_needle(self, heading: float):
        angle_rad = math.radians(90 - heading)
        nx = 300 + 70 * math.cos(angle_rad)
        ny = 100 - 70 * math.sin(angle_rad)
        self._canvas.coords(self._heading_needle, 300, 100, nx, ny)
        self._canvas.itemconfig(self._heading_dial_text, text=f"{heading:.1f}°")

    def _build_mode_panel(self):
        self._mode = tk.StringVar(value='STANDBY')
        frame = ttk.LabelFrame(self.root, text=' System State ', padding=10)
        frame.pack(fill=tk.X, padx=20, pady=5)
        for label, value in [('STANDBY (Read-Only)', 'STANDBY'),
                             ('TWIST CONTROL', 'TWIST'),
                             ('MANUAL CAN', 'CAN')]:
            ttk.Radiobutton(frame, text=label, variable=self._mode,
                            value=value, command=self._on_mode_change).pack(side=tk.LEFT, padx=10)

    def _on_mode_change(self):
        mode = self._mode.get()
        self._send_can_btn.config(state=tk.NORMAL if mode == 'CAN' else tk.DISABLED)
        self._speed_slider.config(state=tk.NORMAL if mode == 'TWIST' else tk.DISABLED)
        self._steer_slider.config(state=tk.NORMAL if mode == 'TWIST' else tk.DISABLED)

    def _build_cmd_panel(self):
        frame = ttk.LabelFrame(self.root, text=' Twist Control (W / A / S / D) ', padding=10)
        frame.pack(fill=tk.X, padx=20, pady=5)

        self._cmd_speed = tk.DoubleVar(value=0.0)
        self._cmd_steer = tk.IntVar(value=0)

        sf = ttk.Frame(frame)
        sf.pack(fill=tk.X, pady=5)
        ttk.Label(sf, text='Speed (m/s):', width=12).pack(side=tk.LEFT)
        ttk.Spinbox(
            sf,
            from_=-5.0,
            to=5.0,
            increment=0.1,
            textvariable=self._cmd_speed,
            width=8,
            format='%.1f',
        ).pack(side=tk.LEFT, padx=5)
        self._speed_slider = tk.Scale(
            sf,
            from_=-5.0,
            to=5.0,
            resolution=0.1,
            orient=tk.HORIZONTAL,
            variable=self._cmd_speed,
            state=tk.DISABLED,
        )
        self._speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        tf = ttk.Frame(frame)
        tf.pack(fill=tk.X, pady=5)
        ttk.Label(tf, text='Steer (deg):', width=12).pack(side=tk.LEFT)
        ttk.Spinbox(tf, from_=-STEER_LIM, to=STEER_LIM, increment=1,
                    textvariable=self._cmd_steer, width=8).pack(side=tk.LEFT, padx=5)
        self._steer_slider = tk.Scale(
            tf,
            from_=-STEER_LIM,
            to=STEER_LIM,
            resolution=1,
            orient=tk.HORIZONTAL,
            variable=self._cmd_steer,
            state=tk.DISABLED,
        )
        self._steer_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.root.bind('<w>', lambda e: self._nudge_speed(0.1))
        self.root.bind('<Up>', lambda e: self._nudge_speed(0.1))
        self.root.bind('<s>', lambda e: self._nudge_speed(-0.1))
        self.root.bind('<Down>', lambda e: self._nudge_speed(-0.1))
        self.root.bind('<a>', lambda e: self._nudge_steer(-1))
        self.root.bind('<Left>', lambda e: self._nudge_steer(-1))
        self.root.bind('<d>', lambda e: self._nudge_steer(1))
        self.root.bind('<Right>', lambda e: self._nudge_steer(1))
        self.root.bind('<space>', lambda e: self._cmd_steer.set(0))

    def _nudge_speed(self, delta: float):
        if self._mode.get() == 'TWIST':
            self._cmd_speed.set(max(-5.0, min(5.0, round(self._cmd_speed.get() + delta, 1))))

    def _nudge_steer(self, delta: int):
        if self._mode.get() == 'TWIST':
            self._cmd_steer.set(max(-STEER_LIM, min(STEER_LIM, self._cmd_steer.get() + delta)))

    def _build_can_panel(self):
        frame = ttk.LabelFrame(self.root, text=' Manual CAN Layer ', padding=10)
        frame.pack(fill=tk.X, padx=20, pady=10)

        self._can_entries = []
        for i, label in enumerate(
            [
                'Gear (0/1/2)',
                f"Brake (0-{CAN_CMD_MAX})",
                f"Pedal (0-{CAN_CMD_MAX})",
                f"Steer (-{STEER_LIM}–{STEER_LIM})",
            ]
        ):
            ttk.Label(frame, text=label).grid(row=0, column=i, padx=4)
            e = ttk.Entry(frame, width=10)
            e.insert(0, '0')
            e.grid(row=1, column=i, padx=4, pady=4)
            self._can_entries.append(e)

        self._send_can_btn = tk.Button(frame, text='SEND', bg='#ff9900',
                                       state=tk.DISABLED, command=self._on_send_can)
        self._send_can_btn.grid(row=0, column=4, rowspan=2, padx=10)

    def _build_tuning_panel(self):
        frame = ttk.LabelFrame(self.root, text=' ADRC Live Tuning ', padding=10)
        frame.pack(fill=tk.X, padx=20, pady=5)

        self._var_b0 = tk.DoubleVar(value=self._gains['b0'])
        self._var_wcl = tk.DoubleVar(value=self._gains['w_cl'])
        self._var_keso = tk.DoubleVar(value=self._gains['k_eso'])

        for label, var in [('b0 (Gain):', self._var_b0),
                           ('w_cl (Bandwidth):', self._var_wcl),
                           ('k_eso (Observer):', self._var_keso)]:
            row = ttk.Frame(frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=18).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=var, width=10).pack(side=tk.LEFT, padx=5)

        tk.Button(frame, text='APPLY TUNING', bg='#0066cc', fg='white',
                  command=self._on_apply_tuning_btn).pack(side=tk.RIGHT, padx=10)

    # ====================================================================== #
    #  GUI event handlers                                                      #
    # ====================================================================== #

    def _cmd_tick(self):
        """10 Hz timer: reads the sliders and pushes the command to the controller."""
        if self._mode.get() == 'TWIST':
            self._set_cmd(self._cmd_speed.get(), float(self._steer_slider.get()))

    def _on_apply_tuning_btn(self):
        self._retune_adrc(self._var_b0.get(), self._var_wcl.get(), self._var_keso.get())

    def _on_send_can(self):
        try:
            gear = int(self._can_entries[0].get())
            brake = int(self._can_entries[1].get())
            pedal = int(self._can_entries[2].get())
            steer = int(self._can_entries[3].get())
        except ValueError:
            self.get_logger().error('Manual CAN: all fields must be integers')
            return

        msg = create_command_frame(gear, brake, pedal, steer,
                                   state_label='manual_can',
                                   time_msg=self.get_clock().now().to_msg())
        self.can_pub.publish(msg)
        data_hex = ' '.join([f'{x:02X}' for x in msg.data])
        self.get_logger().info(f"Manual CAN sent: ID=0x{msg.id:X}  Data=[{data_hex}]")

    def _on_estop_btn(self):
        self._mode.set('STANDBY')
        self._on_mode_change()
        for slider in (self._speed_slider, self._steer_slider):
            slider.config(state=tk.NORMAL)
        self._cmd_speed.set(0.0)
        self._cmd_steer.set(0)
        for slider in (self._speed_slider, self._steer_slider):
            slider.config(state=tk.DISABLED)
        self.trigger_estop()
        self.get_logger().warn('ESTOP TRIGGERED')

    # ====================================================================== #
    #  Run loop                                                                #
    # ====================================================================== #

    def run(self):
        while rclpy.ok():
            self.root.update_idletasks()
            self.root.update()
            rclpy.spin_once(self, timeout_sec=0.01)


def run_tuner(gains: dict, zero_speed_tol: float, args=None):
    """Entry point of `longitudinal_test tune`."""
    rclpy.init(args=args)
    node = LongitudinalTuner(gains, zero_speed_tol)
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C race: rclpy's SIGINT handler can shut the context down mid-construction or
        # mid-spin (RCLError). With the context still alive it is a real error.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()    # Ctrl-C: rclpy's SIGINT handler has already shut the context down
