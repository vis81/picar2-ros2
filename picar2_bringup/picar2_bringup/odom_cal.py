#!/usr/bin/env python3
"""Odometry calibration GUI for PICAR-2.

    ros2 run picar2_bringup odom_cal.py
    make odom-cal   (from ros2/ — launches in Docker with display forwarding)
"""

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ── ROS node ─────────────────────────────────────────────────────────────────

class OdomCalNode(Node):
    def __init__(self):
        super().__init__('odom_cal')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self._odom_cb, 1,
        )
        self.create_timer(0.1, self._vel_timer)

        self._lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.origin_x = self.origin_y = 0.0
        self._speed = 0.0        # current commanded speed (signed)
        self._target = None      # distance target (metres, positive)
        self.moving = False
        self.on_arrival = None   # called(actual_dist) when target reached

    # ── odometry callback ─────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y ** 2 + q.z ** 2),
            )
            if self.moving and self._target is not None:
                dist = math.hypot(self.x - self.origin_x, self.y - self.origin_y)
                if dist >= self._target:
                    self._speed = 0.0
                    self.moving = False
                    cb = self.on_arrival
                    dist_snap = dist
                    if cb:
                        threading.Thread(target=cb, args=(dist_snap,), daemon=True).start()

    # ── periodic velocity publisher ───────────────────────────────────────

    def _vel_timer(self):
        t = Twist()
        with self._lock:
            t.linear.x = self._speed
        self.pub.publish(t)

    # ── public API ────────────────────────────────────────────────────────

    def move(self, distance: float, speed: float, on_arrival=None):
        """Start moving |distance| metres at |speed| m/s. Sign of distance sets direction."""
        with self._lock:
            self.origin_x = self.x
            self.origin_y = self.y
            self._target = abs(distance)
            self._speed = abs(speed) if distance >= 0 else -abs(speed)
            self.moving = True
            self.on_arrival = on_arrival

    def stop(self):
        with self._lock:
            self._speed = 0.0
            self.moving = False
            self._target = None

    def reset_origin(self):
        with self._lock:
            self.origin_x = self.x
            self.origin_y = self.y

    def get_state(self):
        with self._lock:
            dist = math.hypot(self.x - self.origin_x, self.y - self.origin_y)
            return self.x, self.y, self.yaw, dist, self.moving, self._target


# ── GUI ───────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    POLL_MS = 100

    def __init__(self, node: OdomCalNode):
        super().__init__()
        self.node = node
        self.title('Odometry Calibration — PICAR-2')
        self.resizable(False, False)
        self._build()
        self._poll()

    # ── layout ────────────────────────────────────────────────────────────

    def _build(self):
        P = dict(padx=8, pady=5)

        # Parameters
        fp = ttk.LabelFrame(self, text='Move parameters')
        fp.grid(row=0, column=0, sticky='ew', **P)

        ttk.Label(fp, text='Distance (m):').grid(row=0, column=0, sticky='e', padx=(8, 2))
        self.v_dist = tk.DoubleVar(value=1.0)
        ttk.Spinbox(fp, textvariable=self.v_dist, from_=0.1, to=10.0,
                    increment=0.1, width=7, format='%.2f').grid(row=0, column=1, padx=(2, 12))

        ttk.Label(fp, text='Speed (m/s):').grid(row=0, column=2, sticky='e', padx=(0, 2))
        self.v_speed = tk.DoubleVar(value=0.1)
        ttk.Spinbox(fp, textvariable=self.v_speed, from_=0.05, to=0.5,
                    increment=0.05, width=7, format='%.2f').grid(row=0, column=3, padx=(2, 8))

        # Controls
        fc = ttk.LabelFrame(self, text='Control')
        fc.grid(row=1, column=0, sticky='ew', **P)

        self.btn_bwd = ttk.Button(fc, text='◄ Backward', width=14, command=self._backward)
        self.btn_bwd.grid(row=0, column=0, **P)

        self.btn_stop = ttk.Button(fc, text='■ Stop', width=10, command=self._stop)
        self.btn_stop.grid(row=0, column=1, **P)

        self.btn_fwd = ttk.Button(fc, text='Forward ►', width=14, command=self._forward)
        self.btn_fwd.grid(row=0, column=2, **P)

        ttk.Button(fc, text='↺ Reset origin', width=14,
                   command=self._reset).grid(row=0, column=3, **P)

        # Odometry
        fo = ttk.LabelFrame(self, text='Odometry')
        fo.grid(row=2, column=0, sticky='ew', **P)

        fields = [
            ('X', 'm'),
            ('Y', 'm'),
            ('Yaw', '°'),
            ('Trip distance', 'm'),
        ]
        self._oval = []
        for i, (name, unit) in enumerate(fields):
            ttk.Label(fo, text=f'{name}:', anchor='e', width=14).grid(
                row=i, column=0, sticky='e', padx=(8, 2), pady=2)
            var = tk.StringVar(value='—')
            self._oval.append(var)
            ttk.Label(fo, textvariable=var, anchor='w', width=10,
                      font=('Courier', 11)).grid(row=i, column=1, sticky='w', padx=(2, 4))
            ttk.Label(fo, text=unit, anchor='w').grid(row=i, column=2, sticky='w', padx=(0, 8))

        # Progress bar
        self.v_prog = tk.DoubleVar(value=0.0)
        self.prog = ttk.Progressbar(fo, variable=self.v_prog, maximum=1.0, length=220)
        self.prog.grid(row=len(fields), column=0, columnspan=3,
                       padx=8, pady=(4, 6), sticky='ew')

        # Status
        self.v_status = tk.StringVar(value='Ready')
        ttk.Label(self, textvariable=self.v_status, relief='sunken',
                  anchor='w', padding=(6, 2)).grid(row=3, column=0,
                  sticky='ew', padx=8, pady=(2, 8))

    # ── button handlers ───────────────────────────────────────────────────

    def _params(self):
        try:
            d = float(self.v_dist.get())
            s = float(self.v_speed.get())
            if d <= 0 or s <= 0:
                raise ValueError
            return d, s
        except (ValueError, tk.TclError):
            self.v_status.set('Invalid distance or speed')
            return None

    def _forward(self):
        p = self._params()
        if p is None:
            return
        d, s = p
        self.v_status.set(f'Moving forward {d:.2f} m at {s:.2f} m/s…')
        self.node.move(d, s, on_arrival=self._arrived)

    def _backward(self):
        p = self._params()
        if p is None:
            return
        d, s = p
        self.v_status.set(f'Moving backward {d:.2f} m at {s:.2f} m/s…')
        self.node.move(-d, s, on_arrival=self._arrived)

    def _stop(self):
        self.node.stop()
        self.v_status.set('Stopped')

    def _reset(self):
        self.node.reset_origin()
        self.v_prog.set(0.0)
        self.v_status.set('Origin reset')

    def _arrived(self, dist: float):
        self.after(0, lambda: self.v_status.set(
            f'Done — odom: {dist:.4f} m  ·  measure actual distance and compare'))

    # ── polling update ────────────────────────────────────────────────────

    def _poll(self):
        x, y, yaw, dist, moving, target = self.node.get_state()
        self._oval[0].set(f'{x:+.4f}')
        self._oval[1].set(f'{y:+.4f}')
        self._oval[2].set(f'{math.degrees(yaw):+.2f}')
        self._oval[3].set(f'{dist:.4f}')

        if target and target > 0:
            self.v_prog.set(min(dist / target, 1.0))
        elif not moving:
            self.v_prog.set(0.0)

        state = 'MOVING' if moving else 'stopped'
        if moving:
            self.btn_fwd.state(['disabled'])
            self.btn_bwd.state(['disabled'])
        else:
            self.btn_fwd.state(['!disabled'])
            self.btn_bwd.state(['!disabled'])

        self.after(self.POLL_MS, self._poll)


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = OdomCalNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = App(node)
    try:
        app.mainloop()
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
