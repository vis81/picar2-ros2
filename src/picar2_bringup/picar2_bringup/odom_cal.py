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
from sensor_msgs.msg import Imu


# ── ROS node ─────────────────────────────────────────────────────────────────

class OdomCalNode(Node):
    def __init__(self):
        super().__init__('odom_cal')
        self.declare_parameter('wheelbase', 0.235)
        self._wheelbase = self.get_parameter('wheelbase').value

        self.declare_parameter('steer_lut_us',
                               [-595, -347, -298, -149, 0, 136, 272, 408, 545])
        self.declare_parameter('steer_lut_rad',
                               [0.6756, 0.4224, 0.3368, 0.1728, 0.0,
                                -0.1239, -0.2443, -0.4014, -0.6109])
        _us  = list(self.get_parameter('steer_lut_us').value)
        _rad = list(self.get_parameter('steer_lut_rad').value)
        self._steer_lut = list(zip(_us, _rad))  # [(us, rad)...] us asc, rad desc

        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self._odom_cb, 1,
        )
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_timer(0.1, self._vel_timer)

        self._lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.origin_x = self.origin_y = 0.0
        self._speed = 0.0        # current commanded speed (signed)
        self._angular_z = 0.0   # current commanded angular.z (rad/s)
        self._target = None      # distance target (metres, positive)
        self.moving = False
        self.on_arrival = None   # called(actual_dist) when target reached

        # circle test state
        self._imu_t         = None  # last IMU stamp (float seconds)
        self._imu_accum     = 0.0   # integrated gyro Z (rad) — ground truth
        self._odom_yaw_prev = 0.0   # previous odom yaw for unwrapping
        self._odom_accum    = 0.0   # accumulated (unwrapped) odom yaw (rad)
        self._circle_target  = 0.0  # ±2π
        self._circle_running = False
        self.on_circle_done  = None  # called(imu_accum, odom_accum, pos_err, arc_length)
        self._arc_length     = 0.0   # path length during circle (m)
        self._odom_t         = None  # odom stamp for arc_length dt

    @property
    def wheelbase(self):
        return self._wheelbase

    def lut_rad_to_us(self, rad_val: float) -> int:
        lut = self._steer_lut
        if len(lut) < 2:
            return 0
        if rad_val >= lut[0][1]:
            return lut[0][0]
        if rad_val <= lut[-1][1]:
            return lut[-1][0]
        for i in range(1, len(lut)):
            if rad_val >= lut[i][1]:
                t = (rad_val - lut[i-1][1]) / (lut[i][1] - lut[i-1][1])
                return round(lut[i-1][0] + t * (lut[i][0] - lut[i-1][0]))
        return lut[-1][0]

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
            if self._circle_running:
                dyaw = self.yaw - self._odom_yaw_prev
                if dyaw >  math.pi: dyaw -= 2 * math.pi
                if dyaw < -math.pi: dyaw += 2 * math.pi
                self._odom_accum += dyaw
                # arc length: integrate wheel speed (steering-independent)
                t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if self._odom_t is not None:
                    dt = t_msg - self._odom_t
                    if 0 < dt < 0.5:
                        self._arc_length += abs(msg.twist.twist.linear.x) * dt
                self._odom_t = t_msg
            self._odom_yaw_prev = self.yaw
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

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            if self._imu_t is not None and self._circle_running:
                dt = t - self._imu_t
                if 0 < dt < 0.5:
                    self._imu_accum += msg.angular_velocity.z * dt
                    if abs(self._imu_accum) >= abs(self._circle_target):
                        self._speed = 0.0
                        self._angular_z = 0.0
                        self._circle_running = False
                        cb = self.on_circle_done
                        snap = (self._imu_accum, self._odom_accum,
                                math.hypot(self.x - self.origin_x,
                                           self.y - self.origin_y),
                                self._arc_length)
                        if cb:
                            threading.Thread(target=cb, args=snap,
                                             daemon=True).start()
            self._imu_t = t

    def _vel_timer(self):
        t = Twist()
        with self._lock:
            t.linear.x = self._speed
            t.angular.z = self._angular_z
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
            self._angular_z = 0.0
            self.moving = False
            self._target = None

    def reset_origin(self):
        with self._lock:
            self.origin_x = self.x
            self.origin_y = self.y

    def start_circle(self, linear: float, angular: float, on_done=None):
        with self._lock:
            self.origin_x = self.x
            self.origin_y = self.y
            self._odom_yaw_prev = self.yaw
            self._imu_accum  = 0.0
            self._odom_accum = 0.0
            self._arc_length = 0.0
            self._odom_t     = None
            self._circle_target  = math.copysign(2 * math.pi, angular)
            self._speed      = linear
            self._angular_z  = angular
            self._circle_running = True
            self.on_circle_done  = on_done

    def stop_circle(self):
        with self._lock:
            self._speed = 0.0
            self._angular_z = 0.0
            self._circle_running = False

    def get_state(self):
        with self._lock:
            dist = math.hypot(self.x - self.origin_x, self.y - self.origin_y)
            return (self.x, self.y, self.yaw, dist, self.moving, self._target,
                    self._imu_accum, self._odom_accum, self._circle_running,
                    self._arc_length)


# ── GUI ───────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    POLL_MS = 100

    def __init__(self, node: OdomCalNode):
        super().__init__()
        self.node = node
        self.title('Odometry Calibration — PICAR-2')
        self.resizable(False, False)
        self._last_circle_speed = 0.0
        self._last_circle_turn  = 0.0   # signed rad/s as commanded
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

        # Circle test
        fci = ttk.LabelFrame(self, text='Circle test')
        fci.grid(row=3, column=0, sticky='ew', **P)

        ttk.Label(fci, text='Speed (m/s):').grid(row=0, column=0, sticky='e', padx=(8, 2))
        self.v_cspeed = tk.DoubleVar(value=0.20)
        ttk.Spinbox(fci, textvariable=self.v_cspeed, from_=0.05, to=0.5,
                    increment=0.05, width=7, format='%.2f').grid(row=0, column=1, padx=(2, 12))

        ttk.Label(fci, text='Turn (rad/s):').grid(row=0, column=2, sticky='e', padx=(0, 2))
        self.v_cturn = tk.DoubleVar(value=0.60)
        ttk.Spinbox(fci, textvariable=self.v_cturn, from_=0.1, to=1.2,
                    increment=0.05, width=7, format='%.2f').grid(row=0, column=3, padx=(2, 8))

        self.btn_ccw = ttk.Button(fci, text='◄ CCW', width=10, command=self._circle_ccw)
        self.btn_ccw.grid(row=1, column=0, **P)
        self.btn_cstop = ttk.Button(fci, text='■ Stop', width=10, command=self._circle_stop)
        self.btn_cstop.grid(row=1, column=1, **P)
        self.btn_cw = ttk.Button(fci, text='CW ►', width=10, command=self._circle_cw)
        self.btn_cw.grid(row=1, column=2, **P)

        # Row 2: yaw / pos error results
        cr = ttk.Frame(fci)
        cr.grid(row=2, column=0, columnspan=6, sticky='ew', padx=8, pady=(4, 2))
        self.v_imu_yaw  = tk.StringVar(value='—')
        self.v_odom_yaw = tk.StringVar(value='—')
        self.v_pos_err  = tk.StringVar(value='—')
        for col, (label, var, unit) in enumerate([
            ('IMU yaw:',  self.v_imu_yaw,  '°'),
            ('Odom yaw:', self.v_odom_yaw, '°'),
            ('Pos error:', self.v_pos_err,  'm'),
        ]):
            ttk.Label(cr, text=label, anchor='e').grid(row=0, column=col*3,   sticky='e', padx=(4, 2))
            ttk.Label(cr, textvariable=var, anchor='w', width=8,
                      font=('Courier', 11)).grid(row=0, column=col*3+1, sticky='w')
            ttk.Label(cr, text=unit).grid(row=0, column=col*3+2, sticky='w', padx=(0, 8))

        # Row 3: turn radius / steer angle results
        rr = ttk.Frame(fci)
        rr.grid(row=3, column=0, columnspan=6, sticky='ew', padx=8, pady=(2, 2))
        self.v_turn_r       = tk.StringVar(value='—')
        self.v_actual_steer = tk.StringVar(value='—')
        self.v_cmd_steer    = tk.StringVar(value='—')
        for col, (label, var, unit) in enumerate([
            ('Turn radius R:', self.v_turn_r,       'm'),
            ('Actual steer:',  self.v_actual_steer, '°'),
            ('Cmd steer:',     self.v_cmd_steer,    '°'),
        ]):
            ttk.Label(rr, text=label, anchor='e').grid(row=0, column=col*3,   sticky='e', padx=(4, 2))
            ttk.Label(rr, textvariable=var, anchor='w', width=8,
                      font=('Courier', 11)).grid(row=0, column=col*3+1, sticky='w')
            ttk.Label(rr, text=unit).grid(row=0, column=col*3+2, sticky='w', padx=(0, 8))

        # Row 4: LUT point output
        lr = ttk.Frame(fci)
        lr.grid(row=4, column=0, columnspan=6, sticky='ew', padx=8, pady=(2, 2))
        ttk.Label(lr, text='LUT point:', anchor='e').pack(side='left', padx=(0, 4))
        self.v_lut_point = tk.StringVar(value='—')
        ttk.Label(lr, textvariable=self.v_lut_point, anchor='w',
                  font=('Courier', 11, 'bold')).pack(side='left')

        # Row 5: progress bar
        self.v_cprog = tk.DoubleVar(value=0.0)
        ttk.Progressbar(fci, variable=self.v_cprog, maximum=1.0,
                        length=220).grid(row=5, column=0, columnspan=6,
                                         padx=8, pady=(2, 6), sticky='ew')

        # Status
        self.v_status = tk.StringVar(value='Ready')
        ttk.Label(self, textvariable=self.v_status, relief='sunken',
                  anchor='w', padding=(6, 2)).grid(row=4, column=0,
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

    def _circle_params(self):
        try:
            s = float(self.v_cspeed.get())
            t = float(self.v_cturn.get())
            if s <= 0 or t <= 0:
                raise ValueError
            return s, t
        except (ValueError, tk.TclError):
            self.v_status.set('Invalid circle speed or turn rate')
            return None

    def _circle_cw(self):
        p = self._circle_params()
        if p is None:
            return
        s, t = p
        self._last_circle_speed = s
        self._last_circle_turn  = -t
        self.v_status.set(f'Circle CW — speed {s:.2f} m/s, turn {t:.2f} rad/s…')
        self.v_turn_r.set('—')
        self.v_actual_steer.set('—')
        self.v_cmd_steer.set('—')
        self.v_lut_point.set('—')
        self.node.start_circle(s, -t, on_done=self._circle_done)

    def _circle_ccw(self):
        p = self._circle_params()
        if p is None:
            return
        s, t = p
        self._last_circle_speed = s
        self._last_circle_turn  = t
        self.v_status.set(f'Circle CCW — speed {s:.2f} m/s, turn {t:.2f} rad/s…')
        self.v_turn_r.set('—')
        self.v_actual_steer.set('—')
        self.v_cmd_steer.set('—')
        self.v_lut_point.set('—')
        self.node.start_circle(s, t, on_done=self._circle_done)

    def _circle_stop(self):
        self.node.stop_circle()
        self.v_status.set('Circle stopped')

    def _circle_done(self, imu_accum: float, odom_accum: float,
                     pos_err: float, arc_length: float):
        imu_deg  = math.degrees(abs(imu_accum))
        odom_deg = math.degrees(abs(odom_accum))

        # turn radius and actual steer angle from arc length
        wb = self.node.wheelbase
        if arc_length > 0.01:
            R = arc_length / (2 * math.pi)
            actual_steer_rad = math.atan(wb / R)
            actual_steer_rad = math.copysign(actual_steer_rad, self._last_circle_turn)
            actual_steer_deg = math.degrees(actual_steer_rad)
            r_str  = f'{R:.4f}'
            as_str = f'{actual_steer_deg:+.2f}'
        else:
            R = None
            actual_steer_rad = None
            r_str = as_str = '—'

        # commanded steer angle from linear/angular cmd_vel ratio
        v   = self._last_circle_speed
        w   = self._last_circle_turn
        if abs(v) > 0.001:
            cmd_steer_rad = math.atan(wb * w / v)
            cmd_steer_deg = math.degrees(cmd_steer_rad)
            cmd_delta_us  = self.node.lut_rad_to_us(cmd_steer_rad)
            cs_str = f'{cmd_steer_deg:+.2f}'
        else:
            cmd_steer_rad = None
            cmd_delta_us  = None
            cs_str = '—'

        # LUT point: (delta_us, actual_steer_rad)
        if cmd_delta_us is not None and actual_steer_rad is not None:
            lut_str = f'({cmd_delta_us}, {actual_steer_rad:.4f})  # {actual_steer_deg:+.2f}°'
        else:
            lut_str = '—'

        def _update():
            self.v_imu_yaw.set(f'{imu_deg:+.1f}')
            self.v_odom_yaw.set(f'{odom_deg:+.1f}')
            self.v_pos_err.set(f'{pos_err:.4f}')
            self.v_turn_r.set(r_str)
            self.v_actual_steer.set(as_str)
            self.v_cmd_steer.set(cs_str)
            self.v_lut_point.set(lut_str)
            self.v_cprog.set(0.0)
            self.v_status.set(
                f'Circle done — R={r_str} m  actual={as_str}°  cmd={cs_str}°  '
                f'pos_err={pos_err:.3f} m')
        self.after(0, _update)

    # ── polling update ────────────────────────────────────────────────────

    def _poll(self):
        x, y, yaw, dist, moving, target, imu_accum, odom_accum, circle_running, \
            arc_length = self.node.get_state()

        self._oval[0].set(f'{x:+.4f}')
        self._oval[1].set(f'{y:+.4f}')
        self._oval[2].set(f'{math.degrees(yaw):+.2f}')
        self._oval[3].set(f'{dist:.4f}')

        if target and target > 0:
            self.v_prog.set(min(dist / target, 1.0))
        elif not moving:
            self.v_prog.set(0.0)

        if moving:
            self.btn_fwd.state(['disabled'])
            self.btn_bwd.state(['disabled'])
        else:
            self.btn_fwd.state(['!disabled'])
            self.btn_bwd.state(['!disabled'])

        # circle panel
        if circle_running:
            self.btn_cw.state(['disabled'])
            self.btn_ccw.state(['disabled'])
            self.v_imu_yaw.set(f'{math.degrees(imu_accum):+.1f}')
            self.v_odom_yaw.set(f'{math.degrees(odom_accum):+.1f}')
            # live arc length → live R estimate
            wb = self.node.wheelbase
            if arc_length > 0.01 and abs(imu_accum) > 0.1:
                R_live = arc_length / abs(imu_accum)
                self.v_turn_r.set(f'{R_live:.4f}')
            progress = abs(imu_accum) / (2 * math.pi)
            self.v_cprog.set(min(progress, 1.0))
        else:
            self.btn_cw.state(['!disabled'])
            self.btn_ccw.state(['!disabled'])

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
