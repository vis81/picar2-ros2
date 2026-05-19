#!/usr/bin/env python3
"""
ROS2 driver for the LDS02RR / Neato XV-11 LiDAR.

Reads packets from the serial port, drives the spinning motor via GPIO PWM
with a PI controller to hold the desired RPM, and publishes
sensor_msgs/LaserScan on /scan.

The RPM feedback used by the PI controller comes from speed bytes embedded in
every LiDAR packet, so no extra sensor is needed.

Parameters
----------
port        : str   Serial device (default /dev/serial0)
baud        : int   Baud rate (default 115200)
frame_id    : str   LaserScan frame (default laser_link)
range_min   : float Minimum valid range in m (default 0.06)
range_max   : float Maximum valid range in m (default 5.0)
target_rpm  : float Desired motor RPM (default 300.0)
kp          : float PI proportional gain (default 0.454)
ki          : float PI integral gain (default 0.050)
ff_duty     : float Feed-forward PWM duty cycle % (default 67.7)
angle_offset: float Extra scan rotation in radians added on top of gap-based auto-calibration (default 0.0)
pwm_pin     : int   BCM GPIO pin for motor PWM (default 18)
stby_pin    : int   BCM GPIO pin for motor STBY (default 23)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import serial

# ── Protocol constants ────────────────────────────────────────────────────────
_PACKET_BYTES     = 22
_INDEX_FIRST      = 0xA0
_INDEX_LAST       = 0xF9
_INDICES_PER_REV  = _INDEX_LAST - _INDEX_FIRST + 1   # 90
_READINGS_PER_PKT = 4
_READINGS_PER_REV = _INDICES_PER_REV * _READINGS_PER_PKT  # 360
DEG_PER_INDEX     = 360.0 / _INDICES_PER_REV              # 4.0

_RPM_MIN = 50.0
_RPM_MAX = 600.0

# ── Motor constants ───────────────────────────────────────────────────────────
_PWM_FREQ_HZ = 1000
_DUTY_MIN    = 10.0
_DUTY_MAX    = 90.0


def _checksum(pkt: bytes) -> int:
    """Neato XV-11 / LDS02RR 15-bit checksum over the first 20 bytes."""
    chk32 = 0
    for i in range(10):
        word = pkt[2 * i] | (pkt[2 * i + 1] << 8)
        chk32 = (chk32 << 1) + word
    checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
    return checksum & 0x7FFF


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('port',       '/dev/serial0')
        self.declare_parameter('baud',       115200)
        self.declare_parameter('frame_id',   'laser_link')
        self.declare_parameter('range_min',  0.06)
        self.declare_parameter('range_max',  5.0)
        self.declare_parameter('target_rpm', 300.0)
        self.declare_parameter('kp',           0.454)
        self.declare_parameter('ki',           0.050)
        self.declare_parameter('ff_duty',      67.7)
        self.declare_parameter('angle_offset', 0.0)
        self.declare_parameter('pwm_pin',      18)
        self.declare_parameter('stby_pin',     23)

        port       = self.get_parameter('port').value
        baud       = self.get_parameter('baud').value
        self._frame_id   = self.get_parameter('frame_id').value
        self._range_min  = self.get_parameter('range_min').value
        self._range_max  = self.get_parameter('range_max').value
        self._target_rpm = self.get_parameter('target_rpm').value
        self._kp           = self.get_parameter('kp').value
        self._ki           = self.get_parameter('ki').value
        self._ff_duty      = self.get_parameter('ff_duty').value
        self._base_offset  = 0.0   # auto-updated from blind-spot gap each revolution
        pwm_pin            = self.get_parameter('pwm_pin').value
        stby_pin           = self.get_parameter('stby_pin').value

        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._rpm_pub  = self.create_publisher(Float32,   'scan/rpm', 10)

        # GPIO motor setup
        self._stby_pin = stby_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pwm_pin,  GPIO.OUT)
        GPIO.setup(stby_pin, GPIO.OUT)
        GPIO.output(stby_pin, GPIO.HIGH)
        self._pwm = GPIO.PWM(pwm_pin, _PWM_FREQ_HZ)
        self._pwm.start(0)

        # PI controller state (accessed only from reader thread)
        self._integral  = 0.0
        self._last_pi_t = time.monotonic()

        # Serial port
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._ser.reset_input_buffer()

        # Scan accumulator (reader thread only — no lock needed)
        self._reset_scan()

        self._running = True
        self._thread  = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'lidar_node started  port={port}  target={self._target_rpm} RPM  '
            f'Kp={self._kp}  Ki={self._ki}  ff={self._ff_duty}%'
        )

    # ── Scan accumulator ──────────────────────────────────────────────────────

    def _reset_scan(self):
        self._ranges           = [float('inf')] * _READINGS_PER_REV
        self._intensities      = [0.0]          * _READINGS_PER_REV
        self._scan_stamp       = None
        self._scan_start_mono  = None
        self._received_indices = set()

    # ── Blind-spot gap detection ──────────────────────────────────────────────

    def _find_gap_start(self, received: set):
        """Return start index (0..89) of the 3-index blind-spot gap, or None."""
        n_missing = _INDICES_PER_REV - len(received)
        if not (3 <= n_missing <= 8):   # 3 expected; allow small noise margin
            return None
        for start in range(_INDICES_PER_REV):
            if all((start + j) % _INDICES_PER_REV not in received for j in range(3)):
                if (start - 1) % _INDICES_PER_REV in received:
                    return start
        return None

    def _update_base_offset(self):
        """Recompute _base_offset so the blind-spot gap lands at π (behind robot)."""
        gap = self._find_gap_start(self._received_indices)
        if gap is None:
            return
        # Gap spans 3 indices × 4 readings = 12°; centre is 6° past gap start
        gap_center_deg = gap * DEG_PER_INDEX + 6.0
        self._base_offset = math.pi - math.radians(gap_center_deg)
        self.get_logger().debug(f'blind-spot gap at index {gap}  ({gap_center_deg:.1f}°)  base_offset={math.degrees(self._base_offset):.1f}°')

    # ── PI motor control ──────────────────────────────────────────────────────

    def _pi_update(self, rpm: float) -> float:
        now = time.monotonic()
        dt  = max(now - self._last_pi_t, 1e-6)
        self._last_pi_t = now
        error = self._target_rpm - rpm
        self._integral += error * dt
        duty = self._ff_duty + self._kp * error + self._ki * self._integral
        if duty <= _DUTY_MIN or duty >= _DUTY_MAX:
            self._integral *= 0.95   # anti-windup
        return max(_DUTY_MIN, min(_DUTY_MAX, duty))

    # ── Packet parser ─────────────────────────────────────────────────────────

    def _parse_packet(self, pkt: bytes):
        """Return (rpm, index 0..89, readings) or None on any error."""
        if pkt[0] != 0xFA:
            return None
        if not (_INDEX_FIRST <= pkt[1] <= _INDEX_LAST):
            return None
        if _checksum(pkt[:20]) != (pkt[20] | (pkt[21] << 8)):
            return None

        speed_raw = pkt[2] | (pkt[3] << 8)
        rpm = speed_raw / 64.0
        if not (_RPM_MIN <= rpm <= _RPM_MAX):
            return None

        index = pkt[1] - _INDEX_FIRST   # 0..89
        readings = []
        for i in range(_READINGS_PER_PKT):
            b = 4 + i * 4
            dist_word = pkt[b] | (pkt[b + 1] << 8)
            invalid   = (dist_word >> 15) & 1
            dist_mm   = dist_word & 0x3FFF
            signal    = pkt[b + 2] | (pkt[b + 3] << 8)
            dist_m    = float('inf') if (invalid or dist_mm == 0) else dist_mm * 1e-3
            readings.append((dist_m, float(signal)))
        return rpm, index, readings

    # ── Scan publisher ────────────────────────────────────────────────────────

    def _publish_scan(self, scan_time: float):
        inc = 2.0 * math.pi / _READINGS_PER_REV
        msg = LaserScan()
        msg.header.stamp    = self._scan_stamp
        msg.header.frame_id = self._frame_id
        offset = self._base_offset + self.get_parameter('angle_offset').value
        msg.angle_min       = offset
        msg.angle_max       = offset + 2.0 * math.pi - inc
        msg.angle_increment = inc
        msg.time_increment  = scan_time / _READINGS_PER_REV if scan_time > 0.0 else 0.0
        msg.scan_time       = float(scan_time)
        msg.range_min       = self._range_min
        msg.range_max       = self._range_max
        msg.ranges          = list(self._ranges)
        msg.intensities     = list(self._intensities)
        self._scan_pub.publish(msg)

    # ── Reader thread ─────────────────────────────────────────────────────────

    def _reader_loop(self):
        buf = bytearray()
        self._pwm.ChangeDutyCycle(self._ff_duty)

        while self._running:
            data = self._ser.read(128)
            if data:
                buf += data

            while len(buf) >= _PACKET_BYTES:
                if buf[0] != 0xFA:
                    buf.pop(0)
                    continue

                pkt = bytes(buf[:_PACKET_BYTES])
                result = self._parse_packet(pkt)
                if result is None:
                    buf.pop(0)   # false 0xFA in payload — retry from next byte
                    continue
                buf = buf[_PACKET_BYTES:]

                rpm, index, readings = result

                # Motor PI control — runs on every valid packet (~450 Hz)
                duty = self._pi_update(rpm)
                self._pwm.ChangeDutyCycle(duty)

                # Publish RPM periodically (throttle to index 0 to avoid spam)
                if index == 0:
                    self._rpm_pub.publish(Float32(data=float(rpm)))

                # Detect revolution boundary: index wraps 89 → 0
                if index == 0:
                    now_msg   = self.get_clock().now().to_msg()
                    now_mono  = time.monotonic()
                    if self._scan_stamp is not None:
                        self._update_base_offset()   # use gap from completed revolution
                        scan_time = now_mono - self._scan_start_mono
                        self._publish_scan(scan_time)
                    self._reset_scan()               # also clears _received_indices
                    self._scan_stamp      = now_msg
                    self._scan_start_mono = now_mono

                if self._scan_stamp is None:
                    continue   # wait for first index=0 before filling

                # Fill accumulator (4 readings per packet)
                self._received_indices.add(index)
                base = index * _READINGS_PER_PKT
                for i, (dist_m, signal) in enumerate(readings):
                    self._ranges[base + i]      = dist_m
                    self._intensities[base + i] = signal

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        try:
            self._pwm.ChangeDutyCycle(0)
            time.sleep(0.1)
            self._pwm.stop()
            GPIO.output(self._stby_pin, GPIO.LOW)
            GPIO.cleanup()
        except Exception:
            pass
        finally:
            type(self._pwm).__del__ = lambda self: None
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
