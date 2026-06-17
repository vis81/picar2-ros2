#!/usr/bin/env python3
"""
DualShock 4 feedback bridge — rumble + lightbar.

joy_node uses SDL2 for /joy/set_feedback, which collapses both rumble motors
into a single magnitude (only the "strong" id 0 motor responds). We bypass it
by going direct to evdev for two-channel rumble, and to the kernel /sys leds
interface for the RGB lightbar.

Subscriptions:
  /joy/rumble  std_msgs/Float32MultiArray  [strong, weak, duration_sec]
                 strong, weak in [0.0, 1.0]; duration in [0.05, 5.0].
                 Publish (0,0,*) to stop early.
  /joy/led     std_msgs/ColorRGBA          r, g, b in [0.0, 1.0]; a unused.

Looks up the DS4 dynamically from /proc/bus/input/devices so it survives
USB↔BT reconnects.

Permissions:
  - /dev/input/event* needs the `input` group (Makefile adds --group-add).
  - /sys/class/leds/inputN:{red,green,blue}/brightness defaults to root:root
    0644 — a udev rule in etc/99-picar.rules chgrp's them to input on hotplug.
    Make sure `make install-uarts` was run after the rule was added.
"""

from __future__ import annotations

import glob
import os
import re
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA, Float32MultiArray

try:
    import evdev
except ImportError:  # python3-evdev not installed
    evdev = None


# Sony DualShock 4 USB IDs (both gen-1 09CC and gen-2 09CC firmware revs).
DS4_VENDOR = '054C'
DS4_PRODUCT_RE = re.compile(r'09[CcDd][C-Fc-f]')   # 09CC, 09CD…


def find_ds4() -> tuple[str | None, int | None]:
    """Return (event_path, input_index) for the DS4, or (None, None)."""
    try:
        blob = Path('/proc/bus/input/devices').read_text()
    except FileNotFoundError:
        return None, None
    for block in blob.split('\n\n'):
        if 'Wireless Controller' not in block or 'FF=' not in block:
            continue
        event = None
        idx = None
        for line in block.splitlines():
            if line.startswith('H:'):
                # H: Handlers=event4 js0  — the event number is in a token
                # like "Handlers=event4", not standalone, so match by regex.
                m = re.search(r'\bevent(\d+)\b', line)
                if m:
                    event = f'/dev/input/event{m.group(1)}'
            if line.startswith('S:'):
                m = re.search(r'/input/input(\d+)', line)
                if m:
                    idx = int(m.group(1))
        if event:
            return event, idx
    return None, None


def find_led(channel: str, input_idx: int | None) -> Path | None:
    """Find /sys/class/leds/inputN:<channel>/brightness for the DS4."""
    candidates = []
    if input_idx is not None:
        p = Path(f'/sys/class/leds/input{input_idx}:{channel}/brightness')
        if p.exists():
            candidates.append(p)
    if not candidates:
        for p in glob.glob(f'/sys/class/leds/input*:{channel}/brightness'):
            candidates.append(Path(p))
    return candidates[0] if candidates else None


class JoyFeedback(Node):
    def __init__(self):
        super().__init__('joy_feedback')

        self.declare_parameter('device_path', '')
        self.declare_parameter('input_index', -1)
        device_param = self.get_parameter('device_path').get_parameter_value().string_value
        idx_param = self.get_parameter('input_index').get_parameter_value().integer_value

        self.dev = None
        self.current_effect = None
        self.leds: dict[str, Path] = {}

        if device_param:
            event_path, input_idx = device_param, (idx_param if idx_param >= 0 else None)
        else:
            event_path, input_idx = find_ds4()
            if idx_param >= 0:
                input_idx = idx_param

        self._open_evdev(event_path)
        self._open_leds(input_idx)

        self.create_subscription(
            Float32MultiArray, '/joy/rumble', self.on_rumble, 10)
        self.create_subscription(
            ColorRGBA, '/joy/led', self.on_led, 10)

        self.get_logger().info(
            'feedback ready — rumble: {}, leds: {}'.format(
                'on' if self.dev else 'OFF',
                ','.join(k for k in ('red','green','blue') if k in self.leds) or 'NONE'))

    # ── device discovery / open ────────────────────────────────────────────
    def _open_evdev(self, path: str | None):
        if not path:
            self.get_logger().warn('no DS4 evdev found (rumble disabled)')
            return
        if evdev is None:
            self.get_logger().warn('python3-evdev not installed (rumble disabled)')
            return
        try:
            self.dev = evdev.InputDevice(path)
        except (PermissionError, FileNotFoundError, OSError) as e:
            self.get_logger().warn(f'cannot open {path}: {e}')
            return
        caps = self.dev.capabilities().get(evdev.ecodes.EV_FF, [])
        if evdev.ecodes.FF_RUMBLE not in caps:
            self.get_logger().warn(f'{path}: no FF_RUMBLE in caps {caps}')
            self.dev = None

    def _open_leds(self, input_idx: int | None):
        for ch in ('red', 'green', 'blue'):
            p = find_led(ch, input_idx)
            if p and os.access(p, os.W_OK):
                self.leds[ch] = p
            elif p:
                self.get_logger().warn(
                    f'{p} not writable — install etc/99-picar.rules (`make install-uarts`)')

    def _rediscover(self):
        """Re-run device discovery after a write error (controller may have
        reconnected with a different input index). Closes the stale evdev
        and lets the next call re-open everything fresh."""
        if self.dev is not None:
            try: self.dev.close()
            except OSError: pass
        self.dev = None
        self.current_effect = None
        self.leds.clear()
        event_path, input_idx = find_ds4()
        self._open_evdev(event_path)
        self._open_leds(input_idx)
        if self.dev or self.leds:
            self.get_logger().info(f'rediscovered → rumble: {"on" if self.dev else "OFF"}, '
                                   f'leds: {",".join(self.leds) or "NONE"}')

    # ── handlers ───────────────────────────────────────────────────────────
    def on_rumble(self, msg: Float32MultiArray):
        if self.dev is None:
            self._rediscover()
            if self.dev is None:
                return
        data = list(msg.data) + [0.0, 0.0, 0.5]
        strong = _clip01(data[0])
        weak   = _clip01(data[1])
        dur_s  = max(0.05, min(5.0, data[2]))

        # Drop any prior effect so the next replay starts cleanly.
        if self.current_effect is not None:
            try: self.dev.erase_effect(self.current_effect)
            except OSError: pass
            self.current_effect = None

        if strong == 0.0 and weak == 0.0:
            return    # explicit stop — old effect already erased

        effect = evdev.ff.Effect(
            evdev.ecodes.FF_RUMBLE, -1, 0,
            evdev.ff.Trigger(0, 0),
            evdev.ff.Replay(int(dur_s * 1000), 0),
            evdev.ff.EffectType(
                ff_rumble_effect=evdev.ff.Rumble(
                    strong_magnitude=int(strong * 0xFFFF),
                    weak_magnitude=int(weak   * 0xFFFF))))
        try:
            self.current_effect = self.dev.upload_effect(effect)
            self.dev.write(evdev.ecodes.EV_FF, self.current_effect, 1)
        except OSError as e:
            self.get_logger().warn(f'rumble write failed: {e} — rediscovering')
            self._rediscover()

    def on_led(self, msg: ColorRGBA):
        # Trigger rediscovery once per message if any path is stale; a single
        # ENOENT from a reconnect breaks all three otherwise.
        for ch, val in (('red', msg.r), ('green', msg.g), ('blue', msg.b)):
            p = self.leds.get(ch)
            if p is None or not p.exists():
                self._rediscover()
                p = self.leds.get(ch)
                if p is None:
                    return
            try:
                p.write_text(str(int(_clip01(val) * 255)))
            except OSError as e:
                self.get_logger().warn(f'led {ch} write failed: {e} — rediscovering')
                self._rediscover()
                return


def _clip01(v: float) -> float:
    if v < 0.0: return 0.0
    if v > 1.0: return 1.0
    return v


def main():
    rclpy.init()
    node = JoyFeedback()
    try:
        rclpy.spin(node)
    finally:
        # On shutdown, snuff the LEDs so the controller doesn't stay glowing.
        for p in node.leds.values():
            try: p.write_text('0')
            except OSError: pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
