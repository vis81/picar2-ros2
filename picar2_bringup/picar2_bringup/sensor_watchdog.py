#!/usr/bin/python3
"""Keep the USB sensors alive across an unplug and replug.

The LD19 and the SEN0628 are lifecycle nodes that open their serial port
in on_configure and close it in on_cleanup. When the cable comes out the
device node (/dev/ldlidar, /dev/sen0628 - udev symlinks) disappears, the
driver keeps a dead file descriptor and logs timeouts ten times a second
until someone restarts the whole bringup; plugging the cable back in
changes nothing, because nobody re-opens the port.

This node watches the device paths once a second and drives the drivers'
lifecycle from that:

  device appears   -> cleanup (if needed), configure, activate
  device vanishes  -> deactivate, cleanup   (port closed, no timeout spam,
                                             no CPU spent polling a corpse)
  device present but node not active (every check_period) -> same as
                      "appears": covers a driver that failed to open at
                      boot, or a transition that failed half way

It also does the initial bring-up, so the sensors need neither a
lifecycle manager nor launch-time transition events: a sensor that is
unplugged at boot simply comes up when it is plugged in.

Every transition is a service call and takes as long as the driver's
on_configure (the SEN0628 waits up to 5 s for a first frame), so the work
runs on a thread per sensor and the timer never blocks. One wake-up per
second is all this costs while nothing changes.
"""

import os
import threading
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

SETTLE_S = 2.0          # udev and the device's own boot after it appears


class Sensor:
    def __init__(self, node: Node, name: str, device: str, lc_node: str):
        self.name = name
        self.device = device
        self.lc_node = lc_node
        self.present = None                     # unknown until first check
        self.appeared_at = None
        self.last_check = 0.0
        self.last_attempt = 0.0
        self.lock = threading.Lock()            # one sequence at a time
        self.get_state = node.create_client(GetState, f'/{lc_node}/get_state')
        self.change = node.create_client(ChangeState, f'/{lc_node}/change_state')


class SensorWatchdog(Node):
    def __init__(self):
        super().__init__('sensor_watchdog')
        self.declare_parameter('names', ['ld19', 'sen0628'])
        self.declare_parameter('devices', ['/dev/ldlidar', '/dev/sen0628'])
        # An empty node name disables that entry (a launch-time condition).
        self.declare_parameter('nodes', ['lidar_node', 'tof_imager'])
        self.declare_parameter('period', 1.0)
        self.declare_parameter('check_period', 10.0)   # state audit
        self.declare_parameter('retry', 15.0)          # min gap between attempts
        names = list(self.get_parameter('names').value)
        devices = list(self.get_parameter('devices').value)
        nodes = list(self.get_parameter('nodes').value)
        self.check_period = float(self.get_parameter('check_period').value)
        self.retry = float(self.get_parameter('retry').value)
        self.sensors = [Sensor(self, n, d, l)
                        for n, d, l in zip(names, devices, nodes) if l]
        for s in self.sensors:
            self.get_logger().info(f'{s.name}: {s.device} -> /{s.lc_node}')
        self.create_timer(float(self.get_parameter('period').value), self.tick)

    # ── lifecycle plumbing (worker threads only; never block the executor) ──
    def _call(self, client, req, timeout):
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                return None
            done = threading.Event()
            fut = client.call_async(req)
            fut.add_done_callback(lambda _f: done.set())
            if not done.wait(timeout):
                return None
            return fut.result()
        except Exception:                        # noqa: BLE001 - incl. shutdown mid-call
            return None

    def state_of(self, s: Sensor):
        res = self._call(s.get_state, GetState.Request(), 5.0)
        return None if res is None else res.current_state.id

    def transition(self, s: Sensor, tid: int, label: str, timeout=20.0) -> bool:
        res = self._call(s.change, ChangeState.Request(transition=Transition(id=tid)), timeout)
        ok = bool(res is not None and res.success)
        self.get_logger().info(f'{s.name}: {label} {"ok" if ok else "FAILED"}')
        return ok

    def bring_up(self, s: Sensor):
        """From whatever state to active, re-opening the port on the way."""
        with s.lock:
            s.last_attempt = time.monotonic()
            st = self.state_of(s)
            if st is None:
                self.get_logger().warn(f'{s.name}: /{s.lc_node} not answering')
                return
            if st == State.PRIMARY_STATE_ACTIVE:
                # Active but the device was just replugged: the port it
                # holds is the old one. Cycle it.
                if not self.transition(s, Transition.TRANSITION_DEACTIVATE, 'deactivate'):
                    return
                st = State.PRIMARY_STATE_INACTIVE
            if st == State.PRIMARY_STATE_INACTIVE:
                if not self.transition(s, Transition.TRANSITION_CLEANUP, 'cleanup'):
                    return
                st = State.PRIMARY_STATE_UNCONFIGURED
            if st != State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().warn(f'{s.name}: in state {st}, leaving it alone')
                return
            if not self.transition(s, Transition.TRANSITION_CONFIGURE, 'configure'):
                return                           # port not there yet; retry later
            self.transition(s, Transition.TRANSITION_ACTIVATE, 'activate')

    def take_down(self, s: Sensor):
        """Close the port of a device that is gone."""
        with s.lock:
            st = self.state_of(s)
            if st == State.PRIMARY_STATE_ACTIVE:
                if not self.transition(s, Transition.TRANSITION_DEACTIVATE, 'deactivate'):
                    return
                st = State.PRIMARY_STATE_INACTIVE
            if st == State.PRIMARY_STATE_INACTIVE:
                self.transition(s, Transition.TRANSITION_CLEANUP, 'cleanup')

    def _spawn(self, target, s: Sensor):
        if s.lock.locked():
            return                               # a sequence is running
        threading.Thread(target=target, args=(s,), daemon=True).start()

    # ── the 1 Hz tick ──────────────────────────────────────────────────────
    def tick(self):
        now = time.monotonic()
        for s in self.sensors:
            present = os.path.exists(s.device)
            if present != s.present:
                s.present = present
                if present:
                    self.get_logger().info(f'{s.name}: {s.device} appeared')
                    s.appeared_at = now
                    s.last_check = now          # the audit waits its turn
                else:
                    self.get_logger().warn(f'{s.name}: {s.device} gone - closing the port')
                    s.appeared_at = None
                    self._spawn(self.take_down, s)
                continue
            if not present:
                continue
            if s.appeared_at is not None and now - s.appeared_at >= SETTLE_S:
                s.appeared_at = None
                s.last_check = now
                self._spawn(self.bring_up, s)
                continue
            if now - s.last_check >= self.check_period and now - s.last_attempt >= self.retry:
                s.last_check = now
                self._spawn(self.audit, s)

    def audit(self, s: Sensor):
        """Device present: is the driver active? If not, bring it up."""
        if s.lock.locked():
            return
        st = self.state_of(s)
        if st is None or st == State.PRIMARY_STATE_ACTIVE:
            return
        self.get_logger().info(f'{s.name}: device present, driver in state {st} - bringing it up')
        self.bring_up(s)


def main():
    rclpy.init()
    node = SensorWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
