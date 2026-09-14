#!/usr/bin/python3
"""CPU load of the robot computer, on /diagnostics.

Once a second: total and per-core load, load average, clock, SoC
temperature and the Pi firmware's throttling flags, plus the top-N
processes by CPU with the ROS node name where there is one. Standard
DiagnosticArray, so Foxglove / rqt_robot_monitor show it as is and a bag
that records /diagnostics carries the load profile of the run next to
the data. psutil reads /proc directly; nothing is shelled out.

Levels: WARN when idle has stayed under warn_idle for hold seconds, ERROR
under error_idle - a one-second spike is normal, five seconds of it is
the controller missing cycles. Under-voltage or thermal throttling is an
ERROR on its own: the Pi silently drops the clock and everything gets
slower for no visible reason.
"""

import os
import re

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

THERMAL = '/sys/class/thermal/thermal_zone0/temp'
THROTTLED = '/sys/devices/platform/soc/soc:firmware/get_throttled'
FREQ = '/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq'
# vcgencmd get_throttled bits (Raspberry Pi firmware).
THROTTLE_BITS = {
    0: 'under-voltage', 1: 'freq capped', 2: 'throttled', 3: 'soft temp limit',
    16: 'under-voltage occurred', 17: 'freq cap occurred',
    18: 'throttling occurred', 19: 'soft temp limit occurred',
}
NODE_ARG = re.compile(r'__node:=(\S+)')


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def process_label(p):
    """Something a person recognises: the ROS node name, the script, or
    the executable."""
    try:
        cmd = p.cmdline()
    except (psutil.Error, OSError):
        cmd = []
    joined = ' '.join(cmd)
    m = NODE_ARG.search(joined)
    if m:
        return m.group(1)
    for a in cmd[:3]:
        base = os.path.basename(a)
        if base.endswith('.py'):
            # The scripts of this workspace: server.py is the web UI.
            return 'webui' if base == 'server.py' else base[:-3]
    # "python3 /opt/ros/.../ros2 bag record ..." -> "ros2 bag"
    if len(cmd) >= 3 and os.path.basename(cmd[1]) == 'ros2':
        return 'ros2 ' + cmd[2]
    if cmd:
        return os.path.basename(cmd[0])
    try:
        return p.name()
    except psutil.Error:
        return str(p.pid)


class CpuMonitor(Node):
    def __init__(self):
        super().__init__('cpu_monitor')
        self.declare_parameter('period', 1.0)
        self.declare_parameter('top_n', 10)
        self.declare_parameter('warn_idle', 20.0)     # % of all cores
        self.declare_parameter('error_idle', 10.0)
        self.declare_parameter('hold', 5.0)           # s under threshold
        self.declare_parameter('warn_temp', 75.0)     # deg C
        self.period = float(self.get_parameter('period').value)
        self.top_n = int(self.get_parameter('top_n').value)
        self.warn_idle = float(self.get_parameter('warn_idle').value)
        self.error_idle = float(self.get_parameter('error_idle').value)
        self.hold = float(self.get_parameter('hold').value)
        self.warn_temp = float(self.get_parameter('warn_temp').value)

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.procs: dict[int, psutil.Process] = {}
        self.low_since = None
        self.ncpu = psutil.cpu_count() or 1
        psutil.cpu_percent(percpu=True)               # prime the delta
        self.create_timer(self.period, self.tick)

    def processes(self):
        """(label, cpu %, rss MB, D-state) per process, cpu % since the last
        tick, as top does. Process objects are kept between ticks because
        psutil measures against each object's previous call."""
        seen = set()
        out = []
        dstate = 0
        for p in psutil.process_iter():
            seen.add(p.pid)
            proc = self.procs.get(p.pid)
            if proc is None:
                proc = self.procs[p.pid] = p
                try:
                    proc.cpu_percent(None)            # prime; 0 this tick
                except psutil.Error:
                    continue
                continue
            try:
                with proc.oneshot():
                    pct = proc.cpu_percent(None)
                    rss = proc.memory_info().rss / 1e6
                    if proc.status() == psutil.STATUS_DISK_SLEEP:
                        dstate += 1
            except psutil.Error:
                continue
            if pct > 0.0:
                out.append((process_label(proc), pct, rss, proc.pid))
        for pid in list(self.procs):
            if pid not in seen:
                del self.procs[pid]
        out.sort(key=lambda r: -r[1])
        return out, dstate

    def tick(self):
        per_core = psutil.cpu_percent(percpu=True)
        total = sum(per_core) / len(per_core)
        idle = 100.0 - total
        load1, load5, load15 = os.getloadavg()
        temp = _read(THERMAL)
        temp = float(temp) / 1000.0 if temp else None
        freq = _read(FREQ)
        freq = int(freq) // 1000 if freq else None
        thr = _read(THROTTLED)
        thr = int(thr, 16) if thr else 0
        active = [n for b, n in THROTTLE_BITS.items() if thr >> b & 1 and b < 16]
        procs, dstate = self.processes()
        now = self.get_clock().now()

        # Level: sustained, not instantaneous.
        if idle < self.warn_idle:
            self.low_since = self.low_since or now
            low_for = (now - self.low_since).nanoseconds * 1e-9
        else:
            self.low_since = None
            low_for = 0.0
        level = DiagnosticStatus.OK
        why = []
        if low_for >= self.hold:
            level = (DiagnosticStatus.ERROR if idle < self.error_idle
                     else DiagnosticStatus.WARN)
            why.append(f'idle {idle:.0f}% for {low_for:.0f} s')
        if active:
            level = DiagnosticStatus.ERROR
            why.append(', '.join(active))
        if temp is not None and temp >= self.warn_temp:
            level = max(level, DiagnosticStatus.WARN)
            why.append(f'{temp:.0f} C')

        cpu = DiagnosticStatus(name='cpu', hardware_id='pi', level=level)
        cpu.message = (f'{total:.0f}% used, idle {idle:.0f}%, load {load1:.1f}'
                       + (f', {temp:.0f} C' if temp is not None else '')
                       + (' - ' + '; '.join(why) if why else ''))
        kv = [('total_pct', f'{total:.1f}'), ('idle_pct', f'{idle:.1f}'),
              ('cores', str(self.ncpu)),
              ('per_core_pct', ' '.join(f'{c:.0f}' for c in per_core)),
              ('load_1_5_15', f'{load1:.2f} {load5:.2f} {load15:.2f}'),
              ('d_state', str(dstate))]
        if freq is not None:
            kv.append(('freq_mhz', str(freq)))
        if temp is not None:
            kv.append(('temp_c', f'{temp:.1f}'))
        kv.append(('throttled', f'0x{thr:x}'
                   + (' ' + ', '.join(active) if active else '')))
        cpu.values = [KeyValue(key=k, value=v) for k, v in kv]

        top = DiagnosticStatus(name='cpu/top', hardware_id='pi',
                               level=DiagnosticStatus.OK)
        head = procs[:self.top_n]
        top.message = ', '.join(f'{n} {c:.0f}%' for n, c, _, _ in head[:4])
        # Values are % of ONE core, as top shows them; total_pct above is of
        # all of them.
        top.values = [KeyValue(key=n, value=f'{c:.1f}% {r:.0f}MB pid {pid}')
                      for n, c, r, pid in head]

        msg = DiagnosticArray()
        msg.header.stamp = now.to_msg()
        msg.status = [cpu, top]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CpuMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
