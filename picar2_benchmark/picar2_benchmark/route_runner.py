#!/usr/bin/env python3
"""Run one waypoint-route trial and emit a JSON result.

A route is not a goal. Nav2 can report SUCCEEDED on a route it drove badly —
that is not hypothetical, it is what a rolling NavigateThroughPoses window did
on the real robot: the behaviour tree trims goals inside RemovePassedGoals'
radius, a window trimmed empty reports success, and a three-waypoint route
"completed" with the robot 0.75 m from the nearest waypoint.

So nothing here trusts the action result to decide whether a waypoint was
reached. Ground truth says where the robot actually went, and a waypoint counts
as passed only if the robot came within the capture radius of it, in order. The
action result is recorded, but it is evidence about Nav2, not about the route.

Both driving modes ship in the web UI and both are measured here:

  stop  one NavigateToPose per waypoint, Nav2's goal checker deciding arrival.
  flow  a rolling NavigateThroughPoses window, so the controller sees one
        continuous path and never decelerates for an intermediate waypoint.

Comparing them is the point of the scenario. On the robot, flow was 15% faster
per lap and four times more accurate, which is the opposite of the trade one
expects — worth a regression test precisely because it is unintuitive.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient

from . import gates, map_gen, spec, world_gen
from .recorder import Recorder
from .runner import (BAG_TOPICS, Stack, _hlog, _wait_topic, reset_pose,
                     wait_for_quiet_domain)

# A frozen /clock means the simulator died; watch for that rather than guessing
# a wall-clock budget from rtf. Same value and reasoning as runner.py.
SIM_CLOCK_TIMEOUT = 60.0


class RouteRunner:
    """Drives a waypoint list and measures what the robot actually did."""

    def __init__(self, ctx: gates.GateContext, sc, cfg: dict, rec: Recorder):
        self.ctx = ctx
        self.sc = sc
        self.rec = rec
        self.wps = sc.route_waypoints
        self.loop = bool(cfg.get('loop', False))
        self.flow = bool(cfg.get('flow', False))
        self.laps = int(cfg.get('laps', 1))
        self.capture = float(cfg.get('capture_radius_m', 0.45))
        self.recede = float(cfg.get('recede_m', 0.08))
        self.window = int(cfg.get('window', 3))
        self.timeout_s = float(cfg.get('timeout_s', sc.timeout_s))

        self.to_pose = ActionClient(ctx, NavigateToPose, 'navigate_to_pose')
        self.through = ActionClient(ctx, NavigateThroughPoses,
                                    'navigate_through_poses')
        self.idx = 0
        self.passed = 0
        self.path_m = 0.0
        self._last = None
        self._gt_seq = -1
        self._min_d = None
        self._sent_idx = None
        self._handle = None
        # One entry per visit: how close the robot actually came. This is the
        # measurement the whole trial exists to produce.
        self.approaches: list[tuple[int, float]] = []
        self.arrivals: list[float] = []          # sim time of each pass
        self.statuses: list[str] = []            # what Nav2 said, per goal

    # ── geometry ─────────────────────────────────────────────────────────
    def _pose_stamped(self, p) -> PoseStamped:
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.ctx.get_clock().now().to_msg()
        m.pose.position.x = float(p.x)
        m.pose.position.y = float(p.y)
        m.pose.orientation.z = math.sin(float(p.yaw) / 2.0)
        m.pose.orientation.w = math.cos(float(p.yaw) / 2.0)
        return m

    def _sim_now(self) -> float:
        return self.ctx.get_clock().now().nanoseconds * 1e-9

    def _accumulate(self):
        """Ground-truth path length, sampled once per new pose."""
        if self.ctx.gt_seq == self._gt_seq:
            return
        self._gt_seq = self.ctx.gt_seq
        x, y, yaw = self.ctx.gt_pose()
        if self._last is not None:
            self.path_m += math.dist((x, y), self._last)
        self._last = (x, y)
        self.rec.sample_pose(x, y, yaw)

    def _target(self):
        return self.wps[self.idx % len(self.wps)]

    def _dist_to_target(self) -> float:
        x, y, _ = self.ctx.gt_pose()
        t = self._target()
        return math.dist((x, y), (t.x, t.y))

    # ── driving ──────────────────────────────────────────────────────────
    def _window_poses(self):
        n = len(self.wps)
        size = min(self.window, n)
        if self.loop:
            return [self.wps[(self.idx + i) % n] for i in range(size)]
        return self.wps[self.idx:self.idx + size]

    def _send(self) -> bool:
        """Send the next goal or window. False if the action refused it."""
        if self.flow:
            poses = self._window_poses()
            if not poses:
                return False
            g = NavigateThroughPoses.Goal(
                poses=[self._pose_stamped(p) for p in poses])
            fut = self.through.send_goal_async(g)
            self._sent_idx = self.idx
        else:
            g = NavigateToPose.Goal()
            g.pose = self._pose_stamped(self._target())
            fut = self.to_pose.send_goal_async(g)
        end = time.time() + 15
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.ctx, timeout_sec=0.05)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        self._handle = gh
        self._result = gh.get_result_async()
        return True

    def _cancel(self):
        if self._handle is not None:
            try:
                self._handle.cancel_goal_async()
            except Exception:                                 # noqa: BLE001
                pass
            self._handle = None

    def _pass_current(self, d: float):
        """Record a waypoint as passed and move on."""
        self.approaches.append((self.idx % len(self.wps), round(d, 3)))
        self.arrivals.append(self._sim_now())
        self.passed += 1
        self._min_d = None
        self.idx += 1

    # ── the trial ────────────────────────────────────────────────────────
    def run(self) -> dict:
        client = self.through if self.flow else self.to_pose
        name = 'navigate_through_poses' if self.flow else 'navigate_to_pose'
        if not client.wait_for_server(timeout_sec=30.0):
            return {'outcome': 'SIM_DEGRADED', 'detail': f'{name} absent'}

        n = len(self.wps)
        target_passes = n * self.laps
        if not self._send():
            return {'outcome': 'SIM_DEGRADED', 'detail': 'goal not accepted'}

        t0 = self._sim_now()
        t0_wall = time.time()
        last_sim, last_sim_wall = t0, t0_wall
        self._last = None
        outcome = None

        while self.passed < target_passes:
            rclpy.spin_once(self.ctx, timeout_sec=0.05)
            self._accumulate()

            # Arrival is decided here, from ground truth, never from the action
            # result. In flow mode the waypoint is counted at its closest
            # approach: track the minimum distance and confirm once the robot
            # is moving away again, which puts the count at the nearest point
            # rather than at the edge of some radius.
            d = self._dist_to_target()
            if self._min_d is None or d < self._min_d:
                self._min_d = d
            last_of_oneshot = (not self.loop) and self.idx == n - 1
            if self.flow and not last_of_oneshot:
                if self._min_d < self.capture and d > self._min_d + self.recede:
                    self._pass_current(self._min_d)
                    consumed = self.idx - (self._sent_idx or 0)
                    if consumed >= self.window - 1:
                        self._cancel()
                        if not self._send():
                            outcome = 'SIM_DEGRADED'
                            break
                    continue

            if self._result.done():
                st = self._result.result().status
                self.statuses.append(
                    {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                     GoalStatus.STATUS_CANCELED: 'CANCELED',
                     GoalStatus.STATUS_ABORTED: 'ABORTED'}.get(st, f'STATUS_{st}'))
                if st == GoalStatus.STATUS_SUCCEEDED:
                    # Nav2 says it arrived. Whether it really did is the
                    # min-distance number, recorded either way — a pass beyond
                    # the capture radius is exactly the defect this exists to
                    # catch, so it is counted and flagged, not discarded.
                    self._pass_current(self._min_d if self._min_d is not None else d)
                else:
                    outcome = 'ABORTED'
                    break
                if self.passed >= target_passes:
                    break
                if not self._send():
                    outcome = 'SIM_DEGRADED'
                    break
                continue

            now = self._sim_now()
            if now > last_sim:
                last_sim, last_sim_wall = now, time.time()
            elif time.time() - last_sim_wall > SIM_CLOCK_TIMEOUT:
                self._cancel()
                return {'outcome': 'SIM_DEGRADED',
                        'detail': f'sim clock frozen for {SIM_CLOCK_TIMEOUT:.0f}s '
                                  f'of wall time; the simulator died mid-trial'}
            if now - t0 >= self.timeout_s:
                outcome = 'TIMEOUT'
                break

        elapsed = self._sim_now() - t0
        wall = time.time() - t0_wall
        self._cancel()
        if outcome is None:
            outcome = 'SUCCEEDED'

        return {'outcome': outcome, **self.metrics(elapsed, wall, target_passes)}

    def metrics(self, elapsed: float, wall: float, target: int) -> dict:
        n = len(self.wps)
        d = [a for _, a in self.approaches]
        # A waypoint counted but passed beyond the capture radius means the
        # count came from Nav2 claiming success rather than from the robot
        # actually getting there. Reported separately because it is the
        # difference between a route driven and a route merely reported.
        missed = [(i, a) for i, a in self.approaches if a > self.capture]
        laps = []
        if self.loop and len(self.arrivals) > n:
            laps = [self.arrivals[i + n] - self.arrivals[i]
                    for i in range(len(self.arrivals) - n)]
        # Legs, not just laps: a lap time hides which corner cost the seconds,
        # and the per-leg splits are what a hardware run is compared against.
        # arrivals[k] is the pass of waypoint k % n, so the gap to arrivals[k+1]
        # is the leg out of that waypoint.
        legs: dict[str, list[float]] = {}
        for k in range(len(self.arrivals) - 1):
            j = k % n
            key = f'wp{j}->wp{(j + 1) % n}'
            legs.setdefault(key, []).append(self.arrivals[k + 1] - self.arrivals[k])
        per_wp: dict[str, float] = {}
        for i in range(n):
            v = [a for j, a in self.approaches if j == i]
            if v:
                per_wp[f'wp{i}'] = round(max(v), 3)
        return {
            'mode': 'flow' if self.flow else 'stop',
            'waypoints': n,
            'target_passes': target,
            'passes': self.passed,
            'completed': self.passed >= target,
            'time_s': round(elapsed, 2),
            'wall_time_s': round(wall, 2),
            'rtf_achieved': round(elapsed / wall, 3) if wall > 0 else None,
            'gt_path_m': round(self.path_m, 2),
            'mean_speed_mps': round(self.path_m / max(elapsed, 1e-6), 3),
            'approach_mean_m': round(sum(d) / len(d), 3) if d else None,
            'approach_max_m': round(max(d), 3) if d else None,
            'approach_worst_wp': per_wp,
            'missed_waypoints': len(missed),
            'missed_detail': [{'wp': i, 'closest_m': a} for i, a in missed],
            'lap_time_s': round(sum(laps) / len(laps), 2) if laps else None,
            'lap_time_min_s': round(min(laps), 2) if laps else None,
            'lap_time_max_s': round(max(laps), 2) if laps else None,
            'leg_time_s': {k: round(sum(v) / len(v), 2) for k, v in legs.items()},
            'leg_times_all_s': {k: [round(x, 2) for x in v] for k, v in legs.items()},
            'nav_statuses': self.statuses,
        }


def run_trial(scenario: str, out_dir: Path, gen_dir: Path, mode: str = 'flow',
              keep_up: bool = False, sensor_noise: float = 1.0,
              bag: bool = True, overlay: str = '') -> dict:
    sc = spec.load(scenario)
    if not sc.route:
        raise SystemExit(f'{sc.name} is not a route scenario (no `route:` block)')
    cfg = dict(sc.route)
    cfg['flow'] = (mode == 'flow')

    gen = gen_dir / sc.name
    gen.mkdir(parents=True, exist_ok=True)
    world = gen / f'{sc.name}.sdf'
    world.write_text(world_gen.to_sdf(sc))
    _, map_yaml = map_gen.write(sc, gen)

    logs = out_dir / 'logs' / f'{sc.name}_{mode}_{int(time.time())}'
    result: dict = {'scenario': sc.name, 'route_mode': mode,
                    'localisation': 'ground_truth', 'sensor_noise': sensor_noise,
                    'config': Path(overlay).stem if overlay else 'baseline'}
    # A missing overlay is not an error to launch_ros: it warns "Parameter file
    # path is not a file" and runs the baseline, so the trial would record a
    # config it never used. Same guard as runner.py, for the same reason.
    if overlay and not Path(overlay).is_file():
        return {**result, 'outcome': 'RUNNER_ERROR',
                'detail': f'overlay file not found: {overlay} - if you just '
                          f'added it, rebuild the package so it installs'}
    busy = wait_for_quiet_domain()
    if busy:
        return {**result, 'outcome': 'SIM_DEGRADED',
                'detail': f'domain already has nodes ({busy})'}

    import os
    os.environ['ROS_LOG_DIR'] = str(logs / 'ros')
    _hlog(logs, f'route trial start scenario={sc.name} mode={mode} '
                f'waypoints={len(sc.route_waypoints)} laps={cfg.get("laps", 1)}')
    stack = Stack()
    try:
        stack.launch([
            'ros2', 'launch', 'picar2_bringup', 'sim.launch.py', 'headless:=true',
            'lidar:=ld19', f'world:={world}', f'spawn_x:={sc.start.x}',
            f'spawn_y:={sc.start.y}', f'spawn_yaw:={sc.start.yaw}',
            f'sensor_noise:={sensor_noise}'], logs / 'sim.log')
        if not _wait_topic('/lidar_node/scan', 150):
            raise gates.GateFailure('simulator never produced a scan')

        # ground_truth only, and deliberately: every metric here is a distance
        # between where the robot really was and where a waypoint really is.
        # Under slam the map frame is anchored at the start pose, so world
        # waypoints would mean something else and every approach number would
        # be measuring localisation error instead of driving accuracy.
        stack.launch([
            'ros2', 'launch', 'picar2_benchmark', 'benchmark_localization.launch.py',
            'mode:=ground_truth', f'map_yaml:={map_yaml}', 'use_sim_time:=true'],
            logs / 'loc.log')
        stack.launch([
            'ros2', 'launch', 'picar2_bringup', 'nav2.launch.py',
            'use_sim_time:=true']
            + ([f'params_overlay:={overlay}'] if overlay else []),
            logs / 'nav2.log')
        if bag:
            stack.launch(['ros2', 'bag', 'record', '-o', str(logs / 'bag'),
                          '--include-hidden-topics', '--max-cache-size', '10485760',
                          *BAG_TOPICS], logs / 'bag.log')

        rclpy.init()
        ctx = gates.GateContext()
        gates.wait_for_ground_truth(ctx)
        gates.gate_spawn_pose(ctx, sc.start)
        gates.gate_settle(ctx)
        gates.gate_motion(ctx)
        reset_pose(sc)
        ctx.spin(2.0)
        gates.gate_settle(ctx)
        gates.gate_spawn_pose(ctx, sc.start)
        # The waypoints, not sc.goal: a route scenario's goal is a placeholder
        # kept only so the loader has one, and gating on it would check that
        # the costmap covers a pose the trial never visits.
        gates.gate_costmap(ctx, sc, 'ground_truth', targets=sc.route_waypoints)
        _hlog(logs, 'gates passed')
        result['gates'] = 'passed'

        rec = Recorder(ctx, sc.all_boxes, 'ground_truth')
        ctx.spin(1.0)
        out = RouteRunner(ctx, sc, cfg, rec).run()
        result.update(out)
        result['metrics'] = rec.metrics()
        rec.dump_trajectory(logs / 'trajectory.json')
        result['trajectory'] = str(logs / 'trajectory.json')
        if bag:
            result['bag'] = str(logs / 'bag')
        _hlog(logs, f"outcome={result.get('outcome')} "
                    f"passes={result.get('passes')}/{result.get('target_passes')} "
                    f"missed={result.get('missed_waypoints')} "
                    f"approach_max={result.get('approach_max_m')}")
    except gates.GateFailure as e:
        _hlog(logs, f'GATE FAILED: {e}')
        result.update({'outcome': 'SIM_DEGRADED', 'detail': str(e)})
    except Exception as e:                                   # noqa: BLE001
        _hlog(logs, f'RUNNER ERROR: {e!r}')
        result.update({'outcome': 'RUNNER_ERROR', 'detail': repr(e)})
    finally:
        try:
            rclpy.shutdown()
        except Exception:                                    # noqa: BLE001
            pass
        if not keep_up:
            stack.teardown()
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Run one waypoint-route trial.')
    ap.add_argument('scenario')
    ap.add_argument('--route-mode', default='flow', choices=['flow', 'stop'],
                    help='flow: rolling NavigateThroughPoses window. '
                         'stop: one NavigateToPose per waypoint.')
    ap.add_argument('-o', '--out', default='/tmp/picar2_bench/route')
    ap.add_argument('--sensor-noise', type=float, default=1.0)
    ap.add_argument('--no-bag', dest='bag', action='store_false')
    ap.add_argument('--keep-up', action='store_true')
    ap.add_argument('--overlay', default='',
                    help='nav2 params overlay layered over nav2.yaml, e.g. the '
                         'installed configs/speed_060.yaml to drive the route at '
                         'the speed the robot was measured at')
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res = run_trial(a.scenario, out, Path('/tmp/picar2_bench'), a.route_mode,
                    a.keep_up, a.sensor_noise, a.bag, a.overlay)
    exp = spec.check_expectations(spec.load(a.scenario), res)
    if exp:
        res['expectations'] = exp
    name = (f"{res['scenario']}_route_{a.route_mode}_{res['config']}_"
            f"n{a.sensor_noise}_{int(time.time())}.json")
    (out / name).write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    # A route that completed but missed waypoints is a failure, however
    # cheerfully Nav2 reported it. That distinction is the whole point.
    if exp and not exp['passed']:
        for c in exp['checks']:
            if not c['ok']:
                print(f"  EXPECTATION FAILED  {c['check']}: want {c['want']}, "
                      f"got {c['got']}")
    ok = (res.get('outcome') == 'SUCCEEDED'
          and not res.get('missed_waypoints')
          and (not exp or exp['passed']))
    return 0 if ok else 1
