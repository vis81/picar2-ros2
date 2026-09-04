#!/usr/bin/env python3
"""Run one benchmark trial and emit a JSON result.

Sequence: generate -> launch sim -> launch localisation -> launch nav2 -> gates
-> send one goal -> measure -> tear down.

A trial that fails a gate returns outcome SIM_DEGRADED and is meant to be
discarded by the caller rather than averaged in.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from . import attribution, gates, map_gen, spec, world_gen
from .recorder import Recorder

# Kill only what this runner started. An earlier version matched process names
# system-wide, which tore down a simulation someone else was running on the same
# machine — every trial began by killing anything that looked like a sim.
class Stack:
    """Process groups this trial launched, and nothing else."""

    def __init__(self):
        self.groups: list[int] = []

    def launch(self, args: list[str], log: Path) -> subprocess.Popen:
        log.parent.mkdir(parents=True, exist_ok=True)
        p = subprocess.Popen(args, stdout=log.open('w'), stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        self.groups.append(os.getpgid(p.pid))
        return p

    def _alive(self) -> list[int]:
        out = []
        for g in self.groups:
            try:
                os.killpg(g, 0)
                out.append(g)
            except (ProcessLookupError, PermissionError):
                pass
        return out

    def teardown(self) -> None:
        """SIGINT for a clean ROS shutdown, then SIGKILL what remains."""
        for g in self.groups:
            try:
                os.killpg(g, signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        for _ in range(20):
            if not self._alive():
                return
            time.sleep(0.5)
        for g in self._alive():
            try:
                os.killpg(g, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(2)


# Nodes that mean a competing robot stack is already running. Viewers and
# tooling (rviz, rqt, tf listeners, teleop) are deliberately not here: watching
# a trial in RViz is a normal thing to want, and blocking it would make the
# guard something people route around.
CONFLICTING_NODES = (
    'bt_navigator', 'controller_server', 'planner_server', 'behavior_server',
    'smoother_server', 'waypoint_follower', 'map_server', 'amcl',
    'cartographer', 'slam_toolbox', 'robot_state_publisher',
    'controller_manager', 'ros_gz', 'gz_', 'gt_localizer', 'ekf',
)


def domain_is_busy() -> str | None:
    """Another robot stack on our domain, or None.

    Two bt_navigators on one domain silently fight over goals: a trial recorded
    ABORTED while its own log said 'Goal succeeded', because the two outcomes
    belonged to different stacks. Refuse rather than produce a number that
    describes neither run.
    """
    r = subprocess.run(['ros2', 'node', 'list'], capture_output=True,
                       text=True, timeout=25)
    clashes = [n for n in r.stdout.split()
               if any(c in n for c in CONFLICTING_NODES)]
    return ', '.join(sorted(set(clashes))[:6]) if clashes else None


def reset_pose(sc, robot='picar2') -> None:
    """Teleport the robot back to the scenario start.

    The motion gate has to actually drive the robot to prove it can move, which
    leaves it ~1 m downrange. Starting the measurement from there would silently
    change the scenario, so put it back exactly where the spec says.
    """
    import math as _m
    qz, qw = _m.sin(sc.start.yaw / 2), _m.cos(sc.start.yaw / 2)
    req = (f'name: "{robot}", position: {{x: {sc.start.x}, y: {sc.start.y}, z: 0.05}}, '
           f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}')
    subprocess.run(
        ['gz', 'service', '-s', f'/world/{spec.WORLD_NAME}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', req],
        capture_output=True, text=True)


def effective_params() -> dict:
    """Read back the parameters that are actually in force.

    A params overlay with a wrong node path, or a parameter whose name does not
    exist, is silently ignored — that is exactly how nav2.yaml ran for months
    with max_allowed_time_to_collision_up_to_goal doing nothing. Recording the
    values the running nodes report makes an inert overlay visible in the data
    instead of quietly producing a duplicate of the baseline.
    """
    wanted = [
        ('/global_costmap/global_costmap', 'inflation_layer.inflation_radius'),
        ('/global_costmap/global_costmap', 'inflation_layer.cost_scaling_factor'),
        ('/controller_server', 'FollowPath.plugin'),
        ('/controller_server', 'controller_frequency'),
        ('/planner_server', 'GridBased.motion_model_for_search'),
    ]
    out = {}
    for node, param in wanted:
        r = subprocess.run(['ros2', 'param', 'get', node, param],
                           capture_output=True, text=True, timeout=20)
        val = r.stdout.strip().split('value is:')[-1].strip()
        out[f'{node.rsplit("/", 1)[-1]}.{param}'] = val or 'unavailable'
    return out


def _wait_topic(topic: str, timeout: float) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        out = subprocess.run(['ros2', 'topic', 'list'],
                             capture_output=True, text=True).stdout
        if topic in out.split():
            return True
        time.sleep(2)
    return False


class GoalRunner:
    """Sends one NavigateToPose goal and measures the outcome.

    Uses the stale-goal defences from scripts/loop_waypoints.py: Jazzy reports a
    preempted goal as ABORTED with empty error info, and NavigateToPose's result
    declares only NONE=0, so the handle identity is the only reliable signal that
    a result belongs to the goal we sent.
    """

    def __init__(self, ctx: gates.GateContext, timeout_s: float, rec: Recorder):
        self.ctx = ctx
        self.timeout_s = timeout_s
        self.rec = rec
        self.client = ActionClient(ctx, NavigateToPose, 'navigate_to_pose')
        self.handle = None
        self.status = None
        self.path_m = 0.0
        self._last = None

    def _pose(self, p) -> PoseStamped:
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.ctx.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(p.x), float(p.y)
        m.pose.orientation.z = math.sin(p.yaw / 2)
        m.pose.orientation.w = math.cos(p.yaw / 2)
        return m

    def _accumulate(self):
        x, y, yaw = self.ctx.gt_pose()
        if self._last is not None:
            self.path_m += math.dist(self._last, (x, y))
        self._last = (x, y)
        self.rec.sample_pose(x, y, yaw)

    def run(self, goal) -> dict:
        if not self.client.wait_for_server(timeout_sec=30.0):
            return {'outcome': 'SIM_DEGRADED', 'detail': 'navigate_to_pose absent'}
        g = NavigateToPose.Goal()
        g.pose = self._pose(goal)
        send = self.client.send_goal_async(g)
        end = time.time() + 15
        while time.time() < end and not send.done():
            rclpy.spin_once(self.ctx, timeout_sec=0.05)
        gh = send.result()
        if gh is None or not gh.accepted:
            return {'outcome': 'SIM_DEGRADED', 'detail': 'goal not accepted'}
        self.handle = gh
        result_fut = gh.get_result_async()

        t0 = time.time()
        self._last = None
        sx, sy, _ = self.ctx.gt_pose()          # true start, for the detour ratio
        while time.time() - t0 < self.timeout_s:
            rclpy.spin_once(self.ctx, timeout_sec=0.05)
            self._accumulate()
            if result_fut.done():
                break
        elapsed = time.time() - t0

        gx, gy, gyaw = self.ctx.gt_pose()
        dist_to_goal = math.dist((gx, gy), (goal.x, goal.y))
        yaw_err = abs(math.atan2(math.sin(gyaw - goal.yaw), math.cos(gyaw - goal.yaw)))

        if not result_fut.done():
            gh.cancel_goal_async()
            outcome = 'TIMEOUT'
        else:
            st = result_fut.result().status
            outcome = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                       GoalStatus.STATUS_CANCELED: 'CANCELED',
                       GoalStatus.STATUS_ABORTED: 'ABORTED'}.get(st, f'STATUS_{st}')
        return {
            'outcome': outcome,
            'time_s': round(elapsed, 2),
            'gt_path_m': round(self.path_m, 2),
            'straight_line_m': round(math.dist((sx, sy), (goal.x, goal.y)), 2),
            'detour_ratio': round(self.path_m / max(math.dist((sx, sy), (goal.x, goal.y)), 1e-6), 3),
            'mean_speed_mps': round(self.path_m / max(elapsed, 1e-6), 3),
            'final_xy_error_m': round(dist_to_goal, 3),
            'final_yaw_error_deg': round(math.degrees(yaw_err), 1),
        }


def run_trial(scenario: str, mode: str, out_dir: Path, gen_dir: Path,
              keep_up: bool = False, sensor_noise: float = 1.0,
              overlay: str = '') -> dict:
    sc = spec.load(scenario)
    gen = gen_dir / sc.name
    gen.mkdir(parents=True, exist_ok=True)
    world = gen / f'{sc.name}.sdf'
    world.write_text(world_gen.to_sdf(sc))
    _, map_yaml = map_gen.write(sc, gen)

    # Per-trial log directory. A single shared 'logs/' overwrote each trial's
    # evidence with the next one's, which destroyed the first dead_end_reverse
    # failure before it could be attributed.
    logs = out_dir / 'logs' / f'{sc.name}_{mode}_{int(time.time())}'
    result: dict = {'scenario': sc.name, 'mode': mode, 'sensor_noise': sensor_noise,
                    'config': Path(overlay).stem if overlay else 'baseline'}
    busy = domain_is_busy()
    if busy:
        return {**result, 'outcome': 'SIM_DEGRADED',
                'detail': f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")} '
                          f'already has nodes ({busy}); refusing to start so two '
                          f'stacks do not fight over goals'}
    stack = Stack()
    try:
        stack.launch([
            'ros2', 'launch', 'picar2_bringup', 'sim.launch.py', 'headless:=true',
            'lidar:=ld19', f'world:={world}', f'spawn_x:={sc.start.x}',
            f'spawn_y:={sc.start.y}', f'spawn_yaw:={sc.start.yaw}',
            f'sensor_noise:={sensor_noise}'], logs / 'sim.log')
        if not _wait_topic('/lidar_node/scan', 150):
            raise gates.GateFailure('simulator never produced a scan')

        stack.launch([
            'ros2', 'launch', 'picar2_benchmark', 'benchmark_localization.launch.py',
            f'mode:={mode}', f'map_yaml:={map_yaml}', 'use_sim_time:=true'],
            logs / 'loc.log')
        stack.launch([
            'ros2', 'launch', 'picar2_bringup', 'nav2.launch.py', 'use_sim_time:=true']
            + ([f'params_overlay:={overlay}'] if overlay else []),
            logs / 'nav2.log')

        rclpy.init()
        ctx = gates.GateContext()
        gates.wait_for_ground_truth(ctx)
        gates.gate_spawn_pose(ctx, sc.start)
        gates.gate_settle(ctx)
        gates.gate_motion(ctx)
        reset_pose(sc)
        ctx.spin(2.0)
        gates.gate_settle(ctx)
        # back on the exact start pose, so the measurement matches the spec
        gates.gate_spawn_pose(ctx, sc.start)
        gates.gate_single_map_odom(ctx, mode)
        gates.gate_costmap(ctx, sc, mode)
        result['gates'] = 'passed'
        try:
            result['effective'] = effective_params()
        except Exception as e:                               # noqa: BLE001
            result['effective'] = {'error': repr(e)}

        rec = Recorder(ctx, sc.all_boxes)
        ctx.spin(1.0)                       # let the subscriptions connect
        run = GoalRunner(ctx, sc.timeout_s, rec).run(sc.goal)
        result.update(run)
        m = rec.metrics()
        result['metrics'] = m
        cls, why = attribution.classify(
            run.get('outcome', 'UNKNOWN'), m,
            run.get('final_xy_error_m', 99.0), run.get('final_yaw_error_deg', 0.0))
        result['class'] = cls
        result['why'] = why
        ev = attribution.cusp_evidence(m)
        if ev:
            result['cusp_evidence'] = ev
    except gates.GateFailure as e:
        result.update({'outcome': 'SIM_DEGRADED', 'detail': str(e)})
    except Exception as e:                                   # noqa: BLE001
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
    ap = argparse.ArgumentParser(description='Run one navigation benchmark trial.')
    ap.add_argument('scenario')
    ap.add_argument('--mode', default='ground_truth',
                    choices=['ground_truth', 'slam', 'amcl'])
    ap.add_argument('-o', '--out', default='/tmp/picar2_bench/results')
    ap.add_argument('--sensor-noise', type=float, default=1.0,
                    help='scale simulated sensor noise; 0.0 isolates scheduling variance')
    ap.add_argument('--overlay', default='',
                    help='nav2 params overlay layered over nav2.yaml')
    ap.add_argument('--keep-up', action='store_true',
                    help='leave the stack running for inspection')
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res = run_trial(a.scenario, a.mode, out, Path('/tmp/picar2_bench'), a.keep_up,
                    a.sensor_noise, a.overlay)
    name = (f"{res['scenario']}_{a.mode}_{res['config']}_"
            f"n{a.sensor_noise}_{int(time.time())}.json")
    (out / name).write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    # A failed navigation is a legitimate result, not a tooling error — only an
    # unmeasurable trial should make the caller (or make) report failure.
    return 1 if res.get('outcome') in ('SIM_DEGRADED', 'RUNNER_ERROR') else 0
