#!/usr/bin/env python3
"""Run one benchmark trial and emit a JSON result.

Sequence: generate -> launch sim -> launch localisation -> launch nav2 -> gates
-> send one goal -> measure -> tear down.

A trial that fails a gate returns outcome SIM_DEGRADED and is meant to be
discarded by the caller rather than averaged in.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
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
from rclpy.duration import Duration

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
                break
            time.sleep(0.5)
        else:
            for g in self._alive():
                try:
                    os.killpg(g, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        # Settle on every path, not only after SIGKILL. Returning the instant
        # the last process exits leaves DDS and the ros2 daemon still listing
        # them, and the next trial's busy check then refuses to start.
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


def wait_for_quiet_domain(timeout: float = 45.0) -> str | None:
    """Poll until no competing stack is visible, or give up and name it.

    Checking once was too strict. The ros2 daemon caches discovery, so nodes
    from the previous trial linger for seconds after their processes are gone.
    That discarded 6 of 30 trials in an envelope run - and progressively more in
    later arms, as a busier machine made the daemon lag further behind - even
    though the domain was genuinely clear moments later. Waiting distinguishes
    "a stack is running" from "the last one has not finished disappearing".
    """
    end = time.time() + timeout
    while True:
        busy = domain_is_busy()
        if not busy:
            return None
        if time.time() >= end:
            return busy
        time.sleep(2.0)


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
    # In parallel: each `ros2 param get` spawns a python process that takes
    # over a second to reach the point of asking, and five of them in series
    # cost more than the drive being measured. They are independent reads.
    def _one(node_param):
        node, param = node_param
        r = subprocess.run(['ros2', 'param', 'get', node, param],
                           capture_output=True, text=True, timeout=20)
        return node_param, r
    out = {}
    with cf.ThreadPoolExecutor(max_workers=len(wanted)) as ex:
        results = list(ex.map(_one, wanted))
    for (node, param), r in results:
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


# Topics recorded for every trial so a run can be re-examined after the fact.
# Deliberately not the costmaps: /global_costmap/costmap alone is ~100 kB a
# message and the update streams dwarf everything else, which would turn a
# 48-trial sweep into gigabytes. The scenario geometry is in the spec and the
# occupancy is reproducible from it, so what is worth keeping is what the stack
# decided: plans, commands, the BT's own account, and ground truth to check it
# against. The three action topics are hidden and need the explicit flag.
BAG_TOPICS = [
    '/behavior_tree_log', '/plan', '/received_global_plan', '/local_plan',
    '/cmd_vel', '/gt/odom', '/odom', '/lidar_node/scan', '/joint_states',
    '/tf', '/tf_static', '/rosout',
    '/navigate_to_pose/_action/status', '/follow_path/_action/status',
    '/compute_path_to_pose/_action/status',
]

# Wall seconds with a frozen /clock before a trial is called degraded.
SIM_CLOCK_TIMEOUT = 15.0


class GoalRunner:
    """Sends one NavigateToPose goal and measures the outcome.

    Uses the stale-goal defences from scripts/loop_waypoints.py: Jazzy reports a
    preempted goal as ABORTED with empty error info, and NavigateToPose's result
    declares only NONE=0, so the handle identity is the only reliable signal that
    a result belongs to the goal we sent.
    """

    def __init__(self, ctx: gates.GateContext, timeout_s: float, rec: Recorder,
                 map_warmup_s: float = 0.0):
        self.ctx = ctx
        self.timeout_s = timeout_s
        self.rec = rec
        self.map_warmup_s = map_warmup_s
        self.client = ActionClient(ctx, NavigateToPose, 'navigate_to_pose')
        self.handle = None
        self.status = None
        self.path_m = 0.0
        self._last = None
        self._gt_seq = -1

    def goal_in_map(self, goal, start) -> tuple[float, float, float]:
        """Express a scenario goal in whatever frame the robot is actually using.

        Scenario coordinates are world coordinates. Under ground_truth the map
        frame IS the world, so they pass through unchanged. Under SLAM,
        cartographer anchors `map` at the robot's start pose, so a world goal
        means something different — sending it unchanged had the planner
        reporting a start of (0.34, -0.01) for a robot spawned at (-1.0, 0).

        Converting through the body frame is correct in both cases: "so many
        metres ahead of, and left of, where I started".
        """
        dx, dy = goal.x - start.x, goal.y - start.y
        c, sn = math.cos(-start.yaw), math.sin(-start.yaw)
        bx, by = dx * c - dy * sn, dx * sn + dy * c        # goal in body frame
        byaw = goal.yaw - start.yaw
        try:
            tf = self.ctx.buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=3.0))
        except Exception:                                   # noqa: BLE001
            return (goal.x, goal.y, goal.yaw)               # best effort
        t, q = tf.transform.translation, tf.transform.rotation
        myaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c2, s2 = math.cos(myaw), math.sin(myaw)
        return (t.x + bx * c2 - by * s2, t.y + bx * s2 + by * c2, myaw + byaw)

    def goal_is_mappable(self, gx: float, gy: float) -> bool:
        """Is the goal inside the current costmap? Under SLAM it often is not —
        the area simply has not been seen yet — and Smac then rejects the goal
        in 30 ms with 'Goal Coordinates outside of the map', which is not a
        navigation result and should not be recorded as one."""
        m = self.ctx.costmap
        if m is None:
            return False
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        return (ox <= gx <= ox + m.info.width * m.info.resolution
                and oy <= gy <= oy + m.info.height * m.info.resolution)

    def _pose(self, p) -> PoseStamped:
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.ctx.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(p.x), float(p.y)
        m.pose.orientation.z = math.sin(p.yaw / 2)
        m.pose.orientation.w = math.cos(p.yaw / 2)
        return m

    def _sim_now(self) -> float:
        return self.ctx.get_clock().now().nanoseconds * 1e-9

    def _accumulate(self):
        # Once per ground-truth message, not once per spin. This loop polls far
        # faster than /gt/odom publishes at 50 Hz, so re-reading the cache
        # repeats a position while the sim clock advances -- indistinguishable
        # from standing still. That made the stall detector report 10.9 s of
        # stall in a 10.88 s trial while the controller was commanding motion
        # 99.5% of the time, and inflated path_m with the same duplicates.
        if self.ctx.gt_seq == self._gt_seq:
            return
        self._gt_seq = self.ctx.gt_seq
        x, y, yaw = self.ctx.gt_pose()
        if self._last is not None:
            self.path_m += math.dist(self._last, (x, y))
        self._last = (x, y)
        self.rec.sample_pose(x, y, yaw)

    def run(self, goal, start=None) -> dict:
        if not self.client.wait_for_server(timeout_sec=30.0):
            return {'outcome': 'SIM_DEGRADED', 'detail': 'navigate_to_pose absent'}
        # `sent` is the goal in the robot's own frame; `goal` stays in world
        # coordinates. Keeping them apart matters: rebinding `goal` here made
        # every downstream metric compare a world ground-truth pose against a
        # map-frame goal. Under ground_truth the frames coincide so it was
        # invisible, but under SLAM it reported final_xy_error_m 2.63 m for a
        # run that actually finished 0.26 m from the goal.
        sent = goal
        if start is not None:
            gx, gy, gyaw = self.goal_in_map(goal, start)
            sent = type(goal)(gx, gy, gyaw)
        # SLAM warm-up: give cartographer time to observe the goal region before
        # sending the goal. The map genuinely grows — 132 -> 139 cells was
        # observed while a 4.3 m goal sat just outside it — so checking once and
        # giving up mis-reports a timing problem as an unmappable scenario.
        # Cartographer's start-up is also not navigation time and should not be
        # charged to the trial.
        warm_end = time.time() + self.map_warmup_s
        while not self.goal_is_mappable(sent.x, sent.y) and time.time() < warm_end:
            rclpy.spin_once(self.ctx, timeout_sec=0.2)
        if not self.goal_is_mappable(sent.x, sent.y):
            m = self.ctx.costmap
            extent = (f'{m.info.width * m.info.resolution:.1f}x'
                      f'{m.info.height * m.info.resolution:.1f} m' if m else 'none')
            return {'outcome': 'SCENARIO_UNMAPPABLE',
                    'detail': f'goal ({sent.x:.2f},{sent.y:.2f}) still outside the '
                              f'costmap ({extent}) after {self.map_warmup_s:.0f}s of '
                              f'mapping; that area cannot be observed from the start '
                              f'pose, so no goal there is plannable under SLAM'}
        g = NavigateToPose.Goal()
        g.pose = self._pose(sent)
        send = self.client.send_goal_async(g)
        end = time.time() + 15
        while time.time() < end and not send.done():
            rclpy.spin_once(self.ctx, timeout_sec=0.05)
        gh = send.result()
        if gh is None or not gh.accepted:
            return {'outcome': 'SIM_DEGRADED', 'detail': 'goal not accepted'}
        self.handle = gh
        result_fut = gh.get_result_async()

        # Measure in SIM time, not wall time. rtf is a target the simulator can
        # miss under host load, so wall-clock results drift with machine load -
        # exactly the uncontrolled variable trials are run serially to avoid.
        # At rtf 0.5 the two differ by 2x, which is why a 21.75 s trial reported
        # 10.9 s of recorder-measured stall: one inconsistency, not two bugs.
        t0 = self._sim_now()
        t0_wall = time.time()
        last_sim, last_sim_wall = t0, t0_wall
        self._last = None
        sx, sy, _ = self.ctx.gt_pose()          # true start, for the detour ratio
        while True:
            rclpy.spin_once(self.ctx, timeout_sec=0.05)
            self._accumulate()
            if result_fut.done():
                break
            now = self._sim_now()
            # A pure sim-time timeout would hang forever if the simulator dies,
            # because /clock simply stops - which is what a gazebo exit -2
            # mid-trial looks like. Watch for a frozen clock instead of guessing
            # a wall budget from rtf.
            if now > last_sim:
                last_sim, last_sim_wall = now, time.time()
            elif time.time() - last_sim_wall > SIM_CLOCK_TIMEOUT:
                gh.cancel_goal_async()
                return {'outcome': 'SIM_DEGRADED',
                        'detail': f'sim clock frozen for {SIM_CLOCK_TIMEOUT:.0f}s '
                                  f'of wall time; the simulator died mid-trial'}
            if now - t0 >= self.timeout_s:
                break
        elapsed = self._sim_now() - t0
        wall = time.time() - t0_wall

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
            'time_s': round(elapsed, 2),                  # simulated seconds
            'wall_time_s': round(wall, 2),
            # Achieved real-time factor. The scenario pins a target; a large
            # shortfall means the host could not keep up, so the trial ran under
            # different conditions from its peers.
            'rtf_achieved': round(elapsed / wall, 3) if wall > 0 else None,
            'gt_path_m': round(self.path_m, 2),
            'straight_line_m': round(math.dist((sx, sy), (goal.x, goal.y)), 2),
            'detour_ratio': round(self.path_m / max(math.dist((sx, sy), (goal.x, goal.y)), 1e-6), 3),
            'mean_speed_mps': round(self.path_m / max(elapsed, 1e-6), 3),
            'final_xy_error_m': round(dist_to_goal, 3),
            'final_yaw_error_deg': round(math.degrees(yaw_err), 1),
        }


def _hlog(logs, msg: str) -> None:
    """The harness's own account of a trial: what it decided and when.

    The result JSON says what a trial concluded; this says how it got there,
    which is what you need when a trial behaves differently from its peers.
    """
    logs.mkdir(parents=True, exist_ok=True)
    with (logs / 'harness.log').open('a') as f:
        f.write(f'{time.strftime("%H:%M:%S")} {msg}\n')


def run_trial(scenario: str, mode: str, out_dir: Path, gen_dir: Path,
              keep_up: bool = False, sensor_noise: float = 1.0,
              overlay: str = '', bt: str = '',
              trajectory: bool = True, bag: bool = True) -> dict:
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
                    'config': (Path(bt).stem if bt else
                               Path(overlay).stem if overlay else 'baseline')}
    # A missing overlay is not an error to launch_ros: it warns "Parameter file
    # path is not a file" and runs the baseline, so the trial records a config
    # it never used. Two mitigation results were read as null that way - the
    # files existed in source but colcon had not re-globbed them into share/.
    for label, path in (('overlay', overlay), ('bt', bt)):
        if path and not Path(path).is_file():
            return {**result, 'outcome': 'RUNNER_ERROR',
                    'detail': f'{label} file not found: {path} - if you just '
                              f'added it, rebuild the package so it installs'}

    busy = wait_for_quiet_domain()
    if busy:
        return {**result, 'outcome': 'SIM_DEGRADED',
                'detail': f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")} '
                          f'already has nodes ({busy}); refusing to start so two '
                          f'stacks do not fight over goals'}
    # Give the trial its own ROS log tree. The node-level logs are richer than
    # the captured stdout and are otherwise scattered under ~/.ros/log by
    # launch timestamp, which is painful to match back to a trial.
    os.environ['ROS_LOG_DIR'] = str(logs / 'ros')
    _hlog(logs, f'trial start scenario={sc.name} mode={mode} noise={sensor_noise}'
                f' overlay={overlay or "-"} bt={bt or "-"}')
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
            + ([f'params_overlay:={overlay}'] if overlay else [])
            + ([f'nav_to_pose_bt:={bt}'] if bt else []),
            logs / 'nav2.log')

        if bag:
            # SIGINT-terminated by Stack.teardown, which is what closes the
            # bag cleanly; a SIGKILL would leave it unindexed.
            stack.launch(['ros2', 'bag', 'record', '-o', str(logs / 'bag'),
                          '--include-hidden-topics', '--max-cache-size', '10485760',
                          *BAG_TOPICS], logs / 'bag.log')

        rclpy.init()
        ctx = gates.GateContext()
        _t = time.time()
        gates.wait_for_ground_truth(ctx)
        gates.gate_spawn_pose(ctx, sc.start)
        gates.gate_settle(ctx)
        gates.gate_motion(ctx)
        reset_pose(sc)
        # gate_settle below already waits for the robot to be still, so this
        # only has to let the teleport land.
        ctx.spin(0.5)
        gates.gate_settle(ctx)
        # back on the exact start pose, so the measurement matches the spec
        gates.gate_spawn_pose(ctx, sc.start)
        gates.gate_single_map_odom(ctx, mode)
        gates.gate_costmap(ctx, sc, mode)
        _hlog(logs, f'gates passed in {time.time() - _t:.1f}s')
        _t = time.time()
        result['gates'] = 'passed'
        try:
            result['effective'] = effective_params()
        except Exception as e:                               # noqa: BLE001
            result['effective'] = {'error': repr(e)}

        rec = Recorder(ctx, sc.all_boxes, mode)
        ctx.spin(0.5)                       # let the subscriptions connect
        # only SLAM has to wait for the world to be observed
        warmup = 60.0 if mode == 'slam' else 0.0
        _hlog(logs, f'setup done in {time.time() - _t:.1f}s')
        _hlog(logs, f'goal sent x={sc.goal.x} y={sc.goal.y} yaw={sc.goal.yaw}')
        run = GoalRunner(ctx, sc.timeout_s, rec, warmup).run(sc.goal, sc.start)
        result.update(run)
        _hlog(logs, f"outcome={run.get('outcome')} time_s={run.get('time_s')}")
        m = rec.metrics()
        result['metrics'] = m
        if trajectory:
            traj = logs / 'trajectory.json'
            rec.dump_trajectory(traj)
            result['trajectory'] = str(traj)
        if bag:
            result['bag'] = str(logs / 'bag')
        onset = rec.failure_onset()
        if onset:
            repro = spec.write_repro(sc, onset, logs)
            result['repro_scenario'] = str(repro)
            _hlog(logs, f'control broke down at ({onset[0]:.2f},{onset[1]:.2f},'
                        f'{onset[2]:.2f}); wrote {repro.name}')
        cls, why = attribution.classify(
            run.get('outcome', 'UNKNOWN'), m,
            run.get('final_xy_error_m', 99.0), run.get('final_yaw_error_deg', 0.0))
        result['class'] = cls
        result['why'] = why
        ev = attribution.cusp_evidence(m)
        if ev:
            result['cusp_evidence'] = ev
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
    ap = argparse.ArgumentParser(description='Run one navigation benchmark trial.')
    ap.add_argument('scenario')
    ap.add_argument('--mode', default='ground_truth',
                    choices=['ground_truth', 'slam', 'amcl'])
    ap.add_argument('-o', '--out', default='/tmp/picar2_bench/results')
    ap.add_argument('--sensor-noise', type=float, default=1.0,
                    help='scale simulated sensor noise; 0.0 isolates scheduling variance')
    ap.add_argument('--overlay', default='',
                    help='nav2 params overlay layered over nav2.yaml')
    ap.add_argument('--bt', default='',
                    help='navigate_to_pose behaviour tree XML (replan cadence)')
    ap.add_argument('--keep-up', action='store_true',
                    help='leave the stack running for inspection')
    ap.add_argument('--no-trajectory', dest='trajectory', action='store_false',
                    help='skip the raw pose/command dump (on by default)')
    ap.add_argument('--no-bag', dest='bag', action='store_false',
                    help='skip the rosbag (on by default)')
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res = run_trial(a.scenario, a.mode, out, Path('/tmp/picar2_bench'), a.keep_up,
                    a.sensor_noise, a.overlay, a.bt, a.trajectory, a.bag)
    name = (f"{res['scenario']}_{a.mode}_{res['config']}_"
            f"n{a.sensor_noise}_{int(time.time())}.json")
    (out / name).write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    # A failed navigation is a legitimate result, not a tooling error — only an
    # unmeasurable trial should make the caller (or make) report failure.
    return 1 if res.get('outcome') in ('SIM_DEGRADED', 'RUNNER_ERROR') else 0
