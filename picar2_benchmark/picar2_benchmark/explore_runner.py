"""One measured exploration trial.

Navigation trials end on arrival; an exploration trial has no goal pose, so it
ends when the room is mapped, when the map stops growing, or when the clock runs
out. The headline number is coverage against the *true* free area, computed from
the scenario geometry - the robot only knows what it has already mapped, so it
cannot tell a finished room from one it has given up on.

Exploration runs under SLAM by construction: there has to be a map being built,
and cartographer owns map->odom while it does. Ground truth is still collected
throughout, and is what path, revisits and coverage are measured against.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from . import gates, map_gen, spec, world_gen
from .recorder import Recorder
from .runner import (BAG_TOPICS, Stack, _hlog, _wait_topic, reset_pose,
                     wait_for_quiet_domain)

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
# Cartographer publishes graded occupancy probabilities, not a trinary map:
# free space sits around 20-29 and walls around 70-79, measured on a live map.
# Anything known and below the wall threshold counts as mapped free space.
FREE_MAX = 65
CELL = 0.3          # revisit bucket, metres


class Coverage(rclpy.node.Node):
    def __init__(self, true_free_m2: float):
        super().__init__('bench_coverage')
        self.true_free = true_free_m2
        self.curve: list[tuple[float, float]] = []     # t, mapped m2
        self.latest = 0.0
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)

    def _on_map(self, m):
        res = m.info.resolution
        free = sum(1 for c in m.data if 0 <= c < FREE_MAX)
        self.latest = free * res * res


def run_trial(scenario: str, out_dir: Path, gen_dir: Path,
              explorer: str = 'explore_lite', keep_up: bool = False,
              sensor_noise: float = 1.0, bag: bool = True) -> dict:
    sc = spec.load(scenario)
    if not sc.explore:
        raise SystemExit(f'{sc.name} is not an exploration scenario '
                         f'(no `explore:` block)')
    cfg = sc.explore
    gen = gen_dir / sc.name
    gen.mkdir(parents=True, exist_ok=True)
    world = gen / f'{sc.name}.sdf'
    world.write_text(world_gen.to_sdf(sc))
    map_gen.write(sc, gen)
    true_free = spec.free_area_m2(sc)

    logs = out_dir / 'logs' / f'{sc.name}_{explorer}_{int(time.time())}'
    result: dict = {'scenario': sc.name, 'explorer': explorer, 'mode': 'slam',
                    'sensor_noise': sensor_noise, 'true_free_m2': round(true_free, 2)}
    busy = wait_for_quiet_domain()
    if busy:
        return {**result, 'outcome': 'SIM_DEGRADED',
                'detail': f'domain already has nodes ({busy})'}

    import os
    os.environ['ROS_LOG_DIR'] = str(logs / 'ros')
    os.environ['PICAR_EXPLORER'] = explorer
    _hlog(logs, f'explore trial start scenario={sc.name} explorer={explorer} '
                f'true_free={true_free:.1f}m2')
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
            'mode:=slam', 'use_sim_time:=true'], logs / 'loc.log')
        stack.launch([
            'ros2', 'launch', 'picar2_bringup', 'nav2.launch.py',
            'use_sim_time:=true'], logs / 'nav2.log')
        if bag:
            stack.launch(['ros2', 'bag', 'record', '-o', str(logs / 'bag'),
                          '--include-hidden-topics', '--max-cache-size', '10485760',
                          *BAG_TOPICS, '/map'], logs / 'bag.log')

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
        _hlog(logs, 'gates passed')
        result['gates'] = 'passed'

        cov = Coverage(true_free)
        rec = Recorder(ctx, sc.all_boxes, 'slam')
        ctx.spin(1.0)

        # The explorer is launched last: it starts driving the moment it sees a
        # map, and anything before this point is setup, not exploration.
        stack.launch(['ros2', 'launch', 'picar2_bringup', 'explore.launch.py',
                      'use_sim_time:=true', f'explorer:={explorer}'],
                     logs / 'explore.log')
        _hlog(logs, f'explorer launched: {explorer}')

        out = _explore_loop(ctx, cov, rec, sc, cfg, true_free, logs)
        result.update(out)
        result['metrics'] = rec.metrics()
        rec.dump_trajectory(logs / 'trajectory.json')
        (logs / 'coverage.json').write_text(json.dumps(
            {'true_free_m2': true_free,
             'curve': [[round(t, 2), round(a, 3)] for t, a in cov.curve]}))
        result['trajectory'] = str(logs / 'trajectory.json')
        result['coverage_curve'] = str(logs / 'coverage.json')
        if bag:
            result['bag'] = str(logs / 'bag')
        _hlog(logs, f"outcome={result.get('outcome')} "
                    f"coverage={result.get('final_coverage_pct')}%")
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


def _explore_loop(ctx, cov, rec, sc, cfg, true_free, logs) -> dict:
    duration = float(cfg.get('duration_s', 240))
    target = float(cfg.get('target_coverage', 0.90))
    plateau = float(cfg.get('plateau_s', 45))
    sim = lambda: ctx.get_clock().now().nanoseconds * 1e-9

    t0 = sim()
    wall0 = time.time()
    best = 0.0
    best_t = t0
    last_sim, last_sim_wall = t0, wall0
    visits: Counter = Counter()
    last_cell = None
    path = 0.0
    prev_xy = None
    gt_seq = -1
    marks = {}                       # coverage fraction -> sim seconds

    while True:
        rclpy.spin_once(ctx, timeout_sec=0.05)
        rclpy.spin_once(cov, timeout_sec=0.0)
        now = sim()

        # A frozen clock means the simulator died; a sim-time deadline alone
        # would wait for a tick that never comes.
        if now > last_sim:
            last_sim, last_sim_wall = now, time.time()
        elif time.time() - last_sim_wall > 15.0:
            return {'outcome': 'SIM_DEGRADED',
                    'detail': 'sim clock frozen; the simulator died mid-trial'}

        # Once per ground-truth message, not once per loop. This loop spins far
        # faster than /gt/odom publishes at 50 Hz, so re-reading the cached pose
        # repeats a position while the clock advances - which reads as standing
        # still and marked entire navigation runs as stalled before the same fix
        # was applied there.
        if ctx.gt_seq != gt_seq:
            gt_seq = ctx.gt_seq
            x, y, yaw = ctx.gt_pose()
            rec.sample_pose(x, y, yaw)
            if prev_xy is not None:
                path += math.dist(prev_xy, (x, y))
            prev_xy = (x, y)
            c = (round(x / CELL), round(y / CELL))
            if c != last_cell:
                visits[c] += 1
                last_cell = c

        if cov.latest > best + 1e-9:
            best, best_t = cov.latest, now
        # one sample a second: the curve is for reading afterwards, not for
        # the loop rate, which is thousands of times faster
        if not cov.curve or (now - t0) - cov.curve[-1][0] >= 1.0:
            cov.curve.append((now - t0, cov.latest))
        frac = cov.latest / true_free if true_free else 0.0
        for m in (0.5, 0.7, 0.8, 0.9):
            if m not in marks and frac >= m:
                marks[m] = round(now - t0, 2)
                _hlog(logs, f'{int(m*100)}% coverage at {marks[m]}s')

        if frac >= target:
            outcome = 'COVERED'
            break
        if now - best_t >= plateau:
            outcome = 'PLATEAUED'
            break
        if now - t0 >= duration:
            outcome = 'TIMEOUT'
            break

    elapsed = sim() - t0
    entries = sum(visits.values())
    redundant = sum(v - 1 for v in visits.values() if v > 1)
    return {
        'outcome': outcome,
        'time_s': round(elapsed, 2),
        'wall_time_s': round(time.time() - wall0, 2),
        'mapped_m2': round(cov.latest, 2),
        'final_coverage_pct': round(100 * cov.latest / true_free, 1) if true_free else None,
        'coverage_rate_m2_per_min': round(cov.latest / max(elapsed, 1e-6) * 60, 2),
        'gt_path_m': round(path, 2),
        # How much driving each square metre of map cost. Two explorers can
        # reach the same coverage with very different amounts of driving, and
        # that difference is the whole point of comparing them.
        'path_per_m2': round(path / cov.latest, 3) if cov.latest > 0.1 else None,
        'distinct_cells': len(visits),
        'redundant_entry_pct': round(100 * redundant / entries, 1) if entries else None,
        **{f'time_to_{int(m*100)}pct_s': marks.get(m) for m in (0.5, 0.7, 0.8, 0.9)},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Run one exploration trial.')
    ap.add_argument('scenario')
    ap.add_argument('--explorer', default='explore_lite',
                    choices=['explore_lite', 'frontier'])
    ap.add_argument('-o', '--out', default='/tmp/picar2_bench/explore')
    ap.add_argument('--sensor-noise', type=float, default=1.0)
    ap.add_argument('--no-bag', dest='bag', action='store_false')
    ap.add_argument('--keep-up', action='store_true')
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res = run_trial(a.scenario, out, Path('/tmp/picar2_bench'), a.explorer,
                    a.keep_up, a.sensor_noise, a.bag)
    name = (f"{res['scenario']}_{a.explorer}_n{a.sensor_noise}_"
            f"{int(time.time())}.json")
    (out / name).write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    return 0 if res.get('outcome') in ('COVERED', 'PLATEAUED', 'TIMEOUT') else 1
