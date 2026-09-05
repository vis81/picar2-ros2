"""Records the evidence needed to attribute a navigation failure.

NavigateToPose cannot tell us anything: its result declares only NONE=0. The
sub-actions carry rich error codes, but bt_navigator is their client and results
travel over a service we cannot snoop. So the primary source is
/behavior_tree_log, which reports which BT node changed state and when — and our
tree contains WouldAPlannerRecoveryHelp / WouldAControllerRecoveryHelp, i.e.
Nav2's own verdict on whose fault it was.

The action *status* topics are hidden (absent from plain `ros2 topic list`);
they must be subscribed explicitly.
"""
from __future__ import annotations

import math

import rclpy

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav2_msgs.msg import BehaviorTreeLog

from .geometry import clearance


def path_cusps(path: Path) -> int:
    """Count direction reversals in a path.

    A Reeds-Shepp plan encodes a reverse leg as a cusp: the travel direction
    flips sign relative to the pose heading. Comparing the cusp count of the
    planner's /plan against RPP's pruned /received_global_plan is what turns
    "RPP skips reverse segments" from a hypothesis into a measurement.
    """
    poses = path.poses
    if len(poses) < 3:
        return 0
    signs = []
    for a, b in zip(poses, poses[1:]):
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        q = a.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        if abs(along) > 1e-4:
            signs.append(1 if along > 0 else -1)
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def path_curvatures(path: Path) -> list[float]:
    """Curvature (1/R) demanded at each triple of plan poses.

    Compared against the executed |wz|/|vx|, this says whether the controller is
    steering as hard as the plan asks. A plan full of 2.0 (the 0.5 m minimum
    radius) executed at ~0 means the robot is shuffling, not turning.
    """
    p = [(q.pose.position.x, q.pose.position.y) for q in path.poses]
    out = []
    for a, b, c in zip(p, p[1:], p[2:]):
        d1, d2, d3 = math.dist(a, b), math.dist(b, c), math.dist(a, c)
        if d1 < 1e-6 or d2 < 1e-6 or d3 < 1e-6:
            continue
        # Menger curvature: 4 * triangle area / product of side lengths
        area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
        out.append(4 * area / (d1 * d2 * d3))
    return out


def path_length(path: Path) -> float:
    p = [(q.pose.position.x, q.pose.position.y) for q in path.poses]
    return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))


class Recorder:
    """Subscribes on an existing node and accumulates a trial timeline."""

    def __init__(self, node, boxes, mode: str = 'ground_truth'):
        self.node = node
        self.mode = mode
        self.boxes = boxes
        self.bt: list[tuple[float, str, str]] = []      # t, node_name, status
        self.cmd: list[tuple[float, float, float]] = []  # t, vx, wz
        self.pose: list[tuple[float, float, float, float]] = []  # t, x, y, yaw
        # (gt_x, gt_y, gt_yaw, believed_x, believed_y, believed_yaw)
        self.loc_pairs: list[tuple] = []
        self.clearances: list[tuple[float, float]] = []
        self.plans: list[tuple[float, int, float]] = []   # t, cusps, length
        self.plan_curv: list[float] = []                 # 1/R asked for by the plan
        self.pruned: list[tuple[float, int, float]] = []
        self.planner_status: list[int] = []
        self.controller_status: list[int] = []
        self._seen_goals: dict[str, int] = {}

        node.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._on_bt, 50)
        node.create_subscription(Twist, '/cmd_vel', self._on_cmd, 20)
        node.create_subscription(Path, '/plan', self._on_plan, 5)
        node.create_subscription(Path, '/received_global_plan', self._on_pruned, 5)
        node.create_subscription(GoalStatusArray, '/compute_path_to_pose/_action/status',
                                 lambda m: self._on_status(m, self.planner_status), 20)
        node.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                                 lambda m: self._on_status(m, self.controller_status), 20)

    def _t(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_bt(self, msg):
        for ev in msg.event_log:
            self.bt.append((self._t(), ev.node_name, ev.current_status))

    def _on_cmd(self, msg):
        # /cmd_vel is unstamped Twist (enable_stamped_cmd_vel defaults false),
        # so it has to be timestamped on receipt.
        self.cmd.append((self._t(), msg.linear.x, msg.angular.z))

    def _on_plan(self, msg):
        self.plans.append((self._t(), path_cusps(msg), path_length(msg)))
        self.plan_curv.extend(path_curvatures(msg))

    def _on_pruned(self, msg):
        self.pruned.append((self._t(), path_cusps(msg), path_length(msg)))

    def _on_status(self, msg, sink):
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()[:12]
            if self._seen_goals.get(gid) != s.status:
                self._seen_goals[gid] = s.status
                sink.append(s.status)

    def sample_pose(self, x: float, y: float, yaw: float) -> None:
        t = self._t()
        self.pose.append((t, x, y, yaw))
        self.clearances.append((t, clearance((x, y, yaw), self.boxes)))
        self._sample_loc_error(x, y, yaw)

    def _sample_loc_error(self, gx: float, gy: float, gyaw: float) -> None:
        """Ground truth vs what the robot believes, stored as pose pairs.

        Reduced in metrics() two ways, because no single number is honest
        in all three modes:

        loc_error - absolute distance between the poses. The real number
        under ground_truth and amcl, where the map served to the stack IS
        the world map, so a constant offset is genuine error. Meaningless
        under slam: cartographer anchors `map` wherever the robot happened
        to be at init, so the offset is arbitrary.

        loc_drift - the residual after aligning the two trajectories with a
        rigid SE(2) transform. The one quantity that means the same thing
        in every mode, so it is what to compare across them.

        The alignment must include rotation, not just translation: a yaw
        offset in cartographer's map frame shows up as apparent position
        error of roughly 2*r*sin(yaw/2), which is metres at r=3 m. Aligning
        translation alone reported 1.24 m mean and 6.54 m max drift for a
        slam run that finished 0.44 m from the goal - impossible, since Nav2
        drives to the goal in the believed frame. Medians, not means, so one
        bad sample during cartographer start-up cannot move the fit.
        """
        try:
            tf = self.node.buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:                                    # noqa: BLE001
            return
        t, q = tf.transform.translation, tf.transform.rotation
        byaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))
        self.loc_pairs.append((gx, gy, gyaw, t.x, t.y, byaw))


    def dump_trajectory(self, path) -> None:
        """Write the raw pose and command series next to the result.

        Aggregates alone cannot answer "what did it actually do" - the corner
        scenarios produce a bimodal reversal count, and telling a clean arc from
        a recovery thrash needs the path itself, not a median.
        """
        import json as _json
        _json.dump({
            'pose': [[round(v, 4) for v in p] for p in self.pose],   # t,x,y,yaw
            'cmd': [[round(v, 4) for v in c] for c in self.cmd],     # t,vx,wz
            'clearance': [[round(v, 4) for v in c] for c in self.clearances],
            'plans': [[round(v, 4) for v in p] for p in self.plans],  # t,cusps,len
            'bt': [[round(e[0], 4), e[1], e[2]] for e in self.bt],    # t,node,status
        }, open(path, 'w'))

    def failure_onset(self):
        """Pose just before control broke down; see spec.pose_before."""
        from .spec import pose_before
        return pose_before(self.bt, self.pose)

    # ── derived metrics ─────────────────────────────────────────────────
    def metrics(self) -> dict:
        m: dict = {}
        if self.loc_pairs:
            def _med(v):
                v = sorted(v)
                return v[len(v) // 2]
            # robust SE(2) fit: median yaw offset, then median translation
            dyaw = _med([math.atan2(math.sin(p[2] - p[5]), math.cos(p[2] - p[5]))
                         for p in self.loc_pairs])
            cs, sn = math.cos(dyaw), math.sin(dyaw)
            rot = [(p[3] * cs - p[4] * sn, p[3] * sn + p[4] * cs)
                   for p in self.loc_pairs]
            tx = _med([p[0] - r[0] for p, r in zip(self.loc_pairs, rot)])
            ty = _med([p[1] - r[1] for p, r in zip(self.loc_pairs, rot)])
            series = {'loc_drift': [math.hypot(p[0] - (r[0] + tx),
                                               p[1] - (r[1] + ty))
                                    for p, r in zip(self.loc_pairs, rot)]}
            if self.mode != 'slam':
                series['loc_error'] = [math.hypot(p[0] - p[3], p[1] - p[4])
                                       for p in self.loc_pairs]
            m['loc_align_yaw_deg'] = round(math.degrees(dyaw), 2)
            for label, vals in series.items():
                e = sorted(vals)
                m[f'{label}_mean_m'] = round(sum(e) / len(e), 3)
                m[f'{label}_p95_m'] = round(
                    e[min(len(e) - 1, int(0.95 * len(e)))], 3)
                m[f'{label}_max_m'] = round(e[-1], 3)
        if self.cmd:
            moving = [c for c in self.cmd if abs(c[1]) > 0.02]
            m['cmd_samples'] = len(self.cmd)
            m['cmd_zero_pct'] = round(100 * (1 - len(moving) / len(self.cmd)), 1)
            m['reverse_pct'] = round(
                100 * sum(1 for c in moving if c[1] < 0) / max(len(moving), 1), 1)
            signs = [1 if c[1] > 0.02 else -1 for c in self.cmd if abs(c[1]) > 0.02]
            m['direction_reversals'] = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        # Steering behaviour. A car turning round in a confined space must
        # alternate full lock with direction — hard one way in reverse, hard the
        # other going forward. Shuffling back and forth near-straight changes
        # position but gains almost no heading, so executed curvature is the
        # measurement that distinguishes a real multi-point turn from a shuffle.
        moving = [c for c in self.cmd if abs(c[1]) > 0.02]
        if moving:
            curv = [abs(c[2]) / abs(c[1]) for c in moving]      # |wz|/|vx| = 1/R
            fwd = [abs(c[2]) for c in moving if c[1] > 0]
            rev = [abs(c[2]) for c in moving if c[1] < 0]
            m['exec_curvature_median'] = round(sorted(curv)[len(curv) // 2], 3)
            m['exec_curvature_max'] = round(max(curv), 3)
            # 1/R for the 0.5 m minimum turning radius is 2.0
            m['pct_near_straight'] = round(
                100 * sum(1 for c in curv if c < 0.5) / len(curv), 1)
            m['pct_near_full_lock'] = round(
                100 * sum(1 for c in curv if c > 1.5) / len(curv), 1)
            if fwd:
                m['mean_abs_wz_forward'] = round(sum(fwd) / len(fwd), 3)
            if rev:
                m['mean_abs_wz_reverse'] = round(sum(rev) / len(rev), 3)

            # Steering alternation. yaw_rate = v*tan(delta)/L, so to keep turning
            # the SAME way through a direction change the steering angle must
            # flip — which shows up as wz keeping its sign while vx flips. If wz
            # flips too, the steering did not alternate and the robot simply
            # retraces the same arc: maximum motion, zero net rotation.
            kept = flipped = 0
            prev = None
            for t, vx, wz in moving:
                cur = (1 if vx > 0 else -1, 1 if wz > 0 else -1)
                if prev is not None and cur[0] != prev[0]:      # direction changed
                    if cur[1] == prev[1]:
                        kept += 1        # steering alternated -> rotation accrues
                    else:
                        flipped += 1     # steering held -> arc retraced
                prev = cur
            if kept + flipped:
                m['turn_alternation_pct'] = round(100 * kept / (kept + flipped), 1)
                m['reversals_accruing_yaw'] = kept
                m['reversals_retracing'] = flipped

        if len(self.pose) > 2:
            # One sample per /gt/odom message: GoalRunner._accumulate now
            # drops re-reads of the cached pose, which otherwise repeat a
            # position while the sim clock advances and score as zero
            # speed. That reported 10.9 s of stall in a 10.88 s trial with
            # the controller commanding motion 99.5% of the time; a repeat
            # inside one clock tick also gave dt == 0, which `continue`
            # skipped without closing the stall, fusing the run into one
            # contiguous block (stall_count == 1 in all 30 trials).
            stalls, cur = [], 0.0
            for a, b in zip(self.pose, self.pose[1:]):
                dt = b[0] - a[0]
                if dt <= 0 or dt > 2.0:
                    continue
                if math.dist(a[1:3], b[1:3]) / dt < 0.02:
                    cur += dt
                elif cur:
                    stalls.append(cur); cur = 0.0
            if cur:
                stalls.append(cur)
            m['stall_count'] = len(stalls)
            m['stall_total_s'] = round(sum(stalls), 1)
            m['longest_stall_s'] = round(max(stalls), 1) if stalls else 0.0
        # Achieved curvature from ground truth: d(yaw)/d(distance). Commanded
        # |wz|/|vx| is what the controller *asks* for; if the steering saturates
        # the robot turns less than that, and the visible result is shuffling
        # back and forth without the heading actually coming round.
        if len(self.pose) > 3:
            # Accumulate over a 5 cm baseline rather than comparing adjacent
            # samples: at ~0.1 m/s and ~20 Hz, consecutive poses are ~5 mm apart,
            # so a per-sample threshold discarded every pair and the metric came
            # out empty.
            ach, dyaw_tot, dist_tot = [], 0.0, 0.0
            anchor = self.pose[0]
            acc_yaw = 0.0
            for a, b in zip(self.pose, self.pose[1:]):
                step = math.dist(a[1:3], b[1:3])
                acc_yaw += abs(math.atan2(math.sin(b[3] - a[3]),
                                          math.cos(b[3] - a[3])))
                dist_tot += step
                dyaw_tot += abs(math.atan2(math.sin(b[3] - a[3]),
                                           math.cos(b[3] - a[3])))
                span = math.dist(anchor[1:3], b[1:3])
                if span >= 0.05:
                    ach.append(acc_yaw / span)
                    anchor, acc_yaw = b, 0.0
            if ach:
                ach.sort()
                m['gt_curvature_median'] = round(ach[len(ach) // 2], 3)
                m['gt_curvature_p90'] = round(ach[int(len(ach) * 0.9)], 3)
                # total heading swept per metre driven: the honest summary of
                # whether all that shuffling actually turned the robot round
                m['yaw_swept_deg_per_m'] = round(math.degrees(dyaw_tot) / dist_tot, 1)
                # net heading achieved vs total heading churned through: near 1
                # means every degree of rotation counted, near 0 means the robot
                # rotated one way then straight back again
                net = abs(math.atan2(math.sin(self.pose[-1][3] - self.pose[0][3]),
                                     math.cos(self.pose[-1][3] - self.pose[0][3])))
                m['yaw_net_deg'] = round(math.degrees(net), 1)
                m['yaw_abs_deg'] = round(math.degrees(dyaw_tot), 1)
                m['yaw_efficiency'] = round(net / dyaw_tot, 3) if dyaw_tot > 0 else None

        if self.clearances:
            vals = [c for _, c in self.clearances]
            m['min_clearance_m'] = round(min(vals), 3)
            m['time_below_5cm_s'] = round(sum(
                1 for v in vals if v < 0.05) * 0.05, 1)
        if self.plan_curv:
            c = sorted(self.plan_curv)
            m['plan_curvature_median'] = round(c[len(c) // 2], 3)
            m['plan_curvature_max'] = round(c[-1], 3)
        if self.plans:
            m['plans_received'] = len(self.plans)
            m['first_plan_cusps'] = self.plans[0][1]
            m['max_plan_cusps'] = max(p[1] for p in self.plans)
        if self.pruned:
            m['pruned_plans'] = len(self.pruned)
            m['max_pruned_cusps'] = max(p[1] for p in self.pruned)
        # NOT failures. The BT re-sends follow_path on every replan cycle and
        # each new goal preempts the running one, which Jazzy reports as
        # ABORTED — measured ratio to plans_received is exactly 1.00 in every
        # trial. Reading these as failures is the same mistake that made the
        # explorer blacklist frontiers it had merely replanned past.
        m['planner_goals_preempted'] = sum(1 for s in self.planner_status
                                           if s == GoalStatus.STATUS_ABORTED)
        m['controller_goals_preempted'] = sum(1 for s in self.controller_status
                                              if s == GoalStatus.STATUS_ABORTED)
        # The real failure signal: BT nodes that actually reported FAILURE.
        m['follow_path_failures'] = sum(
            1 for e in self.bt if e[1] == 'FollowPath' and e[2] == 'FAILURE')
        m['compute_path_failures'] = sum(
            1 for e in self.bt if e[1] == 'ComputePathToPose' and e[2] == 'FAILURE')
        m['bt_events'] = len(self.bt)
        m['bt_failures'] = sum(1 for e in self.bt if e[2] == 'FAILURE')
        m['bt_controller_recovery_votes'] = sum(
            1 for e in self.bt
            if 'WouldAControllerRecoveryHelp' in e[1] and e[2] == 'SUCCESS')
        m['bt_planner_recovery_votes'] = sum(
            1 for e in self.bt
            if 'WouldAPlannerRecoveryHelp' in e[1] and e[2] == 'SUCCESS')
        m['recoveries_run'] = sum(
            1 for e in self.bt
            if e[1] in ('BackUp', 'Wait') and e[2] == 'RUNNING')
        failed = sorted({e[1] for e in self.bt if e[2] == 'FAILURE'})
        if failed:
            m['bt_failed_nodes'] = failed[:8]
        return m
