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


def path_length(path: Path) -> float:
    p = [(q.pose.position.x, q.pose.position.y) for q in path.poses]
    return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))


class Recorder:
    """Subscribes on an existing node and accumulates a trial timeline."""

    def __init__(self, node, boxes):
        self.node = node
        self.boxes = boxes
        self.bt: list[tuple[float, str, str]] = []      # t, node_name, status
        self.cmd: list[tuple[float, float, float]] = []  # t, vx, wz
        self.pose: list[tuple[float, float, float, float]] = []  # t, x, y, yaw
        self.clearances: list[tuple[float, float]] = []
        self.plans: list[tuple[float, int, float]] = []   # t, cusps, length
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

    # ── derived metrics ─────────────────────────────────────────────────
    def metrics(self) -> dict:
        m: dict = {}
        if self.cmd:
            moving = [c for c in self.cmd if abs(c[1]) > 0.02]
            m['cmd_samples'] = len(self.cmd)
            m['cmd_zero_pct'] = round(100 * (1 - len(moving) / len(self.cmd)), 1)
            m['reverse_pct'] = round(
                100 * sum(1 for c in moving if c[1] < 0) / max(len(moving), 1), 1)
            signs = [1 if c[1] > 0.02 else -1 for c in self.cmd if abs(c[1]) > 0.02]
            m['direction_reversals'] = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        if len(self.pose) > 2:
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
        if self.clearances:
            vals = [c for _, c in self.clearances]
            m['min_clearance_m'] = round(min(vals), 3)
            m['time_below_5cm_s'] = round(sum(
                1 for v in vals if v < 0.05) * 0.05, 1)
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
