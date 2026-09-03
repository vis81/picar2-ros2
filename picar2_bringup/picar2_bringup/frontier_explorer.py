#!/usr/bin/env python3
"""Frontier exploration for an Ackermann robot.

Written to replace explore_lite, whose assumptions are all reasonable for a
differential-drive robot and all wrong for a car:

  * it aims at the frontier centroid itself — which sits on the free/unknown
    boundary, i.e. usually against a wall, where RPP's collision check then
    refuses to command anything;
  * it sends yaw=0 for every goal, an arbitrary heading a car must actually
    achieve;
  * it ranks frontiers by straight-line distance, so a frontier 2 m behind
    scores the same as 2 m ahead though it costs a multi-point turn;
  * it re-plans on a timer and re-sends whenever the centroid moves >1 cm,
    preempting the current goal and leaving the robot swinging between
    directions instead of committing;
  * it traverses only cells of exactly FREE_SPACE, so any inflation
    disconnects the search;
  * and it blacklists a frontier permanently on any abort, then stops for
    good once the list covers everything.

This node instead: stands off from the frontier into known-free space and
faces the unknown, scores candidates with a kinematic heuristic confirmed by
the real planner, commits to a goal until it is reached or clearly beaten,
treats traversability as a cost threshold, and lets blacklist entries expire.

It never stops permanently: with no frontiers it idles and keeps looking.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

UNKNOWN = -1


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Frontier:
    __slots__ = ("cells", "centroid", "goal", "size_m", "score", "path_len")

    def __init__(self, cells, centroid, goal, size_m):
        self.cells = cells
        self.centroid = centroid      # (x, y) of the frontier itself
        self.goal = goal              # (x, y, yaw) the robot should drive to
        self.size_m = size_m
        self.score = math.inf
        self.path_len = None


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        p = self.declare_parameters("", [
            ("costmap_topic", "/global_costmap/costmap"),
            ("robot_base_frame", "base_footprint"),
            ("plan_period", 2.0),        # how often to reconsider (s)
            ("free_threshold", 50),      # costmap value at or below = traversable
            ("lethal_threshold", 90),    # at or above = obstacle
            ("min_frontier_size", 0.4),  # metres of frontier edge to bother with
            ("min_goal_distance", 0.5),  # a goal nearer than this is already reached
            ("turn_radius", 0.5),        # minimum turning radius, for scoring
            ("turn_weight", 1.0),        # how much a required turn costs vs metres
            ("gain_weight", 2.0),        # reward per metre of frontier size
            ("commit_seconds", 10.0),    # min time on a goal before reconsidering
            ("switch_margin", 1.5),      # metres of path a rival must beat it by
            ("useful_radius", 1.0),      # goal is pointless once no unknown is this close
            ("arrive_radius", 0.4),      # close enough — don't chase the goal heading
            ("stuck_seconds", 25.0),     # no progress for this long = give up on it
            ("stuck_distance", 0.2),     # ...where progress means moving this far
            ("futile_distance", 0.15),   # a goal that moved us less than this bought nothing
            ("verify_top_k", 3),         # candidates costed with the real planner
            ("blacklist_seconds", 45.0), # failures expire rather than being forever
            ("blacklist_radius", 0.5),
        ])
        self.cfg = {d.name: d.value for d in p}

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.cfg["costmap_topic"],
                                 self._on_costmap, qos)
        self.create_subscription(Bool, "explore/resume", self._on_resume, 10)
        self.status_pub = self.create_publisher(ExploreStatus, "explore/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "explore/frontiers", 1)

        cb = ReentrantCallbackGroup()
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                callback_group=cb)
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose",
                                    callback_group=cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grid: OccupancyGrid | None = None
        self.enabled = True
        self.goal_handle = None
        self.current: Frontier | None = None
        self.blacklist: list[tuple[float, float, float]] = []   # x, y, expiry
        self._published = None
        self.goal_started = 0.0
        self._preempting = False
        self._fail_count: dict[tuple[int, int], int] = {}
        self._sent_from: tuple[float, float] | None = None
        self._last_xy = (0.0, 0.0)
        self._track: list[tuple[float, float, float]] = []   # t, x, y

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        # the web UI shows whatever was last published, so say something now
        # rather than leaving it blank until the first planning cycle
        self._status(ExploreStatus.EXPLORATION_STARTED)
        self.get_logger().info("frontier_explorer ready")

    def _loop(self):
        while not self._stop.wait(self.cfg["plan_period"]):
            try:
                self._tick()
            except Exception as e:                       # never let the loop die
                self.get_logger().error(f"tick failed: {e}")

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _wait(self, fut, timeout):
        """Wait on a future from the planning thread.

        Not spin_until_future_complete: the executor is already spinning this
        node in other threads, and re-entering it here would deadlock.
        """
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end and not self._stop.is_set():
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    # ── inputs ───────────────────────────────────────────────────────────
    def _on_costmap(self, msg):
        self.grid = msg

    def _on_resume(self, msg):
        self.enabled = bool(msg.data)
        self.get_logger().info(f"explore {'resumed' if self.enabled else 'paused'}")
        if not self.enabled:
            self._cancel_goal()

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", self.cfg["robot_base_frame"], Time())
        except Exception as e:
            self.get_logger().warn(f"no robot pose: {e}", throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        self._last_xy = (t.x, t.y)
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _status(self, s):
        if s != self._published:
            self._published = s
            self.status_pub.publish(ExploreStatus(status=s))

    # ── frontier extraction ──────────────────────────────────────────────
    def _frontiers(self) -> list[Frontier]:
        g = self.grid
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        a = np.frombuffer(bytes(g.data), dtype=np.int8).reshape(h, w).astype(np.int16)

        free = (a >= 0) & (a <= self.cfg["free_threshold"])
        unknown = a == UNKNOWN

        # a frontier cell is FREE and touches UNKNOWN — free, so a goal derived
        # from it can actually be stood on
        nb = np.zeros_like(unknown)
        nb[1:, :] |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:, :]
        nb[:, 1:] |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        edge = free & nb
        if not edge.any():
            return []

        out = []
        min_cells = max(2, int(self.cfg["min_frontier_size"] / res))
        for cells in self._clusters(edge):
            ys = np.fromiter((c[0] for c in cells), dtype=np.int32, count=len(cells))
            xs = np.fromiter((c[1] for c in cells), dtype=np.int32, count=len(cells))
            if len(ys) < min_cells:
                continue
            cx = ox + (xs.mean() + 0.5) * res
            cy = oy + (ys.mean() + 0.5) * res
            if self._blacklisted(cx, cy):
                continue
            goal = self._goal_for(a, xs, ys, ox, oy, res, w, h)
            if goal is None:
                continue
            out.append(Frontier(len(ys), (cx, cy), goal, len(ys) * res))
        return out

    @staticmethod
    def _clusters(mask):
        """4-connected components, walking only the set cells.

        Scanning every cell of a 400x400 costmap in Python costs more than the
        whole rest of the cycle; frontier cells are a few hundred at most.
        """
        ys, xs = np.nonzero(mask)
        todo = set(zip(ys.tolist(), xs.tolist()))
        while todo:
            stack = [todo.pop()]
            cells = list(stack)
            while stack:
                y, x = stack.pop()
                for n in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                    if n in todo:
                        todo.discard(n)
                        cells.append(n)
                        stack.append(n)
            yield cells

    def _goal_for(self, a, xs, ys, ox, oy, res, w, h):
        """Aim at the frontier cell with the most clearance, facing the unknown.

        Every frontier cell is free by construction, so one is always drivable;
        picking the cheapest gives the widest berth from walls. What we must
        not do is step *back* towards known space — the robot is usually
        already standing there, so the goal lands behind it, Nav2 reports
        success without moving, and the frontier is never consumed.
        """
        costs = a[ys, xs]
        order = np.lexsort((np.abs(xs - xs.mean()) + np.abs(ys - ys.mean()), costs))
        cx_i, cy_i = int(xs[order[0]]), int(ys[order[0]])
        # direction from the frontier into unknown space, from the local window
        y0, y1 = max(0, cy_i - 6), min(h, cy_i + 7)
        x0, x1 = max(0, cx_i - 6), min(w, cx_i + 7)
        win = a[y0:y1, x0:x1]
        uy, ux = np.nonzero(win == UNKNOWN)
        if len(uy) == 0:
            return None
        vx = (ux.mean() + x0) - cx_i
        vy = (uy.mean() + y0) - cy_i
        n = math.hypot(vx, vy)
        if n < 1e-6:
            return None
        yaw = math.atan2(vy / n, vx / n)
        return (ox + (cx_i + 0.5) * res, oy + (cy_i + 0.5) * res, yaw)

    # ── scoring ──────────────────────────────────────────────────────────
    def _heuristic(self, robot, f: Frontier) -> float:
        rx, ry, ryaw = robot
        gx, gy, gyaw = f.goal
        d = math.hypot(gx - rx, gy - ry)
        bearing = math.atan2(gy - ry, gx - rx)
        # what the car must turn to set off towards it, and to arrive facing right
        turn = abs(wrap(bearing - ryaw)) + abs(wrap(gyaw - bearing))
        turn_cost = self.cfg["turn_weight"] * self.cfg["turn_radius"] * turn
        return d + turn_cost - self.cfg["gain_weight"] * f.size_m

    def _plan_length(self, goal) -> float | None:
        """True path cost from the planner that will actually drive it."""
        if not self.planner.wait_for_server(timeout_sec=2.0):
            return None
        g = ComputePathToPose.Goal()
        g.goal = self._pose(goal)
        g.use_start = False
        gh = self._wait(self.planner.send_goal_async(g), 5.0)
        if gh is None or not gh.accepted:
            return None
        r = self._wait(gh.get_result_async(), 8.0)
        if r is None or not r.result.path.poses:
            return None
        pts = [(p.pose.position.x, p.pose.position.y) for p in r.result.path.poses]
        return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

    def _pose(self, goal) -> PoseStamped:
        x, y, yaw = goal
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        return ps

    # ── blacklist (expiring, not permanent) ──────────────────────────────
    def _blacklisted(self, x, y) -> bool:
        now = time.monotonic()
        self.blacklist = [b for b in self.blacklist if b[2] > now]
        r = self.cfg["blacklist_radius"]
        return any(math.hypot(x - bx, y - by) < r for bx, by, _ in self.blacklist)

    def _blacklist(self, x, y, escalate=True):
        """Suppress a spot, for longer each time it actually fails.

        A fixed timeout means a genuinely unreachable frontier is retried
        forever; a permanent one is what stranded explore_lite. Escalating
        backs off without ever closing the door. Merely standing on a frontier
        is not a failure, so it gets the flat timeout — otherwise normal
        progress would ban half the map.
        """
        if escalate:
            key = (int(x * 2), int(y * 2))
            n = self._fail_count[key] = self._fail_count.get(key, 0) + 1
            secs = min(self.cfg["blacklist_seconds"] * n, 600.0)
            why = f"failure {n}"
        else:
            secs = self.cfg["blacklist_seconds"]
            why = "already here"
        self.blacklist.append((x, y, time.monotonic() + secs))
        self.get_logger().info(
            f"frontier ({x:.2f},{y:.2f}) suppressed {secs:.0f}s ({why})")

    # ── goal handling ────────────────────────────────────────────────────
    def _cancel_goal(self):
        """Give up the current goal. Our own doing, so not a frontier failure."""
        self._preempting = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.current = None

    def _send(self, f: Frontier):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose unavailable")
            return
        g = NavigateToPose.Goal()
        g.pose = self._pose(f.goal)
        self.current = f
        self.goal_started = time.monotonic()
        self._sent_from = self._last_xy
        self._preempting = False
        fut = self.nav.send_goal_async(g)
        fut.add_done_callback(lambda fu: self._on_accepted(fu, f))
        self.get_logger().info(
            f"goal ({f.goal[0]:.2f},{f.goal[1]:.2f}) yaw {math.degrees(f.goal[2]):.0f}deg "
            f"size {f.size_m:.2f}m path {f.path_len if f.path_len is None else round(f.path_len,2)}")

    def _on_accepted(self, fut, f):
        gh = fut.result()
        if gh is None or not gh.accepted:
            if self.current is f:
                self.current = None
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda fu: self._on_result(fu, f))

    def _on_result(self, fut, f):
        res = fut.result()
        if self.current is f:
            self.current, self.goal_handle = None, None
        if res is None:
            return
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            moved = math.dist((self._sent_from or (0.0, 0.0)), self._last_xy)
            if moved < self.cfg["futile_distance"]:
                self.get_logger().info(
                    f"goal succeeded without moving ({moved:.2f}m) — suppressing it")
                self._blacklist(*f.centroid, escalate=False)
            else:
                self.get_logger().info(f"goal reached, moved {moved:.2f}m")
            return
        # A goal we replaced comes back ABORTED, not CANCELED — bt_navigator
        # reports a preempted goal as a failure. Blacklisting on that is what
        # eventually left explore_lite with nowhere to go.
        if res.status == GoalStatus.STATUS_CANCELED or self._preempting:
            self._preempting = False
            return
        self._blacklist(*f.centroid)

    def _stuck(self, robot) -> bool:
        """True when the robot has been shuffling on the spot.

        Nav2's recovery BT will happily back up and retry for minutes; from
        outside we can see it is getting nowhere and spend the time on a
        different frontier instead.
        """
        if len(self._track) < 4 or self._track[0][0] > self.goal_started:
            return False              # not enough history on this goal yet
        xs = [p[1] for p in self._track]
        ys = [p[2] for p in self._track]
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return span < self.cfg["stuck_distance"]

    def _still_useful(self, f: Frontier) -> bool:
        """A goal is worth finishing while it is reachable and still faces unknown.

        Driving towards a frontier usually reveals it before arrival; without
        this the robot would dutifully complete a goal that no longer buys
        any information.
        """
        g = self.grid
        res, w, h = g.info.resolution, g.info.width, g.info.height
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        a = np.frombuffer(bytes(g.data), dtype=np.int8).reshape(h, w).astype(np.int16)
        gx = int((f.goal[0] - ox) / res)
        gy = int((f.goal[1] - oy) / res)
        if not (0 <= gx < w and 0 <= gy < h):
            return False
        if a[gy, gx] > self.cfg["free_threshold"] or a[gy, gx] < 0:
            return False              # something moved into it
        r = int(self.cfg["useful_radius"] / res)
        win = a[max(0, gy - r):gy + r + 1, max(0, gx - r):gx + r + 1]
        return bool((win == UNKNOWN).any())

    # ── main loop ────────────────────────────────────────────────────────
    def _tick(self):
        if not self.enabled or self.grid is None:
            return
        robot = self._robot_pose()
        if robot is None:
            return
        # Sampled every cycle, not just while a goal is live: goals that
        # succeed instantly would otherwise keep resetting the history and
        # no stall would ever be detected.
        now = time.monotonic()
        self._track.append((now, robot[0], robot[1]))
        self._track = [p for p in self._track if p[0] >= now - self.cfg["stuck_seconds"]]

        cands = self._frontiers()
        self._markers(cands)
        if not cands:
            if self.blacklist:        # suppressed, not exhausted — they come back
                self.get_logger().info("all frontiers temporarily suppressed",
                                       throttle_duration_sec=20.0)
                return
            self._status(ExploreStatus.EXPLORATION_COMPLETE)
            self.get_logger().info("no frontiers; idling", throttle_duration_sec=20.0)
            return
        self._status(ExploreStatus.EXPLORATION_IN_PROGRESS)

        # Commitment. An Ackermann robot spends most of a short goal just
        # lining up, so abandoning goals every cycle means never finishing a
        # manoeuvre — this is what made explore_lite look indecisive.
        if self.current is not None:
            gx, gy, _ = self.current.goal
            near = math.hypot(gx - robot[0], gy - robot[1])
            if near < self.cfg["arrive_radius"]:
                # Close enough. The goal yaw was only ever a hint about which
                # way to face the unknown; a car cannot turn on the spot, so
                # insisting on it means shuffling back and forth to shave off
                # the last few degrees.
                self.get_logger().info(f"arrived within {near:.2f}m — next frontier")
                self._cancel_goal()
            elif self._stuck(robot):
                self.get_logger().info("no progress — abandoning this frontier")
                self._blacklist(*self.current.centroid)
                self._cancel_goal()
            elif not self._still_useful(self.current):
                self.get_logger().info("current goal no longer faces unknown space")
                self._cancel_goal()
            elif time.monotonic() - self.goal_started < self.cfg["commit_seconds"]:
                return

        # A goal we have already reached makes Nav2 report instant success
        # without moving, and the frontier then survives to be chosen again
        # next cycle — forever.
        near = [f for f in cands
                if math.dist(f.goal[:2], robot[:2]) < self.cfg["min_goal_distance"]]
        for f in near:
            self._blacklist(*f.centroid, escalate=False)
        cands = [f for f in cands if f not in near]
        if not cands:
            self.get_logger().info("only frontiers we already stand on",
                                   throttle_duration_sec=10.0)
            return

        for f in cands:
            f.score = self._heuristic(robot, f)
        cands.sort(key=lambda f: f.score)

        # confirm the shortlist against the planner that will drive it, so an
        # unreachable frontier is discarded before it wastes a goal
        best = None
        for f in cands[: int(self.cfg["verify_top_k"])]:
            L = self._plan_length(f.goal)
            if L is None:
                continue
            f.path_len = L
            f.score = L - self.cfg["gain_weight"] * f.size_m
            if best is None or f.score < best.score:
                best = f
        if best is None:
            self.get_logger().info("no reachable frontier this cycle",
                                   throttle_duration_sec=10.0)
            return

        # Only abandon a live goal for one that is better by a real margin,
        # scored the same way (path metres less frontier reward) so the two
        # numbers are actually comparable.
        if self.current is not None:
            cur_len = self._plan_length(self.current.goal)
            cur = (math.inf if cur_len is None
                   else cur_len - self.cfg["gain_weight"] * self.current.size_m)
            if best.score > cur - self.cfg["switch_margin"]:
                self.goal_started = time.monotonic()
                return
            self.get_logger().info(
                f"switching: {best.score:.2f} beats {cur:.2f} by more than "
                f"{self.cfg['switch_margin']:.1f}m")
            self._cancel_goal()

        self._send(best)

    # ── visualisation ────────────────────────────────────────────────────
    def _markers(self, cands):
        arr = MarkerArray()
        m = Marker(); m.header.frame_id = "map"; m.ns = "frontiers"; m.id = 0
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        for i, f in enumerate(cands[:20]):
            a = Marker()
            a.header.frame_id = "map"
            a.header.stamp = self.get_clock().now().to_msg()
            a.ns, a.id, a.type, a.action = "frontiers", i + 1, Marker.ARROW, Marker.ADD
            a.pose = self._pose(f.goal).pose
            a.scale.x, a.scale.y, a.scale.z = 0.35, 0.06, 0.06
            best = (i == 0)
            a.color = ColorRGBA(r=0.0 if best else 1.0, g=1.0 if best else 0.6,
                                b=0.0, a=0.9)
            arr.markers.append(a)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = FrontierExplorer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
