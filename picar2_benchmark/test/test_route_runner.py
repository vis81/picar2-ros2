"""Route logic without ROS.

The parts worth pinning down here are pure arithmetic — which waypoints go in a
window, when a pass is counted, and whether the metrics flag a waypoint that was
credited without being reached. Stubbing ROS keeps them testable on any machine
rather than only inside a sourced workspace, and they are the parts a sim run is
too slow and too noisy to exercise exhaustively.
"""
import sys
import types


def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
    def __call__(self, *a, **k): return _Any()


class _PoseStamped:
    class _V:
        def __init__(self): self.x = self.y = self.z = 0.0; self.w = 1.0

    def __init__(self):
        self.header = _Any()
        self.pose = types.SimpleNamespace(position=_PoseStamped._V(),
                                          orientation=_PoseStamped._V())


if 'rclpy' not in sys.modules:
    _mod('rclpy', init=_Any(), shutdown=_Any(), spin_once=_Any())
    _mod('rclpy.action', ActionClient=_Any)
    _mod('rclpy.node', Node=object)
    _mod('rclpy.qos', QoSProfile=_Any, DurabilityPolicy=_Any(),
         ReliabilityPolicy=_Any(), HistoryPolicy=_Any())
    _mod('action_msgs.msg', GoalStatus=types.SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
         GoalStatusArray=_Any)
    _mod('action_msgs')
    _mod('geometry_msgs.msg', PoseStamped=_PoseStamped, Twist=_Any)
    _mod('geometry_msgs')
    _mod('nav2_msgs.action', NavigateToPose=_Any,
         NavigateThroughPoses=types.SimpleNamespace(Goal=_Any))
    _mod('nav2_msgs')
    _mod('nav_msgs.msg', OccupancyGrid=_Any, Odometry=_Any, Path=_Any)
    _mod('nav_msgs')
    _mod('nav2_msgs.msg', BehaviorTreeLog=_Any)
    _mod('rclpy.duration', Duration=_Any)
    _mod('rclpy.parameter', Parameter=_Any)
    _mod('tf2_ros', Buffer=_Any, TransformListener=_Any)
    _mod('sensor_msgs.msg', LaserScan=_Any, PointCloud2=_Any)
    _mod('sensor_msgs')

from picar2_benchmark import spec                       # noqa: E402
from picar2_benchmark.route_runner import RouteRunner    # noqa: E402


class _Ctx:
    """Just enough GateContext to drive the arithmetic."""
    def __init__(self):
        self.gt_seq = 0
        self._p = (0.0, 0.0, 0.0)
        self.t = 0.0

    def gt_pose(self): return self._p
    def get_clock(self):
        outer = self
        class _C:
            def now(self):
                class _N:
                    nanoseconds = int(outer.t * 1e9)
                return _N()
        return _C()


def _runner(wps, loop=False, flow=True, laps=1, capture=0.45):
    sc = spec.Scenario(
        name='t', size=(20.0, 20.0), start=spec.Pose(0, 0, 0),
        goal=spec.Pose(1, 1, 0),
        route={'waypoints': [{'x': x, 'y': y} for x, y in wps],
               'loop': loop, 'laps': laps, 'capture_radius_m': capture})
    ctx = _Ctx()
    r = RouteRunner(ctx, sc, dict(sc.route, flow=flow), rec=_Any())
    return r, ctx


def _at(r, ctx, x, y):
    """Move the robot and run one iteration of the capture rule."""
    ctx._p = (x, y, 0.0)
    ctx.t += 0.1
    d = r._dist_to_target()
    if r._min_d is None or d < r._min_d:
        r._min_d = d
    if r._min_d < r.capture and d > r._min_d + r.recede:
        r._pass_current(r._min_d)
        return True
    return False


def test_window_holds_three_and_wraps_when_looping():
    r, _ = _runner([(0, 0), (2, 0), (4, 0), (6, 0)], loop=True)
    assert [(p.x, p.y) for p in r._window_poses()] == [(0, 0), (2, 0), (4, 0)]
    r.idx = 3
    assert [(p.x, p.y) for p in r._window_poses()] == [(6, 0), (0, 0), (2, 0)]


def test_window_runs_out_at_the_end_of_a_one_shot():
    """A one-shot route must not wrap; the window simply shortens."""
    r, _ = _runner([(0, 0), (2, 0), (4, 0)], loop=False)
    r.idx = 2
    assert [(p.x, p.y) for p in r._window_poses()] == [(4, 0)]


def test_a_waypoint_is_counted_at_its_closest_approach():
    r, ctx = _runner([(0, 0), (3, 0)], loop=True)
    assert not _at(r, ctx, -0.40, 0.0)      # approaching
    assert not _at(r, ctx, -0.03, 0.0)      # nearest point
    assert _at(r, ctx, 0.20, 0.0)           # receding -> counted
    assert r.passed == 1
    assert r.approaches == [(0, 0.03)]      # the closest approach, not the last


def test_a_distant_pass_is_not_counted():
    r, ctx = _runner([(0, 0), (3, 0)], loop=True)
    _at(r, ctx, 0.0, 1.2)
    _at(r, ctx, 0.6, 1.4)
    assert r.passed == 0


def test_metrics_flag_a_waypoint_credited_without_being_reached():
    """The defect this whole runner exists to catch: Nav2 reporting SUCCEEDED
    on a route it drove nowhere near. The pass is counted, and flagged."""
    r, _ = _runner([(0, 0), (3, 0)], loop=False, capture=0.45)
    r.approaches = [(0, 0.05), (1, 0.75)]
    r.passed = 2
    m = r.metrics(elapsed=20.0, wall=40.0, target=2)
    assert m['missed_waypoints'] == 1
    assert m['missed_detail'] == [{'wp': 1, 'closest_m': 0.75}]
    assert m['approach_max_m'] == 0.75
    assert m['completed'] is True          # completed, but not clean


def test_metrics_report_no_misses_when_every_waypoint_was_reached():
    r, _ = _runner([(0, 0), (3, 0)], loop=False)
    r.approaches = [(0, 0.05), (1, 0.11)]
    r.passed = 2
    m = r.metrics(elapsed=20.0, wall=40.0, target=2)
    assert m['missed_waypoints'] == 0
    assert m['approach_mean_m'] == 0.08


def test_lap_time_comes_from_one_pass_per_waypoint_apart():
    """A lap is the gap between successive visits to the same waypoint, so it
    is measured n arrivals apart — not by dividing the total."""
    r, _ = _runner([(0, 0), (3, 0), (3, 3)], loop=True, laps=2)
    r.arrivals = [0.0, 5.0, 11.0, 20.0, 25.0, 31.0, 40.0]
    r.approaches = [(i % 3, 0.05) for i in range(7)]
    r.passed = 7
    m = r.metrics(elapsed=40.0, wall=80.0, target=6)
    assert m['lap_time_s'] == 20.0
    assert m['lap_time_min_s'] == 20.0


def test_incomplete_route_is_not_reported_as_completed():
    r, _ = _runner([(0, 0), (3, 0), (3, 3)], loop=True, laps=2)
    r.approaches = [(0, 0.05), (1, 0.06)]
    r.passed = 2
    m = r.metrics(elapsed=99.0, wall=198.0, target=6)
    assert m['completed'] is False
    assert m['passes'] == 2 and m['target_passes'] == 6
