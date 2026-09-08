from pathlib import Path
import pytest

from picar2_benchmark import spec


def test_slam_envelope_matches_measured_map():
    """The model behind the slam validation, checked against hardware-in-sim
    numbers: on the original 14x8 open world with a 5 m lidar and walls 4 m to
    each side, only rays within 53 deg of the side walls return, so the map
    reaches sqrt(5^2-4^2)=3.0 m ahead. Measured 147/360 finite rays."""
    sc = spec.Scenario(
        name='probe', size=(14.0, 8.0),
        start=spec.Pose(-2.0, 0.0, 0.0), goal=spec.Pose(2.0, 0.0, 0.0))
    x0, y0, x1, y1 = spec.slam_envelope(sc)
    # forward reach from the start, limited by oblique hits on the side walls
    assert 2.8 < x1 - sc.start.x < 3.2
    # sideways it sees the walls themselves
    assert abs(y1 - 4.0) < 0.1 and abs(y0 + 4.0) < 0.1


def test_validate_rejects_goal_outside_slam_envelope():
    sc = spec.Scenario(
        name='too_far', size=(14.0, 8.0),
        start=spec.Pose(-5.0, 0.0, 0.0), goal=spec.Pose(5.0, 0.0, 0.0))
    with pytest.raises(ValueError, match='lidar can observe'):
        spec.validate(sc)


def test_shipped_scenarios_are_runnable_in_every_mode():
    """Every scenario must validate, which now includes being observable under
    slam — otherwise ground_truth and slam cannot be compared on it."""
    from pathlib import Path
    d = Path(__file__).resolve().parent.parent / 'scenarios'
    files = sorted(d.glob('*.yaml'))
    assert files, 'no scenarios found'
    for f in files:
        spec.validate(spec.load(f))


def test_write_repro_emits_a_loadable_scenario(tmp_path):
    """The repro path only runs when a trial fails, which is rare enough
    (3 of 19 corner_right runs) that it would otherwise ship unverified."""
    from pathlib import Path
    sc = spec.load(Path(__file__).resolve().parent.parent
                   / 'scenarios' / 'corner_right.yaml')
    out = spec.write_repro(sc, (0.42, -0.63, -1.1), tmp_path)
    again = spec.load(out)
    spec.validate(again)
    assert again.name == 'corner_right_repro'
    assert abs(again.start.x - 0.42) < 1e-6 and abs(again.start.y + 0.63) < 1e-6
    assert (again.goal.x, again.goal.y) == (sc.goal.x, sc.goal.y)
    assert len(again.obstacles) == len(sc.obstacles)
    assert again.size == sc.size


def test_write_repro_handles_a_scenario_with_no_obstacles(tmp_path):
    sc = spec.load(Path(__file__).resolve().parent.parent
                   / 'scenarios' / 'open_straight.yaml') if False else spec.Scenario(
        name='bare', size=(5.5, 4.5),
        start=spec.Pose(-2.0, 0.0, 0.0), goal=spec.Pose(2.0, 0.0, 0.0))
    out = spec.write_repro(sc, (0.0, 0.0, 0.0), tmp_path)
    again = spec.load(out)
    assert again.obstacles == []


def test_pose_before_picks_the_pose_ahead_of_the_failure():
    bt = [(5.0, 'FollowPath', 'SUCCESS'), (9.0, 'FollowPath', 'FAILURE'),
          (11.0, 'FollowPath', 'FAILURE')]
    poses = [(t / 2.0, t / 2.0, 0.0, 0.0) for t in range(0, 40)]
    x, y, yaw = spec.pose_before(bt, poses)
    assert abs(x - 8.0) < 0.6                     # ~1 s of run-up before t=9
    assert spec.pose_before([], poses) is None    # no failure -> nothing to repro
    assert spec.pose_before(bt, []) is None


def test_explore_scenario_validates_without_a_reachable_goal():
    """Exploration has no goal pose, so the slam-envelope and goal-distance
    checks that every navigation scenario must pass do not apply to it."""
    sc = spec.load(Path(__file__).resolve().parent.parent
                   / 'scenarios' / 'explore_room.yaml')
    spec.validate(sc)
    assert sc.explore is not None
    assert sc.explore['duration_s'] > 0


def test_explore_block_is_validated():
    base = dict(name='x', size=(6.0, 5.0),
                start=spec.Pose(-2.0, 0.0, 0.0), goal=spec.Pose(2.0, 0.0, 0.0))
    with pytest.raises(ValueError, match='duration_s'):
        spec.validate(spec.Scenario(**base, explore={'duration_s': 0}))
    with pytest.raises(ValueError, match='target_coverage'):
        spec.validate(spec.Scenario(**base,
                                    explore={'duration_s': 60, 'target_coverage': 1.5}))


def test_free_area_matches_the_geometry():
    """An empty room's free area is its interior, so this is checkable by hand
    and pins the rasteriser the coverage metric depends on."""
    sc = spec.Scenario(name='r', size=(8.0, 6.0),
                       start=spec.Pose(-3.0, -2.0, 0.0), goal=spec.Pose(0.0, 0.0, 0.0))
    assert abs(spec.free_area_m2(sc) - 48.0) < 1.0     # 8 x 6 less wall rounding


def _route(**over):
    """A minimal valid route scenario, for the rejection tests to spoil."""
    r = {'loop': False, 'laps': 1,
         'waypoints': [{'x': -1.5, 'y': 0.0}, {'x': 1.5, 'y': 0.0}]}
    r.update(over)
    return spec.Scenario(name='x', size=(8.0, 6.0),
                         start=spec.Pose(-2.5, 0.0, 0.0),
                         goal=spec.Pose(0.0, 0.0, 0.0), route=r)


def test_route_scenario_validates_without_a_reachable_goal():
    """A route has no single goal, so the goal-distance and slam-envelope
    checks a navigation scenario must pass do not apply. Route trials run in
    ground_truth only, where the costmap comes from the static map."""
    spec.validate(_route())


def test_route_needs_at_least_two_waypoints():
    with pytest.raises(ValueError, match='at least two waypoints'):
        spec.validate(_route(waypoints=[{'x': 1.0, 'y': 0.0}]))


def test_route_rejects_waypoints_too_close_to_tell_apart():
    """Closer than twice the capture radius and the robot is inside both at
    once, so a pass through one cannot be attributed."""
    with pytest.raises(ValueError, match='capture radius'):
        spec.validate(_route(waypoints=[{'x': 0.0, 'y': 0.0},
                                        {'x': 0.5, 'y': 0.0}]))


def test_route_rejects_a_waypoint_outside_the_world():
    with pytest.raises(ValueError, match='outside the world'):
        spec.validate(_route(waypoints=[{'x': -1.5, 'y': 0.0},
                                        {'x': 99.0, 'y': 0.0}]))


def test_route_rejects_a_waypoint_inside_an_obstacle():
    sc = spec.Scenario(name='x', size=(8.0, 6.0),
                       start=spec.Pose(-2.5, 0.0, 0.0),
                       goal=spec.Pose(0.0, 0.0, 0.0),
                       obstacles=[spec.Box(1.5, 0.0, 0.6, 0.6)],
                       route={'loop': False, 'laps': 1,
                              'waypoints': [{'x': -1.5, 'y': 0.0},
                                            {'x': 1.5, 'y': 0.0}]})
    with pytest.raises(ValueError, match='overlaps an obstacle'):
        spec.validate(sc)


def test_route_laps_require_a_loop():
    """Driving the list twice without looping would just stop at the end."""
    with pytest.raises(ValueError, match='requires route.loop'):
        spec.validate(_route(laps=2, loop=False))
    spec.validate(_route(laps=2, loop=True))


def test_a_scenario_is_a_route_or_an_exploration_not_both():
    with pytest.raises(ValueError, match='not both'):
        spec.validate(spec.Scenario(
            name='x', size=(8.0, 6.0), start=spec.Pose(-2.5, 0.0, 0.0),
            goal=spec.Pose(0.0, 0.0, 0.0),
            explore={'duration_s': 60},
            route={'waypoints': [{'x': -1.5, 'y': 0.0}, {'x': 1.5, 'y': 0.0}]}))


def test_route_waypoints_carry_their_heading():
    sc = _route(waypoints=[{'x': -1.5, 'y': 0.0, 'yaw': 1.0},
                           {'x': 1.5, 'y': 0.0}])
    w = sc.route_waypoints
    assert (w[0].x, w[0].yaw) == (-1.5, 1.0)
    assert w[1].yaw == 0.0          # absent heading defaults, never crashes


def test_a_low_box_does_not_occlude_the_lidar():
    """A 6 cm box is below the LD19's 0.1485 m scan plane, so the lidar sees
    straight over it. Ray casting has no notion of height, so it has to be told,
    or a low obstacle would appear to hide everything behind it from a sensor
    that can see it perfectly well."""
    tall = spec.Scenario(name='x', size=(6.0, 6.0),
                         start=spec.Pose(-2.0, 0.0, 0.0),
                         goal=spec.Pose(2.0, 0.0, 0.0),
                         obstacles=[spec.Box(0.0, 0.0, 0.5, 1.2)])
    low = spec.Scenario(name='x', size=(6.0, 6.0),
                        start=spec.Pose(-2.0, 0.0, 0.0),
                        goal=spec.Pose(2.0, 0.0, 0.0),
                        obstacles=[spec.Box(0.0, 0.0, 0.5, 1.2, sz=0.06)])
    from picar2_benchmark.geometry import ray_hit
    # Straight ahead, where the box actually sits: the tall one stops the ray
    # at its near face, the low one does not stop it at all.
    o = (-2.0, 0.0)
    assert ray_hit(o, 0.0, tall.all_boxes, 5.0) == pytest.approx(1.75, abs=0.01)
    assert ray_hit(o, 0.0, [b for b in low.all_boxes
                            if b.sz >= spec.LIDAR_HEIGHT_M], 5.0) \
        == pytest.approx(5.0, abs=0.01)
    # ...so the envelope reaches further with the low box than the tall one.
    assert spec.slam_envelope(low)[2] > spec.slam_envelope(tall)[2]


def test_an_unmapped_obstacle_stays_out_of_the_static_map():
    """The only way to test that a sensor still works: a mapped obstacle is
    planned around whether or not anything ever detects it."""
    from picar2_benchmark import map_gen
    base = dict(name='x', size=(5.0, 5.0), start=spec.Pose(-1.5, 0.0, 0.0),
                goal=spec.Pose(1.5, 0.0, 0.0))
    seen = spec.Scenario(**base, obstacles=[spec.Box(0, 0, .5, 1.2, .06, True)])
    hidden = spec.Scenario(**base, obstacles=[spec.Box(0, 0, .5, 1.2, .06, False)])
    a, _ = map_gen.rasterise(seen, 0.05)
    b, _ = map_gen.rasterise(hidden, 0.05)
    assert (a != map_gen.FREE).sum() > (b != map_gen.FREE).sum()


def test_expectations_compare_against_the_result_and_its_metrics():
    sc = spec.Scenario(name='x', size=(6.0, 5.0), start=spec.Pose(-2.0, 0, 0),
                       goal=spec.Pose(2.0, 0, 0),
                       expect={'outcome': 'SUCCEEDED',
                               'max_direction_reversals': 10})
    ok = spec.check_expectations(
        sc, {'outcome': 'SUCCEEDED', 'metrics': {'direction_reversals': 0}})
    assert ok['passed'] and ok['failed'] == 0

    bad = spec.check_expectations(
        sc, {'outcome': 'SUCCEEDED', 'metrics': {'direction_reversals': 105}})
    assert not bad['passed'] and bad['failed'] == 1
    assert [c['check'] for c in bad['checks'] if not c['ok']] \
        == ['max_direction_reversals']


def test_an_unmeasured_metric_fails_rather_than_passes_silently():
    """A bound on something that was never recorded must not read as met."""
    sc = spec.Scenario(name='x', size=(6.0, 5.0), start=spec.Pose(-2.0, 0, 0),
                       goal=spec.Pose(2.0, 0, 0),
                       expect={'max_nonexistent_thing': 1})
    r = spec.check_expectations(sc, {'outcome': 'SUCCEEDED'})
    assert not r['passed']
    assert 'not measured' in r['checks'][0]['detail']


def test_a_scenario_without_expectations_cannot_fail_them():
    sc = spec.Scenario(name='x', size=(6.0, 5.0), start=spec.Pose(-2.0, 0, 0),
                       goal=spec.Pose(2.0, 0, 0))
    assert spec.check_expectations(sc, {'outcome': 'ABORTED'}) == {}


def test_min_bounds_are_honoured():
    sc = spec.Scenario(name='x', size=(6.0, 5.0), start=spec.Pose(-2.0, 0, 0),
                       goal=spec.Pose(2.0, 0, 0),
                       expect={'min_min_clearance_m': 0.0})
    assert spec.check_expectations(sc, {'metrics': {'min_clearance_m': 0.12}})['passed']
    assert not spec.check_expectations(
        sc, {'metrics': {'min_clearance_m': -0.03}})['passed']
