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
