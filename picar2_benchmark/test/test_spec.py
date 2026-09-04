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
