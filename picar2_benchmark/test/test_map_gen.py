import numpy as np

from picar2_benchmark.map_gen import FREE, OCCUPIED, UNKNOWN, rasterise
from picar2_benchmark.spec import Box, Pose, Scenario


def _sc(obstacles=None):
    return Scenario(name='t', size=(10.0, 6.0), start=Pose(-3, 0), goal=Pose(3, 0),
                    obstacles=obstacles or [])


def _at(img, origin, res, x, y):
    return img[int((y - origin[1]) / res), int((x - origin[0]) / res)]


def test_interior_is_free_and_border_is_unknown():
    sc = _sc()
    img, origin = rasterise(sc)
    assert _at(img, origin, 0.05, 0.0, 0.0) == FREE
    # beyond the boundary wall there must be unknown, not free: Smac's
    # allow_unknown path has to be exercised
    assert img[0, 0] == UNKNOWN
    assert UNKNOWN in np.unique(img)


def test_boundary_walls_are_occupied():
    sc = _sc()
    img, origin = rasterise(sc)
    assert _at(img, origin, 0.05, 5.1, 0.0) == OCCUPIED     # right wall at x=5.1
    assert _at(img, origin, 0.05, 0.0, 3.1) == OCCUPIED     # top wall at y=3.1


def test_obstacle_is_stamped():
    sc = _sc([Box(1.0, 0.0, 0.4, 0.4)])
    img, origin = rasterise(sc)
    assert _at(img, origin, 0.05, 1.0, 0.0) == OCCUPIED
    assert _at(img, origin, 0.05, 2.0, 0.0) == FREE


def test_a_doorway_gap_stays_free():
    """The gap between two wall segments must survive rasterisation, or the
    scenario is unsolvable for reasons that have nothing to do with Nav2."""
    gap = 0.9
    y = gap / 2 + 1.5
    sc = _sc([Box(0.0, y, 0.2, 3.0), Box(0.0, -y, 0.2, 3.0)])
    img, origin = rasterise(sc)
    assert _at(img, origin, 0.05, 0.0, 0.0) == FREE
    assert _at(img, origin, 0.05, 0.0, y) == OCCUPIED
