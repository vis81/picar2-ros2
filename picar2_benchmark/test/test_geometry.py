import math

from picar2_benchmark.geometry import clearance, poly_box_distance, transform
from picar2_benchmark.spec import Box


def test_footprint_transform_translates_and_rotates():
    at_origin = transform((0.0, 0.0, 0.0))
    assert (0.30, 0.095) in [(round(x, 6), round(y, 6)) for x, y in at_origin]
    # 90 deg: the +x extent becomes +y
    turned = [(round(x, 6), round(y, 6)) for x, y in transform((0.0, 0.0, math.pi / 2))]
    assert (-0.095, 0.30) in turned


def test_clearance_positive_when_clear():
    box = Box(3.0, 0.0, 0.2, 2.0)          # wall at x=3
    # footprint nose reaches x=0.30, wall face at x=2.9 -> 2.6 m of clearance
    assert abs(clearance((0.0, 0.0, 0.0), [box]) - 2.6) < 1e-6


def test_clearance_negative_when_overlapping():
    box = Box(0.1, 0.0, 0.4, 0.4)
    assert clearance((0.0, 0.0, 0.0), [box]) < 0.0


def test_clearance_accounts_for_orientation():
    """A gap the robot fits through lengthwise but not sideways."""
    box_l = Box(0.0, 0.6, 4.0, 0.2)
    box_r = Box(0.0, -0.6, 4.0, 0.2)
    # pointing along the corridor: half-width 0.095 vs gap half 0.5 -> clear
    assert clearance((0.0, 0.0, 0.0), [box_l, box_r]) > 0.3
    # pointing across it: the 0.30 m nose reaches into the wall
    assert clearance((0.0, 0.0, math.pi / 2), [box_l, box_r]) < 0.3


def test_distance_is_symmetric_across_the_box_faces():
    box = Box(0.0, 0.0, 1.0, 1.0)
    poly = transform((3.0, 0.0, 0.0))
    left = poly_box_distance(poly, box)
    poly2 = transform((-3.0, 0.0, math.pi))
    assert abs(left - poly_box_distance(poly2, box)) < 1e-9
