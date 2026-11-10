import math
from pillar_camera_sim.geometry import project_point, estimate_from_two_centers


def test_symmetric_pillars():
    left = project_point(3, 0.33, 0.9, 0, 0, 0.9, 0, 700, 700, 800, 600)
    right = project_point(3, -0.33, 0.9, 0, 0, 0.9, 0, 700, 700, 800, 600)
    result = estimate_from_two_centers(left[0], right[0], .66, 700, 800)
    assert math.isclose(result[0], 3, abs_tol=1e-8)
    assert math.isclose(result[1], 0, abs_tol=1e-8)


def test_camera_moves_forward():
    u0 = project_point(3, -.33, .9, 0, 0, .9, 0, 700, 700, 800, 600)[0]
    u1 = project_point(3, -.33, .9, 1, 0, .9, 0, 700, 700, 800, 600)[0]
    assert u1 > u0


def test_behind_camera_hidden():
    assert project_point(-1, 0, 1, 0, 0, 1, 0, 700, 700, 800, 600) is None
