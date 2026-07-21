import math

from boat_navigation.guidance import (
    los_heading,
    segment_geometry,
    should_advance_segment,
    speed_scale_for_heading_error,
    wrap_angle,
)
from boat_navigation.path_utils import patrol_route, path_points
from boat_navigation.pid import PIDController


def test_wrap_angle():
    assert abs(wrap_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-6


def test_cross_track_sign():
    along, cross, length = segment_geometry((0.0, 0.0), (10.0, 0.0), (5.0, 2.0))
    assert length == 10.0
    assert along == 5.0
    assert cross > 0.0


def test_los_heading_pulls_to_path():
    desired, _, cross = los_heading((0.0, 0.0), (10.0, 0.0), (5.0, 2.0), 5.0)
    assert cross > 0.0
    assert desired < 0.0
    desired_center, _, cross_center = los_heading(
        (0.0, 0.0), (10.0, 0.0), (5.0, 0.0), 5.0
    )
    assert abs(cross_center) < 1e-6
    assert abs(desired_center) < 1e-6


def test_advance_on_along_track():
    assert should_advance_segment(
        (0.0, 0.0),
        (10.0, 0.0),
        (9.0, 0.0),
        9.0,
        10.0,
        8.0,
        2.0,
    )


def test_patrol_geometry():
    path = patrol_route(120.0)
    points = path_points(path)
    xs = [p[0] for p in points[:-1]]
    ys = [p[1] for p in points[:-1]]
    assert min(xs) == -120.0
    assert max(xs) == 120.0
    assert min(ys) == -120.0
    assert max(ys) == 120.0
    assert points[0] == points[-1]


def test_pid_anti_windup():
    pid = PIDController(1.0, 1.0, 0.0, -1.0, 1.0, 0.5)
    for _ in range(20):
        pid.update(5.0, 0.1)
    assert abs(pid.integral) <= 0.5 + 1e-6


def test_speed_scale_for_heading_error():
    assert speed_scale_for_heading_error(0.0) == 1.0
    assert speed_scale_for_heading_error(math.pi / 2.0) == 0.2
