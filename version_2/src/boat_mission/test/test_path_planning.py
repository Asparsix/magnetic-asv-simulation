"""Unit tests for coverage and hunt path generators."""

import math

from boat_mission.path_planning import (
    generate_expanding_spiral,
    generate_info_gain_candidates,
    generate_lawnmower,
    path_length,
    plan_info_gain_waypoint,
)


def test_lawnmower_covers_bounds_and_alternates():
    path = generate_lawnmower(-100.0, 100.0, -100.0, 100.0, spacing=50.0)
    assert len(path) >= 4
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    assert min(xs) >= -100.0 - 1e-9
    assert max(xs) <= 100.0 + 1e-9
    assert min(ys) >= -100.0 - 1e-9
    assert max(ys) <= 100.0 + 1e-9
    # First lane left→right, second right→left
    assert path[0] == (-100.0, -100.0)
    assert path[1] == (100.0, -100.0)
    assert path[2] == (100.0, -50.0)
    assert path[3] == (-100.0, -50.0)
    assert path_length(path) > 800.0


def test_lawnmower_rejects_bad_spacing():
    try:
        generate_lawnmower(0.0, 10.0, 0.0, 10.0, spacing=0.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_spiral_expands_around_center():
    center = (10.0, -5.0)
    spiral = generate_expanding_spiral(
        center,
        step_spacing=20.0,
        max_radius=40.0,
        ring_spacing=20.0,
        min_points_per_ring=8,
    )
    assert len(spiral) >= 8
    radii = [math.hypot(x - center[0], y - center[1]) for x, y in spiral]
    assert max(radii) <= 40.0 + 1e-6
    assert min(radii) >= 20.0 - 1e-6


def test_verification_orbit_in_path_planning():
    from boat_mission.path_planning import generate_verification_orbit
    orbit = generate_verification_orbit((0.0, 0.0), 10.0, num_points=8)
    assert len(orbit) >= 8
    assert orbit[0] == orbit[-1]


def test_info_gain_prefers_peak_direction():
    current = (0.0, 0.0)
    peak = (50.0, 0.0)
    waypoint, score = plan_info_gain_waypoint(
        current, peak, radii=[10.0, 20.0], num_angles=8
    )
    assert waypoint[0] > 0.0
    assert abs(waypoint[1]) < 15.0
    assert score < 50.0


def test_info_gain_candidates_respect_bounds():
    candidates = generate_info_gain_candidates(
        (0.0, 0.0),
        radii=[30.0],
        num_angles=4,
        bounds=((-10.0, -10.0), (10.0, 10.0)),
    )
    for x, y in candidates:
        assert -10.0 <= x <= 10.0
        assert -10.0 <= y <= 10.0
