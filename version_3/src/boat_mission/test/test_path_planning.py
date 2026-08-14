"""Unit tests for coverage and hunt path generators."""

import math

from boat_mission.path_planning import (
    generate_expanding_spiral,
    generate_info_gain_candidates,
    generate_lawnmower,
    generate_region_lawnmower,
    generate_split_lawnmower,
    generate_strip_lawnmower,
    path_length,
    plan_info_gain_waypoint,
    strip_coverage_bounds,
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


def test_split_lawnmower_partitions_lanes_without_overlap():
    shared = dict(
        min_x=-100.0,
        max_x=100.0,
        min_y=-100.0,
        max_y=100.0,
        spacing=50.0,
        num_asvs=2,
    )
    path0 = generate_split_lawnmower(asv_index=0, **shared)
    path1 = generate_split_lawnmower(asv_index=1, **shared)

    ys0 = sorted({round(y, 6) for _, y in path0})
    ys1 = sorted({round(y, 6) for _, y in path1})
    assert ys0 == [-100.0, 0.0, 100.0]
    assert ys1 == [-50.0, 50.0]
    assert set(ys0).isdisjoint(ys1)

    # Each ASV still zig-zags on its own lane sequence.
    assert path0[0] == (-100.0, -100.0)
    assert path0[1] == (100.0, -100.0)
    assert path0[2] == (100.0, 0.0)
    assert path0[3] == (-100.0, 0.0)
    assert path1[0] == (-100.0, -50.0)
    assert path1[1] == (100.0, -50.0)


def test_split_lawnmower_single_asv_matches_full():
    full = generate_lawnmower(-80.0, 80.0, -60.0, 60.0, spacing=30.0)
    split = generate_split_lawnmower(
        -80.0, 80.0, -60.0, 60.0, spacing=30.0, asv_index=0, num_asvs=1
    )
    assert split == full


def test_strip_lawnmower_three_asvs_leave_gap_corridor():
    """Vertical strips never share an x-seam; gap corridor stays empty."""
    shared = dict(
        min_x=-120.0,
        max_x=120.0,
        min_y=-120.0,
        max_y=120.0,
        spacing=20.0,
        num_asvs=3,
        gap_m=20.0,
    )
    paths = [
        generate_strip_lawnmower(asv_index=i, **shared) for i in range(3)
    ]
    for path in paths:
        assert len(path) >= 4
    # Pairwise x-ranges do not overlap.
    ranges = [
        (min(x for x, _ in p), max(x for x, _ in p)) for p in paths
    ]
    assert ranges[0][1] < ranges[1][0]
    assert ranges[1][1] < ranges[2][0]
    assert ranges[1][0] - ranges[0][1] >= 19.0
    assert ranges[2][0] - ranges[1][1] >= 19.0
    # Bounds helper matches.
    left, right, _, _ = strip_coverage_bounds(
        -120, 120, -120, 120, 1, 3, gap_m=20.0
    )
    assert abs(left - ranges[1][0]) < 1.0
    assert abs(right - ranges[1][1]) < 1.0


def test_region_lawnmower_two_asvs_are_geographic_halves():
    """South-boundary seeds → west / east Voronoi regions, no lane interleave."""
    shared = dict(
        min_x=-120.0,
        max_x=120.0,
        min_y=-120.0,
        max_y=120.0,
        spacing=40.0,
        num_asvs=2,
        seeds=[(-110.0, -110.0), (110.0, -110.0)],
    )
    west = generate_region_lawnmower(asv_index=0, **shared)
    east = generate_region_lawnmower(asv_index=1, **shared)
    assert len(west) >= 4
    assert len(east) >= 4

    # West region stays left of the seam; east stays right (with margin).
    assert max(x for x, _ in west) <= -10.0
    assert min(x for x, _ in east) >= 10.0

    # Mean x clearly separates the two regional paths.
    mean_west = sum(x for x, _ in west) / len(west)
    mean_east = sum(x for x, _ in east) / len(east)
    assert mean_west < -20.0
    assert mean_east > 20.0
    assert mean_east > mean_west

    # First waypoint is near each shoreline seed (no cross-region commute).
    assert math.hypot(west[0][0] - (-110.0), west[0][1] - (-110.0)) < 40.0
    assert math.hypot(east[0][0] - 110.0, east[0][1] - (-110.0)) < 40.0
    # East boat starts at the east end of its first sweep and moves west.
    assert east[0][0] > east[1][0]


def test_region_lawnmower_safety_margin_separates_shared_seam():
    """Inset margin keeps west/east first-lane tips from meeting at x=0."""
    shared = dict(
        min_x=-120.0,
        max_x=120.0,
        min_y=-120.0,
        max_y=120.0,
        spacing=20.0,
        num_asvs=2,
        seeds=[(-110.0, -110.0), (110.0, -110.0)],
        safety_margin_m=15.0,
    )
    west = generate_region_lawnmower(asv_index=0, **shared)
    east = generate_region_lawnmower(asv_index=1, **shared)
    gap = min(x for x, _ in east) - max(x for x, _ in west)
    assert gap >= 20.0  # ~2 * margin across the seam


def test_region_lawnmower_single_asv_matches_full():
    full = generate_lawnmower(-80.0, 80.0, -60.0, 60.0, spacing=30.0)
    region = generate_region_lawnmower(
        -80.0, 80.0, -60.0, 60.0, spacing=30.0, asv_index=0, num_asvs=1
    )
    assert region == full


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


def test_spiral_near_shore_stays_inside_bounds():
    """East-shore peak must not push spiral waypoints onto / past x=120."""
    from boat_mission.path_planning import generate_expanding_spiral

    center = (98.5, 15.0)
    spiral = generate_expanding_spiral(
        center,
        step_spacing=10.0,
        max_radius=80.0,
        ring_spacing=15.0,
        margin_min=(-120.0, -120.0),
        margin_max=(120.0, 120.0),
        inland_inset_m=10.0,
    )
    assert spiral
    for x, y in spiral:
        assert -110.0 <= x <= 110.0, (x, y)
        assert -110.0 <= y <= 110.0, (x, y)
    # Usable radius room east of center is 120-98.5-10 = 11.5 m.
    radii = [math.hypot(x - center[0], y - center[1]) for x, y in spiral]
    assert max(radii) <= 11.5 + 1e-6


def test_verification_orbit_shrinks_near_boundary():
    from boat_mission.path_planning import generate_verification_orbit

    orbit = generate_verification_orbit(
        (95.0, 15.0),
        radius=20.0,
        num_points=12,
        margin_min=(-120.0, -120.0),
        margin_max=(120.0, 120.0),
        inland_inset_m=10.0,
    )
    for x, y in orbit:
        assert -110.0 <= x <= 110.0
        assert -110.0 <= y <= 110.0
    radii = [math.hypot(x - 95.0, y - 15.0) for x, y in orbit[:-1]]
    assert max(radii) <= 15.0 + 1e-6  # 120-95-10 = 15


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
