"""Unit tests for coverage and hunt path generators."""

import math

from boat_mission.path_planning import (
    generate_expanding_spiral,
    generate_info_gain_candidates,
    generate_lawnmower,
    generate_region_lawnmower,
    generate_split_lawnmower,
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


def test_region_lawnmower_two_asvs_are_geographic_halves():
    """South-boundary seeds → west / east Voronoi regions, no lane interleave."""
    shared = dict(
        min_x=-450.0,
        max_x=450.0,
        min_y=-450.0,
        max_y=450.0,
        spacing=100.0,
        num_asvs=2,
        seeds=[(-450.0, -450.0), (450.0, -450.0)],
    )
    west = generate_region_lawnmower(asv_index=0, **shared)
    east = generate_region_lawnmower(asv_index=1, **shared)
    assert len(west) >= 4
    assert len(east) >= 4

    # West region stays on x <= 0 (+tolerance); east on x >= 0.
    assert max(x for x, _ in west) <= 1.0
    assert min(x for x, _ in east) >= -1.0

    # Mean x clearly separates the two regional paths.
    mean_west = sum(x for x, _ in west) / len(west)
    mean_east = sum(x for x, _ in east) / len(east)
    assert mean_west < -50.0
    assert mean_east > 50.0
    assert mean_east > mean_west


def test_region_lawnmower_three_asvs_cover_distinct_corners():
    shared = dict(
        min_x=-450.0,
        max_x=450.0,
        min_y=-450.0,
        max_y=450.0,
        spacing=150.0,
        num_asvs=3,
        seeds=[(-450.0, -450.0), (450.0, -450.0), (450.0, 450.0)],
    )
    paths = [
        generate_region_lawnmower(asv_index=i, **shared) for i in range(3)
    ]
    assert all(len(p) >= 4 for p in paths)
    means = [
        (sum(x for x, _ in p) / len(p), sum(y for _, y in p) / len(p))
        for p in paths
    ]
    # SW / SE / NE centroids should separate in the expected quadrants.
    assert means[0][0] < 0.0 and means[0][1] < 50.0
    assert means[1][0] > 0.0 and means[1][1] < 50.0
    assert means[2][0] > -50.0 and means[2][1] > 0.0


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
