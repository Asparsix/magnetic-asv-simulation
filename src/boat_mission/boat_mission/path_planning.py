"""Coverage and hunt path generators (lawnmower, spiral, info-gain)."""

from __future__ import annotations

import math


def generate_lawnmower(
    min_x,
    max_x,
    min_y,
    max_y,
    spacing,
    start_from_bottom=True,
):
    """Generate a rectangular boustrophedon (lawnmower) polyline."""
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    if max_x <= min_x or max_y <= min_y:
        raise ValueError('invalid bounds')

    path = []
    if start_from_bottom:
        y = min_y
        y_end = max_y
        y_step = spacing
    else:
        y = max_y
        y_end = min_y
        y_step = -spacing

    moving_right = True
    # Include final lane even if spacing does not land exactly on bound.
    while (y_step > 0 and y <= y_end + 1e-9) or (y_step < 0 and y >= y_end - 1e-9):
        y_clamped = min(max(y, min_y), max_y)
        if moving_right:
            path.append((min_x, y_clamped))
            path.append((max_x, y_clamped))
        else:
            path.append((max_x, y_clamped))
            path.append((min_x, y_clamped))
        moving_right = not moving_right
        y += y_step

    return _dedupe(path)


def generate_expanding_spiral(
    center,
    step_spacing,
    max_radius,
    ring_spacing,
    min_points_per_ring=8,
    margin_min=None,
    margin_max=None,
):
    """Concentric ring waypoints around a peak for densified sampling."""
    cx, cy = center
    waypoints = []
    radius = ring_spacing
    while radius <= max_radius + 1e-9:
        circumference = 2.0 * math.pi * radius
        n_points = max(min_points_per_ring, int(circumference / max(step_spacing, 1e-6)))
        for i in range(n_points):
            angle = 2.0 * math.pi * i / n_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            if margin_min is not None and margin_max is not None:
                x = min(max(x, margin_min[0]), margin_max[0])
                y = min(max(y, margin_min[1]), margin_max[1])
            waypoints.append((x, y))
        radius += ring_spacing
    if not waypoints:
        waypoints.append((cx, cy))
    return _dedupe(waypoints)


def generate_verification_orbit(
    center,
    radius,
    num_points=12,
    margin_min=None,
    margin_max=None,
):
    """Closed orbit around a candidate for Phase 8 verification densify."""
    if radius <= 0.0:
        raise ValueError('radius must be positive')
    if num_points < 3:
        raise ValueError('num_points must be >= 3')
    cx, cy = center
    waypoints = []
    for i in range(num_points):
        angle = 2.0 * math.pi * i / num_points
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        if margin_min is not None and margin_max is not None:
            x = min(max(x, margin_min[0]), margin_max[0])
            y = min(max(y, margin_min[1]), margin_max[1])
        waypoints.append((x, y))
    # Close the loop for LOS path following.
    waypoints.append(waypoints[0])
    return _dedupe(waypoints)


def generate_info_gain_candidates(current_pos, radii, num_angles=16, bounds=None):
    """Local ring candidates around the current pose for information surfing."""
    cx, cy = current_pos
    candidates = [(cx, cy)]
    for radius in radii:
        for i in range(num_angles):
            angle = 2.0 * math.pi * i / num_angles
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            if bounds is not None:
                (min_x, min_y), (max_x, max_y) = bounds
                x = min(max(x, min_x), max_x)
                y = min(max(y, min_y), max_y)
            candidates.append((x, y))
    return candidates


def plan_info_gain_waypoint(current_pos, peak_xy, radii, num_angles=16, bounds=None):
    """
    Peak-seeking fallback among local ring candidates.

    Prefer boat_mission.info_gain.InfoGainPlanner for true mutual information.
    Kept for unit tests and callers that lack a belief map.
    """
    candidates = generate_info_gain_candidates(
        current_pos, radii, num_angles=num_angles, bounds=bounds
    )
    px, py = peak_xy
    best = candidates[0]
    best_score = float('inf')
    for candidate in candidates:
        dist_peak = math.hypot(candidate[0] - px, candidate[1] - py)
        dist_move = math.hypot(candidate[0] - current_pos[0], candidate[1] - current_pos[1])
        score = dist_peak - 0.15 * dist_move
        if score < best_score:
            best_score = score
            best = candidate
    return best, best_score


def interpolate_segment(start, end, step_size):
    sx, sy = start
    ex, ey = end
    length = math.hypot(ex - sx, ey - sy)
    if length < 1e-9:
        return [start]
    n = max(1, int(length / max(step_size, 1e-6)))
    points = []
    for i in range(n + 1):
        t = i / n
        points.append((sx + t * (ex - sx), sy + t * (ey - sy)))
    return points


def path_length(points):
    if len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(len(points) - 1):
        total += math.hypot(
            points[i + 1][0] - points[i][0],
            points[i + 1][1] - points[i][1],
        )
    return total


def _dedupe(points):
    cleaned = []
    for point in points:
        if not cleaned or (
            abs(cleaned[-1][0] - point[0]) > 1e-9
            or abs(cleaned[-1][1] - point[1]) > 1e-9
        ):
            cleaned.append((float(point[0]), float(point[1])))
    return cleaned
