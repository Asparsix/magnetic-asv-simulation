"""Coverage and hunt path generators (lawnmower, spiral, info-gain)."""

from __future__ import annotations

import math

from shapely.geometry import LineString, MultiPoint, Point, Polygon
from shapely.ops import substring, voronoi_diagram

# Spawn poses = first lawnmower waypoint of each vertical strip (yaw east = 0).
DEFAULT_REGION_SEEDS_3 = (
    (-120.0, -120.0),   # ASV1 — west strip start
    (-33.333, -120.0),  # ASV2 — center strip start
    (53.333, -120.0),   # ASV3 — east strip start
)
DEFAULT_REGION_SEEDS_2 = (
    (-120.0, -120.0),
    (53.333, -120.0),
)


def strip_coverage_bounds(
    min_x,
    max_x,
    min_y,
    max_y,
    asv_index,
    num_asvs,
    gap_m=20.0,
):
    """
    Split [min_x, max_x] into ``num_asvs`` vertical strips with a dead corridor
    of width ``gap_m`` between neighbours so lawnmowers cannot meet.
    """
    num_asvs = int(num_asvs)
    asv_index = int(asv_index)
    if num_asvs < 1:
        raise ValueError('num_asvs must be >= 1')
    if asv_index < 0 or asv_index >= num_asvs:
        raise ValueError('asv_index must satisfy 0 <= asv_index < num_asvs')
    if max_x <= min_x or max_y <= min_y:
        raise ValueError('invalid bounds')
    if num_asvs == 1:
        return float(min_x), float(max_x), float(min_y), float(max_y)

    gap = max(float(gap_m), 0.0)
    total_w = float(max_x) - float(min_x)
    usable = total_w - gap * (num_asvs - 1)
    if usable <= 1.0:
        raise ValueError('coverage too narrow for strip gaps')
    strip_w = usable / num_asvs
    left = float(min_x) + asv_index * (strip_w + gap)
    right = left + strip_w
    return left, right, float(min_y), float(max_y)


def generate_strip_lawnmower(
    min_x,
    max_x,
    min_y,
    max_y,
    spacing,
    asv_index=0,
    num_asvs=1,
    gap_m=20.0,
    start_from_bottom=True,
):
    """Lawnmower confined to this ASV's vertical strip (no shared seam)."""
    sx0, sx1, sy0, sy1 = strip_coverage_bounds(
        min_x, max_x, min_y, max_y, asv_index, num_asvs, gap_m=gap_m
    )
    return generate_lawnmower(
        sx0, sx1, sy0, sy1, spacing, start_from_bottom=start_from_bottom
    )


def generate_lawnmower(
    min_x,
    max_x,
    min_y,
    max_y,
    spacing,
    start_from_bottom=True,
):
    """Generate a rectangular boustrophedon (lawnmower) polyline."""
    return generate_split_lawnmower(
        min_x,
        max_x,
        min_y,
        max_y,
        spacing,
        asv_index=0,
        num_asvs=1,
        start_from_bottom=start_from_bottom,
    )


def generate_split_lawnmower(
    min_x,
    max_x,
    min_y,
    max_y,
    spacing,
    asv_index=0,
    num_asvs=1,
    start_from_bottom=True,
):
    """
    Lawnmower coverage with optional lane partitioning across ASVs.

    Lanes are indexed from the start bound (bottom or top). ASV ``asv_index``
    receives every lane where ``lane_id % num_asvs == asv_index``. Directions
    alternate within that ASV's own lane sequence so each boat still zig-zags.
    """
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    if max_x <= min_x or max_y <= min_y:
        raise ValueError('invalid bounds')
    num_asvs = int(num_asvs)
    asv_index = int(asv_index)
    if num_asvs < 1:
        raise ValueError('num_asvs must be >= 1')
    if asv_index < 0 or asv_index >= num_asvs:
        raise ValueError('asv_index must satisfy 0 <= asv_index < num_asvs')

    if start_from_bottom:
        y = min_y
        y_end = max_y
        y_step = spacing
    else:
        y = max_y
        y_end = min_y
        y_step = -spacing

    lane_ys = []
    while (y_step > 0 and y <= y_end + 1e-9) or (y_step < 0 and y >= y_end - 1e-9):
        lane_ys.append(min(max(y, min_y), max_y))
        y += y_step

    assigned = [
        lane_y
        for lane_id, lane_y in enumerate(lane_ys)
        if lane_id % num_asvs == asv_index
    ]
    if not assigned:
        # Degenerate box / oversplit — fall back to a single mid lane.
        assigned = [0.5 * (min_y + max_y)]

    path = []
    moving_right = True
    for lane_y in assigned:
        if moving_right:
            path.append((min_x, lane_y))
            path.append((max_x, lane_y))
        else:
            path.append((max_x, lane_y))
            path.append((min_x, lane_y))
        moving_right = not moving_right

    return _dedupe(path)


def default_region_seeds(num_asvs):
    """Boundary launch seeds for the current multi-ASV count."""
    num_asvs = int(num_asvs)
    if num_asvs <= 1:
        return [DEFAULT_REGION_SEEDS_3[0]]
    if num_asvs == 2:
        return list(DEFAULT_REGION_SEEDS_2)
    if num_asvs == 3:
        return list(DEFAULT_REGION_SEEDS_3)
    raise ValueError('region seeds are only defined for 1–3 ASVs')


def generate_voronoi_regions(boundary_poly, seeds):
    """Clip a Voronoi diagram of ``seeds`` to ``boundary_poly`` (one poly per seed)."""
    points = MultiPoint([Point(p) for p in seeds])
    vor = voronoi_diagram(points, envelope=boundary_poly)
    clipped_regions = [poly.intersection(boundary_poly) for poly in vor.geoms]

    ordered_regions = []
    for seed in seeds:
        seed_point = Point(seed)
        matched = None
        for region in clipped_regions:
            if region.is_empty:
                continue
            if region.covers(seed_point) or region.distance(seed_point) < 1e-6:
                matched = region
                break
        if matched is None:
            raise RuntimeError(f'no Voronoi cell found for seed {seed}')
        ordered_regions.append(matched)
    return ordered_regions


def _perimeter_path(ext, p1, p2):
    d1 = ext.project(Point(p1))
    d2 = ext.project(Point(p2))
    length = ext.length

    if d1 <= d2:
        if (d2 - d1) <= (d1 + (length - d2)):
            geom = substring(ext, d1, d2)
        else:
            geom = LineString(
                list(substring(ext, d1, 0).coords)
                + list(substring(ext, length, d2).coords)
            )
    else:
        if (d1 - d2) <= ((length - d1) + d2):
            geom = substring(ext, d1, d2)
        else:
            geom = LineString(
                list(substring(ext, d1, length).coords)
                + list(substring(ext, 0, d2).coords)
            )

    clean_coords = []
    for coord in list(geom.coords):
        if not clean_coords or coord != clean_coords[-1]:
            clean_coords.append(coord)
    if len(clean_coords) >= 2:
        return clean_coords[1:-1]
    return []


def generate_polygon_lawnmower(polygon, spacing, start_near=None):
    """Boustrophedon coverage clipped to a Shapely polygon (region lawnmower).

    If ``start_near`` is given (e.g. the ASV spawn / region seed), the first
    sweep starts at the endpoint closer to that point so boats do not commute
    across their region before coverage begins.
    """
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    if polygon.is_empty:
        return []

    minx, miny, maxx, maxy = polygon.bounds
    path = []
    y = miny + spacing / 2.0
    moving_right = True
    start_dir_chosen = False
    previous_endpoint = None
    ext = polygon.exterior

    while y <= maxy + 1e-9:
        sweep_line = LineString([(minx, y), (maxx, y)])
        intersection = sweep_line.intersection(polygon)
        segments = []

        if not intersection.is_empty:
            if intersection.geom_type == 'LineString':
                segments.append(intersection)
            elif intersection.geom_type == 'MultiLineString':
                segments.extend(list(intersection.geoms))

        segments = sorted(segments, key=lambda s: s.centroid.x)
        if (
            not start_dir_chosen
            and start_near is not None
            and segments
        ):
            first = list(segments[0].coords)
            left = first[0]
            right = first[-1]
            sx, sy = float(start_near[0]), float(start_near[1])
            d_left = math.hypot(left[0] - sx, left[1] - sy)
            d_right = math.hypot(right[0] - sx, right[1] - sy)
            # Prefer starting at the nearer end of the first sweep.
            moving_right = d_left <= d_right
            start_dir_chosen = True
        for segment in segments:
            coords = list(segment.coords)
            if not moving_right:
                coords.reverse()
            if previous_endpoint is not None:
                path.extend(_perimeter_path(ext, previous_endpoint, coords[0]))
            path.extend(coords)
            previous_endpoint = coords[-1]
            moving_right = not moving_right
        y += spacing

    return _dedupe([(float(x), float(y)) for x, y in path])


def generate_region_lawnmower(
    min_x,
    max_x,
    min_y,
    max_y,
    spacing,
    asv_index=0,
    num_asvs=1,
    seeds=None,
    safety_margin_m=15.0,
):
    """
    Voronoi-partition the coverage box and lawnmower only this ASV's region.

    ``seeds`` are launch / region-centre points (flat list or Nx2). Defaults to
    shoreline starts for the dual/triple ASV sim.

    ``safety_margin_m`` erodes each Voronoi cell before pathing so neighbouring
    lawnmowers leave a corridor at shared seams and do not drive head-on into
    each other (e.g. both racing to x=0 on the same y-lane).
    """
    num_asvs = int(num_asvs)
    asv_index = int(asv_index)
    if num_asvs < 1:
        raise ValueError('num_asvs must be >= 1')
    if asv_index < 0 or asv_index >= num_asvs:
        raise ValueError('asv_index must satisfy 0 <= asv_index < num_asvs')
    if max_x <= min_x or max_y <= min_y:
        raise ValueError('invalid bounds')

    if num_asvs == 1:
        return generate_lawnmower(min_x, max_x, min_y, max_y, spacing)

    if seeds is None:
        seed_list = default_region_seeds(num_asvs)
    else:
        flat = list(seeds)
        if flat and isinstance(flat[0], (list, tuple)):
            seed_list = [(float(p[0]), float(p[1])) for p in flat]
        else:
            if len(flat) % 2 != 0:
                raise ValueError('seeds must be a flat [x1,y1,x2,y2,...] list')
            seed_list = [
                (float(flat[i]), float(flat[i + 1]))
                for i in range(0, len(flat), 2)
            ]
    if len(seed_list) < num_asvs:
        raise ValueError('need at least one seed per ASV')
    seed_list = seed_list[:num_asvs]

    boundary = Polygon([
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ])
    regions = generate_voronoi_regions(boundary, seed_list)
    region = regions[asv_index]

    margin = max(float(safety_margin_m), 0.0)
    if margin > 0.0:
        eroded = region.buffer(-margin)
        # Prefer the polygon piece that still contains / is nearest the seed.
        if not eroded.is_empty:
            seed_pt = Point(seed_list[asv_index])
            if eroded.geom_type == 'MultiPolygon':
                candidates = list(eroded.geoms)
                eroded = min(
                    candidates,
                    key=lambda g: g.distance(seed_pt),
                )
            if eroded.area > 1.0:
                region = eroded

    return generate_polygon_lawnmower(
        region,
        spacing,
        start_near=seed_list[asv_index],
    )


def max_radius_inside_bounds(center, margin_min, margin_max, inset=0.0):
    """Largest radius whose full circle stays inside [margin_min, margin_max].

    ``inset`` further shrinks the usable room so thruster overshoot / LOS
    lookahead does not press the shoreline collider.
    """
    if margin_min is None or margin_max is None:
        return float('inf')
    cx, cy = center
    room = min(
        cx - margin_min[0],
        margin_max[0] - cx,
        cy - margin_min[1],
        margin_max[1] - cy,
    )
    return max(0.0, room - float(inset))


def generate_expanding_spiral(
    center,
    step_spacing,
    max_radius,
    ring_spacing,
    min_points_per_ring=8,
    margin_min=None,
    margin_max=None,
    inland_inset_m=10.0,
):
    """Concentric ring waypoints around a peak for densified sampling.

    When bounds are given, the spiral radius is shrunk so every waypoint stays
    strictly inside the box (no wall-clamping). Clamping used to pile points on
    the shoreline and drive boats into Gazebo shore colliders.
    """
    cx, cy = center
    fit = max_radius_inside_bounds(
        center, margin_min, margin_max, inset=inland_inset_m
    )
    usable_max = min(float(max_radius), fit)
    waypoints = []
    if usable_max < max(ring_spacing, 1.0) * 0.5:
        # Peak too close to the boundary for a real spiral — loiter in place.
        waypoints.append((cx, cy))
        waypoints.append((cx + min(1.0, usable_max * 0.5), cy))
        return _dedupe(waypoints)

    radius = min(ring_spacing, usable_max)
    while radius <= usable_max + 1e-9:
        circumference = 2.0 * math.pi * radius
        n_points = max(min_points_per_ring, int(circumference / max(step_spacing, 1e-6)))
        for i in range(n_points):
            angle = 2.0 * math.pi * i / n_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            waypoints.append((x, y))
        if radius >= usable_max - 1e-9:
            break
        radius = min(radius + ring_spacing, usable_max)
    if not waypoints:
        waypoints.append((cx, cy))
    return _dedupe(waypoints)


def generate_verification_orbit(
    center,
    radius,
    num_points=12,
    margin_min=None,
    margin_max=None,
    start_angle=0.0,
    inland_inset_m=10.0,
):
    """Closed orbit around a candidate for Phase 8 verification densify.

    ``start_angle`` (rad) rotates the first waypoint — used so a peer verifier
    can enter from the side opposite the discoverer. Radius is shrunk to keep
    the full orbit inside the coverage box (no shoreline clamping).
    """
    if radius <= 0.0:
        raise ValueError('radius must be positive')
    if num_points < 3:
        raise ValueError('num_points must be >= 3')
    cx, cy = center
    fit = max_radius_inside_bounds(
        center, margin_min, margin_max, inset=inland_inset_m
    )
    radius = min(float(radius), fit)
    if radius < 1.0:
        # Degenerate: hold near the candidate itself.
        return _dedupe([(cx, cy), (cx + 0.5, cy), (cx, cy)])
    waypoints = []
    for i in range(num_points):
        angle = start_angle + 2.0 * math.pi * i / num_points
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        waypoints.append((x, y))
    # Close the loop for LOS path following.
    waypoints.append(waypoints[0])
    return _dedupe(waypoints)


def opposite_approach_angle(candidate_xy, discoverer_xy):
    """Orbit entry angle on the side opposite the discoverer."""
    dx = candidate_xy[0] - discoverer_xy[0]
    dy = candidate_xy[1] - discoverer_xy[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def transit_waypoints(start, end, spacing_m=40.0):
    """Dense linear hops from start → end (inclusive of both endpoints)."""
    x0, y0 = start
    x1, y1 = end
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist < 1e-6:
        return [(float(x0), float(y0))]
    spacing = max(float(spacing_m), 1.0)
    n_hops = max(1, int(math.ceil(dist / spacing)))
    points = []
    for i in range(n_hops + 1):
        t = i / n_hops
        points.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return points


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
