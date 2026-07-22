"""Line-of-sight guidance and path geometry helpers."""

import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def segment_geometry(start, end, position):
    """Return along-track distance, cross-track error, and segment length."""
    sx, sy = start
    ex, ey = end
    px, py = position

    tx = ex - sx
    ty = ey - sy
    length = math.hypot(tx, ty)
    if length < 1e-6:
        return 0.0, 0.0, 0.0

    tx /= length
    ty /= length
    rx = px - sx
    ry = py - sy
    along = rx * tx + ry * ty
    cross = tx * ry - ty * rx
    return along, cross, length


def path_heading(start, end):
    sx, sy = start
    ex, ey = end
    return math.atan2(ey - sy, ex - sx)


def los_heading(start, end, position, lookahead):
    along, cross, _ = segment_geometry(start, end, position)
    heading = path_heading(start, end)
    if lookahead <= 1e-6:
        return heading, along, cross
    return heading + math.atan2(-cross, lookahead), along, cross


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def should_advance_segment(
    start,
    end,
    position,
    along,
    length,
    accept_radius,
    pass_epsilon,
):
    if distance(position, end) <= accept_radius:
        return True
    if length > 1e-6 and along >= length - pass_epsilon:
        return True
    return False


def speed_scale_for_heading_error(heading_error, min_scale=0.2):
    return max(min_scale, math.cos(abs(heading_error)))
