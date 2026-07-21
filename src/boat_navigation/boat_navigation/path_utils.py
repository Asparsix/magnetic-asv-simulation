"""Path utilities and patrol route generation."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def make_path(points, frame_id='map'):
    path = Path()
    path.header.frame_id = frame_id
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def path_points(path):
    return [
        (pose.pose.position.x, pose.pose.position.y)
        for pose in path.poses
    ]


def valid_path(path):
    return path is not None and len(path.poses) >= 2


def patrol_route(half_size=120.0):
    """Closed rectangular patrol inset from a 300 m lake shoreline."""
    h = half_size
    points = [
        (-h, -h),
        (h, -h),
        (h, h),
        (-h, h),
        (-h, -h),
    ]
    return make_path(points)


def nearest_segment_index(path_points_list, position):
    if len(path_points_list) < 2:
        return 0

    best_index = 0
    best_distance = float('inf')
    px, py = position

    for index in range(len(path_points_list) - 1):
        sx, sy = path_points_list[index]
        ex, ey = path_points_list[index + 1]
        tx = ex - sx
        ty = ey - sy
        length = math.hypot(tx, ty)
        if length < 1e-6:
            dist = math.hypot(px - sx, py - sy)
        else:
            tx /= length
            ty /= length
            rx = px - sx
            ry = py - sy
            along = rx * tx + ry * ty
            cross = tx * ry - ty * rx
            if along < 0.0:
                dist = math.hypot(px - sx, py - sy)
            elif along > length:
                dist = math.hypot(px - ex, py - ey)
            else:
                dist = abs(cross)
        if dist < best_distance:
            best_distance = dist
            best_index = index
    return best_index
