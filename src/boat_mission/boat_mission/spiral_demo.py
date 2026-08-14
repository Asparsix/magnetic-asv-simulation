#!/usr/bin/env python3
"""Drive to a fixed point, then exercise the production spiral path."""

import math

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from boat_mission.path_planning import generate_expanding_spiral
from boat_navigation.path_utils import make_path


class SpiralDemo(Node):
    """Publish a transit plan followed by the mission manager's spiral plan."""

    def __init__(self):
        super().__init__('spiral_demo')
        self.declare_parameter('pose_topic', 'pose2d')
        self.declare_parameter('plan_topic', 'plan')
        self.declare_parameter('center_x', 30.0)
        self.declare_parameter('center_y', 20.0)
        self.declare_parameter('arrival_radius_m', 8.0)
        self.declare_parameter('spiral_ring_spacing_m', 15.0)
        self.declare_parameter('spiral_max_radius_m', 80.0)
        self.declare_parameter('spiral_step_spacing_m', 10.0)
        self.declare_parameter('min_x', -120.0)
        self.declare_parameter('max_x', 120.0)
        self.declare_parameter('min_y', -120.0)
        self.declare_parameter('max_y', 120.0)

        self.center = (
            float(self.get_parameter('center_x').value),
            float(self.get_parameter('center_y').value),
        )
        self.arrival_radius = float(
            self.get_parameter('arrival_radius_m').value
        )
        self.pose = None
        self.phase = 'WAITING_FOR_POSE'

        plan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.plan_pub = self.create_publisher(
            Path, self.get_parameter('plan_topic').value, plan_qos
        )
        self.create_subscription(
            Pose2D,
            self.get_parameter('pose_topic').value,
            self.on_pose,
            10,
        )
        self.create_timer(0.5, self.on_timer)
        self.get_logger().info(
            f'Spiral demo ready; transit target=({self.center[0]:.1f},'
            f'{self.center[1]:.1f})'
        )

    def on_pose(self, msg):
        self.pose = msg

    def publish_plan(self, points):
        path = make_path(points, frame_id='map')
        path.header.stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(path)

    def on_timer(self):
        if self.pose is None:
            return

        if self.phase == 'WAITING_FOR_POSE':
            self.publish_plan([
                (self.pose.x, self.pose.y),
                self.center,
            ])
            self.phase = 'TRANSIT'
            self.get_logger().info(
                f'TRANSIT: driving to ({self.center[0]:.1f},'
                f'{self.center[1]:.1f})'
            )
            return

        if self.phase != 'TRANSIT':
            return

        distance = math.hypot(
            self.pose.x - self.center[0],
            self.pose.y - self.center[1],
        )
        if distance > self.arrival_radius:
            return

        # This is the same generator and parameter mapping used by
        # MissionManager._start_spiral().
        points = generate_expanding_spiral(
            self.center,
            step_spacing=float(
                self.get_parameter('spiral_step_spacing_m').value
            ),
            max_radius=float(
                self.get_parameter('spiral_max_radius_m').value
            ),
            ring_spacing=float(
                self.get_parameter('spiral_ring_spacing_m').value
            ),
            margin_min=(
                float(self.get_parameter('min_x').value),
                float(self.get_parameter('min_y').value),
            ),
            margin_max=(
                float(self.get_parameter('max_x').value),
                float(self.get_parameter('max_y').value),
            ),
        )
        points = [(self.pose.x, self.pose.y)] + points
        self.publish_plan(points)
        self.phase = 'SPIRAL'
        self.get_logger().info(
            f'SPIRAL: published {len(points)} production waypoints around '
            f'({self.center[0]:.1f},{self.center[1]:.1f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpiralDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
