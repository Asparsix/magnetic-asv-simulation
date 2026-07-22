#!/usr/bin/env python3
"""Publish the boat Pose2D directly from Gazebo Transport."""

import math

from geometry_msgs.msg import Pose2D
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node


class GazeboPose2D(Node):
    """Extract one model pose from Gazebo's dynamic pose stream."""

    def __init__(self):
        super().__init__('gazebo_pose2d')
        self.declare_parameter('model_name', 'simple_boat')
        self.declare_parameter('world_name', 'niot_world')
        self.declare_parameter('pose_topic', 'pose2d')

        self.model_name = self.get_parameter('model_name').value
        world_name = self.get_parameter('world_name').value
        pose_topic = self.get_parameter('pose_topic').value

        self.pose_pub = self.create_publisher(Pose2D, pose_topic, 10)
        self.gz_node = GzNode()
        gz_topic = f'/world/{world_name}/dynamic_pose/info'
        subscribed = self.gz_node.subscribe(Pose_V, gz_topic, self.on_gz_pose)
        if not subscribed:
            raise RuntimeError(f'Failed to subscribe to Gazebo topic {gz_topic}')

        self.get_logger().info(
            f'Publishing {self.model_name} from {gz_topic} on {pose_topic}'
        )

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def on_gz_pose(self, msg):
        for pose in msg.pose:
            if pose.name != self.model_name:
                continue

            output = Pose2D()
            output.x = pose.position.x
            output.y = pose.position.y
            output.theta = self.quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            self.pose_pub.publish(output)
            return


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPose2D()
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
