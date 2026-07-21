#!/usr/bin/env python3
"""Convert a Twist command into bounded port and starboard thrust."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from boat_control.core import mix_thrust


class ThrustMixer(Node):
    """Map ``cmd_vel`` surge and yaw commands to differential thrust."""

    def __init__(self):
        super().__init__('thrust_mixer')

        self.declare_parameter('model_name', 'simple_boat')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('thrust_scale', 50.0)
        self.declare_parameter('turn_gain', 1.0)
        self.declare_parameter('max_thrust', 100.0)

        model_name = self.get_parameter('model_name').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.thrust_scale = float(self.get_parameter('thrust_scale').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.max_thrust = float(self.get_parameter('max_thrust').value)

        topic_prefix = f'/model/{model_name}/joint'
        self.left_pub = self.create_publisher(
            Float64,
            f'{topic_prefix}/left_propeller_joint/cmd_thrust',
            10,
        )
        self.right_pub = self.create_publisher(
            Float64,
            f'{topic_prefix}/right_propeller_joint/cmd_thrust',
            10,
        )
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)

        self.get_logger().info(
            f'Listening on {cmd_vel_topic}; model={model_name}, '
            f'thrust_scale={self.thrust_scale:.1f}, '
            f'turn_gain={self.turn_gain:.1f}, max_thrust={self.max_thrust:.1f}'
        )

    def on_cmd_vel(self, msg):
        left, right = mix_thrust(
            msg.linear.x,
            msg.angular.z,
            self.thrust_scale,
            self.turn_gain,
            self.max_thrust,
        )
        self.left_pub.publish(Float64(data=left))
        self.right_pub.publish(Float64(data=right))


def main(args=None):
    rclpy.init(args=args)
    node = ThrustMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
