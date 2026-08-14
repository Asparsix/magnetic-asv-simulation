#!/usr/bin/env python3
"""LOS path follower with heading and speed PID control."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from boat_navigation.guidance import (
    los_heading,
    segment_geometry,
    should_advance_segment,
    speed_scale_for_heading_error,
    wrap_angle,
    yaw_from_quaternion,
)
from boat_navigation.path_utils import (
    nearest_segment_index,
    path_points,
    patrol_route,
    valid_path,
)
from boat_navigation.pid import PIDController


class LosPathFollower(Node):
    """Follow a nav_msgs/Path using LOS guidance and PID surge/yaw commands."""

    def __init__(self):
        super().__init__('los_path_follower')

        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('lookahead_m', 15.0)
        self.declare_parameter('accept_radius_m', 8.0)
        self.declare_parameter('pass_epsilon_m', 2.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('u_ref', 0.35)
        self.declare_parameter('u_max', 0.5)
        self.declare_parameter('r_max', 0.3)
        self.declare_parameter('patrol_half_size', 120.0)
        self.declare_parameter('pose_timeout_s', 1.0)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('plan_topic', 'plan')
        self.declare_parameter('active_plan_topic', 'plan/active')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('debug_topic', 'nav/debug')
        self.declare_parameter('use_builtin_path', True)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('ki_yaw', 0.05)
        self.declare_parameter('kd_yaw', 0.2)
        self.declare_parameter('integral_limit_yaw', 1.0)
        self.declare_parameter('kp_u', 1.0)
        self.declare_parameter('ki_u', 0.1)
        self.declare_parameter('kd_u', 0.0)
        self.declare_parameter('integral_limit_u', 1.0)
        self.declare_parameter('speed_mode', 'open_loop')

        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.accept_radius_m = float(self.get_parameter('accept_radius_m').value)
        self.pass_epsilon_m = float(self.get_parameter('pass_epsilon_m').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.u_ref = float(self.get_parameter('u_ref').value)
        self.u_max = float(self.get_parameter('u_max').value)
        self.r_max = float(self.get_parameter('r_max').value)
        self.pose_timeout_s = float(self.get_parameter('pose_timeout_s').value)
        self.speed_mode = self.get_parameter('speed_mode').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        active_plan_topic = self.get_parameter('active_plan_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.u_meas = 0.0
        self.last_odom_time = None
        self.segment_index = 0
        self.active_path = None
        self.active_points = []

        self.yaw_pid = PIDController(
            kp=float(self.get_parameter('kp_yaw').value),
            ki=float(self.get_parameter('ki_yaw').value),
            kd=float(self.get_parameter('kd_yaw').value),
            output_min=-self.r_max,
            output_max=self.r_max,
            integral_limit=float(self.get_parameter('integral_limit_yaw').value),
            reset_error=math.pi / 2.0,
        )
        self.speed_pid = PIDController(
            kp=float(self.get_parameter('kp_u').value),
            ki=float(self.get_parameter('ki_u').value),
            kd=float(self.get_parameter('kd_u').value),
            output_min=-self.u_max,
            output_max=self.u_max,
            integral_limit=float(self.get_parameter('integral_limit_u').value),
            reset_error=self.u_max,
        )

        plan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.active_plan_pub = self.create_publisher(Path, active_plan_topic, plan_qos)
        self.debug_pub = self.create_publisher(Float32MultiArray, debug_topic, 10)

        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(Path, plan_topic, self.on_plan, plan_qos)

        if bool(self.get_parameter('use_builtin_path').value):
            self.set_path(
                patrol_route(float(self.get_parameter('patrol_half_size').value)),
                reindex=False,
            )

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self.on_control_timer)
        self.get_logger().info('LOS path follower ready')

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.u_meas = msg.twist.twist.linear.x
        self.last_odom_time = self.get_clock().now()

    def on_plan(self, msg):
        if not valid_path(msg):
            self.get_logger().warn('Ignoring invalid plan message')
            return
        self.set_path(msg, reindex=True)

    def set_path(self, path, reindex):
        self.active_path = path
        self.active_points = path_points(path)
        self.segment_index = 0
        self.yaw_pid.reset()
        self.speed_pid.reset()
        if reindex and self.last_odom_time is not None:
            self.segment_index = nearest_segment_index(
                self.active_points,
                (self.x, self.y),
            )
        self.active_plan_pub.publish(path)

    def on_control_timer(self):
        if not valid_path(self.active_path):
            self.publish_stop()
            return

        if self.last_odom_time is None:
            self.publish_stop()
            return

        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age > self.pose_timeout_s:
            self.get_logger().warn(
                f'Stale odometry ({age:.2f}s); stopping',
                throttle_duration_sec=2.0,
            )
            self.publish_stop()
            return

        start = self.active_points[self.segment_index]
        end = self.active_points[self.segment_index + 1]
        along, cross, length = segment_geometry(start, end, (self.x, self.y))

        if should_advance_segment(
            start,
            end,
            (self.x, self.y),
            along,
            length,
            self.accept_radius_m,
            self.pass_epsilon_m,
        ):
            self.advance_segment()

        start = self.active_points[self.segment_index]
        end = self.active_points[self.segment_index + 1]
        desired_heading, along, cross = los_heading(
            start,
            end,
            (self.x, self.y),
            self.lookahead_m,
        )
        heading_error = wrap_angle(desired_heading - self.yaw)

        dt = 1.0 / float(self.get_parameter('control_rate_hz').value)
        yaw_cmd = self.yaw_pid.update(heading_error, dt)

        speed_ref = self.u_ref * speed_scale_for_heading_error(heading_error)
        if self.speed_mode == 'pid':
            speed_error = speed_ref - self.u_meas
            surge_cmd = self.speed_pid.update(speed_error, dt)
        else:
            surge_cmd = speed_ref

        cmd = Twist()
        cmd.linear.x = max(-self.u_max, min(self.u_max, surge_cmd))
        cmd.angular.z = yaw_cmd
        self.cmd_pub.publish(cmd)

        debug = Float32MultiArray()
        debug.data = [
            float(cross),
            float(along),
            float(desired_heading),
            float(heading_error),
            float(surge_cmd),
            float(self.segment_index),
        ]
        self.debug_pub.publish(debug)

    def advance_segment(self):
        if self.segment_index < len(self.active_points) - 2:
            self.segment_index += 1
        elif self.loop:
            self.segment_index = 0
        else:
            self.publish_stop()
            return
        self.yaw_pid.reset()
        self.speed_pid.reset()

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LosPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
