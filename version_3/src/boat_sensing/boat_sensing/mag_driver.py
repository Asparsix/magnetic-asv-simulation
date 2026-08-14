#!/usr/bin/env python3
"""Fixed-rate magnetometer driver that stamps MagReading with pose."""

import math
import random

from boat_msgs.msg import MagReading
from geometry_msgs.msg import Pose2D, Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from boat_sensing.dipole import (
    dipole_anomaly_nt,
    planted_dipole_field_nt,
    resolve_dipole_moment,
)
from boat_sensing.filter_core import tesla_to_nt, vector_scalar


class MagDriver(Node):
    """Republish Gazebo magnetometer samples as MagReading at a fixed rate."""

    def __init__(self):
        super().__init__('mag_driver')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('gazebo_mag_topic', 'mag/gazebo')
        self.declare_parameter('pose_topic', 'pose2d')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('raw_topic', 'mag/raw')
        self.declare_parameter('frame_id', 'asv1/mag_link')
        self.declare_parameter('motor_on_threshold', 1.0e-3)

        # Gazebo Harmonic has no local magnetic sources. When planting is enabled,
        # replace the unstable Gazebo field with a synthetic Earth field + dipole.
        # dipole_model: scalar_soft (legacy A/(r^3+s^3) on Bz) or total_field
        # (vector m, full B, scalar = |B|, anomaly = |B0+Bd|-|B0|).
        self.declare_parameter('plant_magnetic_target', True)
        self.declare_parameter('dipole_model', 'scalar_soft')
        self.declare_parameter('target_x', 80.0)
        self.declare_parameter('target_y', -40.0)
        self.declare_parameter('target_z', -1.0)
        self.declare_parameter('sensor_z', 0.0)
        self.declare_parameter('dipole_strength_nt', 1.5e12)
        self.declare_parameter('dipole_soft_m', 1.0)
        self.declare_parameter('dipole_peak_nt', 50.0)
        self.declare_parameter('dipole_mx', 0.0)
        self.declare_parameter('dipole_my', 0.0)
        self.declare_parameter('dipole_mz', 0.0)
        self.declare_parameter('earth_inclination_deg', 15.0)
        self.declare_parameter('earth_declination_deg', -1.0)
        # Inflated units matching Gazebo-as-Tesla conversion (~0.45 "T" → 4.5e8 nT).
        self.declare_parameter('synthetic_background_nt', 4.5e8)
        self.declare_parameter('synthetic_noise_nt', 5.0e4)
        self.declare_parameter('random_seed', 0)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        gazebo_mag_topic = self.get_parameter('gazebo_mag_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        raw_topic = self.get_parameter('raw_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.motor_on_threshold = float(
            self.get_parameter('motor_on_threshold').value
        )
        self.plant_magnetic_target = bool(
            self.get_parameter('plant_magnetic_target').value
        )
        self.dipole_model = str(
            self.get_parameter('dipole_model').value
        ).strip().lower()
        if self.dipole_model not in ('scalar_soft', 'total_field'):
            self.get_logger().warn(
                f'unknown dipole_model={self.dipole_model!r}, '
                'falling back to scalar_soft'
            )
            self.dipole_model = 'scalar_soft'
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.sensor_z = float(self.get_parameter('sensor_z').value)
        self.dipole_strength_nt = float(
            self.get_parameter('dipole_strength_nt').value
        )
        self.dipole_soft_m = float(self.get_parameter('dipole_soft_m').value)
        self.dipole_peak_nt = float(self.get_parameter('dipole_peak_nt').value)
        self.earth_inclination_deg = float(
            self.get_parameter('earth_inclination_deg').value
        )
        self.earth_declination_deg = float(
            self.get_parameter('earth_declination_deg').value
        )
        self.dipole_m = resolve_dipole_moment(
            float(self.get_parameter('dipole_mx').value),
            float(self.get_parameter('dipole_my').value),
            float(self.get_parameter('dipole_mz').value),
            self.dipole_peak_nt,
            self.target_z,
            self.earth_inclination_deg,
            sensor_z=self.sensor_z,
            soft_m=self.dipole_soft_m,
        )
        self.synthetic_background_nt = float(
            self.get_parameter('synthetic_background_nt').value
        )
        self.synthetic_noise_nt = float(
            self.get_parameter('synthetic_noise_nt').value
        )
        seed = int(self.get_parameter('random_seed').value)
        if seed != 0:
            random.seed(seed)

        self.latest_mag = None
        self.latest_pose = None
        self.motor_on = False
        self.published_count = 0

        self.raw_pub = self.create_publisher(MagReading, raw_topic, 10)
        self.create_subscription(
            MagneticField, gazebo_mag_topic, self.on_mag, 50
        )
        self.create_subscription(Pose2D, pose_topic, self.on_pose, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_timer(1.0 / rate_hz, self.on_timer)

        plant_msg = 'disabled'
        if self.plant_magnetic_target:
            if self.dipole_model == 'total_field':
                mx, my, mz = self.dipole_m
                plant_msg = (
                    f'total_field at ({self.target_x:.1f},{self.target_y:.1f},'
                    f'{self.target_z:.1f}) m=({mx:.3g},{my:.3g},{mz:.3g}) '
                    f'peak≈{self.dipole_peak_nt:.1f} nT '
                    f'I={self.earth_inclination_deg:.1f}°'
                )
            else:
                plant_msg = (
                    f'scalar_soft at ({self.target_x:.1f},{self.target_y:.1f}) '
                    f'A={self.dipole_strength_nt:.3g} nT'
                )
        self.get_logger().info(
            f'mag_driver publishing {raw_topic} at {rate_hz:.1f} Hz '
            f'from {gazebo_mag_topic}; dipole {plant_msg}'
        )

    def on_mag(self, msg):
        self.latest_mag = msg

    def on_pose(self, msg):
        self.latest_pose = msg

    def on_cmd_vel(self, msg):
        self.motor_on = (
            abs(msg.linear.x) > self.motor_on_threshold
            or abs(msg.angular.z) > self.motor_on_threshold
        )

    def on_timer(self):
        # Planted mode synthesizes the field from pose alone; Gazebo mag optional.
        if self.plant_magnetic_target:
            if self.latest_pose is None:
                return
            pose_x = self.latest_pose.x
            pose_y = self.latest_pose.y
            heading = self.latest_pose.theta
            if self.dipole_model == 'total_field':
                mx, my, mz = self.dipole_m
                bx, by, bz, _scalar, _dF = planted_dipole_field_nt(
                    pose_x,
                    pose_y,
                    self.sensor_z,
                    self.target_x,
                    self.target_y,
                    self.target_z,
                    mx,
                    my,
                    mz,
                    self.synthetic_background_nt,
                    self.earth_inclination_deg,
                    self.earth_declination_deg,
                    soft_m=self.dipole_soft_m,
                )
                sigma = self.synthetic_noise_nt
                bx += random.gauss(0.0, sigma)
                by += random.gauss(0.0, sigma)
                bz += random.gauss(0.0, sigma)
            else:
                anomaly = dipole_anomaly_nt(
                    pose_x,
                    pose_y,
                    self.target_x,
                    self.target_y,
                    self.dipole_strength_nt,
                    soft_m=self.dipole_soft_m,
                )
                noise = random.gauss(0.0, self.synthetic_noise_nt)
                bx = 0.0
                by = 0.0
                bz = self.synthetic_background_nt + anomaly + noise
        else:
            if self.latest_mag is None:
                return
            mag = self.latest_mag
            bx = tesla_to_nt(mag.magnetic_field.x)
            by = tesla_to_nt(mag.magnetic_field.y)
            bz = tesla_to_nt(mag.magnetic_field.z)
            if self.latest_pose is not None:
                pose_x = self.latest_pose.x
                pose_y = self.latest_pose.y
                heading = self.latest_pose.theta
            else:
                pose_x = math.nan
                pose_y = math.nan
                heading = math.nan

        reading = MagReading()
        reading.header.stamp = self.get_clock().now().to_msg()
        reading.header.frame_id = self.frame_id
        reading.bx = bx
        reading.by = by
        reading.bz = bz
        reading.scalar = vector_scalar(bx, by, bz)
        reading.motor_on = self.motor_on
        reading.x = pose_x
        reading.y = pose_y
        reading.heading = heading

        self.raw_pub.publish(reading)
        self.published_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MagDriver()
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
