#!/usr/bin/env python3
"""Monte Carlo only: slow bias ramp on magnetometer readings.

Inserts between mag_driver (mag/plant) and mag_filter (mag/raw). Not launched
in normal sim.launch or multi_asv_phase4.
"""

from __future__ import annotations

import math

from boat_msgs.msg import MagReading
import rclpy
from rclpy.node import Node

from boat_sensing.filter_core import vector_scalar


class MonteCarloMagDrift(Node):
    """Integrate a constant baseline drift vector onto raw mag samples."""

    def __init__(self):
        super().__init__('monte_carlo_mag_drift')
        self.declare_parameter('input_topic', 'mag/plant')
        self.declare_parameter('output_topic', 'mag/raw')
        self.declare_parameter('drift_nt_per_min', 0.0)
        self.declare_parameter('drift_azimuth_deg', 0.0)
        self.declare_parameter('drift_vertical_fraction', 0.15)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate = float(self.get_parameter('drift_nt_per_min').value) / 60.0
        az = math.radians(float(self.get_parameter('drift_azimuth_deg').value))
        vert_frac = max(0.0, float(self.get_parameter('drift_vertical_fraction').value))

        self.dbx_dt = rate * math.cos(az)
        self.dby_dt = rate * math.sin(az)
        self.dbz_dt = rate * vert_frac
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0
        self.last_time = None

        self.pub = self.create_publisher(MagReading, output_topic, 10)
        self.create_subscription(MagReading, input_topic, self.on_reading, 50)
        self.get_logger().info(
            f'MC baseline drift {rate * 60.0:.3f} nT/min '
            f'az={math.degrees(az):.1f}° on {input_topic} -> {output_topic}'
        )

    def on_reading(self, msg: MagReading) -> None:
        now = self.get_clock().now()
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1.0e-9
            if dt > 0.0:
                self.bias_x += self.dbx_dt * dt
                self.bias_y += self.dby_dt * dt
                self.bias_z += self.dbz_dt * dt
        self.last_time = now

        out = MagReading()
        out.header = msg.header
        out.header.stamp = now.to_msg()
        out.bx = float(msg.bx) + self.bias_x
        out.by = float(msg.by) + self.bias_y
        out.bz = float(msg.bz) + self.bias_z
        out.scalar = vector_scalar(out.bx, out.by, out.bz)
        out.motor_on = msg.motor_on
        out.x = msg.x
        out.y = msg.y
        out.heading = msg.heading
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MonteCarloMagDrift()
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
