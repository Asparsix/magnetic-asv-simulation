#!/usr/bin/env python3
"""Low-pass and spike-reject filter for MagReading streams."""

from boat_msgs.msg import MagReading
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node

from boat_sensing.filter_core import MagnetometerFilterChain


class MagFilter(Node):
    """Filter MagReading samples and publish cleaned MagReading output."""

    def __init__(self):
        super().__init__('mag_filter')

        self.declare_parameter('raw_topic', 'mag/raw')
        self.declare_parameter('filtered_topic', 'mag/filtered')
        self.declare_parameter('status_topic', 'mag/filter_status')
        self.declare_parameter('lowpass_window', 5)
        self.declare_parameter('spike_history', 20)
        self.declare_parameter('spike_n_sigma', 3.0)
        self.declare_parameter('min_std_nt', 1.0)
        self.declare_parameter('status_rate_hz', 1.0)

        raw_topic = self.get_parameter('raw_topic').value
        filtered_topic = self.get_parameter('filtered_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.chain = MagnetometerFilterChain(
            lowpass_window=int(self.get_parameter('lowpass_window').value),
            spike_history=int(self.get_parameter('spike_history').value),
            spike_n_sigma=float(self.get_parameter('spike_n_sigma').value),
            min_std_nt=float(self.get_parameter('min_std_nt').value),
        )
        self.received_count = 0
        self.published_count = 0

        self.filtered_pub = self.create_publisher(MagReading, filtered_topic, 10)
        self.status_pub = self.create_publisher(
            DiagnosticStatus, status_topic, 10
        )
        self.create_subscription(MagReading, raw_topic, self.on_raw, 50)
        status_hz = float(self.get_parameter('status_rate_hz').value)
        self.create_timer(1.0 / status_hz, self.publish_status)

        self.get_logger().info(
            f'mag_filter {raw_topic} -> {filtered_topic} '
            f'(LP window={self.chain.lowpass["bx"].window_size}, '
            f'spike n_sigma={self.chain.spike["bx"].n_sigma})'
        )

    def on_raw(self, msg):
        self.received_count += 1
        filtered = self.chain.update(msg.bx, msg.by, msg.bz)

        out = MagReading()
        out.header = msg.header
        out.bx = filtered['bx']
        out.by = filtered['by']
        out.bz = filtered['bz']
        out.scalar = filtered['scalar']
        out.x = msg.x
        out.y = msg.y
        out.heading = msg.heading
        out.motor_on = msg.motor_on
        self.filtered_pub.publish(out)
        self.published_count += 1

    def publish_status(self):
        status = DiagnosticStatus()
        status.name = 'mag_filter'
        status.hardware_id = self.get_namespace().strip('/') or 'asv'
        status.level = DiagnosticStatus.OK
        status.message = 'filtering'
        status.values = [
            KeyValue(key='received', value=str(self.received_count)),
            KeyValue(key='published', value=str(self.published_count)),
            KeyValue(
                key='spikes_rejected',
                value=str(self.chain.rejected_count),
            ),
            KeyValue(
                key='last_sample_rejected',
                value=str(self.chain.last_rejected),
            ),
        ]
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = MagFilter()
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
