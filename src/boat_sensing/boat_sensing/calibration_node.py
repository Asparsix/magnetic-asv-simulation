#!/usr/bin/env python3
"""Build ambient baseline and publish cleaned magnetic anomalies."""

import math

from boat_msgs.msg import CalibrationStatus, MagAnomaly, MagReading
import rclpy
from rclpy.node import Node

from boat_sensing.calibration_core import (
    BaselineMap,
    MagneticCalibrator,
    TemporalHighPass,
)


class CalibrationNode(Node):
    """Convert filtered MagReading into MagAnomaly with calibration status."""

    def __init__(self):
        super().__init__('calibration_node')

        self.declare_parameter('filtered_topic', 'mag/filtered')
        self.declare_parameter('anomaly_topic', 'mag/anomaly')
        self.declare_parameter('status_topic', 'calibration/status')
        self.declare_parameter('area_size_m', 300.0)
        self.declare_parameter('origin_x', -150.0)
        self.declare_parameter('origin_y', -150.0)
        self.declare_parameter('cell_size_m', 20.0)
        self.declare_parameter('num_heading_bins', 8)
        self.declare_parameter('min_cell_samples', 1)
        self.declare_parameter('reject_residual_nt', 5.0e7)
        self.declare_parameter('temporal_window', 12)
        self.declare_parameter('noise_floor_nt', 0.0)
        self.declare_parameter('ready_coverage_percent', 5.0)
        self.declare_parameter('ready_min_cells', 8)
        self.declare_parameter('freeze_baseline_when_ready', True)
        self.declare_parameter('status_rate_hz', 1.0)

        baseline = BaselineMap(
            area_size_m=float(self.get_parameter('area_size_m').value),
            origin_x=float(self.get_parameter('origin_x').value),
            origin_y=float(self.get_parameter('origin_y').value),
            cell_size_m=float(self.get_parameter('cell_size_m').value),
            num_heading_bins=int(self.get_parameter('num_heading_bins').value),
            min_cell_samples=int(self.get_parameter('min_cell_samples').value),
            reject_residual_nt=float(
                self.get_parameter('reject_residual_nt').value
            ),
        )
        temporal = TemporalHighPass(
            window=int(self.get_parameter('temporal_window').value),
            noise_floor_nt=float(self.get_parameter('noise_floor_nt').value),
        )
        self.calibrator = MagneticCalibrator(
            baseline_map=baseline,
            temporal=temporal,
            ready_coverage_percent=float(
                self.get_parameter('ready_coverage_percent').value
            ),
            ready_min_cells=int(self.get_parameter('ready_min_cells').value),
            freeze_baseline_when_ready=bool(
                self.get_parameter('freeze_baseline_when_ready').value
            ),
        )

        filtered_topic = self.get_parameter('filtered_topic').value
        anomaly_topic = self.get_parameter('anomaly_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.anomaly_pub = self.create_publisher(MagAnomaly, anomaly_topic, 10)
        self.status_pub = self.create_publisher(
            CalibrationStatus, status_topic, 10
        )
        self.create_subscription(MagReading, filtered_topic, self.on_filtered, 50)
        status_hz = float(self.get_parameter('status_rate_hz').value)
        self.create_timer(1.0 / status_hz, self.publish_status)

        self.get_logger().info(
            f'calibration_node {filtered_topic} -> {anomaly_topic} '
            f'(grid={baseline.grid_size}x{baseline.grid_size}, '
            f'heading_bins={baseline.num_heading_bins})'
        )

    def on_filtered(self, msg):
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.heading):
            return

        sample = self.calibrator.process(
            msg.x, msg.y, msg.heading, msg.scalar
        )

        anomaly = MagAnomaly()
        anomaly.header = msg.header
        anomaly.raw_nt = sample.raw_nt
        anomaly.baseline_nt = (
            sample.baseline_nt if sample.is_calibrated else float('nan')
        )
        anomaly.cleaned_anomaly_nt = sample.cleaned_anomaly_nt
        anomaly.is_calibrated = sample.is_calibrated
        anomaly.heading_bin = sample.heading_bin
        anomaly.grid_cell_x = sample.grid_cell_x
        anomaly.grid_cell_y = sample.grid_cell_y
        anomaly.x = sample.x
        anomaly.y = sample.y
        anomaly.heading = sample.heading
        self.anomaly_pub.publish(anomaly)

    def publish_status(self):
        status = CalibrationStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'map'
        status.phase = self.calibrator.phase
        status.cells_sampled = self.calibrator.baseline.cells_sampled()
        status.coverage_percent = float(
            self.calibrator.baseline.coverage_percent()
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
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
