#!/usr/bin/env python3
"""Fuse MagAnomaly samples into a shared Bayesian belief map."""

from boat_msgs.msg import BeliefGrid, MagAnomaly
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from boat_mapping.bayes_core import BeliefMap


class BayesFusionNode(Node):
    """Subscribe to one or more ASV anomaly streams and publish swarm belief."""

    def __init__(self):
        super().__init__('bayes_fusion')

        self.declare_parameter('anomaly_topics', ['/asv1/mag/anomaly'])
        self.declare_parameter('map_topic', '/swarm/belief/map')
        self.declare_parameter('peak_topic', '/swarm/belief/peak')
        self.declare_parameter('peak_probability_topic', '/swarm/belief/peak_probability')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('area_size_m', 300.0)
        self.declare_parameter('origin_x', -150.0)
        self.declare_parameter('origin_y', -150.0)
        self.declare_parameter('cell_size_m', 20.0)
        self.declare_parameter('p_bg', 0.05)
        self.declare_parameter('p_max', 0.95)
        self.declare_parameter('d_half', 30.0)
        # Defaults sized for inflated Gazebo mag magnitudes; retune for field nT.
        self.declare_parameter('hit_threshold_nt', 5.0e7)
        self.declare_parameter('miss_threshold_nt', 1.0e6)
        self.declare_parameter('hit_only', True)

        anomaly_topics = list(self.get_parameter('anomaly_topics').value)
        map_topic = self.get_parameter('map_topic').value
        peak_topic = self.get_parameter('peak_topic').value
        peak_p_topic = self.get_parameter('peak_probability_topic').value
        publish_hz = float(self.get_parameter('publish_rate_hz').value)

        self.belief_map = BeliefMap(
            area_size_m=float(self.get_parameter('area_size_m').value),
            origin_x=float(self.get_parameter('origin_x').value),
            origin_y=float(self.get_parameter('origin_y').value),
            cell_size_m=float(self.get_parameter('cell_size_m').value),
            p_bg=float(self.get_parameter('p_bg').value),
            p_max=float(self.get_parameter('p_max').value),
            d_half=float(self.get_parameter('d_half').value),
            hit_threshold_nt=float(self.get_parameter('hit_threshold_nt').value),
            miss_threshold_nt=float(self.get_parameter('miss_threshold_nt').value),
            hit_only=bool(self.get_parameter('hit_only').value),
        )

        self.map_pub = self.create_publisher(BeliefGrid, map_topic, 10)
        self.peak_pub = self.create_publisher(PoseStamped, peak_topic, 10)
        self.peak_p_pub = self.create_publisher(Float64, peak_p_topic, 10)

        for topic in anomaly_topics:
            self.create_subscription(MagAnomaly, topic, self.on_anomaly, 50)

        self.create_timer(1.0 / publish_hz, self.publish_belief)
        self.get_logger().info(
            f'bayes_fusion listening on {anomaly_topics}; '
            f'grid={self.belief_map.width}x{self.belief_map.height}, '
            f'hit>={self.belief_map.hit_threshold_nt:.3g} nT'
        )

    def on_anomaly(self, msg):
        label = self.belief_map.update(
            msg.x,
            msg.y,
            msg.cleaned_anomaly_nt,
            is_calibrated=msg.is_calibrated,
        )
        if label == 'HIT':
            peak = self.belief_map.peak()
            self.get_logger().info(
                f'HIT update at ({msg.x:.1f},{msg.y:.1f}) '
                f'anomaly={msg.cleaned_anomaly_nt:.3g} nT; '
                f'peak_p={peak.probability:.4f} at ({peak.x:.1f},{peak.y:.1f})',
                throttle_duration_sec=2.0,
            )

    def publish_belief(self):
        peak = self.belief_map.peak()
        stamp = self.get_clock().now().to_msg()

        grid = BeliefGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = 'map'
        grid.resolution = float(self.belief_map.cell_size_m)
        grid.origin_x = self.belief_map.origin_x
        grid.origin_y = self.belief_map.origin_y
        grid.width = self.belief_map.width
        grid.height = self.belief_map.height
        grid.data = self.belief_map.as_row_major()
        self.map_pub.publish(grid)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = peak.x
        pose.pose.position.y = peak.y
        pose.pose.orientation.w = 1.0
        self.peak_pub.publish(pose)

        peak_p = Float64()
        peak_p.data = peak.probability
        self.peak_p_pub.publish(peak_p)


def main(args=None):
    rclpy.init(args=args)
    node = BayesFusionNode()
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
