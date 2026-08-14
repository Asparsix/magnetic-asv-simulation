#!/usr/bin/env python3
"""Fuse MagAnomaly samples into a shared Bayesian belief map."""

from boat_msgs.msg import BeliefGrid, MagAnomaly
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from boat_mapping.bayes_core import BeliefMap
from boat_mapping.dipole_fit import DipoleFitter


class BayesFusionNode(Node):
    """Subscribe to one or more ASV anomaly streams and publish swarm belief."""

    def __init__(self):
        super().__init__('bayes_fusion')

        self.declare_parameter('anomaly_topics', ['/asv1/mag/anomaly'])
        self.declare_parameter('map_topic', '/swarm/belief/map')
        self.declare_parameter('peak_topic', '/swarm/belief/peak')
        self.declare_parameter('peak_probability_topic', '/swarm/belief/peak_probability')
        self.declare_parameter('centroid_topic', '/swarm/belief/centroid')
        self.declare_parameter('centroid_mass_topic', '/swarm/belief/centroid_mass')
        self.declare_parameter('centroid_spread_topic', '/swarm/belief/centroid_spread')
        self.declare_parameter('fix_topic', '/swarm/belief/fix')
        self.declare_parameter('fix_rms_topic', '/swarm/belief/fix_rms')
        self.declare_parameter('fix_samples_topic', '/swarm/belief/fix_samples')
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
        # Cells with belief >= threshold_frac * peak_belief form the centroid region.
        self.declare_parameter('centroid_threshold_frac', 0.5)
        # Dipole least-squares refinement (same plant as mag_driver).
        self.declare_parameter('dipole_fit_enable', True)
        self.declare_parameter('dipole_model', 'scalar_soft')
        self.declare_parameter('dipole_soft_m', 20.0)
        self.declare_parameter('dipole_fit_target_z', -1.0)
        self.declare_parameter('dipole_fit_sensor_z', 0.0)
        self.declare_parameter('dipole_fit_free_depth', False)
        self.declare_parameter('dipole_fit_free_moment', False)
        self.declare_parameter('earth_inclination_deg', 15.0)
        self.declare_parameter('earth_declination_deg', -1.0)
        self.declare_parameter('earth_total_nt', 45000.0)
        self.declare_parameter('dipole_fit_guess_peak_nt', 50.0)
        self.declare_parameter('dipole_fit_min_anomaly_nt', 10.0)
        self.declare_parameter('dipole_fit_min_samples', 12)
        self.declare_parameter('dipole_fit_max_samples', 400)
        self.declare_parameter('dipole_fit_guess_strength_nt', 4.0e5)

        anomaly_topics = list(self.get_parameter('anomaly_topics').value)
        map_topic = self.get_parameter('map_topic').value
        peak_topic = self.get_parameter('peak_topic').value
        peak_p_topic = self.get_parameter('peak_probability_topic').value
        centroid_topic = self.get_parameter('centroid_topic').value
        centroid_mass_topic = self.get_parameter('centroid_mass_topic').value
        centroid_spread_topic = self.get_parameter('centroid_spread_topic').value
        fix_topic = self.get_parameter('fix_topic').value
        fix_rms_topic = self.get_parameter('fix_rms_topic').value
        fix_samples_topic = self.get_parameter('fix_samples_topic').value
        publish_hz = float(self.get_parameter('publish_rate_hz').value)
        self.centroid_threshold_frac = float(
            self.get_parameter('centroid_threshold_frac').value
        )
        self.dipole_fit_enable = bool(
            self.get_parameter('dipole_fit_enable').value
        )
        self.dipole_guess_strength = float(
            self.get_parameter('dipole_fit_guess_strength_nt').value
        )
        self.dipole_guess_peak = float(
            self.get_parameter('dipole_fit_guess_peak_nt').value
        )
        self.dipole_model = str(
            self.get_parameter('dipole_model').value
        ).strip().lower()

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
        self.fitter = DipoleFitter(
            soft_m=float(self.get_parameter('dipole_soft_m').value),
            min_anomaly_nt=float(
                self.get_parameter('dipole_fit_min_anomaly_nt').value
            ),
            min_samples=int(self.get_parameter('dipole_fit_min_samples').value),
            max_samples=int(self.get_parameter('dipole_fit_max_samples').value),
            dipole_model=self.dipole_model,
            target_z=float(self.get_parameter('dipole_fit_target_z').value),
            sensor_z=float(self.get_parameter('dipole_fit_sensor_z').value),
            earth_inclination_deg=float(
                self.get_parameter('earth_inclination_deg').value
            ),
            earth_declination_deg=float(
                self.get_parameter('earth_declination_deg').value
            ),
            earth_total_nt=float(self.get_parameter('earth_total_nt').value),
            free_depth=bool(self.get_parameter('dipole_fit_free_depth').value),
            free_moment=bool(self.get_parameter('dipole_fit_free_moment').value),
            guess_peak_nt=self.dipole_guess_peak,
        )
        self.latest_fix = None

        self.map_pub = self.create_publisher(BeliefGrid, map_topic, 10)
        self.peak_pub = self.create_publisher(PoseStamped, peak_topic, 10)
        self.peak_p_pub = self.create_publisher(Float64, peak_p_topic, 10)
        self.centroid_pub = self.create_publisher(PoseStamped, centroid_topic, 10)
        self.centroid_mass_pub = self.create_publisher(
            Float64, centroid_mass_topic, 10
        )
        self.centroid_spread_pub = self.create_publisher(
            Float64, centroid_spread_topic, 10
        )
        self.fix_pub = self.create_publisher(PoseStamped, fix_topic, 10)
        self.fix_rms_pub = self.create_publisher(Float64, fix_rms_topic, 10)
        self.fix_samples_pub = self.create_publisher(
            Float64, fix_samples_topic, 10
        )

        for topic in anomaly_topics:
            self.create_subscription(MagAnomaly, topic, self.on_anomaly, 50)

        self.create_timer(1.0 / publish_hz, self.publish_belief)
        self.get_logger().info(
            f'bayes_fusion listening on {anomaly_topics}; '
            f'grid={self.belief_map.width}x{self.belief_map.height}, '
            f'hit>={self.belief_map.hit_threshold_nt:.3g} nT, '
            f'centroid_frac={self.centroid_threshold_frac:.2f}, '
            f'dipole_fit={"on" if self.dipole_fit_enable else "off"} '
            f'model={self.dipole_model}'
        )

    def on_anomaly(self, msg):
        label = self.belief_map.update(
            msg.x,
            msg.y,
            msg.cleaned_anomaly_nt,
            is_calibrated=msg.is_calibrated,
        )
        if self.dipole_fit_enable and msg.is_calibrated:
            self.fitter.add_sample(msg.x, msg.y, msg.cleaned_anomaly_nt)
            self._maybe_refit()

        if label == 'HIT':
            peak = self.belief_map.peak()
            centroid = self.belief_map.weighted_centroid(
                self.centroid_threshold_frac
            )
            fix_str = 'n/a'
            if self.latest_fix is not None and self.latest_fix.success:
                fix_str = (
                    f'({self.latest_fix.x:.1f},{self.latest_fix.y:.1f}) '
                    f'rms={self.latest_fix.residual_rms_nt:.1f}nT '
                    f'n={self.latest_fix.num_samples}'
                )
            source = msg.header.frame_id or 'unknown'
            self.get_logger().info(
                f'HIT from {source} at ({msg.x:.1f},{msg.y:.1f}) '
                f'anomaly={msg.cleaned_anomaly_nt:.3g} nT; '
                f'peak_p={peak.probability:.4f} at ({peak.x:.1f},{peak.y:.1f}); '
                f'centroid=({centroid.x:.1f},{centroid.y:.1f}) '
                f'mass={centroid.mass:.3f} spread={centroid.spread_m:.1f}m; '
                f'fix={fix_str}',
                throttle_duration_sec=2.0,
            )

    def _maybe_refit(self):
        if len(self.fitter) < self.fitter.min_samples:
            return
        centroid = self.belief_map.weighted_centroid(
            self.centroid_threshold_frac
        )
        fix = self.fitter.fit(
            guess_xy=(centroid.x, centroid.y),
            guess_strength_nt=self.dipole_guess_strength,
            guess_peak_nt=self.dipole_guess_peak,
        )
        if fix is not None and fix.success:
            self.latest_fix = fix

    def publish_belief(self):
        peak = self.belief_map.peak()
        centroid = self.belief_map.weighted_centroid(self.centroid_threshold_frac)
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

        cpose = PoseStamped()
        cpose.header.stamp = stamp
        cpose.header.frame_id = 'map'
        cpose.pose.position.x = centroid.x
        cpose.pose.position.y = centroid.y
        cpose.pose.orientation.w = 1.0
        self.centroid_pub.publish(cpose)

        mass = Float64()
        mass.data = float(centroid.mass)
        self.centroid_mass_pub.publish(mass)

        spread = Float64()
        spread.data = float(centroid.spread_m)
        self.centroid_spread_pub.publish(spread)

        if self.latest_fix is not None and self.latest_fix.success:
            fpose = PoseStamped()
            fpose.header.stamp = stamp
            fpose.header.frame_id = 'map'
            fpose.pose.position.x = self.latest_fix.x
            fpose.pose.position.y = self.latest_fix.y
            fpose.pose.position.z = float(self.latest_fix.z)
            fpose.pose.orientation.w = 1.0
            self.fix_pub.publish(fpose)

            rms = Float64()
            rms.data = float(self.latest_fix.residual_rms_nt)
            self.fix_rms_pub.publish(rms)

            ns = Float64()
            ns.data = float(self.latest_fix.num_samples)
            self.fix_samples_pub.publish(ns)


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
