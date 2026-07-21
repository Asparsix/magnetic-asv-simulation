#!/usr/bin/env python3
"""Live 2-D trajectory display with true/estimated target and magnetic readings."""

import math

from boat_msgs.msg import MagAnomaly
from geometry_msgs.msg import Pose2D, PoseStamped
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


class TrajectoryPlotter(Node):
    """Plot trajectory, heading, planned route, targets, and magnetic field."""

    def __init__(self):
        super().__init__('trajectory_plotter')
        self.declare_parameter('pose_topic', 'pose2d')
        self.declare_parameter('trajectory_topic', 'trajectory')
        self.declare_parameter('active_plan_topic', 'plan/active')
        self.declare_parameter('anomaly_topic', 'mag/anomaly')
        self.declare_parameter('peak_topic', '/swarm/belief/peak')
        self.declare_parameter(
            'peak_probability_topic', '/swarm/belief/peak_probability'
        )
        self.declare_parameter('lake_half_size_m', 150.0)
        self.declare_parameter('max_history_points', 20000)
        self.declare_parameter('sample_distance_m', 0.2)
        self.declare_parameter('heading_arrow_m', 12.0)
        self.declare_parameter('show_true_target', True)
        self.declare_parameter('target_x', 80.0)
        self.declare_parameter('target_y', -40.0)
        self.declare_parameter('mag_history_points', 6000)
        self.declare_parameter('anomaly_vmax_nt', 55.0)
        self.declare_parameter('anomaly_hit_threshold_nt', 15.0)

        pose_topic = self.get_parameter('pose_topic').value
        trajectory_topic = self.get_parameter('trajectory_topic').value
        active_plan_topic = self.get_parameter('active_plan_topic').value
        anomaly_topic = self.get_parameter('anomaly_topic').value
        peak_topic = self.get_parameter('peak_topic').value
        peak_prob_topic = self.get_parameter('peak_probability_topic').value
        self.lake_half_size = float(
            self.get_parameter('lake_half_size_m').value
        )
        self.max_history = int(
            self.get_parameter('max_history_points').value
        )
        self.sample_distance = float(
            self.get_parameter('sample_distance_m').value
        )
        self.heading_arrow = float(
            self.get_parameter('heading_arrow_m').value
        )
        self.show_true_target = bool(
            self.get_parameter('show_true_target').value
        )
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.mag_history = int(
            self.get_parameter('mag_history_points').value
        )
        self.anomaly_vmax = float(
            self.get_parameter('anomaly_vmax_nt').value
        )
        self.hit_threshold = float(
            self.get_parameter('anomaly_hit_threshold_nt').value
        )

        self.x_history = []
        self.y_history = []
        self.current_pose = None
        self.route_x = []
        self.route_y = []

        # Magnetic readings sampled along the track.
        self.mag_x = []
        self.mag_y = []
        self.mag_val = []
        self.mag_t = []
        self.start_time = None

        # Estimated target (belief peak).
        self.est_x = None
        self.est_y = None
        self.est_p = None

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.trajectory_pub = self.create_publisher(
            Path, trajectory_topic, path_qos
        )
        self.create_subscription(Pose2D, pose_topic, self.on_pose, 10)
        self.create_subscription(
            Path, active_plan_topic, self.on_active_plan, path_qos
        )
        self.create_subscription(
            MagAnomaly, anomaly_topic, self.on_anomaly, 20
        )
        self.create_subscription(
            PoseStamped, peak_topic, self.on_peak, 10
        )
        self.create_subscription(
            Float64, peak_prob_topic, self.on_peak_prob, 10
        )

        plt.ion()
        self.figure, (self.ax_map, self.ax_mag) = plt.subplots(
            1, 2, figsize=(16, 8),
            gridspec_kw={'width_ratios': [1.25, 1.0]},
        )
        self.figure.canvas.manager.set_window_title(
            'Boat Trajectory, Target Estimate, and Magnetic Field'
        )
        self._norm = Normalize(vmin=0.0, vmax=self.anomaly_vmax)
        scalar_map = ScalarMappable(norm=self._norm, cmap='inferno')
        scalar_map.set_array([])
        self._cbar = self.figure.colorbar(
            scalar_map, ax=self.ax_map, fraction=0.046, pad=0.04
        )
        self._cbar.set_label('|cleaned anomaly| (nT)')

        self.create_timer(0.2, self.update_plot)
        self.create_timer(1.0, self.publish_trajectory)
        self.get_logger().info(
            f'Plotting poses from {pose_topic}; close window to hide plot'
        )

    def on_pose(self, msg):
        self.current_pose = msg
        if self.x_history:
            distance = math.hypot(
                msg.x - self.x_history[-1],
                msg.y - self.y_history[-1],
            )
            if distance < self.sample_distance:
                return

        self.x_history.append(msg.x)
        self.y_history.append(msg.y)
        if len(self.x_history) > self.max_history:
            overflow = len(self.x_history) - self.max_history
            del self.x_history[:overflow]
            del self.y_history[:overflow]

    def on_active_plan(self, msg):
        self.route_x = [pose.pose.position.x for pose in msg.poses]
        self.route_y = [pose.pose.position.y for pose in msg.poses]

    def on_anomaly(self, msg):
        if not math.isfinite(msg.x) or not math.isfinite(msg.y):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        self.mag_x.append(msg.x)
        self.mag_y.append(msg.y)
        self.mag_val.append(abs(msg.cleaned_anomaly_nt))
        self.mag_t.append(now - self.start_time)
        if len(self.mag_x) > self.mag_history:
            overflow = len(self.mag_x) - self.mag_history
            del self.mag_x[:overflow]
            del self.mag_y[:overflow]
            del self.mag_val[:overflow]
            del self.mag_t[:overflow]

    def on_peak(self, msg):
        self.est_x = msg.pose.position.x
        self.est_y = msg.pose.position.y

    def on_peak_prob(self, msg):
        self.est_p = float(msg.data)

    def publish_trajectory(self):
        if not self.x_history:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for x, y in zip(self.x_history, self.y_history):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.trajectory_pub.publish(path)

    def update_plot(self):
        if self.current_pose is None or not plt.fignum_exists(
            self.figure.number
        ):
            return

        self._draw_map()
        self._draw_mag()
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def _draw_map(self):
        pose = self.current_pose
        axes = self.ax_map
        axes.clear()

        if self.route_x:
            axes.plot(
                self.route_x,
                self.route_y,
                '--',
                color='orange',
                linewidth=1.5,
                label='LOS route',
            )
        if self.x_history:
            axes.plot(
                self.x_history,
                self.y_history,
                color='tab:blue',
                linewidth=1.5,
                alpha=0.5,
                label='Measured trajectory',
            )

        # Magnetic readings colored by anomaly magnitude.
        if self.mag_x:
            axes.scatter(
                self.mag_x,
                self.mag_y,
                c=self.mag_val,
                cmap='inferno',
                norm=self._norm,
                s=14,
                zorder=3,
                label='Mag readings',
            )

        # True (planted) target.
        if self.show_true_target:
            axes.scatter(
                [self.target_x],
                [self.target_y],
                marker='X',
                s=220,
                color='red',
                edgecolors='black',
                linewidths=1.5,
                zorder=6,
                label='True target',
            )

        # Estimated target (belief peak).
        if self.est_x is not None:
            label = 'Estimated target'
            if self.est_p is not None:
                label = f'Estimated target (p={self.est_p:.2f})'
            axes.scatter(
                [self.est_x],
                [self.est_y],
                marker='*',
                s=360,
                color='magenta',
                edgecolors='black',
                linewidths=1.2,
                zorder=7,
                label=label,
            )
            if self.show_true_target:
                axes.plot(
                    [self.target_x, self.est_x],
                    [self.target_y, self.est_y],
                    ':',
                    color='magenta',
                    linewidth=1.0,
                    zorder=5,
                )

        axes.scatter(
            [pose.x],
            [pose.y],
            color='dodgerblue',
            s=70,
            zorder=8,
            label='Boat',
        )
        dx = self.heading_arrow * math.cos(pose.theta)
        dy = self.heading_arrow * math.sin(pose.theta)
        axes.arrow(
            pose.x,
            pose.y,
            dx,
            dy,
            width=0.8,
            head_width=4.0,
            head_length=5.0,
            color='dodgerblue',
            length_includes_head=True,
            zorder=8,
        )

        limit = self.lake_half_size
        axes.plot(
            [-limit, limit, limit, -limit, -limit],
            [-limit, -limit, limit, limit, -limit],
            color='forestgreen',
            linewidth=3.0,
            label='Lake shore',
        )
        axes.scatter(
            [40.0],
            [30.0],
            marker='o',
            s=180,
            color='forestgreen',
            label='Island',
        )

        heading_deg = math.degrees(pose.theta)
        title = (
            'Map: trajectory, targets, magnetic readings\n'
            f'Position: ({pose.x:.1f}, {pose.y:.1f}) m  |  '
            f'Heading: {heading_deg:.1f}°'
        )
        if self.est_x is not None and self.show_true_target:
            err = math.hypot(
                self.est_x - self.target_x, self.est_y - self.target_y
            )
            title += f'  |  est. error: {err:.1f} m'
        axes.set_title(title)
        axes.set_xlabel('East / X (m)')
        axes.set_ylabel('North / Y (m)')
        axes.set_xlim(-limit - 10.0, limit + 10.0)
        axes.set_ylim(-limit - 10.0, limit + 10.0)
        axes.set_aspect('equal', adjustable='box')
        axes.grid(True, alpha=0.35)
        axes.legend(loc='upper right', fontsize=8)

    def _draw_mag(self):
        axes = self.ax_mag
        axes.clear()
        if self.mag_t:
            axes.plot(
                self.mag_t,
                self.mag_val,
                color='tab:purple',
                linewidth=1.2,
                label='|cleaned anomaly|',
            )
            axes.axhline(
                self.hit_threshold,
                color='red',
                linestyle='--',
                linewidth=1.0,
                label=f'hit threshold ({self.hit_threshold:.0f} nT)',
            )
            latest = self.mag_val[-1]
            axes.set_title(
                f'Magnetic anomaly vs time  |  latest: {latest:.1f} nT'
            )
            axes.legend(loc='upper left', fontsize=8)
        else:
            axes.set_title('Magnetic anomaly vs time (waiting for data)')
        axes.set_xlabel('Time since first reading (s)')
        axes.set_ylabel('|cleaned anomaly| (nT)')
        axes.grid(True, alpha=0.35)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
