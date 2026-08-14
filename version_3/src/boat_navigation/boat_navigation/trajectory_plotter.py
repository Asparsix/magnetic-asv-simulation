#!/usr/bin/env python3
"""Live 2-D trajectory display with true/estimated target and magnetic readings.

Supports one or more ASVs in a single window. Pass ``asv_namespaces`` (e.g.
``['asv1', 'asv2']``) to draw every boat's track / heading / route / mag
readings in its own colour; leave it empty for the classic single-boat mode
that reads relative topics under the node's own namespace.
"""

import math

from boat_msgs.msg import MagAnomaly, MissionState, VerifyResult
from geometry_msgs.msg import Pose2D, PoseStamped
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.gridspec import GridSpec
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String


# Match Gazebo hull colours: ASV1 red, ASV2 blue (then green / purple extras).
BOAT_COLORS = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']
ROUTE_COLORS = ['#fc8d62', '#80b1d3', '#b3de69', '#bc80bd']

# Banner colours for the mission-phase status strip.
PHASE_BANNER = {
    'SEARCHING': ('#1b4f72', '#d6eaf8'),   # dark blue on light blue
    'HUNTING': ('#7d3c98', '#f5eef8'),     # purple
    'HOLD': ('#b9770e', '#fef9e7'),        # amber
    'VERIFYING': ('#b03a2e', '#fdedec'),   # red
    'CONFIRMED': ('#196f3d', '#e8f8f5'),   # green
    'WAITING': ('#566573', '#eaecee'),     # grey
}


class _BoatView:
    """Mutable per-boat plotting state."""

    def __init__(self, label, color, route_color):
        self.label = label
        self.color = color
        self.route_color = route_color
        self.x_history = []
        self.y_history = []
        self.current_pose = None
        self.route_x = []
        self.route_y = []
        self.mag_x = []
        self.mag_y = []
        self.mag_val = []
        self.mag_t = []
        self.start_time = None
        self.trajectory_pub = None
        self.mode = '—'
        self.hunt_phase = ''
        self.confirmations = 0
        self.peak_p = 0.0


class TrajectoryPlotter(Node):
    """Plot trajectory, heading, planned route, targets, and magnetic field."""

    def __init__(self):
        super().__init__('trajectory_plotter')
        self.declare_parameter('pose_topic', 'pose2d')
        self.declare_parameter('trajectory_topic', 'trajectory')
        self.declare_parameter('active_plan_topic', 'plan/active')
        self.declare_parameter('anomaly_topic', 'mag/anomaly')
        # Empty string array placeholder so ROS types this as STRING_ARRAY.
        # Pass real namespaces (e.g. ['asv1','asv2']) for multi-boat mode;
        # leave unset / ['' ] for classic single-boat relative topics.
        self.declare_parameter('asv_namespaces', [''])
        self.declare_parameter('peak_topic', '/swarm/belief/peak')
        self.declare_parameter(
            'peak_probability_topic', '/swarm/belief/peak_probability'
        )
        self.declare_parameter('centroid_topic', '/swarm/belief/centroid')
        self.declare_parameter(
            'centroid_spread_topic', '/swarm/belief/centroid_spread'
        )
        self.declare_parameter('fix_topic', '/swarm/belief/fix')
        self.declare_parameter('fix_rms_topic', '/swarm/belief/fix_rms')
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
        self.declare_parameter('mission_state_topic', 'mission/state')
        self.declare_parameter(
            'verify_result_topic', '/swarm/verify/result'
        )
        self.declare_parameter(
            'mission_complete_topic', '/swarm/mission/complete'
        )
        self.declare_parameter(
            'mission_status_topic', '/swarm/mission/status'
        )
        self.declare_parameter('verify_confirmations_required', 4)

        pose_topic = self.get_parameter('pose_topic').value
        trajectory_topic = self.get_parameter('trajectory_topic').value
        active_plan_topic = self.get_parameter('active_plan_topic').value
        anomaly_topic = self.get_parameter('anomaly_topic').value
        mission_state_topic = self.get_parameter('mission_state_topic').value
        namespaces = [
            ns for ns in (self.get_parameter('asv_namespaces').value or [])
            if str(ns).strip()
        ]
        peak_topic = self.get_parameter('peak_topic').value
        peak_prob_topic = self.get_parameter('peak_probability_topic').value
        centroid_topic = self.get_parameter('centroid_topic').value
        centroid_spread_topic = self.get_parameter(
            'centroid_spread_topic'
        ).value
        fix_topic = self.get_parameter('fix_topic').value
        fix_rms_topic = self.get_parameter('fix_rms_topic').value
        verify_result_topic = self.get_parameter('verify_result_topic').value
        mission_complete_topic = self.get_parameter(
            'mission_complete_topic'
        ).value
        mission_status_topic = self.get_parameter(
            'mission_status_topic'
        ).value
        self.verify_required = int(
            self.get_parameter('verify_confirmations_required').value
        )
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

        # Shared swarm belief overlays (identical regardless of boat count).
        self.est_x = None
        self.est_y = None
        self.est_p = None
        self.peak_x = None
        self.peak_y = None
        self.est_spread = None
        self.fix_x = None
        self.fix_y = None
        self.fix_rms = None
        self.mission_confirmed = False
        self.swarm_status = ''
        self.last_verify_result = None

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Build one view per boat. No namespaces -> single relative-topic boat.
        self.boats = []
        if namespaces:
            specs = [
                (ns, f'/{ns}/{pose_topic}', f'/{ns}/{active_plan_topic}',
                 f'/{ns}/{anomaly_topic}', f'/{ns}/{trajectory_topic}',
                 f'/{ns}/{mission_state_topic}',
                 ns.upper())
                for ns in namespaces
            ]
        else:
            specs = [
                (None, pose_topic, active_plan_topic, anomaly_topic,
                 trajectory_topic, mission_state_topic, 'Boat')
            ]

        for index, (
            ns, pose_t, plan_t, anom_t, traj_t, state_t, label
        ) in enumerate(specs):
            boat = _BoatView(
                label=label,
                color=BOAT_COLORS[index % len(BOAT_COLORS)],
                route_color=ROUTE_COLORS[index % len(ROUTE_COLORS)],
            )
            boat.trajectory_pub = self.create_publisher(
                Path, traj_t, path_qos
            )
            self.create_subscription(
                Pose2D, pose_t,
                lambda msg, b=boat: self.on_pose(msg, b), 10
            )
            self.create_subscription(
                Path, plan_t,
                lambda msg, b=boat: self.on_active_plan(msg, b), path_qos
            )
            self.create_subscription(
                MagAnomaly, anom_t,
                lambda msg, b=boat: self.on_anomaly(msg, b), 20
            )
            self.create_subscription(
                MissionState, state_t,
                lambda msg, b=boat: self.on_mission_state(msg, b), 10
            )
            self.boats.append(boat)

        self.create_subscription(
            PoseStamped, peak_topic, self.on_peak, 10
        )
        self.create_subscription(
            Float64, peak_prob_topic, self.on_peak_prob, 10
        )
        self.create_subscription(
            PoseStamped, centroid_topic, self.on_centroid, 10
        )
        self.create_subscription(
            Float64, centroid_spread_topic, self.on_centroid_spread, 10
        )
        self.create_subscription(PoseStamped, fix_topic, self.on_fix, 10)
        self.create_subscription(Float64, fix_rms_topic, self.on_fix_rms, 10)
        self.create_subscription(
            VerifyResult, verify_result_topic, self.on_verify_result, 10
        )
        self.create_subscription(
            Bool, mission_complete_topic, self.on_mission_complete, path_qos
        )
        self.create_subscription(
            String, mission_status_topic, self.on_swarm_status, path_qos
        )

        plt.ion()
        self.figure = plt.figure(figsize=(16, 9.5))
        grid = GridSpec(
            2, 2, figure=self.figure,
            height_ratios=[1.0, 0.18],
            width_ratios=[1.25, 1.0],
            hspace=0.28, wspace=0.22,
        )
        self.ax_map = self.figure.add_subplot(grid[0, 0])
        self.ax_mag = self.figure.add_subplot(grid[0, 1])
        self.ax_status = self.figure.add_subplot(grid[1, :])
        self.ax_status.set_axis_off()
        self.figure.canvas.manager.set_window_title(
            'Boat Trajectory, Mission Phase, and Magnetic Field'
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
            'Plotting {} boat(s): {}'.format(
                len(self.boats),
                ', '.join(b.label for b in self.boats),
            )
        )

    def on_pose(self, msg, boat):
        boat.current_pose = msg
        if boat.x_history:
            distance = math.hypot(
                msg.x - boat.x_history[-1],
                msg.y - boat.y_history[-1],
            )
            if distance < self.sample_distance:
                return

        boat.x_history.append(msg.x)
        boat.y_history.append(msg.y)
        if len(boat.x_history) > self.max_history:
            overflow = len(boat.x_history) - self.max_history
            del boat.x_history[:overflow]
            del boat.y_history[:overflow]

    def on_active_plan(self, msg, boat):
        boat.route_x = [pose.pose.position.x for pose in msg.poses]
        boat.route_y = [pose.pose.position.y for pose in msg.poses]

    def on_anomaly(self, msg, boat):
        if not math.isfinite(msg.x) or not math.isfinite(msg.y):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if boat.start_time is None:
            boat.start_time = now
        boat.mag_x.append(msg.x)
        boat.mag_y.append(msg.y)
        boat.mag_val.append(abs(msg.cleaned_anomaly_nt))
        boat.mag_t.append(now - boat.start_time)
        if len(boat.mag_x) > self.mag_history:
            overflow = len(boat.mag_x) - self.mag_history
            del boat.mag_x[:overflow]
            del boat.mag_y[:overflow]
            del boat.mag_val[:overflow]
            del boat.mag_t[:overflow]

    def on_peak(self, msg):
        self.peak_x = msg.pose.position.x
        self.peak_y = msg.pose.position.y
        if self.est_x is None:
            self.est_x = self.peak_x
            self.est_y = self.peak_y

    def on_peak_prob(self, msg):
        self.est_p = float(msg.data)

    def on_centroid(self, msg):
        self.est_x = msg.pose.position.x
        self.est_y = msg.pose.position.y

    def on_centroid_spread(self, msg):
        self.est_spread = float(msg.data)

    def on_fix(self, msg):
        self.fix_x = msg.pose.position.x
        self.fix_y = msg.pose.position.y

    def on_fix_rms(self, msg):
        self.fix_rms = float(msg.data)

    def on_mission_state(self, msg, boat):
        boat.mode = str(msg.mode or '—')
        boat.hunt_phase = str(msg.hunt_phase or '')
        boat.confirmations = int(msg.confirmations)
        boat.peak_p = float(msg.peak_p)
        if boat.mode == 'COMPLETE':
            self.mission_confirmed = True

    def on_verify_result(self, msg):
        self.last_verify_result = msg
        if msg.success:
            self.mission_confirmed = True

    def on_mission_complete(self, msg):
        if msg.data:
            self.mission_confirmed = True

    def on_swarm_status(self, msg):
        self.swarm_status = str(msg.data or '')

    def publish_trajectory(self):
        for boat in self.boats:
            if not boat.x_history:
                continue
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = 'map'
            for x, y in zip(boat.x_history, boat.y_history):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            boat.trajectory_pub.publish(path)

    def update_plot(self):
        any_pose = any(b.current_pose is not None for b in self.boats)
        if not any_pose or not plt.fignum_exists(self.figure.number):
            return

        self._draw_map()
        self._draw_mag()
        self._draw_status()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def _phase_summary(self):
        """Return (banner_key, headline, detail_lines)."""
        if self.mission_confirmed:
            detail = []
            if self.last_verify_result is not None and self.last_verify_result.success:
                vr = self.last_verify_result
                detail.append(
                    f'Verifier {vr.verifier_id}: {vr.confirmations}/'
                    f'{self.verify_required} confirmations'
                    f'  |  peak_p={vr.final_peak_p:.2f}'
                )
            if self.swarm_status:
                detail.append(self.swarm_status)
            for boat in self.boats:
                detail.append(f'{boat.label}: {boat.mode}')
            return (
                'CONFIRMED',
                'CONFIRMED  —  target verified',
                detail or ['Mission complete'],
            )

        modes = {b.mode for b in self.boats}
        verifying = [b for b in self.boats if b.mode == 'VERIFY']
        holding = [b for b in self.boats if b.mode == 'HOLD']
        hunting = [
            b for b in self.boats
            if b.mode == 'TARGET_SEARCH'
        ]

        if verifying:
            conf = max(b.confirmations for b in verifying)
            names = ', '.join(b.label for b in verifying)
            detail = [
                f'{b.label}: VERIFY  confirmations {b.confirmations}/'
                f'{self.verify_required}'
                for b in verifying
            ]
            for boat in self.boats:
                if boat not in verifying:
                    phase = boat.hunt_phase or '—'
                    detail.append(f'{boat.label}: {boat.mode} {phase}'.strip())
            return (
                'VERIFYING',
                f'VERIFYING  —  {names} confirming  '
                f'({conf}/{self.verify_required})',
                detail,
            )

        if holding:
            detail = [
                f'{b.label}: HOLD (waiting for peer verify)'
                for b in holding
            ]
            for boat in self.boats:
                if boat not in holding:
                    detail.append(
                        f'{boat.label}: {boat.mode} '
                        f'{boat.hunt_phase or ""}'.strip()
                    )
            return (
                'HOLD',
                'HOLD / DECLARE  —  discoverer waiting, peer will verify',
                detail,
            )

        if hunting:
            detail = []
            for boat in hunting:
                phase = boat.hunt_phase or 'HUNT'
                detail.append(
                    f'{boat.label}: TARGET_SEARCH / {phase}'
                    f'  p={boat.peak_p:.2f}'
                )
            for boat in self.boats:
                if boat not in hunting:
                    detail.append(f'{boat.label}: {boat.mode}')
            phases = sorted({(b.hunt_phase or 'HUNT') for b in hunting})
            return (
                'HUNTING',
                f'HUNTING  —  {" / ".join(phases)}'
                '  (estimate may still oscillate)',
                detail,
            )

        if any(b.mode == 'GLOBAL_SEARCH' for b in self.boats) or modes == {'—'}:
            detail = [
                f'{b.label}: {b.mode}'
                + (f'  p={b.peak_p:.2f}' if b.peak_p > 0 else '')
                for b in self.boats
            ]
            return (
                'SEARCHING',
                'SEARCHING  —  regional lawnmower coverage',
                detail,
            )

        detail = [f'{b.label}: {b.mode} {b.hunt_phase}'.strip() for b in self.boats]
        return ('WAITING', 'MISSION STATUS', detail)

    def _draw_status(self):
        axes = self.ax_status
        axes.clear()
        axes.set_axis_off()
        axes.set_xlim(0, 1)
        axes.set_ylim(0, 1)

        key, headline, details = self._phase_summary()
        fg, bg = PHASE_BANNER.get(key, PHASE_BANNER['WAITING'])
        axes.add_patch(
            plt.Rectangle(
                (0.01, 0.08), 0.98, 0.84,
                transform=axes.transAxes,
                facecolor=bg,
                edgecolor=fg,
                linewidth=2.5,
                zorder=0,
            )
        )
        axes.text(
            0.03, 0.62, headline,
            transform=axes.transAxes,
            fontsize=15,
            fontweight='bold',
            color=fg,
            va='center',
            ha='left',
            zorder=1,
        )
        if details:
            axes.text(
                0.03, 0.28, '   |   '.join(details),
                transform=axes.transAxes,
                fontsize=10,
                color='#2c3e50',
                va='center',
                ha='left',
                zorder=1,
            )

        # Tab-like phase chips on the right.
        chips = [
            ('SEARCH', 'SEARCHING'),
            ('HUNT', 'HUNTING'),
            ('VERIFY', 'VERIFYING'),
            ('CONFIRMED', 'CONFIRMED'),
        ]
        x0 = 0.58
        for i, (label, chip_key) in enumerate(chips):
            active = chip_key == key or (
                key == 'HOLD' and chip_key == 'VERIFYING'
            )
            chip_fg, chip_bg = PHASE_BANNER[chip_key]
            axes.add_patch(
                plt.Rectangle(
                    (x0 + i * 0.105, 0.55), 0.095, 0.32,
                    transform=axes.transAxes,
                    facecolor=chip_fg if active else '#f4f6f7',
                    edgecolor=chip_fg,
                    linewidth=1.5,
                    zorder=1,
                )
            )
            axes.text(
                x0 + i * 0.105 + 0.0475, 0.71,
                label,
                transform=axes.transAxes,
                fontsize=8,
                fontweight='bold',
                color='white' if active else chip_fg,
                ha='center',
                va='center',
                zorder=2,
            )

    def _draw_map(self):
        axes = self.ax_map
        axes.clear()
        multi = len(self.boats) > 1

        # Per-boat routes, tracks, mag scatter, boat marker, heading arrow.
        for boat in self.boats:
            if boat.route_x:
                axes.plot(
                    boat.route_x,
                    boat.route_y,
                    '--',
                    color=boat.route_color,
                    linewidth=1.5,
                    label=f'{boat.label} route' if multi else 'LOS route',
                )
            if boat.x_history:
                axes.plot(
                    boat.x_history,
                    boat.y_history,
                    color=boat.color,
                    linewidth=1.5,
                    alpha=0.6,
                    label=(
                        f'{boat.label} track' if multi
                        else 'Measured trajectory'
                    ),
                )
            if boat.mag_x:
                axes.scatter(
                    boat.mag_x,
                    boat.mag_y,
                    c=boat.mag_val,
                    cmap='inferno',
                    norm=self._norm,
                    s=14,
                    zorder=3,
                    label=None if multi else 'Mag readings',
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

        # Estimated target = weighted centroid of the high-p region.
        if self.est_x is not None:
            label = 'Centroid estimate'
            if self.est_p is not None:
                label = f'Centroid estimate (p={self.est_p:.2f})'
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
            if self.est_spread is not None and self.est_spread > 0.0:
                ring = plt.Circle(
                    (self.est_x, self.est_y),
                    self.est_spread,
                    fill=False,
                    color='magenta',
                    linestyle='--',
                    linewidth=1.2,
                    zorder=6,
                    label=f'Uncertainty σ={self.est_spread:.1f} m',
                )
                axes.add_patch(ring)
            if (
                self.peak_x is not None
                and (
                    abs(self.peak_x - self.est_x) > 0.5
                    or abs(self.peak_y - self.est_y) > 0.5
                )
            ):
                axes.scatter(
                    [self.peak_x],
                    [self.peak_y],
                    marker='+',
                    s=120,
                    color='orchid',
                    zorder=6,
                    label='Peak cell',
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

        # Dipole least-squares fix (refined continuous estimate).
        if self.fix_x is not None:
            label = 'Dipole fix'
            if self.fix_rms is not None:
                label = f'Dipole fix (rms={self.fix_rms:.1f} nT)'
            axes.scatter(
                [self.fix_x],
                [self.fix_y],
                marker='D',
                s=160,
                color='cyan',
                edgecolors='black',
                linewidths=1.2,
                zorder=8,
                label=label,
            )
            if self.show_true_target:
                axes.plot(
                    [self.target_x, self.fix_x],
                    [self.target_y, self.fix_y],
                    ':',
                    color='cyan',
                    linewidth=1.0,
                    zorder=5,
                )

        for boat in self.boats:
            pose = boat.current_pose
            if pose is None:
                continue
            axes.scatter(
                [pose.x],
                [pose.y],
                color=boat.color,
                edgecolors='black',
                linewidths=1.0,
                s=90,
                zorder=8,
                label=boat.label if multi else 'Boat',
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
                color=boat.color,
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

        title = self._map_title()
        axes.set_title(title)
        axes.set_xlabel('East / X (m)')
        axes.set_ylabel('North / Y (m)')
        axes.set_xlim(-limit - 10.0, limit + 10.0)
        axes.set_ylim(-limit - 10.0, limit + 10.0)
        axes.set_aspect('equal', adjustable='box')
        axes.grid(True, alpha=0.35)
        axes.legend(loc='upper right', fontsize=8)

    def _map_title(self):
        parts = []
        for boat in self.boats:
            pose = boat.current_pose
            if pose is None:
                continue
            parts.append(
                f'{boat.label}: ({pose.x:.0f}, {pose.y:.0f}) '
                f'{math.degrees(pose.theta):.0f}°'
            )
        title = 'Map: trajectories, targets, magnetic readings'
        if parts:
            title += '\n' + '   |   '.join(parts)
        if self.est_x is not None and self.show_true_target:
            err = math.hypot(
                self.est_x - self.target_x, self.est_y - self.target_y
            )
            title += f'\ncentroid err: {err:.1f} m'
            if self.fix_x is not None:
                fix_err = math.hypot(
                    self.fix_x - self.target_x, self.fix_y - self.target_y
                )
                title += f'  |  fix err: {fix_err:.1f} m'
        return title

    def _draw_mag(self):
        axes = self.ax_mag
        axes.clear()
        multi = len(self.boats) > 1
        plotted = False
        latest_parts = []
        for boat in self.boats:
            if not boat.mag_t:
                continue
            plotted = True
            axes.plot(
                boat.mag_t,
                boat.mag_val,
                color=boat.color,
                linewidth=1.2,
                label=(
                    f'{boat.label} |anomaly|' if multi else '|cleaned anomaly|'
                ),
            )
            latest_parts.append(f'{boat.label}: {boat.mag_val[-1]:.1f}')
        if plotted:
            axes.axhline(
                self.hit_threshold,
                color='red',
                linestyle='--',
                linewidth=1.0,
                label=f'hit threshold ({self.hit_threshold:.0f} nT)',
            )
            axes.set_title(
                'Magnetic anomaly vs time  |  latest: '
                + ('  '.join(latest_parts))
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
