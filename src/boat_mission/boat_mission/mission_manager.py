#!/usr/bin/env python3
"""Mission state machine: lawnmower → MI info-gain → spiral → verify → complete."""

from __future__ import annotations

import math

from boat_msgs.msg import (
    BeliefGrid,
    CalibrationStatus,
    MagAnomaly,
    MissionState,
    VerifyRequest,
    VerifyResult,
)
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from boat_mission.info_gain import InfoGainPlanner
from boat_mission.path_planning import (
    generate_expanding_spiral,
    generate_region_lawnmower,
    generate_split_lawnmower,
    generate_verification_orbit,
    opposite_approach_angle,
)
from boat_mission.verify_core import VerificationTracker
from boat_navigation.path_utils import make_path


class MissionManager(Node):
    """Drive GLOBAL_SEARCH → TARGET_SEARCH → VERIFY → COMPLETE."""

    MODE_GLOBAL = 'GLOBAL_SEARCH'
    MODE_TARGET = 'TARGET_SEARCH'
    MODE_HOLD = 'HOLD'
    MODE_VERIFY = 'VERIFY'
    MODE_COMPLETE = 'COMPLETE'

    PHASE_INFO = 'INFO_GAIN'
    PHASE_SPIRAL = 'SPIRAL'

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('asv_id', '')
        self.declare_parameter('pose_topic', 'pose2d')
        self.declare_parameter('plan_topic', 'plan')
        self.declare_parameter('state_topic', 'mission/state')
        self.declare_parameter('mode_topic', 'mission/mode')
        self.declare_parameter('info_gain_topic', 'mission/info_gain')
        self.declare_parameter('anomaly_topic', 'mag/anomaly')
        self.declare_parameter('calibration_status_topic', 'calibration/status')
        self.declare_parameter('peak_topic', '/swarm/belief/peak')
        self.declare_parameter(
            'peak_probability_topic', '/swarm/belief/peak_probability'
        )
        self.declare_parameter('centroid_topic', '/swarm/belief/centroid')
        self.declare_parameter('belief_map_topic', '/swarm/belief/map')
        self.declare_parameter('declare_topic', '/swarm/verify/declare')
        self.declare_parameter('verify_request_topic', '/swarm/verify/request')
        self.declare_parameter('verify_result_topic', '/swarm/verify/result')
        self.declare_parameter('halt_topic', '/swarm/mission/halt')

        self.declare_parameter('min_x', -120.0)
        self.declare_parameter('max_x', 120.0)
        self.declare_parameter('min_y', -120.0)
        self.declare_parameter('max_y', 120.0)
        self.declare_parameter('lawnmower_spacing', 40.0)
        self.declare_parameter('lawnmower_asv_index', 0)
        self.declare_parameter('lawnmower_num_asvs', 1)
        # 'voronoi' = geographic regions (default multi-ASV); 'lanes' = interleaved Y
        self.declare_parameter('lawnmower_partition', 'voronoi')
        # Flat [x1,y1,x2,y2,...] shoreline seeds. Non-empty default so ROS types
        # this as DOUBLE_ARRAY (an empty [] would be inferred as BYTE_ARRAY).
        self.declare_parameter(
            'region_seeds',
            [-450.0, -450.0, 450.0, -450.0, 450.0, 450.0],
        )
        self.declare_parameter('control_rate_hz', 2.0)

        self.declare_parameter('p_enter_target_search', 0.25)
        self.declare_parameter('consecutive_high_p_required', 4)
        self.declare_parameter('peak_near_asv_radius_m', 150.0)
        self.declare_parameter('min_seconds_before_switch', 20.0)
        self.declare_parameter('require_calibration_ready', True)
        self.declare_parameter('force_target_search', False)
        self.declare_parameter('force_spiral_complete', False)
        self.declare_parameter('force_verify', False)

        self.declare_parameter('info_gain_max_steps', 45)
        self.declare_parameter('info_gain_min_threshold', 1.0e-5)
        self.declare_parameter('info_gain_peak_convergence_m', 25.0)
        self.declare_parameter('info_gain_radii', [10.0, 20.0, 30.0])
        self.declare_parameter('info_gain_num_angles', 16)
        self.declare_parameter('info_gain_replan_period_s', 5.0)
        self.declare_parameter('info_gain_p_spiral', 0.5)
        self.declare_parameter('p_bg', 0.05)
        self.declare_parameter('p_max', 0.95)
        self.declare_parameter('d_half', 30.0)

        self.declare_parameter('spiral_ring_spacing_m', 15.0)
        self.declare_parameter('spiral_max_radius_m', 80.0)
        self.declare_parameter('spiral_step_spacing_m', 10.0)
        self.declare_parameter('spiral_min_duration_s', 45.0)
        self.declare_parameter('spiral_complete_peak_p', 0.25)
        self.declare_parameter('spiral_complete_radius_m', 40.0)

        self.declare_parameter('self_verify', True)
        self.declare_parameter('verify_orbit_radius_m', 20.0)
        self.declare_parameter('verify_orbit_points', 12)
        self.declare_parameter('verify_confirmations_required', 4)
        self.declare_parameter('verify_arrival_radius_m', 30.0)
        self.declare_parameter('verify_peak_tolerance_m', 50.0)
        self.declare_parameter('verify_confirmation_threshold_nt', 5.0e7)
        self.declare_parameter('verify_min_peak_probability', 0.30)

        asv_id = str(self.get_parameter('asv_id').value).strip()
        if not asv_id:
            ns = self.get_namespace().strip('/')
            asv_id = ns if ns else 'asv1'
        self.asv_id = asv_id

        pose_topic = self.get_parameter('pose_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        state_topic = self.get_parameter('state_topic').value
        mode_topic = self.get_parameter('mode_topic').value
        info_gain_topic = self.get_parameter('info_gain_topic').value
        anomaly_topic = self.get_parameter('anomaly_topic').value
        cal_topic = self.get_parameter('calibration_status_topic').value
        peak_topic = self.get_parameter('peak_topic').value
        peak_p_topic = self.get_parameter('peak_probability_topic').value
        centroid_topic = self.get_parameter('centroid_topic').value
        belief_map_topic = self.get_parameter('belief_map_topic').value
        declare_topic = self.get_parameter('declare_topic').value
        verify_request_topic = self.get_parameter('verify_request_topic').value
        verify_result_topic = self.get_parameter('verify_result_topic').value
        halt_topic = self.get_parameter('halt_topic').value

        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.lawnmower_spacing = float(self.get_parameter('lawnmower_spacing').value)
        self.lawnmower_asv_index = int(
            self.get_parameter('lawnmower_asv_index').value
        )
        self.lawnmower_num_asvs = int(
            self.get_parameter('lawnmower_num_asvs').value
        )
        self.lawnmower_partition = str(
            self.get_parameter('lawnmower_partition').value
        ).strip().lower()
        seeds_raw = self.get_parameter('region_seeds').value
        self.region_seeds = [float(v) for v in seeds_raw] if seeds_raw else []
        # Single-ASV missions don't need region seeds; drop the dual default.
        if self.lawnmower_num_asvs <= 1:
            self.region_seeds = []
        self.p_enter = float(self.get_parameter('p_enter_target_search').value)
        self.consecutive_required = int(
            self.get_parameter('consecutive_high_p_required').value
        )
        self.peak_near_radius = float(
            self.get_parameter('peak_near_asv_radius_m').value
        )
        self.min_seconds_before_switch = float(
            self.get_parameter('min_seconds_before_switch').value
        )
        self.require_calibration_ready = bool(
            self.get_parameter('require_calibration_ready').value
        )
        self.self_verify = bool(self.get_parameter('self_verify').value)

        self.info_gain_max_steps = int(self.get_parameter('info_gain_max_steps').value)
        self.info_gain_min_threshold = float(
            self.get_parameter('info_gain_min_threshold').value
        )
        self.info_gain_peak_convergence_m = float(
            self.get_parameter('info_gain_peak_convergence_m').value
        )
        self.info_gain_radii = [
            float(r) for r in self.get_parameter('info_gain_radii').value
        ]
        self.info_gain_num_angles = int(
            self.get_parameter('info_gain_num_angles').value
        )
        self.info_gain_replan_period_s = float(
            self.get_parameter('info_gain_replan_period_s').value
        )
        self.info_gain_p_spiral = float(
            self.get_parameter('info_gain_p_spiral').value
        )
        self.spiral_ring_spacing_m = float(
            self.get_parameter('spiral_ring_spacing_m').value
        )
        self.spiral_max_radius_m = float(
            self.get_parameter('spiral_max_radius_m').value
        )
        self.spiral_step_spacing_m = float(
            self.get_parameter('spiral_step_spacing_m').value
        )
        self.spiral_min_duration_s = float(
            self.get_parameter('spiral_min_duration_s').value
        )
        self.spiral_complete_peak_p = float(
            self.get_parameter('spiral_complete_peak_p').value
        )
        self.spiral_complete_radius_m = float(
            self.get_parameter('spiral_complete_radius_m').value
        )
        self.verify_orbit_radius_m = float(
            self.get_parameter('verify_orbit_radius_m').value
        )
        self.verify_orbit_points = int(
            self.get_parameter('verify_orbit_points').value
        )

        self.planner = InfoGainPlanner(
            p_bg=float(self.get_parameter('p_bg').value),
            p_max=float(self.get_parameter('p_max').value),
            d_half=float(self.get_parameter('d_half').value),
            radii=self.info_gain_radii,
            num_angles=self.info_gain_num_angles,
        )
        self.verifier = VerificationTracker(
            confirmations_required=int(
                self.get_parameter('verify_confirmations_required').value
            ),
            arrival_radius_m=float(
                self.get_parameter('verify_arrival_radius_m').value
            ),
            peak_tolerance_m=float(
                self.get_parameter('verify_peak_tolerance_m').value
            ),
            confirmation_threshold_nt=float(
                self.get_parameter('verify_confirmation_threshold_nt').value
            ),
            min_peak_probability=float(
                self.get_parameter('verify_min_peak_probability').value
            ),
        )

        self.mode = self.MODE_GLOBAL
        self.hunt_phase = ''
        self.pose = None
        self.peak_xy = None
        self.peak_p = 0.0
        self.centroid_xy = None
        self.latest_anomaly_nt = 0.0
        self.calibration_ready = not self.require_calibration_ready
        self.high_p_streak = 0
        self.info_gain_steps = 0
        self.last_info_gain = 0.0
        self.confirmations = 0
        self.start_time = None
        self._lawnmower_published = False
        self._last_plan_signature = None
        self._last_info_replan = None
        self._spiral_published = False
        self._spiral_start_time = None
        self._declared = False
        self._halted = False
        self._verify_orbit_published = False

        plan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.plan_pub = self.create_publisher(Path, plan_topic, plan_qos)
        self.state_pub = self.create_publisher(MissionState, state_topic, 10)
        self.mode_string_pub = self.create_publisher(String, mode_topic, 10)
        self.info_gain_pub = self.create_publisher(Float64, info_gain_topic, 10)
        self.declare_pub = self.create_publisher(VerifyRequest, declare_topic, 10)
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.result_pub = self.create_publisher(
            VerifyResult, verify_result_topic, result_qos
        )

        self.create_subscription(Pose2D, pose_topic, self.on_pose, 10)
        self.create_subscription(PoseStamped, peak_topic, self.on_peak, 10)
        self.create_subscription(Float64, peak_p_topic, self.on_peak_p, 10)
        self.create_subscription(
            PoseStamped, centroid_topic, self.on_centroid, 10
        )
        self.create_subscription(BeliefGrid, belief_map_topic, self.on_belief, 10)
        self.create_subscription(MagAnomaly, anomaly_topic, self.on_anomaly, 50)
        self.create_subscription(
            CalibrationStatus, cal_topic, self.on_calibration, 10
        )
        self.create_subscription(
            VerifyRequest, verify_request_topic, self.on_verify_request, 10
        )
        self.create_subscription(Bool, halt_topic, self.on_halt, 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'mission_manager ({self.asv_id}) ready: '
            'GLOBAL → TARGET (MI→SPIRAL) → VERIFY → COMPLETE'
        )

    def on_pose(self, msg):
        self.pose = msg

    def on_peak(self, msg):
        self.peak_xy = (msg.pose.position.x, msg.pose.position.y)

    def on_peak_p(self, msg):
        self.peak_p = float(msg.data)

    def on_centroid(self, msg):
        self.centroid_xy = (msg.pose.position.x, msg.pose.position.y)

    def _estimate_xy(self):
        """Prefer the weighted centroid; fall back to the discrete peak cell."""
        if self.centroid_xy is not None:
            return self.centroid_xy
        return self.peak_xy

    def on_belief(self, msg):
        self.planner.update_belief_grid(
            msg.data,
            msg.origin_x,
            msg.origin_y,
            msg.resolution,
            msg.width,
            msg.height,
        )

    def on_anomaly(self, msg):
        self.latest_anomaly_nt = float(msg.cleaned_anomaly_nt)
        if self.mode == self.MODE_VERIFY and self.verifier.active:
            pose_xy = None if self.pose is None else (self.pose.x, self.pose.y)
            if self.verifier.register(
                pose_xy, self.peak_xy, self.peak_p, self.latest_anomaly_nt
            ):
                self.confirmations = self.verifier.confirmations
                self._finish_verify(success=True)

    def on_calibration(self, msg):
        self.calibration_ready = msg.phase == 'READY'

    def on_halt(self, msg):
        if msg.data and self.mode != self.MODE_COMPLETE:
            self._enter_complete(reason='swarm_halt')

    def on_verify_request(self, msg):
        if self.mode in (self.MODE_COMPLETE, self.MODE_VERIFY):
            return
        if msg.verifier_id and msg.verifier_id != self.asv_id:
            return
        discoverer_xy = None
        if math.isfinite(msg.discoverer_x) and math.isfinite(msg.discoverer_y):
            # Zero/zero with empty discoverer means "unset" on older publishers.
            if abs(msg.discoverer_x) > 1e-6 or abs(msg.discoverer_y) > 1e-6:
                discoverer_xy = (msg.discoverer_x, msg.discoverer_y)
        self._start_verify(
            (msg.candidate_x, msg.candidate_y),
            reason=f'request_from_{msg.discoverer_id}',
            discoverer_xy=discoverer_xy,
        )

    def _elapsed_s(self):
        if self.start_time is None:
            return 0.0
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def _spiral_elapsed_s(self):
        if self._spiral_start_time is None:
            return 0.0
        return (self.get_clock().now() - self._spiral_start_time).nanoseconds * 1e-9

    def _publish_plan(self, points, force=False):
        if len(points) < 2:
            return
        signature = (
            len(points),
            round(points[0][0], 2),
            round(points[0][1], 2),
            round(points[-1][0], 2),
            round(points[-1][1], 2),
        )
        if not force and signature == self._last_plan_signature:
            return
        path = make_path(points, frame_id='map')
        path.header.stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(path)
        self._last_plan_signature = signature

    def _publish_hold_plan(self):
        if self.pose is None:
            return
        x, y = self.pose.x, self.pose.y
        self._publish_plan([(x, y), (x + 0.5, y)], force=True)

    def _publish_state(self):
        state = MissionState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.mode = self.mode
        state.hunt_phase = self.hunt_phase
        state.peak_p = self.peak_p
        if self.peak_xy is not None:
            state.peak_x = self.peak_xy[0]
            state.peak_y = self.peak_xy[1]
        state.info_gain_steps = self.info_gain_steps
        state.confirmations = self.confirmations
        self.state_pub.publish(state)
        self.mode_string_pub.publish(String(data=self.mode))
        self.info_gain_pub.publish(Float64(data=self.last_info_gain))

    def _enter_target_search(self, reason):
        self.mode = self.MODE_TARGET
        self.hunt_phase = self.PHASE_INFO
        self.info_gain_steps = 0
        self.last_info_gain = 0.0
        self._last_info_replan = None
        self._spiral_published = False
        self._spiral_start_time = None
        peak_str = 'n/a'
        if self.peak_xy is not None:
            peak_str = f'({self.peak_xy[0]:.1f},{self.peak_xy[1]:.1f})'
        belief_ready = 'yes' if self.planner.ready else 'no'
        self.get_logger().info(
            f'Switch → TARGET_SEARCH (INFO_GAIN) reason={reason} '
            f'peak_p={self.peak_p:.3f} peak={peak_str} belief={belief_ready}'
        )
        self._advance_info_gain(force=True)

    def _try_mode_switch(self):
        if self.mode != self.MODE_GLOBAL:
            return

        self.force_target_search = bool(
            self.get_parameter('force_target_search').value
        )
        if self.force_target_search:
            if self.peak_xy is None and self.pose is not None:
                self.peak_xy = (self.pose.x + 30.0, self.pose.y + 30.0)
                self.peak_p = max(self.peak_p, self.p_enter)
            if self.pose is not None:
                self._enter_target_search('force_target_search')
            return

        if self.require_calibration_ready and not self.calibration_ready:
            self.high_p_streak = 0
            return
        if self._elapsed_s() < self.min_seconds_before_switch:
            self.high_p_streak = 0
            return
        if self.pose is None or self._estimate_xy() is None:
            self.high_p_streak = 0
            return

        estimate = self._estimate_xy()
        dist = math.hypot(
            self.pose.x - estimate[0],
            self.pose.y - estimate[1],
        )
        if self.peak_p >= self.p_enter and dist <= self.peak_near_radius:
            self.high_p_streak += 1
        else:
            self.high_p_streak = 0

        if self.high_p_streak >= self.consecutive_required:
            self._enter_target_search('belief_peak')

    def _should_leave_info_gain(self, dist_peak):
        if self.info_gain_steps >= self.info_gain_max_steps:
            return True, 'max_steps'
        if (
            self.info_gain_steps >= 1
            and self.last_info_gain < self.info_gain_min_threshold
        ):
            return True, 'low_mi'
        if dist_peak <= self.info_gain_peak_convergence_m:
            return True, 'near_peak'
        if (
            self.peak_p >= self.info_gain_p_spiral
            and dist_peak <= self.info_gain_peak_convergence_m * 1.5
        ):
            return True, 'confident_near_peak'
        return False, ''

    def _advance_info_gain(self, force=False):
        if self.pose is None:
            return

        now = self.get_clock().now()
        if (
            not force
            and self._last_info_replan is not None
            and (now - self._last_info_replan).nanoseconds * 1e-9
            < self.info_gain_replan_period_s
        ):
            return

        bounds = ((self.min_x, self.min_y), (self.max_x, self.max_y))
        target, gain = self.planner.plan(
            (self.pose.x, self.pose.y),
            bounds=bounds,
            peak_xy=self._estimate_xy(),
        )
        self.last_info_gain = float(gain)
        self._publish_plan([(self.pose.x, self.pose.y), target], force=True)
        self._last_info_replan = now
        self.info_gain_steps += 1

        estimate = self._estimate_xy()
        if estimate is None:
            dist_peak = float('inf')
        else:
            dist_peak = math.hypot(
                self.pose.x - estimate[0],
                self.pose.y - estimate[1],
            )

        leave, reason = self._should_leave_info_gain(dist_peak)
        if leave:
            self.get_logger().info(
                f'INFO_GAIN done ({self.info_gain_steps} steps, '
                f'MI={self.last_info_gain:.5f}, reason={reason}) → SPIRAL'
            )
            self._start_spiral()

    def _start_spiral(self):
        estimate = self._estimate_xy()
        if estimate is None or self._spiral_published:
            return
        self.hunt_phase = self.PHASE_SPIRAL
        points = generate_expanding_spiral(
            estimate,
            step_spacing=self.spiral_step_spacing_m,
            max_radius=self.spiral_max_radius_m,
            ring_spacing=self.spiral_ring_spacing_m,
            margin_min=(self.min_x, self.min_y),
            margin_max=(self.max_x, self.max_y),
        )
        if self.pose is not None:
            points = [(self.pose.x, self.pose.y)] + points
        self._publish_plan(points, force=True)
        self._spiral_published = True
        self._spiral_start_time = self.get_clock().now()
        self.get_logger().info(
            f'SPIRAL ({len(points)} waypoints) '
            f'around ({estimate[0]:.1f},{estimate[1]:.1f})'
        )

    def _maybe_complete_spiral(self):
        estimate = self._estimate_xy()
        if self.hunt_phase != self.PHASE_SPIRAL or estimate is None:
            return

        force = bool(self.get_parameter('force_spiral_complete').value)
        elapsed = self._spiral_elapsed_s()
        dist = float('inf')
        if self.pose is not None:
            dist = math.hypot(
                self.pose.x - estimate[0],
                self.pose.y - estimate[1],
            )

        ready = force or (
            elapsed >= self.spiral_min_duration_s
            and self.peak_p >= self.spiral_complete_peak_p
            and dist <= self.spiral_complete_radius_m
        )
        # Also allow completion after long dwell even if slightly far.
        if not ready and elapsed >= self.spiral_min_duration_s * 2.0:
            if self.peak_p >= self.spiral_complete_peak_p:
                ready = True

        if not ready:
            return

        self.mode = self.MODE_HOLD
        self.hunt_phase = ''
        self._publish_hold_plan()
        self.get_logger().info(
            f'SPIRAL complete → HOLD (elapsed={elapsed:.1f}s, '
            f'peak_p={self.peak_p:.3f}, dist={dist:.1f}m)'
        )
        self._declare_candidate()

        if self.self_verify or bool(self.get_parameter('force_verify').value):
            estimate = self._estimate_xy()
            if estimate is not None:
                self._start_verify(estimate, reason='self_verify_after_spiral')

    def _declare_candidate(self):
        estimate = self._estimate_xy()
        if self._declared or estimate is None:
            return
        req = VerifyRequest()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = 'map'
        req.discoverer_id = self.asv_id
        req.verifier_id = ''
        req.candidate_x = float(estimate[0])
        req.candidate_y = float(estimate[1])
        req.candidate_peak_p = float(self.peak_p)
        if self.pose is not None:
            req.discoverer_x = float(self.pose.x)
            req.discoverer_y = float(self.pose.y)
        self.declare_pub.publish(req)
        self._declared = True
        self.get_logger().info(
            f'Declared candidate (weighted centroid) at '
            f'({req.candidate_x:.1f},{req.candidate_y:.1f}) '
            f'p={req.candidate_peak_p:.3f}'
        )

    def _start_verify(self, candidate_xy, reason='', discoverer_xy=None):
        if self.mode == self.MODE_COMPLETE:
            return
        self.mode = self.MODE_VERIFY
        self.hunt_phase = ''
        self.verifier.start(candidate_xy)
        self.confirmations = 0
        self._verify_orbit_published = False

        start_angle = 0.0
        if discoverer_xy is not None:
            # Enter the orbit from the side opposite the discoverer.
            start_angle = opposite_approach_angle(candidate_xy, discoverer_xy)
        points = generate_verification_orbit(
            candidate_xy,
            radius=self.verify_orbit_radius_m,
            num_points=self.verify_orbit_points,
            margin_min=(self.min_x, self.min_y),
            margin_max=(self.max_x, self.max_y),
            start_angle=start_angle,
        )
        if self.pose is not None:
            points = [(self.pose.x, self.pose.y)] + points
        self._publish_plan(points, force=True)
        self._verify_orbit_published = True
        side = 'opposite' if discoverer_xy is not None else 'default'
        self.get_logger().info(
            f'Switch → VERIFY reason={reason} candidate='
            f'({candidate_xy[0]:.1f},{candidate_xy[1]:.1f}) '
            f'orbit={len(points)} pts approach={side}'
        )

    def _finish_verify(self, success):
        result = VerifyResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = 'map'
        result.success = bool(success)
        result.verifier_id = self.asv_id
        if self.verifier.candidate_xy is not None:
            result.candidate_x = self.verifier.candidate_xy[0]
            result.candidate_y = self.verifier.candidate_xy[1]
        result.confirmations = int(self.verifier.confirmations)
        result.final_peak_p = float(self.peak_p)
        self.result_pub.publish(result)
        self.confirmations = self.verifier.confirmations
        if success:
            self._enter_complete(reason='verify_success')
        else:
            self.mode = self.MODE_HOLD
            self.get_logger().warn('VERIFY failed → HOLD')

    def _enter_complete(self, reason=''):
        self.mode = self.MODE_COMPLETE
        self.hunt_phase = ''
        self._halted = True
        self._publish_hold_plan()
        self.get_logger().info(f'MISSION COMPLETE reason={reason}')

    def on_timer(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        # Live force-verify shortcut for pipeline checks.
        if (
            bool(self.get_parameter('force_verify').value)
            and self.mode not in (self.MODE_VERIFY, self.MODE_COMPLETE)
            and self._estimate_xy() is not None
        ):
            estimate = self._estimate_xy()
            self._declare_candidate()
            self._start_verify(estimate, reason='force_verify')

        if self.mode == self.MODE_GLOBAL:
            if not self._lawnmower_published:
                if self.lawnmower_partition == 'lanes':
                    points = generate_split_lawnmower(
                        self.min_x,
                        self.max_x,
                        self.min_y,
                        self.max_y,
                        self.lawnmower_spacing,
                        asv_index=self.lawnmower_asv_index,
                        num_asvs=self.lawnmower_num_asvs,
                    )
                    share_desc = (
                        f'lane_share={self.lawnmower_asv_index}/'
                        f'{self.lawnmower_num_asvs}'
                    )
                else:
                    points = generate_region_lawnmower(
                        self.min_x,
                        self.max_x,
                        self.min_y,
                        self.max_y,
                        self.lawnmower_spacing,
                        asv_index=self.lawnmower_asv_index,
                        num_asvs=self.lawnmower_num_asvs,
                        seeds=self.region_seeds or None,
                    )
                    share_desc = (
                        f'region={self.lawnmower_asv_index}/'
                        f'{self.lawnmower_num_asvs} partition=voronoi'
                    )
                if self.pose is not None:
                    points = [(self.pose.x, self.pose.y)] + points
                self._publish_plan(points, force=True)
                self._lawnmower_published = True
                self.get_logger().info(
                    f'Published lawnmower path with {len(points)} vertices, '
                    f'spacing={self.lawnmower_spacing:.1f} m, {share_desc}'
                )
            self._try_mode_switch()
        elif self.mode == self.MODE_TARGET:
            if self.hunt_phase == self.PHASE_INFO:
                self._advance_info_gain()
            elif self.hunt_phase == self.PHASE_SPIRAL:
                self._maybe_complete_spiral()
        elif self.mode == self.MODE_HOLD:
            # Waiting for coordinator assignment unless self_verify already fired.
            pass
        elif self.mode == self.MODE_VERIFY:
            self.confirmations = self.verifier.confirmations
        elif self.mode == self.MODE_COMPLETE:
            pass

        self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
