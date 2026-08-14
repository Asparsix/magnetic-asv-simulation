#!/usr/bin/env python3
"""Swarm verification coordinator (Phase 8) — single-ASV ready."""

from __future__ import annotations

from boat_msgs.msg import VerifyRequest, VerifyResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class VerifyCoordinator(Node):
    """
    Accept candidate declarations, assign a verifier, and terminate on success.

    Single-ASV: verifier_id == discoverer_id (self-verify).
    Multi-ASV later: prefer a different idle ASV from known_asvs.
    """

    def __init__(self):
        super().__init__('verify_coordinator')

        self.declare_parameter('declare_topic', '/swarm/verify/declare')
        self.declare_parameter('request_topic', '/swarm/verify/request')
        self.declare_parameter('result_topic', '/swarm/verify/result')
        self.declare_parameter('complete_topic', '/swarm/mission/complete')
        self.declare_parameter('halt_topic', '/swarm/mission/halt')
        self.declare_parameter('known_asvs', ['asv1'])
        self.declare_parameter('prefer_other_verifier', True)

        declare_topic = self.get_parameter('declare_topic').value
        request_topic = self.get_parameter('request_topic').value
        result_topic = self.get_parameter('result_topic').value
        complete_topic = self.get_parameter('complete_topic').value
        halt_topic = self.get_parameter('halt_topic').value
        self.known_asvs = [
            str(a) for a in self.get_parameter('known_asvs').value
        ]
        self.prefer_other = bool(
            self.get_parameter('prefer_other_verifier').value
        )

        self.candidate = None
        self.mission_complete = False
        self.verification_success = False
        self.active_request = None

        self.request_pub = self.create_publisher(VerifyRequest, request_topic, 10)
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.complete_pub = self.create_publisher(Bool, complete_topic, latch_qos)
        self.halt_pub = self.create_publisher(Bool, halt_topic, latch_qos)
        self.status_pub = self.create_publisher(String, '/swarm/mission/status', latch_qos)

        self.create_subscription(VerifyRequest, declare_topic, self.on_declare, 10)
        self.create_subscription(VerifyResult, result_topic, self.on_result, 10)

        self.get_logger().info(
            f'verify_coordinator ready; known_asvs={self.known_asvs}'
        )

    def _pick_verifier(self, discoverer_id):
        if self.prefer_other:
            for asv_id in self.known_asvs:
                if asv_id and asv_id != discoverer_id:
                    return asv_id
        if discoverer_id:
            return discoverer_id
        return self.known_asvs[0] if self.known_asvs else 'asv1'

    def on_declare(self, msg):
        if self.mission_complete:
            return
        if self.candidate is not None and self.active_request is not None:
            self.get_logger().info('Ignoring declare; verification already active')
            return

        discoverer = msg.discoverer_id or 'asv1'
        verifier = self._pick_verifier(discoverer)
        self.candidate = (msg.candidate_x, msg.candidate_y, msg.candidate_peak_p)

        request = VerifyRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        request.discoverer_id = discoverer
        request.verifier_id = verifier
        request.candidate_x = float(msg.candidate_x)
        request.candidate_y = float(msg.candidate_y)
        request.candidate_peak_p = float(msg.candidate_peak_p)
        request.discoverer_x = float(msg.discoverer_x)
        request.discoverer_y = float(msg.discoverer_y)
        self.active_request = request
        self.request_pub.publish(request)
        self.status_pub.publish(
            String(data=f'VERIFY_ASSIGNED:{verifier}@{request.candidate_x:.1f},{request.candidate_y:.1f}')
        )
        self.get_logger().info(
            f'Candidate from {discoverer} → verifier {verifier} at '
            f'({request.candidate_x:.1f},{request.candidate_y:.1f}) '
            f'p={request.candidate_peak_p:.3f}'
        )

    def on_result(self, msg):
        if self.mission_complete:
            return
        if not msg.success:
            self.get_logger().warn(
                f'Verify failed from {msg.verifier_id}; confirmations={msg.confirmations}'
            )
            self.active_request = None
            return

        self.mission_complete = True
        self.verification_success = True
        self.complete_pub.publish(Bool(data=True))
        self.halt_pub.publish(Bool(data=True))
        self.status_pub.publish(
            String(
                data=(
                    f'MISSION_COMPLETE:verifier={msg.verifier_id}:'
                    f'conf={msg.confirmations}:p={msg.final_peak_p:.3f}'
                )
            )
        )
        self.get_logger().info(
            f'MISSION COMPLETE — {msg.verifier_id} confirmed '
            f'({msg.confirmations} readings, peak_p={msg.final_peak_p:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VerifyCoordinator()
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
