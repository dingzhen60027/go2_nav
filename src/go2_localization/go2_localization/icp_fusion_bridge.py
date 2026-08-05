#!/usr/bin/env python3

from collections import deque
from math import isfinite

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .math_utils import (
    FusionGateLimits,
    FusionInnovationGate,
    Pose3,
    base_pose_to_tracking,
    compose,
    inverse,
    normalize_quaternion,
    pose_innovation,
    tracking_pose_to_base,
)


class IcpFusionBridge(Node):
    def __init__(self):
        super().__init__("icp_fusion_bridge")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tracking_translation", [0.16143, 0.0, 0.12262])
        self.declare_parameter("tracking_rotation", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("max_translation_xy", 0.60)
        self.declare_parameter("max_translation_z", 0.35)
        self.declare_parameter("max_yaw", 0.55)
        self.declare_parameter("max_rotation", 0.70)
        self.declare_parameter("alignment_max_translation_xy", 3.0)
        self.declare_parameter("alignment_max_translation_z", 1.0)
        self.declare_parameter("alignment_max_yaw", 1.75)
        self.declare_parameter("alignment_max_rotation", 1.80)
        self.declare_parameter("ekf_reset_holdoff_sec", 0.50)
        self.declare_parameter("max_prediction_age_sec", 0.50)
        self.declare_parameter("max_measurement_age_sec", 0.75)
        self.declare_parameter("icp_stale_timeout_sec", 1.0)
        self.declare_parameter("local_odom_frame", "odom")
        self.declare_parameter("local_odom_history_sec", 3.0)
        self.declare_parameter("max_local_odom_age_sec", 0.50)
        self.declare_parameter("max_local_odom_sync_error_sec", 0.15)
        self.declare_parameter("seed_rate_hz", 15.0)
        self.declare_parameter("position_stddev_xy", 0.15)
        self.declare_parameter("position_stddev_z", 0.25)
        self.declare_parameter("orientation_stddev_roll_pitch", 0.12)
        self.declare_parameter("orientation_stddev_yaw", 0.15)

        self.map_frame = str(self.get_parameter("map_frame").value)
        translation = tuple(
            float(value)
            for value in self.get_parameter("tracking_translation").value
        )
        rotation = normalize_quaternion(
            self.get_parameter("tracking_rotation").value
        )
        self.base_to_tracking = Pose3(translation, rotation)
        self.max_translation_xy = float(
            self.get_parameter("max_translation_xy").value
        )
        self.max_translation_z = float(
            self.get_parameter("max_translation_z").value
        )
        self.max_yaw = float(self.get_parameter("max_yaw").value)
        self.max_rotation = float(self.get_parameter("max_rotation").value)
        self.alignment_max_translation_xy = float(
            self.get_parameter("alignment_max_translation_xy").value
        )
        self.alignment_max_translation_z = float(
            self.get_parameter("alignment_max_translation_z").value
        )
        self.alignment_max_yaw = float(
            self.get_parameter("alignment_max_yaw").value
        )
        self.alignment_max_rotation = float(
            self.get_parameter("alignment_max_rotation").value
        )
        self.ekf_reset_holdoff = max(
            0.0, float(self.get_parameter("ekf_reset_holdoff_sec").value)
        )
        self.fusion_gate = FusionInnovationGate(
            FusionGateLimits(
                self.max_translation_xy,
                self.max_translation_z,
                self.max_yaw,
                self.max_rotation,
            ),
            FusionGateLimits(
                self.alignment_max_translation_xy,
                self.alignment_max_translation_z,
                self.alignment_max_yaw,
                self.alignment_max_rotation,
            ),
        )
        self.max_prediction_age = float(
            self.get_parameter("max_prediction_age_sec").value
        )
        self.max_measurement_age = float(
            self.get_parameter("max_measurement_age_sec").value
        )
        self.icp_stale_timeout = float(
            self.get_parameter("icp_stale_timeout_sec").value
        )
        self.local_odom_frame = str(
            self.get_parameter("local_odom_frame").value
        )
        self.local_odom_history = max(
            1.0, float(self.get_parameter("local_odom_history_sec").value)
        )
        self.max_local_odom_age = max(
            0.05, float(self.get_parameter("max_local_odom_age_sec").value)
        )
        self.max_local_odom_sync_error = max(
            0.01,
            float(
                self.get_parameter("max_local_odom_sync_error_sec").value
            ),
        )
        self.position_stddev_xy = float(
            self.get_parameter("position_stddev_xy").value
        )
        self.position_stddev_z = float(
            self.get_parameter("position_stddev_z").value
        )
        self.orientation_stddev_rp = float(
            self.get_parameter("orientation_stddev_roll_pitch").value
        )
        self.orientation_stddev_yaw = float(
            self.get_parameter("orientation_stddev_yaw").value
        )

        self.icp_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/icp_pose", 10
        )
        self.icp_seed_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/fused_icp/initialpose",
            10,
        )
        self.icp_prediction_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/fused_icp/prediction",
            10,
        )
        self.global_set_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/global/set_pose", 10
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.raw_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/fused_icp/pose_raw",
            self.on_raw_icp,
            20,
        )
        self.prediction_subscription = self.create_subscription(
            Odometry,
            "/localization/odometry/global",
            self.on_prediction,
            20,
        )
        self.local_odometry_subscription = self.create_subscription(
            Odometry,
            "/localization/odometry/local",
            self.on_local_odometry,
            50,
        )
        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.on_initial_pose,
            10,
        )

        self.initialized = False
        self.reference_pose = None
        self.prediction_pose = None
        self.prediction_time = None
        self.local_pose = None
        self.local_pose_time = None
        self.local_pose_stamp_ns = 0
        self.local_pose_buffer = deque()
        self.reference_local_pose = None
        self.last_raw_stamp_ns = 0
        self.initialized_time = None
        self.initialized_stamp_ns = 0
        self.last_icp_time = None
        self.alignment_time = None
        self.accepted = 0
        self.rejected = 0
        self.consecutive_rejections = 0
        seed_rate = max(1.0, float(self.get_parameter("seed_rate_hz").value))
        self.seed_timer = self.create_timer(
            1.0 / seed_rate, self.publish_prediction
        )
        self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostic)
        self.get_logger().info(
            "ICP fusion bridge ready; waiting for /initialpose before accepting ICP"
        )

    @staticmethod
    def pose_from_message(message_pose) -> Pose3:
        position = message_pose.position
        orientation = message_pose.orientation
        values = (position.x, position.y, position.z)
        if not all(isfinite(value) for value in values):
            raise ValueError("pose position is not finite")
        quaternion = normalize_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        return Pose3(position=values, orientation=quaternion)

    @staticmethod
    def fill_pose(message_pose, pose: Pose3):
        message_pose.position.x = pose.position[0]
        message_pose.position.y = pose.position[1]
        message_pose.position.z = pose.position[2]
        message_pose.orientation.x = pose.orientation[0]
        message_pose.orientation.y = pose.orientation[1]
        message_pose.orientation.z = pose.orientation[2]
        message_pose.orientation.w = pose.orientation[3]

    def on_initial_pose(self, message: PoseWithCovarianceStamped):
        if message.header.frame_id not in ("", self.map_frame):
            self.get_logger().error(
                f"Initial pose frame must be {self.map_frame}, got "
                f"{message.header.frame_id}"
            )
            return
        try:
            pose = self.pose_from_message(message.pose.pose)
        except ValueError as error:
            self.get_logger().error(f"Invalid initial pose: {error}")
            return

        # Treat the operator pose as an ICP acquisition seed only.  Committing
        # it to the global EKF here would move map->odom before scan matching
        # has verified the pose.  The EKF reset is deliberately deferred to
        # the first accepted, fully confirmed ICP alignment in on_raw_icp().
        self.reference_pose = pose
        self.prediction_pose = pose
        now = self.get_clock().now()
        self.prediction_time = now
        self.initialized_time = now
        self.initialized_stamp_ns = now.nanoseconds
        self.last_icp_time = None
        self.alignment_time = None
        self.last_raw_stamp_ns = 0
        self.initialized = True
        self.fusion_gate.reset()
        self.reference_local_pose = None
        self.consecutive_rejections = 0
        self.publish_initial_pose(pose)
        self.get_logger().info(
            "Initial pose queued as fused ICP acquisition seed; "
            "global EKF reset deferred until alignment is verified"
        )

    def on_prediction(self, message: Odometry):
        # Before ICP acquisition is verified, the global EKF may still contain
        # its startup state or the previous localization session.  Do not let
        # that unverified state replace the operator seed.
        if (
            not self.initialized
            or not self.fusion_gate.alignment_locked
            or message.header.frame_id != self.map_frame
        ):
            return
        now = self.get_clock().now()
        if self.alignment_time is not None:
            alignment_age = (
                now - self.alignment_time
            ).nanoseconds * 1.0e-9
            if 0.0 <= alignment_age <= self.ekf_reset_holdoff:
                return
        try:
            self.prediction_pose = self.pose_from_message(message.pose.pose)
        except ValueError:
            return
        self.prediction_time = now

    @staticmethod
    def message_stamp_ns(message) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    def on_local_odometry(self, message: Odometry):
        if message.header.frame_id != self.local_odom_frame:
            return
        try:
            pose = self.pose_from_message(message.pose.pose)
        except ValueError:
            return
        stamp_ns = self.message_stamp_ns(message)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        if self.local_pose_stamp_ns and stamp_ns <= self.local_pose_stamp_ns:
            if stamp_ns == self.local_pose_stamp_ns:
                return
            self.local_pose_buffer.clear()
        self.local_pose_stamp_ns = stamp_ns
        self.local_pose = pose
        self.local_pose_time = self.get_clock().now()
        self.local_pose_buffer.append((stamp_ns, pose))
        keep_after_ns = stamp_ns - int(self.local_odom_history * 1.0e9)
        while (
            len(self.local_pose_buffer) > 2
            and self.local_pose_buffer[1][0] < keep_after_ns
        ):
            self.local_pose_buffer.popleft()

    def local_pose_at(self, stamp_ns):
        if stamp_ns is None:
            if self.local_pose is None or self.local_pose_time is None:
                return None
            age = (
                self.get_clock().now() - self.local_pose_time
            ).nanoseconds * 1.0e-9
            if age < 0.0 or age > self.max_local_odom_age:
                return None
            return self.local_pose
        if not self.local_pose_buffer:
            return None
        nearest_stamp, nearest_pose = min(
            self.local_pose_buffer,
            key=lambda sample: abs(sample[0] - stamp_ns),
        )
        sync_error = abs(nearest_stamp - stamp_ns) * 1.0e-9
        if sync_error > self.max_local_odom_sync_error:
            return None
        return nearest_pose

    def active_prediction(self, target_stamp_ns=None):
        if not self.fusion_gate.alignment_locked:
            return self.reference_pose
        if self.alignment_time is not None:
            alignment_age = (
                self.get_clock().now() - self.alignment_time
            ).nanoseconds * 1.0e-9
            if 0.0 <= alignment_age <= self.ekf_reset_holdoff:
                return self.reference_pose
        # Propagate the last map-anchored ICP pose by the relative local-odom
        # motion since that correction. This follows arbitrary real turns even
        # if several ICP frames are missed, without feeding the global EKF's
        # own drift back into the matcher.
        target_local_pose = self.local_pose_at(target_stamp_ns)
        if (
            self.reference_pose is not None
            and self.reference_local_pose is not None
            and target_local_pose is not None
        ):
            local_delta = compose(
                inverse(self.reference_local_pose), target_local_pose
            )
            return compose(self.reference_pose, local_delta)
        # A global EKF without fresh absolute ICP corrections is dead
        # reckoning. Feeding that increasingly uncertain pose back into ICP
        # creates a positive feedback loop: the ICP initial guess drifts, the
        # strict gate rejects the correction, and the guess drifts even more.
        # Keep the matcher locked in tracking mode, but freeze its seed at the
        # last accepted ICP pose once the correction stream is stale.
        if self.last_icp_time is not None:
            correction_age = (
                self.get_clock().now() - self.last_icp_time
            ).nanoseconds * 1.0e-9
            if correction_age > self.icp_stale_timeout:
                return self.reference_pose
        if self.prediction_pose is not None and self.prediction_time is not None:
            age = (
                self.get_clock().now() - self.prediction_time
            ).nanoseconds * 1.0e-9
            if age <= self.max_prediction_age:
                return self.prediction_pose
        return self.reference_pose

    def tracking_pose_message(self, base_pose):
        tracking_pose = base_pose_to_tracking(
            base_pose, self.base_to_tracking
        )
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.fill_pose(message.pose.pose, tracking_pose)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[14] = 0.25
        message.pose.covariance[21] = 0.25
        message.pose.covariance[28] = 0.25
        message.pose.covariance[35] = 0.25
        return message

    def publish_initial_pose(self, base_pose):
        self.icp_seed_pub.publish(self.tracking_pose_message(base_pose))

    def publish_prediction(self):
        if not self.initialized:
            return
        prediction = self.active_prediction()
        if prediction is None:
            return
        self.icp_prediction_pub.publish(
            self.tracking_pose_message(prediction)
        )

    def on_raw_icp(self, message: PoseWithCovarianceStamped):
        if not self.initialized:
            return
        stamp_ns = self.message_stamp_ns(message)
        if stamp_ns < self.initialized_stamp_ns:
            self.rejected += 1
            return
        if stamp_ns <= self.last_raw_stamp_ns:
            self.rejected += 1
            return
        self.last_raw_stamp_ns = stamp_ns
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1.0e-9
        if age > self.max_measurement_age or age < -0.1:
            self.rejected += 1
            self.consecutive_rejections += 1
            return

        try:
            tracking_pose = self.pose_from_message(message.pose.pose)
        except ValueError:
            self.rejected += 1
            self.consecutive_rejections += 1
            return
        base_pose = tracking_pose_to_base(tracking_pose, self.base_to_tracking)
        prediction = self.active_prediction(stamp_ns)
        if prediction is None:
            return
        innovation = pose_innovation(base_pose, prediction)
        gate_decision = self.fusion_gate.evaluate(innovation)
        if not gate_decision.accepted:
            self.rejected += 1
            self.consecutive_rejections += 1
            self.get_logger().warning(
                "Reject ICP innovation: "
                f"xy={innovation.translation_xy:.3f}m "
                f"z={innovation.translation_z:.3f}m "
                f"yaw={innovation.yaw:.3f}rad "
                f"rotation={innovation.rotation:.3f}rad",
                throttle_duration_sec=1.0,
            )
            return

        output = PoseWithCovarianceStamped()
        output.header = message.header
        output.header.frame_id = self.map_frame
        self.fill_pose(output.pose.pose, base_pose)
        output.pose.covariance[0] = self.position_stddev_xy**2
        output.pose.covariance[7] = self.position_stddev_xy**2
        output.pose.covariance[14] = self.position_stddev_z**2
        output.pose.covariance[21] = self.orientation_stddev_rp**2
        output.pose.covariance[28] = self.orientation_stddev_rp**2
        output.pose.covariance[35] = self.orientation_stddev_yaw**2
        if gate_decision.just_locked:
            reset = PoseWithCovarianceStamped()
            reset.header.stamp = self.get_clock().now().to_msg()
            reset.header.frame_id = self.map_frame
            self.fill_pose(reset.pose.pose, base_pose)
            reset.pose.covariance = list(output.pose.covariance)
            self.global_set_pose_pub.publish(reset)
            now = self.get_clock().now()
            self.reference_pose = base_pose
            self.prediction_pose = base_pose
            self.prediction_time = now
            self.alignment_time = now
            self.get_logger().info(
                "Fused ICP aligned; global EKF reset and strict fusion gate enabled"
            )
        self.icp_pose_pub.publish(output)
        self.reference_pose = base_pose
        local_pose_at_measurement = self.local_pose_at(stamp_ns)
        if local_pose_at_measurement is not None:
            self.reference_local_pose = local_pose_at_measurement
        self.last_icp_time = self.get_clock().now()
        self.accepted += 1
        self.consecutive_rejections = 0

    def publish_diagnostic(self):
        now = self.get_clock().now()
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = now.to_msg()
        status = DiagnosticStatus()
        status.name = "go2_localization/icp_fusion"
        status.hardware_id = "mid360"
        correction_age = None
        if self.last_icp_time is not None:
            correction_age = (now - self.last_icp_time).nanoseconds * 1.0e-9
        elif self.initialized_time is not None:
            correction_age = (now - self.initialized_time).nanoseconds * 1.0e-9

        if not self.initialized:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for initial pose"
        elif (
            not self.fusion_gate.alignment_locked
            and self.consecutive_rejections >= 5
        ):
            status.level = DiagnosticStatus.ERROR
            status.message = "initial fused ICP alignment rejected"
        elif not self.fusion_gate.alignment_locked:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for stable fused ICP acquisition"
        elif self.consecutive_rejections >= 5:
            status.level = DiagnosticStatus.ERROR
            status.message = "ICP degraded; reinitialization may be required"
        elif (
            correction_age is not None
            and correction_age > self.icp_stale_timeout
        ):
            status.level = DiagnosticStatus.ERROR
            status.message = "ICP correction stream is stale"
        elif self.consecutive_rejections:
            status.level = DiagnosticStatus.WARN
            status.message = "recent ICP measurement rejected"
        elif self.last_icp_time is None:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for first ICP correction"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ICP corrections accepted"
        status.values = [
            KeyValue(key="accepted", value=str(self.accepted)),
            KeyValue(key="rejected", value=str(self.rejected)),
            KeyValue(
                key="consecutive_rejections",
                value=str(self.consecutive_rejections),
            ),
            KeyValue(
                key="alignment_locked",
                value=str(self.fusion_gate.alignment_locked).lower(),
            ),
            KeyValue(
                key="last_correction_age_sec",
                value=(
                    "never"
                    if self.last_icp_time is None
                    else f"{correction_age:.3f}"
                ),
            ),
        ]
        diagnostic.status.append(status)
        self.diagnostic_pub.publish(diagnostic)


def main(args=None):
    rclpy.init(args=args)
    node = IcpFusionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
