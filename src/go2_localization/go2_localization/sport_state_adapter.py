#!/usr/bin/env python3

from math import isfinite

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState

from .math_utils import normalize_quaternion, quaternion_conjugate, rotate_vector
from .motion_utils import StationaryGyroCorrector


class SportStateAdapter(Node):
    def __init__(self):
        super().__init__("sport_state_adapter")
        self.declare_parameter("input_topic", "/sportmodestate")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu")
        self.declare_parameter("velocity_in_body_frame", True)
        self.declare_parameter("max_timestamp_skew_sec", 0.5)
        self.declare_parameter("reject_nonzero_error_code", True)
        self.declare_parameter("velocity_stddev_x", 0.08)
        self.declare_parameter("velocity_stddev_y", 0.12)
        self.declare_parameter("orientation_stddev_roll_pitch", 0.035)
        self.declare_parameter("orientation_stddev_yaw", 0.5)
        self.declare_parameter("angular_velocity_stddev", 0.025)
        self.declare_parameter("gyro_bias_initialization_samples", 100)
        self.declare_parameter("gyro_bias_initialization_max_gyro", 0.03)
        self.declare_parameter("stationary_linear_speed", 0.02)
        self.declare_parameter("stationary_gyro_deadband", 0.015)
        self.declare_parameter("stale_timeout_sec", 0.5)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.velocity_in_body_frame = bool(
            self.get_parameter("velocity_in_body_frame").value
        )
        self.max_timestamp_skew = float(
            self.get_parameter("max_timestamp_skew_sec").value
        )
        self.reject_errors = bool(
            self.get_parameter("reject_nonzero_error_code").value
        )
        self.velocity_stddev_x = float(
            self.get_parameter("velocity_stddev_x").value
        )
        self.velocity_stddev_y = float(
            self.get_parameter("velocity_stddev_y").value
        )
        self.orientation_stddev_rp = float(
            self.get_parameter("orientation_stddev_roll_pitch").value
        )
        self.orientation_stddev_yaw = float(
            self.get_parameter("orientation_stddev_yaw").value
        )
        self.angular_velocity_stddev = float(
            self.get_parameter("angular_velocity_stddev").value
        )
        self.gyro_corrector = StationaryGyroCorrector(
            required_samples=int(
                self.get_parameter("gyro_bias_initialization_samples").value
            ),
            initialization_max_gyro=float(
                self.get_parameter("gyro_bias_initialization_max_gyro").value
            ),
            stationary_linear_speed=float(
                self.get_parameter("stationary_linear_speed").value
            ),
            stationary_gyro_deadband=float(
                self.get_parameter("stationary_gyro_deadband").value
            ),
        )
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)

        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/localization/body_twist", 20
        )
        self.imu_pub = self.create_publisher(Imu, "/localization/body_imu", 50)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        input_topic = str(self.get_parameter("input_topic").value)
        self.subscription = self.create_subscription(
            SportModeState,
            input_topic,
            self.on_state,
            qos_profile_sensor_data,
        )
        self.last_output_ns = 0
        self.last_input_time = None
        self.received = 0
        self.rejected = 0
        self.used_arrival_stamp = 0
        self.timer = self.create_timer(1.0, self.publish_diagnostic)
        self.get_logger().info(
            f"Go2 state adapter: {input_topic} -> body_twist + body_imu"
        )

    def select_stamp(self, message: SportModeState):
        now = self.get_clock().now()
        candidate_ns = int(message.stamp.sec) * 1_000_000_000 + int(
            message.stamp.nanosec
        )
        skew = abs(candidate_ns - now.nanoseconds) * 1.0e-9
        if (
            candidate_ns <= self.last_output_ns
            or candidate_ns <= 0
            or skew > self.max_timestamp_skew
        ):
            candidate_ns = max(now.nanoseconds, self.last_output_ns + 1)
            self.used_arrival_stamp += 1
        self.last_output_ns = candidate_ns
        return rclpy.time.Time(nanoseconds=candidate_ns).to_msg()

    def on_state(self, message: SportModeState):
        self.received += 1
        self.last_input_time = self.get_clock().now()
        if self.reject_errors and message.error_code != 0:
            self.rejected += 1
            self.get_logger().warning(
                f"Rejecting SportModeState error_code={message.error_code}",
                throttle_duration_sec=2.0,
            )
            return

        velocity = tuple(float(value) for value in message.velocity)
        gyro = tuple(float(value) for value in message.imu_state.gyroscope)
        if not all(isfinite(value) for value in velocity + gyro):
            self.rejected += 1
            return

        try:
            # Unitree arrays use [w, x, y, z]; ROS messages use [x, y, z, w].
            unitree_q = message.imu_state.quaternion
            orientation = normalize_quaternion(
                (unitree_q[1], unitree_q[2], unitree_q[3], unitree_q[0])
            )
        except ValueError:
            self.rejected += 1
            return

        if not self.velocity_in_body_frame:
            velocity = rotate_vector(quaternion_conjugate(orientation), velocity)

        velocity, gyro, _ = self.gyro_corrector.correct(velocity, gyro)

        stamp = self.select_stamp(message)
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        twist.twist.twist.linear.x = velocity[0]
        twist.twist.twist.linear.y = velocity[1]
        twist.twist.twist.linear.z = velocity[2]
        twist.twist.covariance[0] = self.velocity_stddev_x**2
        twist.twist.covariance[7] = self.velocity_stddev_y**2
        twist.twist.covariance[14] = 1.0
        twist.twist.covariance[21] = 1.0
        twist.twist.covariance[28] = 1.0
        twist.twist.covariance[35] = 1.0
        self.twist_pub.publish(twist)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation.x = orientation[0]
        imu.orientation.y = orientation[1]
        imu.orientation.z = orientation[2]
        imu.orientation.w = orientation[3]
        imu.angular_velocity.x = gyro[0]
        imu.angular_velocity.y = gyro[1]
        imu.angular_velocity.z = gyro[2]
        rp_variance = self.orientation_stddev_rp**2
        imu.orientation_covariance[0] = rp_variance
        imu.orientation_covariance[4] = rp_variance
        imu.orientation_covariance[8] = self.orientation_stddev_yaw**2
        angular_variance = self.angular_velocity_stddev**2
        imu.angular_velocity_covariance[0] = angular_variance
        imu.angular_velocity_covariance[4] = angular_variance
        imu.angular_velocity_covariance[8] = angular_variance
        imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu)

    def publish_diagnostic(self):
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "go2_localization/sport_state"
        status.hardware_id = "go2"
        age = None
        if self.last_input_time is not None:
            age = (self.get_clock().now() - self.last_input_time).nanoseconds * 1.0e-9
        if age is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "waiting for SportModeState"
        elif age > self.stale_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = f"SportModeState stale ({age:.2f}s)"
        elif self.rejected:
            status.level = DiagnosticStatus.WARN
            status.message = "running with rejected samples"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "receiving"
        status.values = [
            KeyValue(key="received", value=str(self.received)),
            KeyValue(key="rejected", value=str(self.rejected)),
            KeyValue(
                key="arrival_timestamp_fallbacks",
                value=str(self.used_arrival_stamp),
            ),
            KeyValue(
                key="gyro_bias_ready",
                value=str(self.gyro_corrector.ready).lower(),
            ),
            KeyValue(
                key="gyro_bias_z",
                value=f"{self.gyro_corrector.bias[2]:.6f}",
            ),
            KeyValue(
                key="stationary_gyro_clamps",
                value=str(self.gyro_corrector.stationary_clamps),
            ),
        ]
        diagnostic.status.append(status)
        self.diagnostic_pub.publish(diagnostic)


def main(args=None):
    rclpy.init(args=args)
    node = SportStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
