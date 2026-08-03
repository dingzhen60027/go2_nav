#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Go2CmdAdapter(Node):

    def __init__(self):
        super().__init__("go2_cmd_adapter")

        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_go2")
        self.declare_parameter("linear_deadband", 0.08)
        self.declare_parameter("min_translation_speed", 0.22)
        self.declare_parameter("max_boost_ratio", 2.2)
        self.declare_parameter("rotate_wz_threshold", 0.25)
        self.declare_parameter("rotate_linear_threshold", 0.16)
        self.declare_parameter("max_linear_accel", 0.8)
        self.declare_parameter("max_angular_accel", 1.5)

        in_topic = self.get_parameter("cmd_vel_in_topic").value
        out_topic = self.get_parameter("cmd_vel_out_topic").value
        self.linear_deadband = self.get_parameter("linear_deadband").value
        self.min_translation_speed = self.get_parameter(
            "min_translation_speed").value
        self.max_boost_ratio = self.get_parameter("max_boost_ratio").value
        self.rotate_wz_threshold = self.get_parameter(
            "rotate_wz_threshold").value
        self.rotate_linear_threshold = self.get_parameter(
            "rotate_linear_threshold").value
        self.max_linear_accel = self.get_parameter("max_linear_accel").value
        self.max_angular_accel = self.get_parameter("max_angular_accel").value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        self.last_cmd = Twist()
        self.last_time = self.get_clock().now()

        self.get_logger().info(f"Go2 cmd adapter: {in_topic} -> {out_topic}")

    def _shape_planar_velocity(self, vx: float, vy: float, wz: float):
        speed = math.hypot(vx, vy)

        if abs(wz) >= self.rotate_wz_threshold and speed < self.rotate_linear_threshold:
            return 0.0, 0.0

        if speed < self.linear_deadband:
            return 0.0, 0.0

        if speed < self.min_translation_speed:
            target_speed = min(self.min_translation_speed,
                               speed * self.max_boost_ratio)
            scale = target_speed / speed
            return vx * scale, vy * scale

        return vx, vy

    @staticmethod
    def _limit_delta(current: float, target: float, max_delta: float):
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def _limit_rate(self, cmd: Twist):
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds * 1e-9, 1e-3)
        self.last_time = now

        linear_delta = self.max_linear_accel * dt
        angular_delta = self.max_angular_accel * dt

        limited = Twist()
        limited.linear.x = self._limit_delta(
            self.last_cmd.linear.x, cmd.linear.x, linear_delta)
        limited.linear.y = self._limit_delta(
            self.last_cmd.linear.y, cmd.linear.y, linear_delta)
        limited.angular.z = self._limit_delta(
            self.last_cmd.angular.z, cmd.angular.z, angular_delta)

        self.last_cmd = limited
        return limited

    def _on_cmd(self, msg: Twist):
        vx, vy = self._shape_planar_velocity(
            msg.linear.x, msg.linear.y, msg.angular.z)

        shaped = Twist()
        shaped.linear.x = vx
        shaped.linear.y = vy
        shaped.angular.z = msg.angular.z

        self.pub.publish(self._limit_rate(shaped))


def main():
    rclpy.init()
    node = Go2CmdAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
