#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request

# Go2 sport API IDs
API_ID = {
    "DAMP": 1001,
    "BALANCE_STAND": 1002,
    "STOP_MOVE": 1003,
    "STAND_UP": 1004,
    "STAND_DOWN": 1005,
    "RECOVERY_STAND": 1006,
    "EULER": 1007,
    "MOVE": 1008,
    "SIT": 1009,
    "RISE_SIT": 1010,
    "SPEED_LEVEL": 1015,
    "HELLO": 1016,
    "SWITCH_JOYSTICK": 1027,
}


class Go2Bridge(Node):

    def __init__(self):
        super().__init__("go2_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_go2")
        self.declare_parameter("sport_request_topic", "/api/sport/request")
        self.declare_parameter("lin_vel_max", 1.5)
        self.declare_parameter("ang_vel_max", 1.5)

        sport_topic = self.get_parameter("sport_request_topic").value
        self.request_pub = self.create_publisher(Request, sport_topic, 10)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self._on_cmd_vel, 10)

        self.lin_max = self.get_parameter("lin_vel_max").value
        self.ang_max = self.get_parameter("ang_vel_max").value

        self.get_logger().info(
            f"Bridge ready: {cmd_vel_topic} -> {sport_topic}")

    def _on_cmd_vel(self, msg: Twist):
        vx = max(-self.lin_max, min(self.lin_max, msg.linear.x))
        vy = max(-self.lin_max, min(self.lin_max, msg.linear.y))
        vz = max(-self.ang_max, min(self.ang_max, msg.angular.z))

        req = Request()
        req.header.identity.api_id = API_ID["MOVE"]
        req.parameter = json.dumps({"x": round(vx, 3),
                                     "y": round(vy, 3),
                                     "z": round(vz, 3)})
        self.request_pub.publish(req)

    def send_command(self, api_id: int, parameter: str = ""):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        self.request_pub.publish(req)

    def stand_up(self):
        self.send_command(API_ID["STAND_UP"])
        self.send_command(API_ID["BALANCE_STAND"])

    def stand_down(self):
        self.send_command(API_ID["STAND_DOWN"])

    def damp(self):
        self.send_command(API_ID["DAMP"])

    def stop(self):
        self.send_command(API_ID["STOP_MOVE"])


def main():
    rclpy.init()
    node = Go2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
