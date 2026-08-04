#!/usr/bin/env python3

import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class NeutralJointStatePublisher(Node):
    def __init__(self):
        super().__init__("neutral_joint_state_publisher")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.message = JointState()
        self.message.name = [
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
        ]
        self.message.position = [0.0, 0.8, -1.6] * 4
        self.timer = self.create_timer(0.1, self.publish_joint_state)

    def publish_joint_state(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main(args=None):
    rclpy.init(args=args)
    node = NeutralJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
