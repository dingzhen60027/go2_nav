import os
from math import cos, pi, sin

os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
import pytest
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from go2_localization.icp_fusion_bridge import IcpFusionBridge
from go2_localization.math_utils import yaw_from_quaternion


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_pose(x=0.0, y=0.0, z=0.0):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = z
    message.pose.pose.orientation.w = 1.0
    return message


def make_local_odometry(node, x=0.0, y=0.0, yaw=0.0):
    message = Odometry()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "odom"
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = sin(yaw / 2.0)
    message.pose.pose.orientation.w = cos(yaw / 2.0)
    return message


def test_global_ekf_reset_is_deferred_until_verified_icp_alignment(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    node = IcpFusionBridge()
    try:
        global_resets = RecordingPublisher()
        node.global_set_pose_pub = global_resets

        initial_pose = make_pose(4.0, -2.0, 0.0)
        node.on_initial_pose(initial_pose)

        assert not global_resets.messages
        assert not node.fusion_gate.alignment_locked
        assert node.reference_pose.position == (4.0, -2.0, 0.0)

        stale_global_prediction = Odometry()
        stale_global_prediction.header.frame_id = "map"
        stale_global_prediction.pose.pose.position.x = 100.0
        stale_global_prediction.pose.pose.orientation.w = 1.0
        node.on_prediction(stale_global_prediction)
        assert node.active_prediction().position == (4.0, -2.0, 0.0)

        # fused_icp_matcher publishes pose_raw only after its own acquisition
        # gate has completed the configured multi-scan confirmation.
        confirmed_icp = node.tracking_pose_message(node.reference_pose)
        node.on_raw_icp(confirmed_icp)

        assert node.fusion_gate.alignment_locked
        assert len(global_resets.messages) == 1
        reset_pose = global_resets.messages[0].pose.pose
        assert reset_pose.position.x == 4.0
        assert reset_pose.position.y == -2.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stale_icp_freezes_prediction_at_last_accepted_pose(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    node = IcpFusionBridge()
    try:
        initial_pose = make_pose(4.0, -2.0, 0.0)
        node.on_initial_pose(initial_pose)
        node.on_raw_icp(node.tracking_pose_message(node.reference_pose))
        assert node.fusion_gate.alignment_locked

        stale_by = Duration(seconds=node.icp_stale_timeout + 0.5)
        node.alignment_time = node.get_clock().now() - stale_by
        node.last_icp_time = node.get_clock().now() - stale_by

        drifting_prediction = Odometry()
        drifting_prediction.header.frame_id = "map"
        drifting_prediction.pose.pose.position.x = 100.0
        drifting_prediction.pose.pose.orientation.w = 1.0
        node.on_prediction(drifting_prediction)

        assert node.prediction_pose.position[0] == 100.0
        assert node.active_prediction().position == (4.0, -2.0, 0.0)
        assert node.fusion_gate.alignment_locked
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "turn_yaw",
    [pi / 9.0, pi / 3.0, 17.0 * pi / 18.0, -3.0 * pi / 4.0],
    ids=["plus_20_deg", "plus_60_deg", "plus_170_deg", "minus_135_deg"],
)
def test_stale_icp_prediction_follows_arbitrary_local_odometry_turn(
    monkeypatch, tmp_path, turn_yaw
):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    node = IcpFusionBridge()
    try:
        node.on_local_odometry(make_local_odometry(node, yaw=0.0))
        initial_pose = make_pose(4.0, -2.0, 0.0)
        node.on_initial_pose(initial_pose)
        node.on_raw_icp(node.tracking_pose_message(node.reference_pose))
        assert node.fusion_gate.alignment_locked
        assert node.reference_local_pose is not None

        stale_by = Duration(seconds=node.icp_stale_timeout + 0.5)
        node.alignment_time = node.get_clock().now() - stale_by
        node.last_icp_time = node.get_clock().now() - stale_by
        node.on_local_odometry(make_local_odometry(node, yaw=turn_yaw))

        prediction = node.active_prediction()
        assert yaw_from_quaternion(prediction.orientation) == pytest.approx(
            turn_yaw, abs=1.0e-6
        )
        accepted_before = node.accepted
        node.on_raw_icp(node.tracking_pose_message(prediction))
        assert node.accepted == accepted_before + 1
        assert node.consecutive_rejections == 0
        assert node.fusion_gate.alignment_locked
    finally:
        node.destroy_node()
        rclpy.shutdown()
