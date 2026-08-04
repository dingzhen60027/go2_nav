from math import pi

import pytest

from go2_localization.math_utils import (
    Pose3,
    base_pose_to_tracking,
    normalize_quaternion,
    passes_gate,
    pose_innovation,
    tracking_pose_to_base,
)


def assert_pose_close(actual, expected):
    assert actual.position == pytest.approx(expected.position, abs=1.0e-9)
    assert abs(
        sum(a * b for a, b in zip(actual.orientation, expected.orientation))
    ) == pytest.approx(1.0, abs=1.0e-9)


def test_tracking_transform_round_trip():
    map_to_base = Pose3(
        position=(2.0, -1.0, 0.2),
        orientation=normalize_quaternion((0.0, 0.0, 0.3826834, 0.9238795)),
    )
    base_to_tracking = Pose3(
        position=(0.16143, 0.0, 0.12262),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    map_to_tracking = base_pose_to_tracking(map_to_base, base_to_tracking)
    recovered = tracking_pose_to_base(map_to_tracking, base_to_tracking)
    assert_pose_close(recovered, map_to_base)


def test_tracking_offset_rotates_with_base():
    map_to_base = Pose3(
        position=(1.0, 2.0, 0.0),
        orientation=normalize_quaternion((0.0, 0.0, 1.0, 1.0)),
    )
    base_to_tracking = Pose3(
        position=(0.2, 0.0, 0.1),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    result = base_pose_to_tracking(map_to_base, base_to_tracking)
    assert result.position == pytest.approx((1.0, 2.2, 0.1), abs=1.0e-9)


def test_gate_rejects_large_yaw_innovation():
    prediction = Pose3((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    measurement = Pose3(
        (0.1, 0.0, 0.0),
        (0.0, 0.0, 0.5**0.5, 0.5**0.5),
    )
    innovation = pose_innovation(measurement, prediction)
    assert innovation.yaw == pytest.approx(pi / 2.0)
    assert not passes_gate(innovation, 0.6, 0.35, 0.55, 0.7)


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError):
        normalize_quaternion((0.0, 0.0, 0.0, 0.0))
