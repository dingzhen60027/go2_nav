#include "fast_icp_loc/imu_deskewer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace fast_icp_loc {
namespace {

constexpr int64_t kMillisecond = 1000000;

std::deque<ImuSample> constantImu(const Eigen::Vector3d &angular_velocity) {
  std::deque<ImuSample> samples;
  for (int64_t time_ms = -5; time_ms <= 105; time_ms += 5) {
    samples.push_back({time_ms * kMillisecond, angular_velocity});
  }
  return samples;
}

TEST(RotationTrajectoryTest, LeavesPointsUnchangedWhenStationary) {
  RotationTrajectory trajectory;
  std::string error;
  ASSERT_TRUE(trajectory.build(
      constantImu(Eigen::Vector3d::Zero()), 0, 100 * kMillisecond,
      Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.02, &error)) << error;

  const Eigen::Vector3d point(2.0, -1.0, 0.5);
  EXPECT_TRUE(trajectory.compensate(point, 37 * kMillisecond).isApprox(point, 1e-12));
}

TEST(RotationTrajectoryTest, CompensatesConstantYawToScanEnd) {
  constexpr double yaw_rate = M_PI / 2.0;
  RotationTrajectory trajectory;
  std::string error;
  ASSERT_TRUE(trajectory.build(
      constantImu(Eigen::Vector3d(0.0, 0.0, yaw_rate)),
      0, 100 * kMillisecond, Eigen::Vector3d::Zero(),
      Eigen::Matrix3d::Identity(), 0.02, &error)) << error;

  const Eigen::Vector3d point_at_start(1.0, 0.0, 0.0);
  const double end_yaw = yaw_rate * 0.1;
  const Eigen::Vector3d expected(std::cos(end_yaw), -std::sin(end_yaw), 0.0);
  EXPECT_TRUE(trajectory.compensate(point_at_start, 0).isApprox(expected, 1e-9));

  const Eigen::Vector3d point_at_end(0.4, -0.2, 1.0);
  EXPECT_TRUE(trajectory.compensate(point_at_end, 100 * kMillisecond)
                  .isApprox(point_at_end, 1e-12));
}

TEST(RotationTrajectoryTest, RemovesConfiguredGyroBias) {
  const Eigen::Vector3d bias(0.01, -0.02, 0.03);
  RotationTrajectory trajectory;
  std::string error;
  ASSERT_TRUE(trajectory.build(
      constantImu(bias), 0, 100 * kMillisecond, bias,
      Eigen::Matrix3d::Identity(), 0.02, &error)) << error;

  const Eigen::Vector3d point(1.0, 2.0, 3.0);
  EXPECT_TRUE(trajectory.compensate(point, 50 * kMillisecond).isApprox(point, 1e-12));
}

TEST(RotationTrajectoryTest, RejectsMissingEndCoverage) {
  auto samples = constantImu(Eigen::Vector3d::Zero());
  while (samples.back().stamp_ns >= 90 * kMillisecond) {
    samples.pop_back();
  }

  RotationTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(trajectory.build(
      samples, 0, 100 * kMillisecond, Eigen::Vector3d::Zero(),
      Eigen::Matrix3d::Identity(), 0.02, &error));
  EXPECT_EQ(error, "IMU does not cover scan end");
}

TEST(RotationTrajectoryTest, RejectsLargeImuGap) {
  auto samples = constantImu(Eigen::Vector3d::Zero());
  samples.erase(
      std::remove_if(samples.begin(), samples.end(), [](const ImuSample &sample) {
        return sample.stamp_ns > 20 * kMillisecond && sample.stamp_ns < 80 * kMillisecond;
      }),
      samples.end());

  RotationTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(trajectory.build(
      samples, 0, 100 * kMillisecond, Eigen::Vector3d::Zero(),
      Eigen::Matrix3d::Identity(), 0.02, &error));
  EXPECT_EQ(error, "IMU gap exceeds configured limit");
}

}  // namespace
}  // namespace fast_icp_loc
