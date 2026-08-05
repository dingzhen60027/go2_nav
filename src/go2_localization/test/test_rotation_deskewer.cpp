#include "go2_localization/rotation_deskewer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace go2_localization {
namespace {

constexpr int64_t kMillisecond = 1000000;

std::deque<AngularVelocitySample> constantImu(
    const Eigen::Vector3d &angular_velocity) {
  std::deque<AngularVelocitySample> samples;
  for (int64_t time_ms = -5; time_ms <= 105; time_ms += 5) {
    samples.push_back({time_ms * kMillisecond, angular_velocity});
  }
  return samples;
}

TEST(RotationDeskewerTest, LeavesStationaryPointsUnchanged) {
  RotationDeskewer deskewer;
  std::string error;
  ASSERT_TRUE(deskewer.build(
      constantImu(Eigen::Vector3d::Zero()), 0, 100 * kMillisecond,
      Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), 0.02, &error))
      << error;
  const Eigen::Vector3d point(2.0, -1.0, 0.5);
  EXPECT_TRUE(
      deskewer.compensate(point, 37 * kMillisecond).isApprox(point, 1e-12));
}

TEST(RotationDeskewerTest, CompensatesConstantYawToScanEnd) {
  constexpr double yaw_rate = M_PI / 2.0;
  RotationDeskewer deskewer;
  std::string error;
  ASSERT_TRUE(deskewer.build(
      constantImu(Eigen::Vector3d(0.0, 0.0, yaw_rate)), 0,
      100 * kMillisecond, Eigen::Vector3d::Zero(),
      Eigen::Matrix3d::Identity(), 0.02, &error))
      << error;
  const double end_yaw = yaw_rate * 0.1;
  const Eigen::Vector3d expected(std::cos(end_yaw), -std::sin(end_yaw), 0.0);
  EXPECT_TRUE(deskewer.compensate(Eigen::Vector3d::UnitX(), 0)
                  .isApprox(expected, 1e-9));
}

TEST(RotationDeskewerTest, RemovesConfiguredGyroBias) {
  const Eigen::Vector3d bias(0.01, -0.02, 0.03);
  RotationDeskewer deskewer;
  std::string error;
  ASSERT_TRUE(deskewer.build(
      constantImu(bias), 0, 100 * kMillisecond, bias,
      Eigen::Matrix3d::Identity(), 0.02, &error))
      << error;
  const Eigen::Vector3d point(1.0, 2.0, 3.0);
  EXPECT_TRUE(deskewer.compensate(point, 50 * kMillisecond)
                  .isApprox(point, 1e-12));
}

TEST(RotationDeskewerTest, RejectsIncompleteImuCoverage) {
  auto samples = constantImu(Eigen::Vector3d::Zero());
  while (samples.back().stamp_ns >= 90 * kMillisecond) {
    samples.pop_back();
  }
  RotationDeskewer deskewer;
  std::string error;
  EXPECT_FALSE(deskewer.build(
      samples, 0, 100 * kMillisecond, Eigen::Vector3d::Zero(),
      Eigen::Matrix3d::Identity(), 0.02, &error));
  EXPECT_EQ(error, "IMU does not cover scan end");
}

}  // namespace
}  // namespace go2_localization
