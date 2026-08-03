#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace fast_icp_loc {

struct ImuSample {
  int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

class RotationTrajectory {
public:
  bool build(const std::deque<ImuSample> &imu_samples,
             int64_t scan_start_ns,
             int64_t scan_end_ns,
             const Eigen::Vector3d &gyro_bias,
             const Eigen::Matrix3d &rotation_lidar_from_imu,
             double max_imu_gap_sec,
             std::string *error);

  Eigen::Vector3d compensate(const Eigen::Vector3d &point,
                             int64_t point_stamp_ns) const;

private:
  struct RotationKnot {
    int64_t stamp_ns;
    Eigen::Quaterniond rotation;
  };

  Eigen::Quaterniond rotationAt(int64_t stamp_ns) const;
  std::vector<RotationKnot> knots_;
};

}  // namespace fast_icp_loc
