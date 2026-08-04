#include "fast_icp_loc/fast_icp_loc.hpp"
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_icp_loc {

static constexpr int IMU_INIT_COUNT = 50;  // 采集 50 帧 IMU（~0.25s @ 200Hz）

static double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

static double yawFromPose(const Eigen::Matrix4d &pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

FastIcpLoc::FastIcpLoc(const rclcpp::NodeOptions &options)
    : Node("fast_icp_loc", options), map_loaded_(false), localized_(false),
      leveling_done_(false), imu_count_(0) {

  map_pcd_path_  = this->declare_parameter("map_pcd", "/home/wjg/go2_nav/maps/clean/pcd_icp_latest.pcd");
  scan_topic_    = this->declare_parameter("scan_topic", "/livox/lidar");
  imu_topic_     = this->declare_parameter("imu_topic", "/livox/imu");
  world_frame_   = this->declare_parameter("world_frame", "camera_init");
  body_frame_    = this->declare_parameter("body_frame", "livox_frame");
  lidar_frame_   = this->declare_parameter("lidar_frame", "livox_frame");
  prediction_topic_ = this->declare_parameter("prediction_topic", "");
  voxel_leaf_    = this->declare_parameter("voxel_leaf", 0.15);
  max_corr_dist_ = this->declare_parameter("max_corr_dist", 2.0);
  max_translation_delta_ = this->declare_parameter("max_translation_delta", 0.45);
  max_yaw_delta_ = this->declare_parameter("max_yaw_delta", 0.60);
  max_fitness_score_ = this->declare_parameter("max_fitness_score", 1.0);
  deskew_enabled_ = this->declare_parameter("deskew_enabled", true);
  max_imu_gap_sec_ = this->declare_parameter("max_imu_gap_sec", 0.02);
  max_scan_duration_sec_ = this->declare_parameter("max_scan_duration_sec", 0.15);
  imu_buffer_duration_sec_ = this->declare_parameter("imu_buffer_duration_sec", 2.0);
  imu_init_max_gyro_ = this->declare_parameter("imu_init_max_gyro", 0.10);
  prediction_timeout_sec_ = this->declare_parameter("prediction_timeout_sec", 0.25);
  publish_only_accepted_pose_ =
      this->declare_parameter("publish_only_accepted_pose", false);
  max_pending_scans_ = this->declare_parameter("max_pending_scans", 3);
  max_iter_      = this->declare_parameter("max_iterations", 30);

  if (max_imu_gap_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "max_imu_gap_sec must be positive; using 0.02 s");
    max_imu_gap_sec_ = 0.02;
  }
  if (max_scan_duration_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "max_scan_duration_sec must be positive; using 0.15 s");
    max_scan_duration_sec_ = 0.15;
  }
  if (imu_buffer_duration_sec_ < max_scan_duration_sec_ + max_imu_gap_sec_) {
    RCLCPP_WARN(get_logger(), "IMU buffer is too short; using 2.0 s");
    imu_buffer_duration_sec_ = 2.0;
  }
  if (imu_init_max_gyro_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "imu_init_max_gyro must be positive; using 0.10 rad/s");
    imu_init_max_gyro_ = 0.10;
  }
  if (max_pending_scans_ < 1) {
    RCLCPP_WARN(get_logger(), "max_pending_scans must be at least 1; using 3");
    max_pending_scans_ = 3;
  }
  if (prediction_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "prediction_timeout_sec must be positive; using 0.25 s");
    prediction_timeout_sec_ = 0.25;
  }

  const auto rotation_values = this->declare_parameter<std::vector<double>>(
      "rotation_lidar_from_imu",
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0});
  rotation_lidar_from_imu_ = Eigen::Matrix3d::Identity();
  if (rotation_values.size() == 9) {
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotation_lidar_from_imu_(row, column) = rotation_values[row * 3 + column];
      }
    }
    const Eigen::Matrix3d orthogonality =
        rotation_lidar_from_imu_ * rotation_lidar_from_imu_.transpose();
    if (!orthogonality.isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
        std::abs(rotation_lidar_from_imu_.determinant() - 1.0) > 1e-3) {
      RCLCPP_WARN(get_logger(),
                  "rotation_lidar_from_imu is invalid; using identity rotation");
      rotation_lidar_from_imu_.setIdentity();
    }
  } else {
    RCLCPP_WARN(get_logger(),
                "rotation_lidar_from_imu must contain 9 values; using identity rotation");
  }

  loadMap();

  // IMU remains subscribed after leveling because every scan needs angular velocity.
  rclcpp::SensorDataQoS imu_qos;
  imu_qos.keep_last(500);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&FastIcpLoc::imuCallback, this, std::placeholders::_1));
  imu_acc_sum_ = Eigen::Vector3d::Zero();
  imu_gyro_sum_ = Eigen::Vector3d::Zero();
  gyro_bias_ = Eigen::Vector3d::Zero();
  R_level_ = Eigen::Matrix4d::Identity();

  rclcpp::SensorDataQoS scan_qos;
  scan_qos.keep_last(10);
  scan_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      scan_topic_, scan_qos,
      std::bind(&FastIcpLoc::scanCallback, this, std::placeholders::_1));

  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&FastIcpLoc::initPoseCallback, this, std::placeholders::_1));

  if (!prediction_topic_.empty()) {
    prediction_pose_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            prediction_topic_, 10,
            std::bind(&FastIcpLoc::predictionPoseCallback, this,
                      std::placeholders::_1));
  }

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/icp_pose", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  // 持续发 TF（直到定位开始），保证实时点云在 RViz 里一直可见
  tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this]() {
    if (localized_) return;  // 定位开始后由 publishPose 负责
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = body_frame_;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);
  });

  // 发布地图点云供 RViz 显示
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 10);
  {
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud_ds_, map_msg);
    map_msg.header.frame_id = world_frame_;
    map_msg.header.stamp = this->now();
    map_cloud_pub_->publish(map_msg);
  }
  // 定时重发（RViz transient local 订阅）
  map_timer_ = this->create_wall_timer(std::chrono::seconds(3),
      [this]() {
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud_ds_, map_msg);
        map_msg.header.frame_id = world_frame_;
        map_msg.header.stamp = this->now();
        map_cloud_pub_->publish(map_msg);
      });

  // CustomMsg → PointCloud2 转换发布，供 Nav2 costmap 障碍物层使用
  scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_converted", 10);
  // 校平后的 PointCloud2（RViz 显示，frame=map，不再倾斜）
  leveled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_leveled", 10);

  last_pose_ = Eigen::Matrix4d::Identity();

  RCLCPP_INFO(get_logger(), "Fast ICP Loc ready. Map: %lu pts", map_cloud_ds_->size());
  RCLCPP_INFO(get_logger(), "Collecting %d IMU samples to estimate leveling...", IMU_INIT_COUNT);
  RCLCPP_INFO(get_logger(), "MID360 rotational deskew: %s, max IMU gap %.0f ms",
              deskew_enabled_ ? "enabled" : "disabled", max_imu_gap_sec_ * 1000.0);
}

void FastIcpLoc::loadMap() {
  auto full = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_pcd_path_, *full) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to load: %s", map_pcd_path_.c_str());
    return;
  }
  map_cloud_ds_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(full);
  vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
  vg.filter(*map_cloud_ds_);
  map_loaded_ = true;
  RCLCPP_INFO(get_logger(), "Map: %lu → %lu pts", full->size(), map_cloud_ds_->size());
}

void FastIcpLoc::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
  const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (stamp_ns <= 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Ignoring IMU sample with invalid timestamp");
    return;
  }
  if (last_imu_stamp_ns_ != 0 && stamp_ns <= last_imu_stamp_ns_) {
    if (stamp_ns == last_imu_stamp_ns_) {
      return;
    }
    RCLCPP_WARN(get_logger(), "IMU time moved backwards; clearing synchronization buffers");
    imu_buffer_.clear();
    pending_scans_.clear();
    if (!leveling_done_) {
      imu_count_ = 0;
      imu_acc_sum_.setZero();
      imu_gyro_sum_.setZero();
    }
  }

  const Eigen::Vector3d gyro(msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z);
  const Eigen::Vector3d acceleration(msg->linear_acceleration.x,
                                     msg->linear_acceleration.y,
                                     msg->linear_acceleration.z);
  if (!gyro.allFinite() || !acceleration.allFinite()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Ignoring IMU sample containing NaN or infinity");
    return;
  }
  imu_buffer_.push_back({stamp_ns, gyro});
  last_imu_stamp_ns_ = stamp_ns;

  if (!leveling_done_) {
    if (gyro.norm() > imu_init_max_gyro_) {
      imu_count_ = 0;
      imu_acc_sum_.setZero();
      imu_gyro_sum_.setZero();
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Keep robot still for IMU initialization (gyro %.3f rad/s)",
                           gyro.norm());
      pruneImuBuffer();
      return;
    }

    imu_acc_sum_ += acceleration;
    imu_gyro_sum_ += gyro;
    imu_count_++;
  }

  if (!leveling_done_ && imu_count_ >= IMU_INIT_COUNT) {
    const Eigen::Vector3d acceleration_lidar = rotation_lidar_from_imu_ * imu_acc_sum_;
    if (acceleration_lidar.norm() < 1e-6) {
      RCLCPP_ERROR(get_logger(), "IMU acceleration is zero; cannot estimate leveling");
      imu_count_ = 0;
      imu_acc_sum_.setZero();
      imu_gyro_sum_.setZero();
      return;
    }

    // Rotate the measured upward direction onto the map's +Z direction.
    Eigen::Vector3d up = acceleration_lidar.normalized();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(up, Eigen::Vector3d::UnitZ());
    R_level_.block<3,3>(0,0) = q.toRotationMatrix();
    gyro_bias_ = imu_gyro_sum_ / static_cast<double>(imu_count_);

    leveling_done_ = true;

    const double angle =
        std::acos(std::clamp(q.w(), -1.0, 1.0)) * 2.0 * 180.0 / M_PI;
    RCLCPP_INFO(get_logger(), "IMU leveling done: %.1f deg, rotation applied to scans", angle);
    RCLCPP_INFO(get_logger(), "Gyro bias: [%.5f, %.5f, %.5f] rad/s",
                gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z());
    RCLCPP_INFO(get_logger(), "Waiting for initial pose via /initialpose (RViz 2D Pose Estimate)...");
  }

  if (leveling_done_) {
    processPendingScans();
  }
  pruneImuBuffer();
}

void FastIcpLoc::initPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  const auto &p = msg->pose.pose.position;
  const auto &o = msg->pose.pose.orientation;
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  last_pose_.block<3,3>(0,0) = q.toRotationMatrix();
  last_pose_(0,3) = p.x;
  last_pose_(1,3) = p.y;
  last_pose_(2,3) = p.z;
  prediction_received_ns_ = 0;
  localized_ = true;
  RCLCPP_INFO(get_logger(), "Initial pose: (%.2f, %.2f, %.2f), start tracking", p.x, p.y, p.z);
}

void FastIcpLoc::predictionPoseCallback(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (!localized_ || (!msg->header.frame_id.empty() &&
                      msg->header.frame_id != world_frame_)) {
    return;
  }

  const auto &p = msg->pose.pose.position;
  const auto &o = msg->pose.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.z) ||
      !std::isfinite(o.w)) {
    return;
  }

  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  if (q.norm() < 1e-9) {
    return;
  }
  q.normalize();
  prediction_pose_.setIdentity();
  prediction_pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
  prediction_pose_(0, 3) = p.x;
  prediction_pose_(1, 3) = p.y;
  prediction_pose_(2, 3) = p.z;
  prediction_received_ns_ = this->now().nanoseconds();
}

void FastIcpLoc::scanCallback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  if (!map_loaded_ || !leveling_done_) return;

  int64_t scan_start_ns = 0;
  int64_t scan_end_ns = 0;
  if (!scanTimeRange(*msg, &scan_start_ns, &scan_end_ns)) {
    ++dropped_scan_count_;
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Dropping point cloud with invalid time or no points");
    return;
  }
  if (last_scan_stamp_ns_ != 0 && scan_start_ns < last_scan_stamp_ns_) {
    RCLCPP_WARN(get_logger(), "LiDAR time moved backwards; clearing pending scans");
    pending_scans_.clear();
  }
  last_scan_stamp_ns_ = scan_start_ns;

  if (static_cast<int>(pending_scans_.size()) >= max_pending_scans_) {
    pending_scans_.pop_front();
    ++dropped_scan_count_;
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                         "Deskew queue full; dropping oldest scan (IMU may be delayed)");
  }
  pending_scans_.push_back(std::move(msg));
  processPendingScans();
}

bool FastIcpLoc::scanTimeRange(const livox_ros_driver2::msg::CustomMsg &msg,
                               int64_t *scan_start_ns,
                               int64_t *scan_end_ns) const {
  if (msg.points.empty() || scan_start_ns == nullptr || scan_end_ns == nullptr) {
    return false;
  }

  const uint64_t base_time = msg.timebase != 0
                                 ? msg.timebase
                                 : static_cast<uint64_t>(rclcpp::Time(msg.header.stamp).nanoseconds());
  const auto max_offset = std::max_element(
      msg.points.begin(), msg.points.end(),
      [](const auto &left, const auto &right) {
        return left.offset_time < right.offset_time;
      })->offset_time;
  if (base_time == 0 ||
      base_time > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) - max_offset) {
    return false;
  }

  *scan_start_ns = static_cast<int64_t>(base_time);
  *scan_end_ns = static_cast<int64_t>(base_time + max_offset);
  return true;
}

void FastIcpLoc::processPendingScans() {
  while (!pending_scans_.empty() && !imu_buffer_.empty()) {
    int64_t scan_start_ns = 0;
    int64_t scan_end_ns = 0;
    if (!scanTimeRange(*pending_scans_.front(), &scan_start_ns, &scan_end_ns)) {
      pending_scans_.pop_front();
      ++dropped_scan_count_;
      continue;
    }

    const double scan_duration_sec =
        static_cast<double>(scan_end_ns - scan_start_ns) * 1e-9;
    if (scan_duration_sec > max_scan_duration_sec_) {
      pending_scans_.pop_front();
      ++dropped_scan_count_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "Dropping scan with unexpected duration %.1f ms (limit %.1f ms)",
          scan_duration_sec * 1000.0, max_scan_duration_sec_ * 1000.0);
      continue;
    }

    if (deskew_enabled_ && scan_end_ns > imu_buffer_.back().stamp_ns) {
      return;
    }

    auto scan = pending_scans_.front();
    pending_scans_.pop_front();
    processScan(scan, scan_start_ns, scan_end_ns);
  }
}

void FastIcpLoc::processScan(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg,
    int64_t scan_start_ns,
    int64_t scan_end_ns) {
  RotationTrajectory trajectory;
  const bool has_duration = scan_end_ns > scan_start_ns;
  if (deskew_enabled_ && has_duration) {
    std::string error;
    if (!trajectory.build(imu_buffer_, scan_start_ns, scan_end_ns, gyro_bias_,
                          rotation_lidar_from_imu_, max_imu_gap_sec_, &error)) {
      ++dropped_scan_count_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "Dropping scan: rotational deskew unavailable (%s)", error.c_str());
      return;
    }
  }

  // Convert each point after moving it to the scan-end orientation.
  auto scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  scan->reserve(msg->points.size());
  for (const auto &p : msg->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    Eigen::Vector3d point(p.x, p.y, p.z);
    if (deskew_enabled_ && has_duration) {
      point = trajectory.compensate(point, scan_start_ns + p.offset_time);
    }
    scan->emplace_back(static_cast<float>(point.x()),
                       static_cast<float>(point.y()),
                       static_cast<float>(point.z()));
  }
  if (scan->size() < 50) {
    ++dropped_scan_count_;
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Dropping scan with only %lu valid points", scan->size());
    return;
  }

  const rclcpp::Time scan_stamp(scan_end_ns);
  ++deskewed_scan_count_;

  // Publish the corrected cloud even before an initial localization pose is set.
  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*scan, scan_msg);
  scan_msg.header.frame_id = lidar_frame_;
  scan_msg.header.stamp = scan_stamp;
  scan_pub_->publish(scan_msg);

  // ---- 校平：旋转实时扫描使其与 PCD 地图坐标系对齐 ----
  pcl::transformPointCloud(*scan, *scan, R_level_);

  // 发布校平后的 PointCloud2（RViz 显示用）
  sensor_msgs::msg::PointCloud2 level_msg;
  pcl::toROSMsg(*scan, level_msg);
  level_msg.header.frame_id = body_frame_;
  level_msg.header.stamp = scan_stamp;
  leveled_pub_->publish(level_msg);

  if (deskewed_scan_count_ % 100 == 0) {
    RCLCPP_INFO(get_logger(),
                "Deskew status: processed=%lu dropped=%lu duration=%.1fms IMU_buffer=%lu",
                deskewed_scan_count_, dropped_scan_count_,
                static_cast<double>(scan_end_ns - scan_start_ns) * 1e-6,
                imu_buffer_.size());
  }

  // ICP 匹配等定位开始后才执行
  if (!localized_) return;

  auto scan_ds = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(scan);
  vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
  vg.filter(*scan_ds);
  if (scan_ds->size() < 50) return;

  // ---- Fast GICP ----
  fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(scan_ds);
  icp.setInputTarget(map_cloud_ds_);
  icp.setMaxCorrespondenceDistance(max_corr_dist_);
  icp.setMaximumIterations(max_iter_);
  icp.setNumThreads(4);

  Eigen::Matrix4d initial_guess = last_pose_;
  const int64_t prediction_age_ns =
      this->now().nanoseconds() - prediction_received_ns_;
  if (prediction_received_ns_ > 0 && prediction_age_ns >= 0 &&
      prediction_age_ns <= static_cast<int64_t>(prediction_timeout_sec_ * 1e9)) {
    initial_guess = prediction_pose_;
  }

  auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  icp.align(*aligned, initial_guess.cast<float>());

  bool accepted = false;
  if (icp.hasConverged()) {
    const Eigen::Matrix4d candidate = icp.getFinalTransformation().cast<double>();
    const Eigen::Vector3d delta_t =
        candidate.block<3, 1>(0, 3) - initial_guess.block<3, 1>(0, 3);
    const double translation_delta = delta_t.head<2>().norm();
    const double yaw_delta =
        std::abs(normalizeAngle(yawFromPose(candidate) - yawFromPose(initial_guess)));
    const double fitness_score = icp.getFitnessScore(max_corr_dist_);

    if (translation_delta > max_translation_delta_ ||
        yaw_delta > max_yaw_delta_ ||
        fitness_score > max_fitness_score_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "Reject ICP jump: trans=%.3fm yaw=%.3frad fitness=%.3f "
          "(limits %.3fm %.3frad %.3f)",
          translation_delta, yaw_delta, fitness_score,
          max_translation_delta_, max_yaw_delta_, max_fitness_score_);
    } else {
      last_pose_ = candidate;
      accepted = true;
    }
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "ICP not converged, keeping last pose");
  }

  if (accepted || !publish_only_accepted_pose_) {
    publishPose(last_pose_, scan_stamp);
  }
}

void FastIcpLoc::pruneImuBuffer() {
  if (imu_buffer_.size() < 3) {
    return;
  }

  int64_t keep_after_ns = imu_buffer_.back().stamp_ns -
      static_cast<int64_t>(imu_buffer_duration_sec_ * 1e9);
  if (!pending_scans_.empty()) {
    int64_t pending_start_ns = 0;
    int64_t pending_end_ns = 0;
    if (scanTimeRange(*pending_scans_.front(), &pending_start_ns, &pending_end_ns)) {
      keep_after_ns = std::min(
          keep_after_ns,
          pending_start_ns - static_cast<int64_t>(max_imu_gap_sec_ * 1e9));
    }
  }

  while (imu_buffer_.size() > 2 && imu_buffer_[1].stamp_ns < keep_after_ns) {
    imu_buffer_.pop_front();
  }
}

void FastIcpLoc::publishPose(const Eigen::Matrix4d &T, const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = world_frame_;
  tf.child_frame_id = body_frame_;
  tf.transform.translation.x = T(0,3);
  tf.transform.translation.y = T(1,3);
  tf.transform.translation.z = T(2,3);
  Eigen::Quaterniond q(T.block<3,3>(0,0));
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = world_frame_;
  pose.pose.pose.position.x = T(0,3);
  pose.pose.pose.position.y = T(1,3);
  pose.pose.pose.position.z = T(2,3);
  pose.pose.pose.orientation = tf.transform.rotation;
  pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[14] = 0.01;
  pose_pub_->publish(pose);
}

}  // namespace fast_icp_loc
