#include "go2_localization/fused_icp_gate.hpp"
#include "go2_localization/rotation_deskewer.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_localization {
namespace {

constexpr int kImuInitializationSamples = 50;

struct TimedPose {
  int64_t stamp_ns{0};
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
};

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromPose(const Eigen::Matrix4d &pose) {
  return std::atan2(pose(1, 0), pose(0, 0));
}

bool finitePose(const geometry_msgs::msg::Pose &pose) {
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

}  // namespace

class FusedIcpMatcher : public rclcpp::Node {
public:
  FusedIcpMatcher()
      : Node("fused_icp_matcher"),
        map_cloud_(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()) {
    declareAndLoadParameters();
    loadMap();

    // GICP can take longer than one LiDAR period. Keep IMU ingestion in a
    // separate mutually-exclusive callback group so deskew data continues to
    // arrive while the default callback group performs scan matching.
    imu_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;
    rclcpp::SensorDataQoS imu_qos;
    imu_qos.keep_last(500);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        std::bind(&FusedIcpMatcher::onImu, this, std::placeholders::_1),
        imu_options);

    rclcpp::SensorDataQoS scan_qos;
    // Old scans are actively harmful to localization: after a slow match they
    // no longer describe the robot's current pose. DDS and the application
    // therefore both retain only the newest scan.
    scan_qos.keep_last(1);
    scan_subscription_ =
        create_subscription<livox_ros_driver2::msg::CustomMsg>(
            scan_topic_, scan_qos,
            std::bind(&FusedIcpMatcher::onScan, this, std::placeholders::_1));

    initial_pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_, 10,
            std::bind(&FusedIcpMatcher::onInitialPose, this,
                      std::placeholders::_1));
    prediction_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            prediction_topic_, 20,
            std::bind(&FusedIcpMatcher::onPrediction, this,
                      std::placeholders::_1));

    pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            output_pose_topic_, 20);
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        map_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
    raw_scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        raw_scan_topic_, rclcpp::SensorDataQoS());
    leveled_scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        leveled_scan_topic_, rclcpp::SensorDataQoS());

    publishMap();
    map_timer_ = create_wall_timer(std::chrono::seconds(3),
                                   [this]() { publishMap(); });
    scan_processing_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { processPendingScan(); });

    RCLCPP_INFO(get_logger(),
                "Independent fused ICP matcher ready: map=%lu points",
                map_cloud_->size());
    RCLCPP_INFO(get_logger(),
                "Waiting for stationary Livox IMU initialization (%d samples)",
                kImuInitializationSamples);
  }

private:
  void declareAndLoadParameters() {
    map_path_ = declare_parameter("map_pcd", "");
    scan_topic_ = declare_parameter("scan_topic", "/livox/lidar");
    imu_topic_ = declare_parameter("imu_topic", "/livox/imu");
    world_frame_ = declare_parameter("world_frame", "map");
    tracking_frame_ =
        declare_parameter("tracking_frame", "icp_tracking_frame");
    lidar_frame_ = declare_parameter("lidar_frame", "livox_frame");
    initial_pose_topic_ = declare_parameter(
        "initial_pose_topic", "/localization/fused_icp/initialpose");
    prediction_topic_ = declare_parameter(
        "prediction_topic", "/localization/fused_icp/prediction");
    output_pose_topic_ = declare_parameter(
        "output_pose_topic", "/localization/fused_icp/pose_raw");
    map_cloud_topic_ = declare_parameter("map_cloud_topic", "/map_cloud");
    raw_scan_topic_ =
        declare_parameter("raw_scan_topic", "/scan_converted");
    leveled_scan_topic_ =
        declare_parameter("leveled_scan_topic", "/scan_leveled");

    voxel_leaf_ = declare_parameter("voxel_leaf", 0.15);
    max_corr_dist_ = declare_parameter("max_corr_dist", 1.5);
    max_translation_delta_ =
        declare_parameter("max_translation_delta", 0.45);
    max_translation_z_delta_ =
        declare_parameter("max_translation_z_delta", 0.35);
    max_yaw_delta_ = declare_parameter("max_yaw_delta", 0.60);
    max_tilt_ = declare_parameter("max_tilt", 0.35);
    max_rotation_delta_ =
        declare_parameter("max_rotation_delta", 0.70);
    max_fitness_score_ = declare_parameter("max_fitness_score", 1.0);
    max_iterations_ = declare_parameter("max_iterations", 15);

    acquisition_max_corr_dist_ =
        declare_parameter("acquisition_max_corr_dist", 3.0);
    acquisition_max_translation_delta_ =
        declare_parameter("acquisition_max_translation_delta", 2.5);
    acquisition_max_translation_z_delta_ =
        declare_parameter("acquisition_max_translation_z_delta", 1.0);
    acquisition_max_yaw_delta_ =
        declare_parameter("acquisition_max_yaw_delta", 1.57);
    acquisition_max_tilt_ =
        declare_parameter("acquisition_max_tilt", 0.35);
    acquisition_max_rotation_delta_ =
        declare_parameter("acquisition_max_rotation_delta", 1.80);
    acquisition_max_fitness_score_ =
        declare_parameter("acquisition_max_fitness_score", 1.0);
    acquisition_max_iterations_ =
        declare_parameter("acquisition_max_iterations", 50);
    acquisition_confirmation_count_ =
        declare_parameter("acquisition_confirmation_count", 3);

    prediction_timeout_sec_ =
        declare_parameter("prediction_timeout_sec", 0.25);
    prediction_buffer_duration_sec_ =
        declare_parameter("prediction_buffer_duration_sec", 2.0);
    deskew_enabled_ = declare_parameter("deskew_enabled", true);
    max_imu_gap_sec_ = declare_parameter("max_imu_gap_sec", 0.02);
    max_scan_duration_sec_ =
        declare_parameter("max_scan_duration_sec", 0.15);
    max_scan_age_sec_ = declare_parameter("max_scan_age_sec", 0.50);
    imu_buffer_duration_sec_ =
        declare_parameter("imu_buffer_duration_sec", 2.0);
    imu_init_max_gyro_ = declare_parameter("imu_init_max_gyro", 0.10);

    const auto rotation_values = declare_parameter<std::vector<double>>(
        "rotation_lidar_from_imu",
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    rotation_lidar_from_imu_.setIdentity();
    if (rotation_values.size() == 9) {
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          rotation_lidar_from_imu_(row, column) =
              rotation_values[row * 3 + column];
        }
      }
    }
    const Eigen::Matrix3d orthogonality =
        rotation_lidar_from_imu_ * rotation_lidar_from_imu_.transpose();
    if (rotation_values.size() != 9 ||
        !orthogonality.isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
        std::abs(rotation_lidar_from_imu_.determinant() - 1.0) > 1e-3) {
      RCLCPP_WARN(get_logger(),
                  "rotation_lidar_from_imu is invalid; using identity");
      rotation_lidar_from_imu_.setIdentity();
    }

    if (map_path_.empty()) {
      throw std::runtime_error("map_pcd parameter is empty");
    }
    if (voxel_leaf_ <= 0.0 || max_corr_dist_ <= 0.0 ||
        max_translation_delta_ <= 0.0 || max_yaw_delta_ <= 0.0 ||
        max_translation_z_delta_ <= 0.0 || max_tilt_ <= 0.0 ||
        max_rotation_delta_ <= 0.0 || max_fitness_score_ <= 0.0 ||
        max_iterations_ < 1) {
      throw std::runtime_error("tracking ICP parameters must be positive");
    }
    acquisition_max_corr_dist_ =
        std::max(acquisition_max_corr_dist_, max_corr_dist_);
    acquisition_max_translation_delta_ = std::max(
        acquisition_max_translation_delta_, max_translation_delta_);
    acquisition_max_translation_z_delta_ = std::max(
        acquisition_max_translation_z_delta_, max_translation_z_delta_);
    acquisition_max_yaw_delta_ =
        std::max(acquisition_max_yaw_delta_, max_yaw_delta_);
    acquisition_max_tilt_ = std::max(acquisition_max_tilt_, max_tilt_);
    acquisition_max_rotation_delta_ = std::max(
        acquisition_max_rotation_delta_, max_rotation_delta_);
    acquisition_max_iterations_ =
        std::max(acquisition_max_iterations_, max_iterations_);
    acquisition_confirmation_count_ =
        std::max(1, acquisition_confirmation_count_);
    if (acquisition_max_fitness_score_ <= 0.0) {
      acquisition_max_fitness_score_ = max_fitness_score_;
    }
    prediction_timeout_sec_ = std::max(0.01, prediction_timeout_sec_);
    prediction_buffer_duration_sec_ = std::max(
        prediction_buffer_duration_sec_, 2.0 * prediction_timeout_sec_);
    max_imu_gap_sec_ = std::max(0.001, max_imu_gap_sec_);
    max_scan_duration_sec_ = std::max(0.01, max_scan_duration_sec_);
    max_scan_age_sec_ = std::max(0.05, max_scan_age_sec_);
    imu_buffer_duration_sec_ = std::max(
        imu_buffer_duration_sec_,
        max_scan_duration_sec_ + 2.0 * max_imu_gap_sec_);
    imu_init_max_gyro_ = std::max(0.001, imu_init_max_gyro_);

    gate_.configure(
        {max_translation_delta_, max_yaw_delta_, max_fitness_score_},
        {acquisition_max_translation_delta_, acquisition_max_yaw_delta_,
         acquisition_max_fitness_score_},
        acquisition_confirmation_count_);
  }

  void loadMap() {
    auto full_map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *full_map) == -1) {
      throw std::runtime_error("failed to load ICP map: " + map_path_);
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(full_map);
    voxel_filter.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    voxel_filter.filter(*map_cloud_);
    if (map_cloud_->size() < 100) {
      throw std::runtime_error("downsampled ICP map has too few points");
    }
    RCLCPP_INFO(get_logger(), "Fused ICP map: %lu -> %lu points",
                full_map->size(), map_cloud_->size());
  }

  void publishMap() {
    if (!map_publisher_) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*map_cloud_, message);
    message.header.frame_id = world_frame_;
    message.header.stamp = now();
    map_publisher_->publish(message);
  }

  void onInitialPose(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    if ((!message->header.frame_id.empty() &&
         message->header.frame_id != world_frame_) ||
        !finitePose(message->pose.pose)) {
      RCLCPP_ERROR(get_logger(), "Ignoring invalid fused ICP initial pose");
      return;
    }
    const auto &position = message->pose.pose.position;
    const auto &orientation = message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    if (quaternion.norm() < 1e-9) {
      RCLCPP_ERROR(get_logger(),
                   "Ignoring fused ICP initial pose with zero quaternion");
      return;
    }
    quaternion.normalize();
    last_pose_.setIdentity();
    last_pose_.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    last_pose_(0, 3) = position.x;
    last_pose_(1, 3) = position.y;
    last_pose_(2, 3) = position.z;
    prediction_buffer_.clear();
    gate_.reset();
    initialized_ = true;
    RCLCPP_INFO(
        get_logger(),
        "Fused ICP acquisition started at (%.2f, %.2f, %.2f); "
        "limits %.2fm %.2frad, confirmations=%d",
        position.x, position.y, position.z,
        acquisition_max_translation_delta_, acquisition_max_yaw_delta_,
        acquisition_confirmation_count_);
  }

  void onPrediction(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    if (!initialized_ ||
        (!message->header.frame_id.empty() &&
         message->header.frame_id != world_frame_) ||
        !finitePose(message->pose.pose)) {
      return;
    }
    const auto &position = message->pose.pose.position;
    const auto &orientation = message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    if (quaternion.norm() < 1e-9) {
      return;
    }
    quaternion.normalize();
    TimedPose prediction;
    prediction.stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (prediction.stamp_ns <= 0) {
      prediction.stamp_ns = now().nanoseconds();
    }
    prediction.pose.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    prediction.pose(0, 3) = position.x;
    prediction.pose(1, 3) = position.y;
    prediction.pose(2, 3) = position.z;
    if (!prediction_buffer_.empty() &&
        prediction.stamp_ns <= prediction_buffer_.back().stamp_ns) {
      if (prediction.stamp_ns == prediction_buffer_.back().stamp_ns) {
        prediction_buffer_.back() = prediction;
        return;
      }
      prediction_buffer_.clear();
    }
    prediction_buffer_.push_back(prediction);
    const int64_t keep_after_ns =
        prediction.stamp_ns -
        static_cast<int64_t>(prediction_buffer_duration_sec_ * 1e9);
    while (prediction_buffer_.size() > 2 &&
           prediction_buffer_[1].stamp_ns < keep_after_ns) {
      prediction_buffer_.pop_front();
    }
  }

  bool predictionAt(int64_t target_stamp_ns, Eigen::Matrix4d *pose) const {
    if (pose == nullptr || prediction_buffer_.empty()) {
      return false;
    }
    const TimedPose *nearest = &prediction_buffer_.front();
    int64_t nearest_delta = std::abs(nearest->stamp_ns - target_stamp_ns);
    for (const auto &candidate : prediction_buffer_) {
      const int64_t delta = std::abs(candidate.stamp_ns - target_stamp_ns);
      if (delta < nearest_delta) {
        nearest = &candidate;
        nearest_delta = delta;
      }
    }
    if (nearest_delta >
        static_cast<int64_t>(prediction_timeout_sec_ * 1e9)) {
      return false;
    }
    *pose = nearest->pose;
    return true;
  }

  void onImu(sensor_msgs::msg::Imu::SharedPtr message) {
    const int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (stamp_ns <= 0) {
      return;
    }
    const Eigen::Vector3d angular_velocity(
        message->angular_velocity.x, message->angular_velocity.y,
        message->angular_velocity.z);
    const Eigen::Vector3d acceleration(
        message->linear_acceleration.x, message->linear_acceleration.y,
        message->linear_acceleration.z);
    if (!angular_velocity.allFinite() || !acceleration.allFinite()) {
      return;
    }
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (last_imu_stamp_ns_ != 0 && stamp_ns <= last_imu_stamp_ns_) {
      if (stamp_ns == last_imu_stamp_ns_) {
        return;
      }
      RCLCPP_WARN(get_logger(),
                  "Livox IMU time moved backwards; clearing fused ICP buffers");
      imu_buffer_.clear();
      pending_scans_.clear();
      if (!leveling_ready_) {
        resetLevelingAccumulator();
      }
    }
    imu_buffer_.push_back({stamp_ns, angular_velocity});
    last_imu_stamp_ns_ = stamp_ns;

    if (!leveling_ready_) {
      if (angular_velocity.norm() > imu_init_max_gyro_) {
        resetLevelingAccumulator();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Keep robot still for fused ICP IMU initialization (gyro %.3f)",
            angular_velocity.norm());
        pruneImuBuffer();
        return;
      }
      acceleration_sum_ += acceleration;
      angular_velocity_sum_ += angular_velocity;
      ++imu_initialization_count_;
      if (imu_initialization_count_ >= kImuInitializationSamples) {
        finishLeveling();
      }
    }

    pruneImuBuffer();
  }

  void resetLevelingAccumulator() {
    imu_initialization_count_ = 0;
    acceleration_sum_.setZero();
    angular_velocity_sum_.setZero();
  }

  void finishLeveling() {
    const Eigen::Vector3d acceleration_lidar =
        rotation_lidar_from_imu_ * acceleration_sum_;
    if (acceleration_lidar.norm() < 1e-6) {
      resetLevelingAccumulator();
      return;
    }
    const Eigen::Vector3d up = acceleration_lidar.normalized();
    const Eigen::Quaterniond leveling = Eigen::Quaterniond::FromTwoVectors(
        up, Eigen::Vector3d::UnitZ());
    leveling_rotation_.setIdentity();
    leveling_rotation_.block<3, 3>(0, 0) = leveling.toRotationMatrix();
    gyro_bias_ = angular_velocity_sum_ /
                 static_cast<double>(imu_initialization_count_);
    leveling_ready_ = true;
    const double angle_degrees =
        2.0 * std::acos(std::clamp(leveling.w(), -1.0, 1.0)) * 180.0 /
        M_PI;
    RCLCPP_INFO(get_logger(),
                "Fused ICP IMU leveling ready: %.1f deg", angle_degrees);
    RCLCPP_INFO(get_logger(),
                "Waiting for /initialpose through the fusion bridge");
  }

  bool scanTimeRange(const livox_ros_driver2::msg::CustomMsg &message,
                     int64_t *start_ns, int64_t *end_ns) const {
    if (message.points.empty() || start_ns == nullptr || end_ns == nullptr) {
      return false;
    }
    const uint64_t base_time =
        message.timebase != 0
            ? message.timebase
            : static_cast<uint64_t>(
                  rclcpp::Time(message.header.stamp).nanoseconds());
    const auto max_offset = std::max_element(
        message.points.begin(), message.points.end(),
        [](const auto &left, const auto &right) {
          return left.offset_time < right.offset_time;
        })->offset_time;
    if (base_time == 0 ||
        base_time >
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) -
                max_offset) {
      return false;
    }
    *start_ns = static_cast<int64_t>(base_time);
    *end_ns = static_cast<int64_t>(base_time + max_offset);
    return true;
  }

  void onScan(livox_ros_driver2::msg::CustomMsg::SharedPtr message) {
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    if (!scanTimeRange(*message, &start_ns, &end_ns)) {
      return;
    }
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!leveling_ready_) {
      return;
    }
    if (last_scan_stamp_ns_ != 0 && start_ns < last_scan_stamp_ns_) {
      pending_scans_.clear();
      RCLCPP_WARN(get_logger(),
                  "Livox scan time moved backwards; clearing fused ICP queue");
    }
    last_scan_stamp_ns_ = start_ns;
    if (!pending_scans_.empty()) {
      pending_scans_.pop_front();
      RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Fused ICP busy; replacing stale scan with newest");
    }
    pending_scans_.push_back(std::move(message));
  }

  void processPendingScan() {
    livox_ros_driver2::msg::CustomMsg::SharedPtr scan_message;
    std::deque<AngularVelocitySample> imu_snapshot;
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (pending_scans_.empty() || imu_buffer_.empty()) {
        return;
      }
      if (!scanTimeRange(*pending_scans_.front(), &start_ns, &end_ns)) {
        pending_scans_.pop_front();
        return;
      }
      const double duration = static_cast<double>(end_ns - start_ns) * 1e-9;
      if (duration > max_scan_duration_sec_) {
        pending_scans_.pop_front();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Dropping fused ICP scan with %.1fms duration", duration * 1000.0);
        return;
      }
      if (deskew_enabled_ && end_ns > imu_buffer_.back().stamp_ns) {
        return;
      }
      scan_message = pending_scans_.front();
      pending_scans_.pop_front();
      imu_snapshot = imu_buffer_;
    }
    const double age_sec =
        static_cast<double>(now().nanoseconds() - end_ns) * 1e-9;
    if (age_sec > max_scan_age_sec_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Dropping stale fused ICP scan (age %.3fs)", age_sec);
      return;
    }
    processScan(scan_message, start_ns, end_ns, imu_snapshot);
  }

  void processScan(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr &message,
      int64_t start_ns, int64_t end_ns,
      const std::deque<AngularVelocitySample> &imu_samples) {
    RotationDeskewer deskewer;
    const bool has_duration = end_ns > start_ns;
    if (deskew_enabled_ && has_duration) {
      std::string error;
      if (!deskewer.build(imu_samples, start_ns, end_ns, gyro_bias_,
                          rotation_lidar_from_imu_, max_imu_gap_sec_, &error)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Fused ICP deskew unavailable: %s",
                             error.c_str());
        return;
      }
    }

    auto scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    scan->reserve(message->points.size());
    for (const auto &point_message : message->points) {
      if (!std::isfinite(point_message.x) ||
          !std::isfinite(point_message.y) ||
          !std::isfinite(point_message.z)) {
        continue;
      }
      Eigen::Vector3d point(point_message.x, point_message.y, point_message.z);
      if (deskew_enabled_ && has_duration) {
        point = deskewer.compensate(point, start_ns + point_message.offset_time);
      }
      scan->emplace_back(static_cast<float>(point.x()),
                         static_cast<float>(point.y()),
                         static_cast<float>(point.z()));
    }
    if (scan->size() < 50) {
      return;
    }

    const rclcpp::Time scan_stamp(end_ns);
    sensor_msgs::msg::PointCloud2 raw_message;
    pcl::toROSMsg(*scan, raw_message);
    raw_message.header.frame_id = lidar_frame_;
    raw_message.header.stamp = scan_stamp;
    raw_scan_publisher_->publish(raw_message);

    pcl::transformPointCloud(*scan, *scan, leveling_rotation_);
    sensor_msgs::msg::PointCloud2 leveled_message;
    pcl::toROSMsg(*scan, leveled_message);
    leveled_message.header.frame_id = tracking_frame_;
    leveled_message.header.stamp = scan_stamp;
    leveled_scan_publisher_->publish(leveled_message);

    if (!initialized_) {
      return;
    }

    auto downsampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(scan);
    voxel_filter.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    voxel_filter.filter(*downsampled);
    if (downsampled->size() < 50) {
      return;
    }

    const bool acquiring = !gate_.locked();
    const double correspondence_distance =
        acquiring ? acquisition_max_corr_dist_ : max_corr_dist_;
    const int iterations =
        acquiring ? acquisition_max_iterations_ : max_iterations_;

    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> matcher;
    matcher.setInputSource(downsampled);
    matcher.setInputTarget(map_cloud_);
    matcher.setMaxCorrespondenceDistance(correspondence_distance);
    matcher.setMaximumIterations(iterations);
    matcher.setNumThreads(4);

    Eigen::Matrix4d initial_guess = last_pose_;
    Eigen::Matrix4d timed_prediction;
    if (!acquiring && predictionAt(end_ns, &timed_prediction)) {
      // During a turn, a current-time prediction can be tens of degrees ahead
      // of the scan being matched. Select the buffered odometry prediction at
      // the LiDAR timestamp instead.
      initial_guess = timed_prediction;
    }

    auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    matcher.align(*aligned, initial_guess.cast<float>());
    if (!matcher.hasConverged()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Fused ICP did not converge");
      return;
    }

    const Eigen::Matrix4d candidate =
        matcher.getFinalTransformation().cast<double>();
    const Eigen::Matrix3d candidate_rotation =
        candidate.block<3, 3>(0, 0);
    const Eigen::Matrix3d initial_rotation =
        initial_guess.block<3, 3>(0, 0);
    const Eigen::Matrix3d rotation_orthogonality =
        candidate_rotation * candidate_rotation.transpose();
    if (!candidate.allFinite() ||
        !rotation_orthogonality.isApprox(Eigen::Matrix3d::Identity(), 1e-3) ||
        std::abs(candidate_rotation.determinant() - 1.0) > 1e-3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Reject fused ICP with invalid 3D transform");
      return;
    }
    const Eigen::Vector3d translation =
        candidate.block<3, 1>(0, 3) - initial_guess.block<3, 1>(0, 3);
    const double translation_delta = translation.head<2>().norm();
    const double translation_z_delta = std::abs(translation.z());
    const double yaw_delta = std::abs(normalizeAngle(
        yawFromPose(candidate) - yawFromPose(initial_guess)));
    const double tilt = std::acos(std::clamp(
        candidate_rotation.col(2).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0));
    const Eigen::AngleAxisd rotation_change(
        initial_rotation.transpose() * candidate_rotation);
    const double rotation_delta = std::abs(rotation_change.angle());
    const double translation_z_limit =
        acquiring ? acquisition_max_translation_z_delta_
                  : max_translation_z_delta_;
    const double tilt_limit = acquiring ? acquisition_max_tilt_ : max_tilt_;
    const double rotation_limit =
        acquiring ? acquisition_max_rotation_delta_ : max_rotation_delta_;
    if (translation_z_delta > translation_z_limit || tilt > tilt_limit ||
        rotation_delta > rotation_limit) {
      // FastGICP is a full 6-DoF optimizer. In sparse or locally planar
      // geometry it can return a 90/180-degree roll or pitch solution whose
      // projected yaw still appears valid. Never let such a solution poison
      // last_pose_; the bridge's innovation gate is a second line of defence,
      // not a substitute for validating the matcher state itself.
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Reject fused ICP 3D jump: z=%.3fm tilt=%.3frad rotation=%.3frad "
          "mode=%s",
          translation_z_delta, tilt, rotation_delta,
          acquiring ? "acquisition" : "tracking");
      return;
    }
    const double fitness = matcher.getFitnessScore(correspondence_distance);
    const IcpGateDecision decision =
        gate_.evaluate(translation_delta, yaw_delta, fitness);
    if (!decision.update_candidate) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Reject fused ICP: xy=%.3fm yaw=%.3frad fitness=%.3f mode=%s",
          translation_delta, yaw_delta, fitness,
          acquiring ? "acquisition" : "tracking");
      return;
    }

    last_pose_ = candidate;
    if (decision.restarted_confirmation) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Fused ICP acquisition moved; restarting confirmation");
    } else if (!decision.publish_pose) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Fused ICP acquisition candidate %d/%d",
          gate_.confirmationCount(), gate_.requiredConfirmations());
    }
    if (decision.just_locked) {
      RCLCPP_INFO(get_logger(),
                  "Fused ICP acquisition locked; enabling EKF prediction mode");
    }
    if (decision.publish_pose) {
      publishPose(last_pose_, scan_stamp);
    }
  }

  void publishPose(const Eigen::Matrix4d &pose,
                   const rclcpp::Time &stamp) {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.pose.pose.position.x = pose(0, 3);
    message.pose.pose.position.y = pose(1, 3);
    message.pose.pose.position.z = pose(2, 3);
    const Eigen::Quaterniond orientation(pose.block<3, 3>(0, 0));
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    message.pose.covariance[0] = 0.01;
    message.pose.covariance[7] = 0.01;
    message.pose.covariance[14] = 0.01;
    message.pose.covariance[21] = 0.01;
    message.pose.covariance[28] = 0.01;
    message.pose.covariance[35] = 0.01;
    pose_publisher_->publish(message);
  }

  void pruneImuBuffer() {
    if (imu_buffer_.size() < 3) {
      return;
    }
    int64_t keep_after_ns =
        imu_buffer_.back().stamp_ns -
        static_cast<int64_t>(imu_buffer_duration_sec_ * 1e9);
    if (!pending_scans_.empty()) {
      int64_t pending_start_ns = 0;
      int64_t pending_end_ns = 0;
      if (scanTimeRange(*pending_scans_.front(), &pending_start_ns,
                        &pending_end_ns)) {
        keep_after_ns = std::min(
            keep_after_ns,
            pending_start_ns -
                static_cast<int64_t>(max_imu_gap_sec_ * 1e9));
      }
    }
    while (imu_buffer_.size() > 2 &&
           imu_buffer_[1].stamp_ns < keep_after_ns) {
      imu_buffer_.pop_front();
    }
  }

  std::string map_path_;
  std::string scan_topic_;
  std::string imu_topic_;
  std::string world_frame_;
  std::string tracking_frame_;
  std::string lidar_frame_;
  std::string initial_pose_topic_;
  std::string prediction_topic_;
  std::string output_pose_topic_;
  std::string map_cloud_topic_;
  std::string raw_scan_topic_;
  std::string leveled_scan_topic_;

  double voxel_leaf_{0.15};
  double max_corr_dist_{1.5};
  double max_translation_delta_{0.45};
  double max_translation_z_delta_{0.35};
  double max_yaw_delta_{0.60};
  double max_tilt_{0.35};
  double max_rotation_delta_{0.70};
  double max_fitness_score_{1.0};
  int max_iterations_{15};
  double acquisition_max_corr_dist_{3.0};
  double acquisition_max_translation_delta_{2.5};
  double acquisition_max_translation_z_delta_{1.0};
  double acquisition_max_yaw_delta_{1.57};
  double acquisition_max_tilt_{0.35};
  double acquisition_max_rotation_delta_{1.80};
  double acquisition_max_fitness_score_{1.0};
  int acquisition_max_iterations_{50};
  int acquisition_confirmation_count_{3};
  double prediction_timeout_sec_{0.25};
  double prediction_buffer_duration_sec_{2.0};
  bool deskew_enabled_{true};
  double max_imu_gap_sec_{0.02};
  double max_scan_duration_sec_{0.15};
  double max_scan_age_sec_{0.50};
  double imu_buffer_duration_sec_{2.0};
  double imu_init_max_gyro_{0.10};

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  FusedIcpGate gate_;
  Eigen::Matrix4d last_pose_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d leveling_rotation_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix3d rotation_lidar_from_imu_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d acceleration_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  std::deque<AngularVelocitySample> imu_buffer_;
  std::deque<TimedPose> prediction_buffer_;
  std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> pending_scans_;
  std::mutex sensor_mutex_;
  bool leveling_ready_{false};
  bool initialized_{false};
  int imu_initialization_count_{0};
  int64_t last_imu_stamp_ns_{0};
  int64_t last_scan_stamp_ns_{0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      prediction_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      raw_scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      leveled_scan_publisher_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr scan_processing_timer_;
};

}  // namespace go2_localization

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<go2_localization::FusedIcpMatcher>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("fused_icp_matcher"), "%s", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
