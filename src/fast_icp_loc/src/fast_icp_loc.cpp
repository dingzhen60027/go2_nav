#include "fast_icp_loc/fast_icp_loc.hpp"
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>

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
  world_frame_   = this->declare_parameter("world_frame", "camera_init");
  body_frame_    = this->declare_parameter("body_frame", "livox_frame");
  voxel_leaf_    = this->declare_parameter("voxel_leaf", 0.15);
  max_corr_dist_ = this->declare_parameter("max_corr_dist", 2.0);
  max_translation_delta_ = this->declare_parameter("max_translation_delta", 0.45);
  max_yaw_delta_ = this->declare_parameter("max_yaw_delta", 0.60);
  max_fitness_score_ = this->declare_parameter("max_fitness_score", 1.0);
  max_iter_      = this->declare_parameter("max_iterations", 30);

  loadMap();

  // IMU — 自动测倾角，采集完自动取消订阅
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", rclcpp::SensorDataQoS(),
      std::bind(&FastIcpLoc::imuCallback, this, std::placeholders::_1));
  imu_acc_sum_ = Eigen::Vector3d::Zero();
  R_level_ = Eigen::Matrix4d::Identity();

  scan_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastIcpLoc::scanCallback, this, std::placeholders::_1));

  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&FastIcpLoc::initPoseCallback, this, std::placeholders::_1));

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
  if (leveling_done_) return;

  imu_acc_sum_ += Eigen::Vector3d(msg->linear_acceleration.x,
                                   msg->linear_acceleration.y,
                                   msg->linear_acceleration.z);
  imu_count_++;

  if (imu_count_ >= IMU_INIT_COUNT) {
    // 计算校平旋转：IMU 的"上"方向 → 世界 +z
    Eigen::Vector3d up = imu_acc_sum_.normalized();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(up, Eigen::Vector3d::UnitZ());
    R_level_.block<3,3>(0,0) = q.toRotationMatrix();

    leveling_done_ = true;
    // 用完就取消 IMU 订阅
    imu_sub_.reset();

    double angle = std::acos(q.w()) * 2.0 * 180.0 / M_PI;
    RCLCPP_INFO(get_logger(), "IMU leveling done: %.1f deg, rotation applied to scans", angle);
    RCLCPP_INFO(get_logger(), "Waiting for initial pose via /initialpose (RViz 2D Pose Estimate)...");
  }
}

void FastIcpLoc::initPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  const auto &p = msg->pose.pose.position;
  const auto &o = msg->pose.pose.orientation;
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  last_pose_.block<3,3>(0,0) = q.toRotationMatrix();
  last_pose_(0,3) = p.x;
  last_pose_(1,3) = p.y;
  last_pose_(2,3) = p.z;
  localized_ = true;
  RCLCPP_INFO(get_logger(), "Initial pose: (%.2f, %.2f, %.2f), start tracking", p.x, p.y, p.z);
}

void FastIcpLoc::scanCallback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  if (!map_loaded_ || !leveling_done_) return;

  // CustomMsg → PointXYZ
  auto scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  scan->reserve(msg->points.size());
  for (const auto &p : msg->points) {
    scan->emplace_back(p.x, p.y, p.z);
  }

  // 始终发布 PointCloud2（RViz + Nav2 都需要，不管是否已定位）
  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*scan, scan_msg);
  scan_msg.header.frame_id = "livox_frame";
  scan_msg.header.stamp = this->now();
  scan_pub_->publish(scan_msg);

  // ---- 校平：旋转实时扫描使其与 PCD 地图坐标系对齐 ----
  pcl::transformPointCloud(*scan, *scan, R_level_);

  // 发布校平后的 PointCloud2（RViz 显示用）
  sensor_msgs::msg::PointCloud2 level_msg;
  pcl::toROSMsg(*scan, level_msg);
  level_msg.header.frame_id = "base_link";
  level_msg.header.stamp = this->now();
  leveled_pub_->publish(level_msg);

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

  auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  icp.align(*aligned, last_pose_.cast<float>());

  if (icp.hasConverged()) {
    const Eigen::Matrix4d candidate = icp.getFinalTransformation().cast<double>();
    const Eigen::Vector3d delta_t =
        candidate.block<3, 1>(0, 3) - last_pose_.block<3, 1>(0, 3);
    const double translation_delta = delta_t.head<2>().norm();
    const double yaw_delta =
        std::abs(normalizeAngle(yawFromPose(candidate) - yawFromPose(last_pose_)));
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
    }
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "ICP not converged, keeping last pose");
  }

  publishPose(last_pose_);
}

void FastIcpLoc::publishPose(const Eigen::Matrix4d &T) {
  auto stamp = this->now();

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
