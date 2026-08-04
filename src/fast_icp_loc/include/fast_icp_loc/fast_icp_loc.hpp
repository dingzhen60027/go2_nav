#pragma once
#include "fast_icp_loc/imu_deskewer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

namespace fast_icp_loc {

class FastIcpLoc : public rclcpp::Node {
public:
  explicit FastIcpLoc(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~FastIcpLoc() = default;

private:
  void loadMap();
  void scanCallback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void initPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void predictionPoseCallback(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);
  void processPendingScans();
  void processScan(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg,
                   int64_t scan_start_ns, int64_t scan_end_ns);
  bool scanTimeRange(const livox_ros_driver2::msg::CustomMsg &msg,
                     int64_t *scan_start_ns, int64_t *scan_end_ns) const;
  void pruneImuBuffer();
  void publishPose(const Eigen::Matrix4d &T, const rclcpp::Time &stamp);

  // ---- params ----
  std::string map_pcd_path_;
  std::string scan_topic_;
  std::string imu_topic_;
  std::string world_frame_;
  std::string body_frame_;
  std::string lidar_frame_;
  std::string prediction_topic_;
  double voxel_leaf_;
  double max_corr_dist_;
  double max_translation_delta_;
  double max_yaw_delta_;
  double max_fitness_score_;
  bool deskew_enabled_;
  double max_imu_gap_sec_;
  double max_scan_duration_sec_;
  double imu_buffer_duration_sec_;
  double imu_init_max_gyro_;
  double prediction_timeout_sec_;
  bool publish_only_accepted_pose_;
  int max_pending_scans_;
  int max_iter_;

  // ---- map ----
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ds_;

  // ---- state ----
  Eigen::Matrix4d last_pose_;
  Eigen::Matrix4d prediction_pose_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d R_level_;        // 实时点云校平旋转矩阵
  bool map_loaded_;
  bool localized_;
  bool leveling_done_;
  int imu_count_;
  Eigen::Vector3d imu_acc_sum_;
  Eigen::Vector3d imu_gyro_sum_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Matrix3d rotation_lidar_from_imu_;
  std::deque<ImuSample> imu_buffer_;
  std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> pending_scans_;
  int64_t last_imu_stamp_ns_{0};
  int64_t last_scan_stamp_ns_{0};
  uint64_t deskewed_scan_count_{0};
  uint64_t dropped_scan_count_{0};
  int64_t prediction_received_ns_{0};

  // ---- pubs/subs ----
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      prediction_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr leveled_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace fast_icp_loc
