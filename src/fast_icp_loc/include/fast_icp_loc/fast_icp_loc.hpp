#pragma once
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
#include <memory>

namespace fast_icp_loc {

class FastIcpLoc : public rclcpp::Node {
public:
  explicit FastIcpLoc(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~FastIcpLoc() = default;

private:
  void loadMap();
  void scanCallback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void initPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);
  void publishPose(const Eigen::Matrix4d &T);

  // ---- params ----
  std::string map_pcd_path_;
  std::string scan_topic_;
  std::string world_frame_;
  std::string body_frame_;
  double voxel_leaf_;
  double max_corr_dist_;
  double max_translation_delta_;
  double max_yaw_delta_;
  double max_fitness_score_;
  int max_iter_;

  // ---- map ----
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ds_;

  // ---- state ----
  Eigen::Matrix4d last_pose_;
  Eigen::Matrix4d R_level_;        // 实时点云校平旋转矩阵
  bool map_loaded_;
  bool localized_;
  bool leveling_done_;
  int imu_count_;
  Eigen::Vector3d imu_acc_sum_;

  // ---- pubs/subs ----
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
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
