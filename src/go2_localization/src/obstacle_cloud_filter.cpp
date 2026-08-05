#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace go2_localization {

class ObstacleCloudFilter final : public rclcpp::Node {
 public:
  ObstacleCloudFilter() : Node("obstacle_cloud_filter") {
    input_topic_ = declare_parameter<std::string>(
        "input_topic", "/scan_leveled");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/scan_obstacles");
    ground_filter_enabled_ = declare_parameter<bool>(
        "ground_filter_enabled", true);
    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.08);
    ground_distance_threshold_ = declare_parameter<double>(
        "ground_distance_threshold", 0.06);
    ground_max_tilt_deg_ = declare_parameter<double>(
        "ground_max_tilt_deg", 18.0);
    ground_z_min_ = declare_parameter<double>("ground_z_min", -0.80);
    ground_z_max_ = declare_parameter<double>("ground_z_max", -0.15);
    min_ground_inliers_ = declare_parameter<int>("min_ground_inliers", 80);

    if (!std::isfinite(voxel_leaf_) || voxel_leaf_ <= 0.0 ||
        !std::isfinite(ground_distance_threshold_) ||
        ground_distance_threshold_ <= 0.0 ||
        !std::isfinite(ground_max_tilt_deg_) || ground_max_tilt_deg_ <= 0.0 ||
        ground_max_tilt_deg_ >= 90.0 || !std::isfinite(ground_z_min_) ||
        !std::isfinite(ground_z_max_) || ground_z_min_ >= ground_z_max_ ||
        min_ground_inliers_ < 10) {
      throw std::invalid_argument("invalid obstacle cloud filter parameters");
    }

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ObstacleCloudFilter::onCloud, this,
                  std::placeholders::_1));
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
        get_logger(),
        "Obstacle cloud filter: %s -> %s, ground=%s, distance=%.3fm, "
        "max_tilt=%.1fdeg, z=[%.2f, %.2f]",
        input_topic_.c_str(), output_topic_.c_str(),
        ground_filter_enabled_ ? "on" : "off", ground_distance_threshold_,
        ground_max_tilt_deg_, ground_z_min_, ground_z_max_);
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    Cloud::Ptr input(new Cloud());
    pcl::fromROSMsg(*message, *input);
    input->erase(std::remove_if(input->begin(), input->end(),
                                [](const Point &point) {
                                  return !std::isfinite(point.x) ||
                                         !std::isfinite(point.y) ||
                                         !std::isfinite(point.z);
                                }),
                 input->end());
    if (input->empty()) {
      return;
    }

    Cloud::Ptr filtered(new Cloud());
    filtered->reserve(input->size());
    bool removed_ground = false;
    std::size_t ground_inliers = 0;

    if (ground_filter_enabled_) {
      Cloud::Ptr sampled(new Cloud());
      pcl::VoxelGrid<Point> voxel;
      voxel.setInputCloud(input);
      voxel.setLeafSize(static_cast<float>(voxel_leaf_),
                        static_cast<float>(voxel_leaf_),
                        static_cast<float>(voxel_leaf_));
      voxel.filter(*sampled);

      // Only let points near the expected floor height vote for the plane.
      // Otherwise a dense horizontal ceiling can win RANSAC and prevent the
      // actual floor from being removed.
      Cloud::Ptr plane_candidates(new Cloud());
      plane_candidates->reserve(sampled->size());
      for (const auto &point : *sampled) {
        if (point.z >= ground_z_min_ && point.z <= ground_z_max_) {
          plane_candidates->push_back(point);
        }
      }

      pcl::SACSegmentation<Point> segmentation;
      segmentation.setOptimizeCoefficients(true);
      segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setAxis(Eigen::Vector3f::UnitZ());
      segmentation.setEpsAngle(
          static_cast<double>(ground_max_tilt_deg_) * M_PI / 180.0);
      segmentation.setDistanceThreshold(ground_distance_threshold_);
      segmentation.setMaxIterations(100);
      segmentation.setInputCloud(plane_candidates);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      if (plane_candidates->size() >=
          static_cast<std::size_t>(min_ground_inliers_)) {
        segmentation.segment(*inliers, *coefficients);
      }
      if (coefficients->values.size() >= 4 &&
          static_cast<int>(inliers->indices.size()) >= min_ground_inliers_) {
        const double a = coefficients->values[0];
        const double b = coefficients->values[1];
        const double c = coefficients->values[2];
        const double d = coefficients->values[3];
        const double normal_norm = std::sqrt(a * a + b * b + c * c);
        const double origin_z =
            std::abs(c) > 1.0e-6
                ? -d / c
                : std::numeric_limits<double>::quiet_NaN();
        const double normal_z =
            normal_norm > 1.0e-6 ? std::abs(c) / normal_norm : 0.0;
        if (std::isfinite(origin_z) && origin_z >= ground_z_min_ &&
            origin_z <= ground_z_max_ && normal_z >=
                std::cos(ground_max_tilt_deg_ * M_PI / 180.0)) {
          removed_ground = true;
          ground_inliers = inliers->indices.size();
          filtered->reserve(input->size());
          for (const auto &point : *input) {
            const double distance =
                std::abs(a * point.x + b * point.y + c * point.z + d) /
                normal_norm;
            if (distance > ground_distance_threshold_) {
              filtered->push_back(point);
            }
          }
        }
      }
    }

    if (!removed_ground) {
      *filtered = *input;
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = message->header;
    publisher_->publish(output);

    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Filtered obstacle cloud: input=%zu output=%zu ground_inliers=%zu",
        input->size(), filtered->size(), ground_inliers);
  }

  std::string input_topic_;
  std::string output_topic_;
  bool ground_filter_enabled_{true};
  double voxel_leaf_{0.08};
  double ground_distance_threshold_{0.06};
  double ground_max_tilt_deg_{18.0};
  double ground_z_min_{-0.80};
  double ground_z_max_{-0.15};
  int min_ground_inliers_{80};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace go2_localization

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_localization::ObstacleCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
