#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace fs = std::filesystem;

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct VoxelAccumulator {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::size_t count = 0;
};

class MapCaptureNode : public rclcpp::Node {
 public:
  MapCaptureNode() : Node("map_capture") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/fastlio2/world_cloud");
    output_directory_ = declare_parameter<std::string>("output_directory", ".");
    file_prefix_ = declare_parameter<std::string>("file_prefix", "mapping_scans");
    latest_name_ = declare_parameter<std::string>("latest_name", "scans.pcd");
    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.08);
    log_every_n_scans_ = std::max(
        1, static_cast<int>(declare_parameter<int64_t>("compact_every_n_scans", 20)));
    queue_depth_ = std::max(
        10, static_cast<int>(declare_parameter<int64_t>("queue_depth", 100)));
    if (!std::isfinite(voxel_leaf_) || voxel_leaf_ <= 0.0) {
      throw std::invalid_argument("voxel_leaf must be positive");
    }

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::QoS(rclcpp::KeepLast(queue_depth_)).best_effort(),
        std::bind(&MapCaptureNode::cloudCallback, this, std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
        "save", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = saveCapture(response->message);
        });

    RCLCPP_INFO(get_logger(), "Map capture ready: %s -> %s", input_topic_.c_str(),
                output_directory_.c_str());
  }

  ~MapCaptureNode() override {
    std::string result;
    if (!saved_ && !saveCapture(result)) {
      RCLCPP_WARN(get_logger(), "Map capture was not saved: %s", result.c_str());
    }
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    Cloud incoming;
    pcl::fromROSMsg(*message, incoming);
    incoming.erase(
        std::remove_if(incoming.begin(), incoming.end(), [](const Point &point) {
          return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
        }),
        incoming.end());
    if (incoming.empty()) {
      return;
    }

    std::lock_guard<std::mutex> guard(mutex_);
    for (const Point &point : incoming) {
      const VoxelKey key{
          static_cast<std::int64_t>(std::floor(point.x / voxel_leaf_)),
          static_cast<std::int64_t>(std::floor(point.y / voxel_leaf_)),
          static_cast<std::int64_t>(std::floor(point.z / voxel_leaf_)),
      };
      VoxelAccumulator &voxel = voxels_[key];
      voxel.x += point.x;
      voxel.y += point.y;
      voxel.z += point.z;
      ++voxel.count;
    }
    input_points_ += incoming.size();
    ++scan_count_;
    if (scan_count_ % static_cast<std::size_t>(log_every_n_scans_) == 0) {
      RCLCPP_INFO(get_logger(), "Map capture: scans=%zu input=%zu retained=%zu",
                  scan_count_, input_points_, voxels_.size());
    }
  }

  Cloud::Ptr snapshotLocked() const {
    Cloud::Ptr snapshot(new Cloud);
    snapshot->reserve(voxels_.size());
    for (const auto &[key, voxel] : voxels_) {
      (void)key;
      if (voxel.count == 0) {
        continue;
      }
      Point point;
      const double count = static_cast<double>(voxel.count);
      point.x = static_cast<float>(voxel.x / count);
      point.y = static_cast<float>(voxel.y / count);
      point.z = static_cast<float>(voxel.z / count);
      snapshot->push_back(point);
    }
    return snapshot;
  }

  static std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t value = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&value, &local);
    std::ostringstream stream;
    stream << std::put_time(&local, "%Y%m%d_%H%M%S");
    return stream.str();
  }

  bool saveCapture(std::string &result) {
    Cloud::Ptr snapshot;
    std::size_t saved_scan_count = 0;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (saved_) {
        result = saved_path_.string();
        return true;
      }
      if (saving_) {
        result = "capture save is already in progress";
        return false;
      }
      snapshot = snapshotLocked();
      if (snapshot->empty()) {
        result = "no FAST-LIO2 world points were received";
        return false;
      }
      saved_scan_count = scan_count_;
      saving_ = true;
    }

    fs::path temporary_path;
    fs::path temporary_link;
    try {
      fs::create_directories(output_directory_);
      fs::path final_path = fs::path(output_directory_) / (file_prefix_ + "_" + timestamp() + ".pcd");
      for (int suffix = 1; fs::exists(final_path); ++suffix) {
        final_path = fs::path(output_directory_) /
                     (file_prefix_ + "_" + timestamp() + "_" + std::to_string(suffix) + ".pcd");
      }
      temporary_path = final_path.parent_path() / ("." + final_path.filename().string() + ".tmp");
      if (pcl::io::savePCDFileBinary(temporary_path.string(), *snapshot) != 0) {
        std::error_code cleanup_error;
        fs::remove(temporary_path, cleanup_error);
        result = "PCL failed to write the capture";
        std::lock_guard<std::mutex> guard(mutex_);
        saving_ = false;
        return false;
      }
      fs::rename(temporary_path, final_path);

      if (!latest_name_.empty()) {
        const fs::path latest_path = final_path.parent_path() / latest_name_;
        temporary_link = final_path.parent_path() / ("." + latest_name_ + ".tmp");
        std::error_code error;
        fs::remove(temporary_link, error);
        fs::create_symlink(final_path.filename(), temporary_link);
        fs::rename(temporary_link, latest_path);
      }

      {
        std::lock_guard<std::mutex> guard(mutex_);
        saved_ = true;
        saving_ = false;
        saved_path_ = final_path;
      }
      result = final_path.string();
      RCLCPP_INFO(get_logger(), "MAPPING_CAPTURE_SAVED path=%s points=%zu scans=%zu",
                  result.c_str(), snapshot->size(), saved_scan_count);
      return true;
    } catch (const std::exception &error) {
      std::error_code cleanup_error;
      if (!temporary_path.empty()) {
        fs::remove(temporary_path, cleanup_error);
      }
      if (!temporary_link.empty()) {
        fs::remove(temporary_link, cleanup_error);
      }
      std::lock_guard<std::mutex> guard(mutex_);
      saving_ = false;
      result = error.what();
      return false;
    }
  }

  std::mutex mutex_;
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_;
  std::string input_topic_;
  std::string output_directory_;
  std::string file_prefix_;
  std::string latest_name_;
  double voxel_leaf_ = 0.08;
  int log_every_n_scans_ = 20;
  int queue_depth_ = 100;
  std::size_t scan_count_ = 0;
  std::size_t input_points_ = 0;
  bool saved_ = false;
  bool saving_ = false;
  fs::path saved_path_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapCaptureNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
