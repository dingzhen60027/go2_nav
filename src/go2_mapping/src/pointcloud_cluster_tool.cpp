#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

struct VoxelKey {
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = std::hash<std::int32_t>{}(key.x);
    seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<std::int32_t>{}(key.z) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

struct VoxelData {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::uint64_t count = 0;
  std::int32_t label = -1;
};

struct ClusterStats {
  std::int32_t id = -1;
  std::uint64_t voxel_count = 0;
  std::uint64_t point_count = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();
  std::array<std::uint8_t, 3> color{};
};

struct Options {
  std::string mode;
  std::string input;
  std::string output;
  std::string preview;
  std::string voxels;
  std::string clusters;
  std::string remove;
  double leaf = 0.15;
  double tolerance = 0.30;
  double z_min = 0.20;
  double z_max = 2.20;
  int min_voxels = 5;
};

template <typename T> void writeBinary(std::ofstream &stream, const T &value) {
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template <typename T> T readBinary(std::ifstream &stream) {
  T value{};
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  if (!stream) {
    throw std::runtime_error("cluster voxel file is truncated");
  }
  return value;
}

std::string requireValue(int argc, char **argv, int &index) {
  if (++index >= argc) {
    throw std::runtime_error(std::string("missing value for ") +
                             argv[index - 1]);
  }
  return argv[index];
}

Options parseOptions(int argc, char **argv) {
  if (argc < 2) {
    throw std::runtime_error(
        "usage: pointcloud_cluster_tool <cluster|filter> [options]");
  }
  Options options;
  options.mode = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--input") {
      options.input = requireValue(argc, argv, index);
    } else if (argument == "--output") {
      options.output = requireValue(argc, argv, index);
    } else if (argument == "--preview") {
      options.preview = requireValue(argc, argv, index);
    } else if (argument == "--voxels") {
      options.voxels = requireValue(argc, argv, index);
    } else if (argument == "--clusters") {
      options.clusters = requireValue(argc, argv, index);
    } else if (argument == "--remove") {
      options.remove = requireValue(argc, argv, index);
    } else if (argument == "--leaf") {
      options.leaf = std::stod(requireValue(argc, argv, index));
    } else if (argument == "--tolerance") {
      options.tolerance = std::stod(requireValue(argc, argv, index));
    } else if (argument == "--min-voxels") {
      options.min_voxels = std::stoi(requireValue(argc, argv, index));
    } else if (argument == "--z-min") {
      options.z_min = std::stod(requireValue(argc, argv, index));
    } else if (argument == "--z-max") {
      options.z_max = std::stod(requireValue(argc, argv, index));
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (options.input.empty() || options.voxels.empty()) {
    throw std::runtime_error("--input and --voxels are required");
  }
  if (options.mode == "cluster") {
    if (options.preview.empty() || options.clusters.empty()) {
      throw std::runtime_error(
          "cluster mode requires --preview and --clusters");
    }
    if (!std::isfinite(options.leaf) || options.leaf <= 0.0 ||
        !std::isfinite(options.tolerance) || options.tolerance <= 0.0 ||
        options.min_voxels < 1 || !std::isfinite(options.z_min) ||
        !std::isfinite(options.z_max) || options.z_min >= options.z_max) {
      throw std::runtime_error("invalid clustering parameters");
    }
  } else if (options.mode == "filter") {
    if (options.output.empty() || options.remove.empty()) {
      throw std::runtime_error("filter mode requires --output and --remove");
    }
  } else {
    throw std::runtime_error("mode must be cluster or filter");
  }
  return options;
}

VoxelKey voxelKey(const pcl::PointXYZ &point, double leaf) {
  return {
      static_cast<std::int32_t>(
          std::floor(static_cast<double>(point.x) / leaf)),
      static_cast<std::int32_t>(
          std::floor(static_cast<double>(point.y) / leaf)),
      static_cast<std::int32_t>(
          std::floor(static_cast<double>(point.z) / leaf)),
  };
}

bool isFinite(const pcl::PointXYZ &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

std::array<std::uint8_t, 3> clusterColor(std::int32_t id) {
  const double hue = std::fmod(id * 0.618033988749895, 1.0);
  const double h = hue * 6.0;
  const double c = 0.82;
  const double x = c * (1.0 - std::abs(std::fmod(h, 2.0) - 1.0));
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
  if (h < 1.0) {
    r = c;
    g = x;
  } else if (h < 2.0) {
    r = x;
    g = c;
  } else if (h < 3.0) {
    g = c;
    b = x;
  } else if (h < 4.0) {
    g = x;
    b = c;
  } else if (h < 5.0) {
    r = x;
    b = c;
  } else {
    r = c;
    b = x;
  }
  // Reserve vivid red for the Web editor's explicit "selected for removal"
  // state so an unselected cluster cannot look like it will be deleted.
  if (r > 0.72 && g < 0.30 && b < 0.30) {
    g = 0.52;
  }
  constexpr double offset = 0.18;
  return {
      static_cast<std::uint8_t>(std::round((r + offset) * 255.0)),
      static_cast<std::uint8_t>(std::round((g + offset) * 255.0)),
      static_cast<std::uint8_t>(std::round((b + offset) * 255.0)),
  };
}

void writePreview(const std::string &path,
                  const std::vector<std::pair<VoxelKey, VoxelData>> &voxels) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open cluster preview: " + path);
  }
  output << "ply\n"
         << "format binary_little_endian 1.0\n"
         << "comment generated by go2 pointcloud_cluster_tool\n"
         << "element vertex " << voxels.size() << "\n"
         << "property float x\nproperty float y\nproperty float z\n"
         << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
         << "property int cluster_id\nend_header\n";
  for (const auto &[key, voxel] : voxels) {
    (void)key;
    const float x = static_cast<float>(voxel.x / voxel.count);
    const float y = static_cast<float>(voxel.y / voxel.count);
    const float z = static_cast<float>(voxel.z / voxel.count);
    const auto color = voxel.label >= 0
                           ? clusterColor(voxel.label)
                           : std::array<std::uint8_t, 3>{92, 104, 120};
    writeBinary(output, x);
    writeBinary(output, y);
    writeBinary(output, z);
    output.write(reinterpret_cast<const char *>(color.data()), color.size());
    writeBinary(output, voxel.label);
  }
  if (!output) {
    throw std::runtime_error("failed to write cluster preview: " + path);
  }
}

void writeVoxelLabels(
    const std::string &path, double leaf,
    const std::vector<std::pair<VoxelKey, VoxelData>> &voxels) {
  std::uint64_t labeled_count = 0;
  for (const auto &[key, voxel] : voxels) {
    (void)key;
    if (voxel.label >= 0) {
      ++labeled_count;
    }
  }
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open voxel labels: " + path);
  }
  const std::array<char, 8> magic{{'G', '2', 'C', 'L', 'S', 'T', '1', '\0'}};
  output.write(magic.data(), magic.size());
  writeBinary(output, leaf);
  writeBinary(output, labeled_count);
  for (const auto &[key, voxel] : voxels) {
    if (voxel.label < 0) {
      continue;
    }
    writeBinary(output, key.x);
    writeBinary(output, key.y);
    writeBinary(output, key.z);
    writeBinary(output, voxel.label);
  }
  if (!output) {
    throw std::runtime_error("failed to write voxel labels: " + path);
  }
}

void writeClusterTable(const std::string &path,
                       const std::vector<ClusterStats> &clusters) {
  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open cluster table: " + path);
  }
  output << "id\tvoxel_count\tpoint_count\tcx\tcy\tcz\tmin_x\tmin_y\tmin_"
            "z\tmax_x\tmax_y\tmax_z\tr\tg\tb\n";
  for (const ClusterStats &cluster : clusters) {
    output << cluster.id << '\t' << cluster.voxel_count << '\t'
           << cluster.point_count << '\t' << cluster.x << '\t' << cluster.y
           << '\t' << cluster.z << '\t' << cluster.min_x << '\t'
           << cluster.min_y << '\t' << cluster.min_z << '\t' << cluster.max_x
           << '\t' << cluster.max_y << '\t' << cluster.max_z << '\t'
           << static_cast<int>(cluster.color[0]) << '\t'
           << static_cast<int>(cluster.color[1]) << '\t'
           << static_cast<int>(cluster.color[2]) << '\n';
  }
}

void clusterCloud(const Options &options) {
  pcl::PCLPointCloud2 raw;
  if (pcl::io::loadPCDFile(options.input, raw) != 0) {
    throw std::runtime_error("failed to load PCD: " + options.input);
  }
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::fromPCLPointCloud2(raw, points);

  std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxel_map;
  voxel_map.reserve(points.size() / 3 + 1);
  for (const pcl::PointXYZ &point : points) {
    if (!isFinite(point)) {
      continue;
    }
    const VoxelKey key = voxelKey(point, options.leaf);
    VoxelData &voxel = voxel_map[key];
    voxel.x += point.x;
    voxel.y += point.y;
    voxel.z += point.z;
    ++voxel.count;
  }

  std::vector<std::pair<VoxelKey, VoxelData>> voxels;
  voxels.reserve(voxel_map.size());
  for (const auto &entry : voxel_map) {
    voxels.push_back(entry);
  }
  std::sort(voxels.begin(), voxels.end(),
            [](const auto &left, const auto &right) {
              if (left.first.x != right.first.x)
                return left.first.x < right.first.x;
              if (left.first.y != right.first.y)
                return left.first.y < right.first.y;
              return left.first.z < right.first.z;
            });

  auto candidates = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<std::size_t> candidate_to_voxel;
  candidates->reserve(voxels.size());
  candidate_to_voxel.reserve(voxels.size());
  for (std::size_t index = 0; index < voxels.size(); ++index) {
    const VoxelData &voxel = voxels[index].second;
    const float x = static_cast<float>(voxel.x / voxel.count);
    const float y = static_cast<float>(voxel.y / voxel.count);
    const float z = static_cast<float>(voxel.z / voxel.count);
    if (z < options.z_min || z > options.z_max) {
      continue;
    }
    candidates->push_back(pcl::PointXYZ{x, y, z});
    candidate_to_voxel.push_back(index);
  }

  std::vector<pcl::PointIndices> extracted;
  if (!candidates->empty()) {
    auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(candidates);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(options.tolerance);
    extraction.setMinClusterSize(options.min_voxels);
    extraction.setMaxClusterSize(static_cast<int>(candidates->size()));
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(candidates);
    extraction.extract(extracted);
  }

  std::sort(extracted.begin(), extracted.end(),
            [&](const auto &left, const auto &right) {
              auto count = [&](const auto &cluster) {
                std::uint64_t total = 0;
                for (int candidate_index : cluster.indices) {
                  total += voxels[candidate_to_voxel[static_cast<std::size_t>(
                                      candidate_index)]]
                               .second.count;
                }
                return total;
              };
              return count(left) > count(right);
            });

  std::vector<ClusterStats> stats;
  stats.reserve(extracted.size());
  for (std::size_t cluster_index = 0; cluster_index < extracted.size();
       ++cluster_index) {
    ClusterStats cluster;
    cluster.id = static_cast<std::int32_t>(cluster_index);
    cluster.color = clusterColor(cluster.id);
    for (int candidate_index : extracted[cluster_index].indices) {
      VoxelData &voxel =
          voxels[candidate_to_voxel[static_cast<std::size_t>(candidate_index)]]
              .second;
      voxel.label = cluster.id;
      const double x = voxel.x / voxel.count;
      const double y = voxel.y / voxel.count;
      const double z = voxel.z / voxel.count;
      ++cluster.voxel_count;
      cluster.point_count += voxel.count;
      cluster.x += x * voxel.count;
      cluster.y += y * voxel.count;
      cluster.z += z * voxel.count;
      cluster.min_x = std::min(cluster.min_x, x);
      cluster.min_y = std::min(cluster.min_y, y);
      cluster.min_z = std::min(cluster.min_z, z);
      cluster.max_x = std::max(cluster.max_x, x);
      cluster.max_y = std::max(cluster.max_y, y);
      cluster.max_z = std::max(cluster.max_z, z);
    }
    cluster.x /= cluster.point_count;
    cluster.y /= cluster.point_count;
    cluster.z /= cluster.point_count;
    stats.push_back(cluster);
  }

  writePreview(options.preview, voxels);
  writeVoxelLabels(options.voxels, options.leaf, voxels);
  writeClusterTable(options.clusters, stats);
  std::cout << "CLUSTER_COMPLETE raw_points=" << points.size()
            << " preview_voxels=" << voxels.size()
            << " candidate_voxels=" << candidates->size()
            << " clusters=" << stats.size() << '\n';
}

std::unordered_set<std::int32_t> parseRemovedLabels(const std::string &value) {
  std::unordered_set<std::int32_t> labels;
  std::size_t start = 0;
  while (start < value.size()) {
    const std::size_t comma = value.find(',', start);
    const std::string token =
        value.substr(start, comma == std::string::npos ? comma : comma - start);
    if (!token.empty()) {
      labels.insert(std::stoi(token));
    }
    if (comma == std::string::npos)
      break;
    start = comma + 1;
  }
  return labels;
}

void filterCloud(const Options &options) {
  std::ifstream labels_file(options.voxels, std::ios::binary);
  if (!labels_file.is_open()) {
    throw std::runtime_error("failed to open voxel labels: " + options.voxels);
  }
  std::array<char, 8> magic{};
  labels_file.read(magic.data(), magic.size());
  const std::array<char, 8> expected{{'G', '2', 'C', 'L', 'S', 'T', '1', '\0'}};
  if (magic != expected) {
    throw std::runtime_error("invalid cluster voxel file");
  }
  const double leaf = readBinary<double>(labels_file);
  const std::uint64_t record_count = readBinary<std::uint64_t>(labels_file);
  const auto removed_labels = parseRemovedLabels(options.remove);
  std::unordered_set<VoxelKey, VoxelKeyHash> removed_voxels;
  removed_voxels.reserve(static_cast<std::size_t>(record_count));
  for (std::uint64_t index = 0; index < record_count; ++index) {
    VoxelKey key;
    key.x = readBinary<std::int32_t>(labels_file);
    key.y = readBinary<std::int32_t>(labels_file);
    key.z = readBinary<std::int32_t>(labels_file);
    const std::int32_t label = readBinary<std::int32_t>(labels_file);
    if (removed_labels.count(label) > 0) {
      removed_voxels.insert(key);
    }
  }

  pcl::PCLPointCloud2 raw;
  if (pcl::io::loadPCDFile(options.input, raw) != 0) {
    throw std::runtime_error("failed to load PCD: " + options.input);
  }
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::fromPCLPointCloud2(raw, points);
  if (points.size() != static_cast<std::size_t>(raw.width) * raw.height) {
    throw std::runtime_error("PCD coordinate and binary record counts differ");
  }

  pcl::PCLPointCloud2 filtered = raw;
  filtered.height = 1;
  filtered.width = 0;
  filtered.row_step = 0;
  filtered.data.clear();
  filtered.data.reserve(raw.data.size());
  std::uint64_t removed_points = 0;
  for (std::uint32_t row = 0; row < raw.height; ++row) {
    for (std::uint32_t column = 0; column < raw.width; ++column) {
      const std::size_t point_index =
          static_cast<std::size_t>(row) * raw.width + column;
      const pcl::PointXYZ &point = points[point_index];
      const bool remove =
          isFinite(point) && removed_voxels.count(voxelKey(point, leaf)) > 0;
      if (remove) {
        ++removed_points;
        continue;
      }
      const std::size_t offset =
          static_cast<std::size_t>(row) * raw.row_step +
          static_cast<std::size_t>(column) * raw.point_step;
      filtered.data.insert(filtered.data.end(), raw.data.begin() + offset,
                           raw.data.begin() + offset + raw.point_step);
      ++filtered.width;
    }
  }
  filtered.row_step = filtered.width * filtered.point_step;
  pcl::PCDWriter writer;
  if (writer.writeBinary(options.output, filtered) != 0) {
    throw std::runtime_error("failed to save filtered PCD: " + options.output);
  }
  std::cout << "FILTER_COMPLETE input_points=" << points.size()
            << " removed_points=" << removed_points
            << " output_points=" << filtered.width << '\n';
}

} // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parseOptions(argc, argv);
    if (options.mode == "cluster") {
      clusterCloud(options);
    } else {
      filterCloud(options);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
