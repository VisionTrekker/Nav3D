#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "map_loader/srv/load_map.hpp"

namespace
{

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto h1 = std::hash<int>{}(key.x);
    const auto h2 = std::hash<int>{}(key.y);
    const auto h3 = std::hash<int>{}(key.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

VoxelKey pointToVoxelKey(const pcl::PointXYZI & point, const double resolution)
{
  return VoxelKey{
    static_cast<int>(std::floor(point.x / resolution)),
    static_cast<int>(std::floor(point.y / resolution)),
    static_cast<int>(std::floor(point.z / resolution))};
}

octomap::point3d voxelCenter(const VoxelKey & key, const double resolution)
{
  return octomap::point3d(
    (static_cast<double>(key.x) + 0.5) * resolution,
    (static_cast<double>(key.y) + 0.5) * resolution,
    (static_cast<double>(key.z) + 0.5) * resolution);
}

std::vector<VoxelKey> filterSmallClusters(
  const std::unordered_set<VoxelKey, VoxelKeyHash> & occupied_voxels,
  const int min_cluster_voxels)
{
  std::vector<VoxelKey> filtered;
  if (occupied_voxels.empty()) {
    return filtered;
  }
  if (min_cluster_voxels <= 1) {
    filtered.reserve(occupied_voxels.size());
    for (const auto & key : occupied_voxels) {
      filtered.push_back(key);
    }
    return filtered;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> visited;
  visited.reserve(occupied_voxels.size());
  std::queue<VoxelKey> pending;
  std::vector<VoxelKey> cluster;
  cluster.reserve(static_cast<std::size_t>(min_cluster_voxels));

  for (const auto & seed : occupied_voxels) {
    if (visited.find(seed) != visited.end()) {
      continue;
    }

    cluster.clear();
    visited.insert(seed);
    pending.push(seed);

    while (!pending.empty()) {
      const auto current = pending.front();
      pending.pop();
      cluster.push_back(current);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            const VoxelKey neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (visited.find(neighbor) != visited.end()) {
              continue;
            }
            if (occupied_voxels.find(neighbor) == occupied_voxels.end()) {
              continue;
            }
            visited.insert(neighbor);
            pending.push(neighbor);
          }
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= min_cluster_voxels) {
      filtered.insert(filtered.end(), cluster.begin(), cluster.end());
    }
  }

  return filtered;
}

}  // namespace

class MapLoaderNode : public rclcpp::Node {
public:
  MapLoaderNode() : Node("map_loader_node") {
    const auto pcd_path = declare_parameter<std::string>("pcd_path", "");
    const auto resolution = declare_parameter<double>("resolution", 0.1);
    voxel_downsample_m_ = declare_parameter<double>("voxel_downsample_m", 0.0);
    min_points_per_voxel_ = declare_parameter<int>("min_points_per_voxel", 1);
    min_cluster_voxels_ = declare_parameter<int>("min_cluster_voxels", 1);
    if (resolution <= 0.0) {
      throw std::invalid_argument("Map Loader resolution must be positive");
    }
    if (voxel_downsample_m_ < 0.0) {
      throw std::invalid_argument("Map Loader voxel_downsample_m must be non-negative");
    }
    if (min_points_per_voxel_ < 1) {
      throw std::invalid_argument("Map Loader min_points_per_voxel must be >= 1");
    }
    if (min_cluster_voxels_ < 1) {
      throw std::invalid_argument("Map Loader min_cluster_voxels must be >= 1");
    }

    srv_ = create_service<map_loader::srv::LoadMap>(
      "~/load_map",
      std::bind(&MapLoaderNode::on_load, this,
                 std::placeholders::_1, std::placeholders::_2));

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/map_loader/octomap", rclcpp::QoS(1).reliable().transient_local());
    pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_loader/pcd_points", rclcpp::QoS(1).reliable().transient_local());
    idx_pub_ = create_publisher<std_msgs::msg::String>(
      "/map_loader/scan_context_index", rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(get_logger(), "MapLoaderNode ready. Service: ~/load_map");

    if (!pcd_path.empty()) {
      auto request = std::make_shared<map_loader::srv::LoadMap::Request>();
      auto response = std::make_shared<map_loader::srv::LoadMap::Response>();
      request->pcd_path = pcd_path;
      request->resolution = static_cast<float>(resolution);
      on_load(request, response);
      if (!response->success) {
        throw std::runtime_error("Failed to load startup PCD: " + pcd_path);
      }
    }
  }

private:
  rclcpp::Service<map_loader::srv::LoadMap>::SharedPtr srv_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idx_pub_;
  double voxel_downsample_m_ = 0.0;
  int min_points_per_voxel_ = 1;
  int min_cluster_voxels_ = 1;

  void on_load(const map_loader::srv::LoadMap::Request::SharedPtr req,
               map_loader::srv::LoadMap::Response::SharedPtr res) {
    RCLCPP_INFO(get_logger(), "Received load_map request: PCD=%s, resolution=%.3f",
               req->pcd_path.c_str(), req->resolution);

    try {
      if (req->pcd_path.empty() || req->resolution <= 0.0F) {
        throw std::invalid_argument("PCD path must be non-empty and resolution positive");
      }
      pcl::PointCloud<pcl::PointXYZI> cloud;
      if (pcl::io::loadPCDFile(req->pcd_path, cloud) < 0 || cloud.empty()) {
        throw std::runtime_error("PCD cannot be loaded or is empty");
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu points from PCD", cloud.size());

      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
      cloud_msg.header.frame_id = "map";
      cloud_msg.header.stamp = this->now();
      pcd_pub_->publish(cloud_msg);
      RCLCPP_INFO(get_logger(), "Published fixed PCD points to /map_loader/pcd_points");

      pcl::PointCloud<pcl::PointXYZI>::Ptr planning_cloud(
        new pcl::PointCloud<pcl::PointXYZI>(cloud));
      if (voxel_downsample_m_ > 0.0) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(planning_cloud);
        const auto leaf = static_cast<float>(voxel_downsample_m_);
        voxel_grid.setLeafSize(leaf, leaf, leaf);
        voxel_grid.filter(*downsampled);
        planning_cloud = downsampled;
        RCLCPP_INFO(
          get_logger(), "Planning preprocessing: voxel_downsample_m=%.3f -> %zu points",
          voxel_downsample_m_, planning_cloud->size());
      }

      std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_counts;
      voxel_counts.reserve(planning_cloud->size());
      for (const auto & p : planning_cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          continue;
        }
        ++voxel_counts[pointToVoxelKey(p, req->resolution)];
      }

      std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
      occupied_voxels.reserve(voxel_counts.size());
      for (const auto & item : voxel_counts) {
        if (item.second >= min_points_per_voxel_) {
          occupied_voxels.insert(item.first);
        }
      }

      const auto filtered_voxels = filterSmallClusters(occupied_voxels, min_cluster_voxels_);
      if (filtered_voxels.empty()) {
        throw std::runtime_error("Planning OctoMap preprocessing produced no occupied voxels");
      }

      RCLCPP_INFO(
        get_logger(),
        "Planning OctoMap preprocessing: resolution=%.3f, min_points_per_voxel=%d, "
        "min_cluster_voxels=%d, occupied_voxels=%zu, filtered_voxels=%zu",
        req->resolution, min_points_per_voxel_, min_cluster_voxels_,
        occupied_voxels.size(), filtered_voxels.size());

      auto tree = std::make_shared<octomap::OcTree>(req->resolution);
      for (const auto & key : filtered_voxels) {
        tree->updateNode(voxelCenter(key, req->resolution), true);
      }
      tree->updateInnerOccupancy();

      octomap_msgs::msg::Octomap octomap_msg;
      if (!octomap_msgs::fullMapToMsg(*tree, octomap_msg)) {
        throw std::runtime_error("Failed to convert OcTree to Octomap message");
      }
      octomap_msg.header.frame_id = "map";
      octomap_msg.header.stamp = this->now();
      octomap_pub_->publish(octomap_msg);
      RCLCPP_INFO(get_logger(), "Published Octomap to /map_loader/octomap");

      std_msgs::msg::String idx_msg;
      idx_msg.data = req->pcd_path;
      idx_pub_->publish(idx_msg);
      RCLCPP_INFO(get_logger(), "Published map source path to /map_loader/scan_context_index");

      res->success = true;
      res->octomap_path = req->pcd_path;
      res->scan_context_index_path = req->pcd_path;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load PCD: %s", e.what());
      res->success = false;
      res->octomap_path = "";
      res->scan_context_index_path = "";
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLoaderNode>());
  rclcpp::shutdown();
  return 0;
}
