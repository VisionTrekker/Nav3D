#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/string.hpp>
#include "map_loader/srv/load_map.hpp"

class MapLoaderNode : public rclcpp::Node {
public:
  MapLoaderNode() : Node("map_loader_node") {
    srv_ = create_service<map_loader::srv::LoadMap>(
      "~/load_map",
      std::bind(&MapLoaderNode::on_load, this,
                 std::placeholders::_1, std::placeholders::_2));

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/map_loader/octomap", rclcpp::QoS(1).transient_local());
    idx_pub_ = create_publisher<std_msgs::msg::String>(
      "/map_loader/scan_context_index", rclcpp::QoS(1).transient_local());

    RCLPP_INFO(get_logger(), "MapLoaderNode ready. Service: ~/load_map");
  }

private:
  rclcpp::Service<map_loader::srv::LoadMap>::SharedPtr srv_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr idx_pub_;

  void on_load(const map_loader::srv::LoadMap::Request::SharedPtr req,
               map_loader::srv::LoadMap::Response::SharedPtr res) {
    RCLPP_INFO(get_logger(), "Received load_map request: PCD=%s, resolution=%.3f",
               req->pcd_path.c_str(), req->resolution);

    try {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::io::loadPCDFile(req->pcd_path, cloud);
      RCLPP_INFO(get_logger(), "Loaded %zu points from PCD", cloud.size());

      auto tree = std::make_shared<octomap::OcTree>(req->resolution);
      for (const auto& p : cloud.points) {
        tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
      }
      tree->updateInnerOccupancy();

      octomap_msgs::msg::Octomap octomap_msg;
      if (!octomap_msgs::fullMapToMsg(*tree, octomap_msg)) {
        throw std::runtime_error("Failed to convert OcTree to Octomap message");
      }
      octomap_msg.header.frame_id = "map";
      octomap_msg.header.stamp = this->now();
      octomap_pub_->publish(octomap_msg);
      RCLPP_INFO(get_logger(), "Published Octomap to /map_loader/octomap");

      std_msgs::msg::String idx_msg;
      idx_msg.data = req->pcd_path;
      idx_pub_->publish(idx_msg);
      RCLPP_INFO(get_logger(), "Published scan_context_index to /map_loader/scan_context_index");

      res->success = true;
      res->octomap_path = req->pcd_path;
      res->scan_context_index_path = req->pcd_path;

    } catch (const std::exception& e) {
      RCLPP_ERROR(get_logger(), "Failed to load PCD: %s", e.what());
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
