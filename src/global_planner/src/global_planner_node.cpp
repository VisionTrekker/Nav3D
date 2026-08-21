#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <filesystem>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "global_planner/global_planner.h"
#include "global_planner/pcd2octomap_converter.h"

namespace
{
constexpr const char * kMapFrame = "map";
constexpr const char * kDefaultPcdMap = "/home/nhy/code/vscode/maps/zhiyuan_rev.pcd";
constexpr const char * kDefaultBtOutput = "/tmp/nav3d_global_planner_zhiyuan_rev.bt";
constexpr double kDefaultMaxEndpointSnapDistance = 2.5;
}  // namespace

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node"),
    planner_(std::make_shared<global_planner::GlobalPlanner>())
  {
    pcd_map_file_ = declare_parameter<std::string>("pcd_map_file", kDefaultPcdMap);
    octomap_output_bt_ = declare_parameter<std::string>("octomap_output_bt", kDefaultBtOutput);
    max_endpoint_snap_distance_ = declare_parameter<double>(
      "max_endpoint_snap_distance",
      kDefaultMaxEndpointSnapDistance);

    load_map();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lio/localization/odom",
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::on_odom, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::on_goal, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_planner/path",
      rclcpp::QoS(1).transient_local().reliable());
  }

private:
  void load_map()
  {
    if (pcd_map_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "PCD map path is empty.");
      throw std::runtime_error("PCD map path is empty");
    }

    if (!std::filesystem::exists(pcd_map_file_)) {
      RCLCPP_ERROR(get_logger(), "PCD map does not exist: %s", pcd_map_file_.c_str());
      throw std::runtime_error("PCD map does not exist");
    }

    pcd2octomap::Pcd2OctomapConverter converter;
    converter.setInputPcdFile(pcd_map_file_);
    converter.setOutputBtFile(octomap_output_bt_);

    if (!converter.convert()) {
      RCLCPP_ERROR(get_logger(), "Failed to convert PCD to OctoMap: %s", pcd_map_file_.c_str());
      throw std::runtime_error("Failed to convert PCD to OctoMap");
    }

    RCLCPP_INFO(get_logger(), "PCD loaded: %s", pcd_map_file_.c_str());

    octree_ = converter.getOctomap();
    if (!octree_) {
      RCLCPP_ERROR(get_logger(), "PCD converter returned null OctoMap.");
      throw std::runtime_error("PCD converter returned null OctoMap");
    }

    planner_->setOctomap(octree_);
    map_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "OctoMap ready: resolution=%.3f leaf_nodes=%zu",
      octree_->getResolution(),
      octree_->getNumLeafNodes());
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.header.frame_id = kMapFrame;
    current_pose_.header.stamp = msg->header.stamp;
    current_pose_.pose = msg->pose.pose;
    have_odom_ = true;

    if (!odom_logged_) {
      RCLCPP_INFO(
        get_logger(),
        "odom received: frame=%s position=(%.3f, %.3f, %.3f)",
        msg->header.frame_id.c_str(),
        current_pose_.pose.position.x,
        current_pose_.pose.position.y,
        current_pose_.pose.position.z);
      odom_logged_ = true;
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    goal_pose_.header.frame_id = kMapFrame;
    have_goal_ = true;

    RCLCPP_INFO(
      get_logger(),
      "goal received: frame=%s position=(%.3f, %.3f, %.3f)",
      msg->header.frame_id.c_str(),
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y,
      goal_pose_.pose.position.z);

    plan_once();
  }

  void plan_once()
  {
    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "planning skipped: OctoMap is not ready.");
      return;
    }

    if (!have_odom_) {
      RCLCPP_WARN(get_logger(), "planning skipped: odom has not been received.");
      return;
    }

    if (!have_goal_) {
      RCLCPP_WARN(get_logger(), "planning skipped: goal has not been received.");
      return;
    }

    const global_planner::PointPose start{
      current_pose_.pose.position.x,
      current_pose_.pose.position.y,
      current_pose_.pose.position.z};
    const global_planner::PointPose goal{
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y,
      goal_pose_.pose.position.z};

    if (!is_inside_map_bounds(goal)) {
      RCLCPP_ERROR(
        get_logger(),
        "planning failed: goal outside OctoMap bounds, goal=(%.3f, %.3f, %.3f)",
        goal.x, goal.y, goal.z);
      return;
    }

    if (is_occupied(goal)) {
      RCLCPP_ERROR(
        get_logger(),
        "planning failed: goal is inside an occupied voxel, goal=(%.3f, %.3f, %.3f)",
        goal.x, goal.y, goal.z);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "planning started: start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
      start.x, start.y, start.z, goal.x, goal.y, goal.z);

    planner_->makePlan(start, goal);

    std::vector<global_planner::PointPose> planner_results;
    planner_->getPlannerResults(planner_results);

    if (planner_results.empty()) {
      RCLCPP_ERROR(get_logger(), "planning failed: path point count=0");
      return;
    }

    const auto & last = planner_results.back();
    const double endpoint_distance = distance(last, goal);
    if (endpoint_distance > max_endpoint_snap_distance_) {
      RCLCPP_ERROR(
        get_logger(),
        "planning failed: planner result endpoint is too far from current goal "
        "(distance=%.3f, limit=%.3f), suppressing stale path",
        endpoint_distance,
        max_endpoint_snap_distance_);
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = kMapFrame;
    path.header.stamp = now();
    path.poses.reserve(planner_results.size());

    for (const auto & point : planner_results) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "planning success: path point count=%zu", path.poses.size());
  }

  bool is_inside_map_bounds(const global_planner::PointPose & point) const
  {
    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);

    return point.x >= min_x && point.x <= max_x &&
           point.y >= min_y && point.y <= max_y &&
           point.z >= min_z && point.z <= max_z;
  }

  bool is_occupied(const global_planner::PointPose & point) const
  {
    const octomap::point3d query(
      static_cast<float>(point.x),
      static_cast<float>(point.y),
      static_cast<float>(point.z));
    const auto * node = octree_->search(query);
    return node != nullptr && octree_->isNodeOccupied(node);
  }

  static double distance(
    const global_planner::PointPose & lhs,
    const global_planner::PointPose & rhs)
  {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  std::string pcd_map_file_;
  std::string octomap_output_bt_;
  double max_endpoint_snap_distance_ = kDefaultMaxEndpointSnapDistance;

  bool map_ready_ = false;
  bool have_odom_ = false;
  bool have_goal_ = false;
  bool odom_logged_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
