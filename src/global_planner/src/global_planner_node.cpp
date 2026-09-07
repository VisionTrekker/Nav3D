#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <filesystem>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "global_planner/global_planner.h"
#include "global_planner/pcd2octomap_converter.h"

namespace
{
constexpr const char * kMapFrame = "map";
constexpr const char * kBodyFrame = "base_link";
constexpr const char * kLocalizationOdomTopic = "/lio/localization/odom";
constexpr const char * kDefaultPcdMap =
  "/home/nhy/code/vscode/Nav3D/maps/campus3_no_elevator.pcd";
constexpr const char * kDefaultBtOutput =
  "/tmp/nav3d_global_planner_campus3_no_elevator.bt";
constexpr double kUnsetExpectedOctomapResolution = -1.0;
constexpr double kDefaultMaxEndpointSnapDistance = 2.5;
constexpr double kDefaultRobotRadius = 0.25;
constexpr int kDefaultMaxIterations = 800000;
constexpr int kDefaultSnapSearchRadiusCells = 12;
constexpr bool kDefaultRequireGroundSupport = true;
constexpr bool kDefaultStrictDirectGroundSupport = false;
constexpr int kDefaultGroundSupportXyRadiusCells = 1;
constexpr int kDefaultGroundSupportDepthCells = 1;
constexpr bool kDefaultEnablePreblockedCostmap = true;
constexpr int kDefaultPreblockedCostmapRadiusCells = 3;
constexpr double kDefaultPreblockedCostmapWeight = 2.5;
constexpr bool kDefaultLowestTraversableOnly = false;
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
    expected_octomap_resolution_ = declare_parameter<double>(
      "expected_octomap_resolution", kUnsetExpectedOctomapResolution);
    if (expected_octomap_resolution_ != kUnsetExpectedOctomapResolution &&
        (!std::isfinite(expected_octomap_resolution_) || expected_octomap_resolution_ <= 0.0)) {
      throw std::invalid_argument(
              "expected_octomap_resolution must be positive or -1 to disable the check");
    }
    max_endpoint_snap_distance_ = declare_parameter<double>(
      "max_endpoint_snap_distance",
      kDefaultMaxEndpointSnapDistance);
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", kLocalizationOdomTopic);
    odom_topic_ = odom_topic;
    require_map_frame_odom_ = declare_parameter<bool>(
      "require_map_frame_odom", odom_topic == kLocalizationOdomTopic);
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const auto path_topic = declare_parameter<std::string>(
      "path_topic", "/global_planner/path");
    const auto octomap_topic = declare_parameter<std::string>(
      "octomap_topic", "/map_loader/octomap");
    const bool use_octomap_topic = declare_parameter<bool>("use_octomap_topic", false);
    const double robot_radius = declare_parameter<double>("robot_radius", kDefaultRobotRadius);
    const int max_iterations = declare_parameter<int>("max_iterations", kDefaultMaxIterations);
    const int snap_search_radius_cells = declare_parameter<int>(
      "snap_search_radius_cells", kDefaultSnapSearchRadiusCells);
    const bool require_ground_support = declare_parameter<bool>(
      "require_ground_support", kDefaultRequireGroundSupport);
    const bool strict_direct_ground_support = declare_parameter<bool>(
      "strict_direct_ground_support", kDefaultStrictDirectGroundSupport);
    const int ground_support_xy_radius_cells = declare_parameter<int>(
      "ground_support_xy_radius_cells", kDefaultGroundSupportXyRadiusCells);
    const int ground_support_depth_cells = declare_parameter<int>(
      "ground_support_depth_cells", kDefaultGroundSupportDepthCells);
    const bool enable_preblocked_costmap = declare_parameter<bool>(
      "enable_preblocked_costmap", kDefaultEnablePreblockedCostmap);
    const int preblocked_costmap_radius_cells = declare_parameter<int>(
      "preblocked_costmap_radius_cells", kDefaultPreblockedCostmapRadiusCells);
    const double preblocked_costmap_weight = declare_parameter<double>(
      "preblocked_costmap_weight", kDefaultPreblockedCostmapWeight);
    const bool lowest_traversable_only = declare_parameter<bool>(
      "lowest_traversable_only", kDefaultLowestTraversableOnly);

    if (odom_topic.empty() || goal_topic.empty() || path_topic.empty() ||
        (use_octomap_topic && octomap_topic.empty())) {
      throw std::invalid_argument("Global planner topic parameters must not be empty");
    }

    planner_->configurePlanningParameters(
      robot_radius,
      max_iterations,
      snap_search_radius_cells,
      require_ground_support,
      strict_direct_ground_support,
      ground_support_xy_radius_cells,
      ground_support_depth_cells,
      enable_preblocked_costmap,
      preblocked_costmap_radius_cells,
      preblocked_costmap_weight,
      lowest_traversable_only);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (use_octomap_topic) {
      octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        octomap_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&GlobalPlannerNode::on_octomap, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Waiting for map-frame OctoMap on %s", octomap_topic.c_str());
    } else {
      load_map();
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::on_odom, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic,
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::on_goal, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      path_topic,
      rclcpp::QoS(1).transient_local().reliable());

    RCLCPP_INFO(
      get_logger(), "Global planner interfaces: odom=%s goal=%s path=%s",
      odom_topic.c_str(), goal_topic.c_str(), path_topic.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Global planner odom source: topic=%s expected_frame=%s child_frame=%s",
      odom_topic_.c_str(), require_map_frame_odom_ ? kMapFrame : "map or odom via tf",
      kBodyFrame);
    RCLCPP_INFO(
      get_logger(),
      "Global planner geometry diagnostics: map_frame=%s body_frame=%s "
      "expected_octomap_resolution=%.3f m",
      kMapFrame, kBodyFrame, expected_octomap_resolution_);
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
    if (expected_octomap_resolution_ != kUnsetExpectedOctomapResolution &&
        std::abs(octree_->getResolution() - expected_octomap_resolution_) > 1.0e-6) {
      RCLCPP_WARN(
        get_logger(),
        "OctoMap resolution consistency WARNING: actual=%.6f m expected=%.6f m",
        octree_->getResolution(), expected_octomap_resolution_);
    } else if (expected_octomap_resolution_ != kUnsetExpectedOctomapResolution) {
      RCLCPP_INFO(
        get_logger(),
        "OctoMap resolution consistency PASS: actual=%.6f m expected=%.6f m",
        octree_->getResolution(), expected_octomap_resolution_);
    }
    RCLCPP_INFO(
      get_logger(),
      "OctoMap ready: resolution=%.3f leaf_nodes=%zu",
      octree_->getResolution(),
      octree_->getNumLeafNodes());
  }

  void on_octomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    if (msg->header.frame_id != kMapFrame) {
      RCLCPP_WARN(
        get_logger(), "Ignoring OctoMap in frame '%s'; expected map",
        msg->header.frame_id.c_str());
      return;
    }

    std::unique_ptr<octomap::AbstractOcTree> decoded(octomap_msgs::msgToMap(*msg));
    auto * decoded_octree = dynamic_cast<octomap::OcTree *>(decoded.get());
    if (!decoded_octree) {
      RCLCPP_ERROR(get_logger(), "Ignoring OctoMap whose tree type is not OcTree");
      return;
    }

    decoded.release();
    octree_.reset(decoded_octree);
    if (octree_->getNumLeafNodes() == 0) {
      RCLCPP_ERROR(get_logger(), "Ignoring empty OctoMap");
      octree_.reset();
      return;
    }

    planner_->setOctomap(octree_);
    map_ready_ = true;
    RCLCPP_INFO(
      get_logger(), "OctoMap received: resolution=%.3f leaf_nodes=%zu",
      octree_->getResolution(), octree_->getNumLeafNodes());

    if (have_odom_ && have_goal_) {
      plan_once();
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id.empty() || msg->child_frame_id != kBodyFrame) {
      RCLCPP_WARN(
        get_logger(), "Ignoring odometry with frames '%s' -> '%s'; expected a framed pose of base_link",
        msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
      return;
    }

    if (require_map_frame_odom_ && msg->header.frame_id != kMapFrame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring localization odometry from frame '%s'; expected map -> %s",
        msg->header.frame_id.c_str(), kBodyFrame);
      return;
    }

    geometry_msgs::msg::PoseStamped source_pose;
    source_pose.header = msg->header;
    source_pose.pose = msg->pose.pose;
    if (msg->header.frame_id == kMapFrame) {
      current_pose_ = source_pose;
    } else {
      try {
        current_pose_ = tf_buffer_->transform(
          source_pose, kMapFrame, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Ignoring odometry until %s -> map is available: %s",
          msg->header.frame_id.c_str(), error.what());
        return;
      }
    }
    latest_odom_pose_ = current_pose_;
    ++odom_sequence_;
    have_odom_ = true;

    if (!odom_logged_) {
      RCLCPP_INFO(
        get_logger(),
        "odom received: topic=%s source_frame=%s stored_frame=%s position=(%.3f, %.3f, %.3f)",
        odom_topic_.c_str(),
        msg->header.frame_id.c_str(),
        current_pose_.header.frame_id.c_str(),
        current_pose_.pose.position.x,
        current_pose_.pose.position.y,
        current_pose_.pose.position.z);
      odom_logged_ = true;
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != kMapFrame) {
      RCLCPP_WARN(
        get_logger(), "Ignoring goal in frame '%s'; expected map",
        msg->header.frame_id.c_str());
      return;
    }
    goal_pose_ = *msg;
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

    const global_planner::PointPose latest_odom_position{
      latest_odom_pose_.pose.position.x,
      latest_odom_pose_.pose.position.y,
      latest_odom_pose_.pose.position.z};
    const global_planner::PointPose start = latest_odom_position;
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
      require_map_frame_odom_
      ? "planning input diagnostics: latest localization odom position=(%.3f, %.3f, %.3f) "
        "actual start passed to tryPlan=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) "
        "odom_seq=%llu"
      : "planning input diagnostics: latest configured odom position=(%.3f, %.3f, %.3f) "
        "actual start passed to tryPlan=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) "
        "odom_seq=%llu",
      latest_odom_position.x, latest_odom_position.y, latest_odom_position.z,
      start.x, start.y, start.z,
      goal.x, goal.y, goal.z,
      static_cast<unsigned long long>(odom_sequence_));
    if (distance(start, latest_odom_position) > 1.0e-9) {
      RCLCPP_WARN(
        get_logger(),
        "Planning start mismatch: actual start passed to tryPlan differs from latest odom "
        "by %.9f m; refusing to use another pose source",
        distance(start, latest_odom_position));
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "planning started: start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
      start.x, start.y, start.z, goal.x, goal.y, goal.z);

    const double resolution = octree_->getResolution();
    const auto grid_center = [resolution](const double coordinate) {
        const auto index = static_cast<int>(std::floor(coordinate / resolution));
        return (static_cast<double>(index) + 0.5) * resolution;
      };
    RCLCPP_INFO(
      get_logger(),
      "Global grid diagnostics: resolution=%.3f m start_cell_center=(%.3f, %.3f, %.3f) "
      "goal_cell_center=(%.3f, %.3f, %.3f)",
      resolution,
      grid_center(start.x), grid_center(start.y), grid_center(start.z),
      grid_center(goal.x), grid_center(goal.y), grid_center(goal.z));

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
    const auto & first = planner_results.front();
    RCLCPP_INFO(
      get_logger(),
      "planning success: path point count=%zu first=(%.3f, %.3f, %.3f) "
      "last=(%.3f, %.3f, %.3f) start_error=%.3f m goal_error=%.3f m",
      path.poses.size(),
      first.x, first.y, first.z,
      last.x, last.y, last.z,
      distance(first, start), endpoint_distance);
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
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped latest_odom_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  std::string odom_topic_;
  std::string pcd_map_file_;
  std::string octomap_output_bt_;
  double expected_octomap_resolution_ = kUnsetExpectedOctomapResolution;
  double max_endpoint_snap_distance_ = kDefaultMaxEndpointSnapDistance;

  bool map_ready_ = false;
  bool have_odom_ = false;
  bool have_goal_ = false;
  bool odom_logged_ = false;
  bool require_map_frame_odom_ = false;
  std::uint64_t odom_sequence_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
