#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "scan_context_loop/SCManager.h"

class ScanContextLoopNode : public rclcpp::Node {
  SCManager sc_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr loop_pub_;
  geometry_msgs::msg::Pose last_pose_;
  std::string last_pose_frame_id_;
  bool have_odom_ = false;

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Ignoring odometry with an empty header.frame_id");
      return;
    }
    last_pose_ = msg->pose.pose;
    last_pose_frame_id_ = msg->header.frame_id;
    have_odom_ = true;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!have_odom_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "Waiting for odometry before publishing Scan Context loop candidates");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    sc_.makeAndSaveScancontextAndKeys(cloud);

    int query_idx = sc_.polarcontextsSize() - 1;
    if (query_idx < sc_.NUM_EXCLUDE_RECENT) {
      return;
    }

    int loop_id = -1;
    float yaw_diff = 0;
    sc_.detectLoopClosureID(query_idx, loop_id, yaw_diff);

    if (loop_id >= 0) {
      geometry_msgs::msg::PoseWithCovarianceStamped out;
      out.header = msg->header;
      // last_pose_ is copied from the configured raw odometry stream.  It is
      // not transformed into map, so preserve the source frame instead of
      // relabeling T_odom_base as T_map_base.
      out.header.frame_id = last_pose_frame_id_;
      out.pose.pose = last_pose_;
      loop_pub_->publish(out);
      RCLCPP_INFO(this->get_logger(), "Loop closure detected: %d -> %d", query_idx, loop_id);
    }
  }

public:
  ScanContextLoopNode() : Node("scan_context_loop_node") {
    const auto cloud_topic = declare_parameter<std::string>(
      "cloud_topic", "/lio/mapping/clouds_lidar");
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/lio/mapping/odom_body");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/scan_context_loop/loop_closure");
    if (cloud_topic.empty() || odom_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("Scan Context topic parameters must not be empty");
    }

    sc_.setSCdistThres(0.2);
    sc_.setMaximumRadius(80.0);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&ScanContextLoopNode::on_odom, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, 10,
      std::bind(&ScanContextLoopNode::on_cloud, this, std::placeholders::_1));
    loop_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "ScanContextLoopNode started: cloud=%s odom=%s output=%s",
      cloud_topic.c_str(), odom_topic.c_str(), output_topic.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanContextLoopNode>());
  rclcpp::shutdown();
  return 0;
}
