#include <rclcpp/rclcpp.hpp>
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

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_pose_ = msg->pose.pose;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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
      out.header.frame_id = "map";
      out.pose.pose = last_pose_;
      loop_pub_->publish(out);
      RCLCPP_INFO(this->get_logger(), "Loop closure detected: %d -> %d", query_idx, loop_id);
    }
  }

public:
  ScanContextLoopNode() : Node("scan_context_loop_node") {
    sc_.setSCdistThres(0.2);
    sc_.setMaximumRadius(80.0);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lio/mapping/odom_body", 10,
      std::bind(&ScanContextLoopNode::on_odom, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lio/mapping/clouds_lidar", 10,
      std::bind(&ScanContextLoopNode::on_cloud, this, std::placeholders::_1));
    loop_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/scan_context_loop/loop_closure", 10);

    RCLCPP_INFO(this->get_logger(), "ScanContextLoopNode started.");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanContextLoopNode>());
  rclcpp::shutdown();
  return 0;
}
