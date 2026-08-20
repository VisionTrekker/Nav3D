#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GlobalPlannerStub : public rclcpp::Node {
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_;
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    current_pose_->header = msg->header;
    current_pose_->pose = msg->pose.pose;
    publish_path();
  }
  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose_ = msg;
    publish_path();
  }
  void publish_path() {
    if (!current_pose_ || !goal_pose_) return;
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    const int N = 20;
    for (int i = 0; i <= N; ++i) {
      double t = static_cast<double>(i) / N;
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = current_pose_->pose.position.x * (1 - t) + goal_pose_->pose.position.x * t;
      p.pose.position.y = current_pose_->pose.position.y * (1 - t) + goal_pose_->pose.position.y * t;
      p.pose.position.z = current_pose_->pose.position.z * (1 - t) + goal_pose_->pose.position.z * t;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    path_pub_->publish(path);
  }
	public:
  GlobalPlannerStub() : Node("global_planner_node") {
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&GlobalPlannerStub::on_goal, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lio/localization/odom", 10, std::bind(&GlobalPlannerStub::on_odom, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_planner/path", rclcpp::QoS(1).transient_local());
  }
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerStub>());
  rclcpp::shutdown();
  return 0;
}
