#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace scan_planner
{
class Go2KinematicSim : public rclcpp::Node
{
public:
  Go2KinematicSim() : Node("go2_kinematic_sim")
  {
    x_ = declare_parameter<double>("init_x", 0.0);
    y_ = declare_parameter<double>("init_y", 0.0);
    z_ = declare_parameter<double>("init_z", 0.3);
    yaw_ = declare_parameter<double>("init_yaw", 0.0);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    const double sim_rate = declare_parameter<double>("sim_rate", 100.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");
    const std::string init_odom_topic = declare_parameter<std::string>("init_odom_topic", "");
    const std::string initial_pose_topic = declare_parameter<std::string>("initial_pose_topic", "");
    require_initial_pose_ = declare_parameter<bool>("require_initial_pose", false);
    if (require_initial_pose_ && initial_pose_topic.empty())
    {
      throw std::runtime_error("require_initial_pose=true requires initial_pose_topic");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 100);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 20, std::bind(&Go2KinematicSim::cmdCallback, this, std::placeholders::_1));
    if (!init_odom_topic.empty())
    {
      init_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          init_odom_topic, rclcpp::SensorDataQoS(),
          std::bind(&Go2KinematicSim::initOdomCallback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Go2 simulator will mirror initial odom from %s until first cmd_vel",
                  init_odom_topic.c_str());
    }
    if (!initial_pose_topic.empty())
    {
      initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          initial_pose_topic, 10,
          std::bind(&Go2KinematicSim::initialPoseCallback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Go2 simulator will wait for initial pose from %s",
                  initial_pose_topic.c_str());
    }
    else
    {
      has_initial_pose_ = true;
    }
    last_cmd_time_ = now();
    last_sim_time_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, sim_rate)),
        std::bind(&Go2KinematicSim::simCallback, this));
    RCLCPP_INFO(get_logger(), "Go2 kinematic simulator ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    const bool active_command =
        std::hypot(msg->linear.x, msg->linear.y) > 1.0e-4 ||
        std::abs(msg->angular.z) > 1.0e-4;
    if (active_command)
      have_cmd_ = true;
    vx_cmd_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    vy_cmd_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    vyaw_cmd_ = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    last_cmd_time_ = now();
  }

  void initOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!msg || have_cmd_)
      return;
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
    yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    frame_id_ = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
    child_frame_id_ = msg->child_frame_id.empty() ? child_frame_id_ : msg->child_frame_id;
    has_initial_pose_ = true;
  }

  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (!msg || have_cmd_)
      return;

    const auto &pose = msg->pose.pose;
    const bool finite =
        std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
        std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
        std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
        std::isfinite(pose.orientation.w);
    if (!finite)
    {
      RCLCPP_WARN(get_logger(), "Ignoring initial pose containing NaN or Inf");
      return;
    }

    x_ = pose.position.x;
    y_ = pose.position.y;
    z_ = pose.position.z;
    yaw_ = tf2::getYaw(pose.orientation);
    frame_id_ = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
    child_frame_id_ = "base_link";
    has_initial_pose_ = true;
    last_sim_time_ = now();
    RCLCPP_INFO(
        get_logger(), "Initial pose accepted: frame=%s position=(%.3f, %.3f, %.3f) yaw=%.3f",
        frame_id_.c_str(), x_, y_, z_, yaw_);
  }

  void publishOdom(const rclcpp::Time &stamp)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    const auto orientation = tf2::toMsg(quaternion);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = vx_world_;
    odom.twist.twist.linear.y = vy_world_;
    odom.twist.twist.angular.z = vyaw_cmd_;
    odom_pub_->publish(odom);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = z_;
      transform.transform.rotation = orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void simCallback()
  {
    const auto current_time = now();
    if (!has_initial_pose_)
      return;

    double dt = (current_time - last_sim_time_).seconds();
    last_sim_time_ = current_time;
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    double vx = vx_cmd_, vy = vy_cmd_, wz = vyaw_cmd_;
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_)
      vx = vy = wz = 0.0;
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    vx_world_ = c * vx - s * vy;
    vy_world_ = s * vx + c * vy;
    x_ += vx_world_ * dt;
    y_ += vy_world_ * dt;
    yaw_ = normalizeAngle(yaw_ + wz * dt);
    publishOdom(current_time);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr init_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_{0.0}, y_{0.0}, z_{0.3}, yaw_{0.0};
  double vx_cmd_{0.0}, vy_cmd_{0.0}, vyaw_cmd_{0.0};
  double vx_world_{0.0}, vy_world_{0.0};
  double max_vx_{0.75}, max_vy_{0.35}, max_vyaw_{1.0}, cmd_timeout_{0.3};
  bool have_cmd_{false};
  bool has_initial_pose_{false};
  bool require_initial_pose_{false};
  bool publish_tf_{false};
  std::string frame_id_, child_frame_id_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::Go2KinematicSim>());
  rclcpp::shutdown();
  return 0;
}
