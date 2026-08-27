#pragma once

#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace robot_rl_simulation {

class A1BodyStatePlugin final : public gazebo::ModelPlugin {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  void publish_body_state();

  gazebo::event::ConnectionPtr update_connection_;
  gazebo::physics::LinkPtr base_link_;
  gazebo::common::Time last_publish_time_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  double publish_period_sec_{0.01};
  std::string topic_name_{"/simulation/a1/body_state"};
};

}  // namespace robot_rl_simulation
