#include "robot_rl_simulation/a1_body_state_plugin.hpp"

#include <utility>

#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>

namespace robot_rl_simulation {

void A1BodyStatePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  base_link_ = model->GetLink("base_link");
  if (!base_link_) {
    gzerr << "A1BodyStatePlugin requires a base_link on model " << model->GetName() << '\n';
    return;
  }

  if (sdf->HasElement("topic_name")) {
    topic_name_ = sdf->Get<std::string>("topic_name");
  }
  if (sdf->HasElement("publish_rate_hz")) {
    const auto rate_hz = sdf->Get<double>("publish_rate_hz");
    if (rate_hz <= 0.0) {
      gzerr << "A1BodyStatePlugin publish_rate_hz must be positive\n";
      return;
    }
    publish_period_sec_ = 1.0 / rate_hz;
  }

  node_ = gazebo_ros::Node::Get(sdf);
  if (!node_) {
    gzerr << "A1BodyStatePlugin could not create its ROS node\n";
    return;
  }
  publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name_, rclcpp::QoS(10));
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&A1BodyStatePlugin::publish_body_state, this));
}

void A1BodyStatePlugin::publish_body_state() {
  const auto sim_time = base_link_->GetWorld()->SimTime();
  if ((sim_time - last_publish_time_).Double() < publish_period_sec_) {
    return;
  }
  last_publish_time_ = sim_time;

  const auto pose = base_link_->WorldPose();
  const auto angular_velocity = base_link_->WorldAngularVel();
  nav_msgs::msg::Odometry message;
  message.header.stamp = rclcpp::Time(sim_time.sec, sim_time.nsec, RCL_ROS_TIME);
  message.header.frame_id = "gazebo_world";
  message.child_frame_id = "base_link";
  message.pose.pose.position.x = pose.Pos().X();
  message.pose.pose.position.y = pose.Pos().Y();
  message.pose.pose.position.z = pose.Pos().Z();
  message.pose.pose.orientation.x = pose.Rot().X();
  message.pose.pose.orientation.y = pose.Rot().Y();
  message.pose.pose.orientation.z = pose.Rot().Z();
  message.pose.pose.orientation.w = pose.Rot().W();
  message.twist.twist.angular.x = angular_velocity.X();
  message.twist.twist.angular.y = angular_velocity.Y();
  message.twist.twist.angular.z = angular_velocity.Z();
  publisher_->publish(message);
}

GZ_REGISTER_MODEL_PLUGIN(A1BodyStatePlugin)

}  // namespace robot_rl_simulation
