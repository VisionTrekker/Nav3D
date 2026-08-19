#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <memory>

using namespace std::placeholders;

class GoalMarkerNode : public rclcpp::Node
{
public:
  GoalMarkerNode()
  : Node("goal_marker_node"),
    current_pose_([] {
      geometry_msgs::msg::Pose p;
      p.orientation.w = 1.0;
      return p;
    }()),
    menu_handler_()
  {
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    makeInteractiveMarker();

    // Attach menu to marker and apply
    menu_handler_.apply(*server_, "goal_marker");
    server_->applyChanges();

    RCLCPP_INFO(this->get_logger(), "Goal marker server ready. "
      "Use RViz InteractiveMarkers display to drag and set goal.");
  }

  ~GoalMarkerNode() override = default;

private:
  void makeInteractiveMarker()
  {
    // Menu entry: "Set Goal"
    menu_handler_.insert(
      "Set Goal",
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & /*feedback*/) {
        this->onMenuSelect();
      });

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = rclcpp::Time(0);
    int_marker.name = "goal_marker";
    int_marker.description = "3D Goal — drag to reposition, right-click -> Set Goal";
    int_marker.scale = 0.5;

    // Start at map origin
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    int_marker.pose.orientation.w = 1.0;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = 0.0;

    // 6-DoF control for 3D dragging
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = "move_3d";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    control.always_visible = true;
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    int_marker.controls.push_back(control);

    server_->insert(int_marker,
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback) {
        this->onFeedback(feedback);
      });
  }

  void onFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      current_pose_ = feedback->pose;
      publishGoalPose(feedback->pose);
    }
  }

  void onMenuSelect()
  {
    // MENU_SELECT — user right-clicked and selected "Set Goal"
    publishGoalPose(current_pose_);
  }

  void publishGoalPose(const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose = pose;

    goal_pose_pub_->publish(pose_msg);

    RCLCPP_INFO(this->get_logger(),
      "Published /goal_pose  position=%.2f %.2f %.2f",
      pose.position.x,
      pose.position.y,
      pose.position.z);
  }

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  geometry_msgs::msg::Pose current_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalMarkerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
