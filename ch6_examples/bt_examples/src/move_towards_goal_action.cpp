#include "bt_examples/move_towards_goal_action.hpp"
#include <cmath>

MoveTowardsGoalAction::MoveTowardsGoalAction(const std::string& name,
                                             const BT::NodeConfiguration& config,
                                             rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);
}

BT::PortsList MoveTowardsGoalAction::providedPorts() {
  return {
    BT::InputPort<double>("goal_x", "Goal X coordinate"),
    BT::InputPort<double>("goal_y", "Goal Y coordinate"),
    BT::InputPort<double>("goal_theta", "Goal orientation (radians)")
  };
}

BT::NodeStatus MoveTowardsGoalAction::onStart() {
  if (!getInput("goal_x", goal_x_) ||
      !getInput("goal_y", goal_y_) ||
      !getInput("goal_theta", goal_theta_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing goal parameters");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
              "Moving towards goal (%.2f, %.2f, %.2f)",
              goal_x_, goal_y_, goal_theta_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTowardsGoalAction::onRunning() {
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "TF error: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  
  double dx = goal_x_ - transform.transform.translation.x;
  double dy = goal_y_ - transform.transform.translation.y;
  double distance = std::sqrt(dx*dx + dy*dy);
  
  if (distance < 0.1) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    
    RCLCPP_INFO(node_->get_logger(), "Goal reached!");
    return BT::NodeStatus::SUCCESS;
  }
  
  double target_angle = std::atan2(dy, dx);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::min(0.3, distance * 0.5);
  cmd.angular.z = target_angle * 0.8;
  cmd_vel_pub_->publish(cmd);
  
  return BT::NodeStatus::RUNNING;
}

void MoveTowardsGoalAction::onHalted() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}
