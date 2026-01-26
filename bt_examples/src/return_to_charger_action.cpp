#include "bt_examples/return_to_charger_action.hpp"
#include <cmath>

ReturnToChargerAction::ReturnToChargerAction(const std::string& name,
                                             const BT::NodeConfiguration& config,
                                             rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);
}

BT::PortsList ReturnToChargerAction::providedPorts() {
  return {
    BT::InputPort<double>("charger_x", 0.0, "Charger X coordinate"),
    BT::InputPort<double>("charger_y", 0.0, "Charger Y coordinate")
  };
}

BT::NodeStatus ReturnToChargerAction::onStart() {
  if (!getInput("charger_x", charger_x_) ||
      !getInput("charger_y", charger_y_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing charger position parameters");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
              "Returning to charger at (%.2f, %.2f)",
              charger_x_, charger_y_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToChargerAction::onRunning() {
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "TF error: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  
  double dx = charger_x_ - transform.transform.translation.x;
  double dy = charger_y_ - transform.transform.translation.y;
  double distance = std::sqrt(dx*dx + dy*dy);
  
  if (distance < 0.15) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    
    RCLCPP_INFO(node_->get_logger(), "Reached charger! Ready to recharge.");
    return BT::NodeStatus::SUCCESS;
  }
  
  double target_angle = std::atan2(dy, dx);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::min(0.2, distance * 0.5);
  cmd.angular.z = target_angle * 0.8;
  cmd_vel_pub_->publish(cmd);
  
  return BT::NodeStatus::RUNNING;
}

void ReturnToChargerAction::onHalted() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}
