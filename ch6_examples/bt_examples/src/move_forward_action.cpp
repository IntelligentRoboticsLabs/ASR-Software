#include "bt_examples/move_forward_action.hpp"

MoveForwardAction::MoveForwardAction(const std::string& name,
                                     const BT::NodeConfiguration& config,
                                     rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);
}

BT::PortsList MoveForwardAction::providedPorts() {
  return {
    BT::InputPort<double>("speed", 0.3, "Forward speed (m/s)")
  };
}

BT::NodeStatus MoveForwardAction::tick() {
  double speed;
  getInput("speed", speed);
  
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = speed;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
  
  return BT::NodeStatus::SUCCESS;
}
