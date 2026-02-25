#include "bt_examples/backup_action.hpp"
#include <algorithm>

BackUpAction::BackUpAction(const std::string& name,
                           const BT::NodeConfiguration& config,
                           rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node),
    duration_(0, 0) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);
}

BT::PortsList BackUpAction::providedPorts() {
  return { 
    BT::InputPort<double>("obstacle_distance", "Distance to obstacle"),
    BT::InputPort<double>("base_distance", 0.3, "Base backup distance")
  };
}

BT::NodeStatus BackUpAction::onStart() {
  double obstacle_dist, base_dist;
  getInput("obstacle_distance", obstacle_dist);
  getInput("base_distance", base_dist);
  
  distance_ = std::max(base_dist, obstacle_dist + 0.2);
  
  start_time_ = node_->now();
  duration_ = rclcpp::Duration::from_seconds(distance_ / 0.2);
  
  RCLCPP_INFO(node_->get_logger(), 
              "Backing up %.2f meters (obstacle at %.2fm)", 
              distance_, obstacle_dist);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BackUpAction::onRunning() {
  auto elapsed = node_->now() - start_time_;
  
  if (elapsed < duration_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -0.2;
    cmd_vel_pub_->publish(cmd);
    return BT::NodeStatus::RUNNING;
  }
  
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd);
  
  RCLCPP_INFO(node_->get_logger(), "Back up complete");
  return BT::NodeStatus::SUCCESS;
}

void BackUpAction::onHalted() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd);
}
