#include "bt_examples/spin_action.hpp"


SpinAction::SpinAction(const std::string& name,
                       const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node),
    duration_(0, 0) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);
}

BT::PortsList SpinAction::providedPorts() {
  return { 
    BT::InputPort<double>("obstacle_distance", "Distance to obstacle"),
    BT::InputPort<double>("base_angle", 1.57, "Base spin angle (radians)")
  };
}

BT::NodeStatus SpinAction::onStart() {
  double obstacle_dist, base_angle;
  getInput("obstacle_distance", obstacle_dist);
  getInput("base_angle", base_angle);
  
  if (obstacle_dist < 0.3) {
    angle_ = 3.14159;
    RCLCPP_WARN(node_->get_logger(), 
                "Obstacle very close (%.2fm), spinning 180 degrees", 
                obstacle_dist);
  } else {
    angle_ = base_angle;
    RCLCPP_INFO(node_->get_logger(), 
                "Obstacle at %.2fm, spinning %.0f degrees", 
                obstacle_dist, angle_ * 180.0 / 3.14159);
  }
  
  start_time_ = node_->now();
  duration_ = rclcpp::Duration::from_seconds(angle_ / 0.5);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinAction::onRunning() {
  auto elapsed = node_->now() - start_time_;
  
  if (elapsed < duration_) {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.5;
    cmd_vel_pub_->publish(cmd);
    return BT::NodeStatus::RUNNING;
  }
  
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
  
  RCLCPP_INFO(node_->get_logger(), "Spin complete");
  return BT::NodeStatus::SUCCESS;
}

void SpinAction::onHalted() {
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}
