#include "bt_examples/is_obstacle_near_condition.hpp"
#include <limits>

IsObstacleNearCondition::IsObstacleNearCondition(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      last_scan_ = msg;
    });
}

BT::PortsList IsObstacleNearCondition::providedPorts() {
  return { 
    BT::InputPort<double>("threshold", 0.5, "Distance threshold"),
    BT::OutputPort<double>("obstacle_distance", "Min distance to obstacle")
  };
}

BT::NodeStatus IsObstacleNearCondition::tick() {
  double threshold;
  getInput("threshold", threshold);
  
  if (!last_scan_) {
    return BT::NodeStatus::FAILURE;
  }
  
  float min_distance = std::numeric_limits<float>::max();
  for (const auto& range : last_scan_->ranges) {
    if (std::isfinite(range) && range < min_distance) {
      min_distance = range;
    }
  }
  
  setOutput("obstacle_distance", static_cast<double>(min_distance));
  
  return (min_distance < threshold) ? BT::NodeStatus::SUCCESS 
                                    : BT::NodeStatus::FAILURE;
}
