#include "bt_examples/is_battery_low_condition.hpp"

IsBatteryLowCondition::IsBatteryLowCondition(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", 10,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      last_battery_ = msg;
    });
}

BT::PortsList IsBatteryLowCondition::providedPorts() {
  return { 
    BT::InputPort<double>("threshold", 0.10, "Battery percentage threshold (0.0-1.0)")
  };
}

BT::NodeStatus IsBatteryLowCondition::tick() {
  double threshold;
  getInput("threshold", threshold);
  
  if (!last_battery_) {
    // Si no hay datos de batería, asumimos que está bien
    return BT::NodeStatus::FAILURE;
  }
  
  // BatteryState.percentage va de 0.0 a 1.0 (0% a 100%)
  bool is_low = last_battery_->percentage < threshold;
  
  if (is_low) {
    RCLCPP_WARN(node_->get_logger(), 
                "Battery low: %.1f%% (threshold: %.1f%%)", 
                last_battery_->percentage * 100.0,
                threshold * 100.0);
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}
