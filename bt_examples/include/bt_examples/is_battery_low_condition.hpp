// bt_nodes/is_battery_low_condition.hpp
#ifndef BT_EXAMPLES__BT_NODES__IS_BATTERY_LOW_CONDITION_HPP_
#define BT_EXAMPLES__BT_NODES__IS_BATTERY_LOW_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

class IsBatteryLowCondition : public BT::ConditionNode {
public:
  IsBatteryLowCondition(const std::string& name,
                        const BT::NodeConfiguration& config,
                        rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_;
};

#endif  // BT_EXAMPLES__BT_NODES__IS_BATTERY_LOW_CONDITION_HPP_
