#include "bt_examples/has_goal_condition.hpp"

HasGoalCondition::HasGoalCondition(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node) {}

BT::PortsList HasGoalCondition::providedPorts() {
  return {
    BT::InputPort<double>("goal_x", "Goal X coordinate"),
    BT::InputPort<double>("goal_y", "Goal Y coordinate")
  };
}

BT::NodeStatus HasGoalCondition::tick() {
  double goal_x, goal_y;
  
  if (!getInput("goal_x", goal_x) || !getInput("goal_y", goal_y)) {
    RCLCPP_WARN(node_->get_logger(), "No goal set");
    return BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::SUCCESS;
}
