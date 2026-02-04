// bt_nodes/has_goal_condition.hpp
#ifndef BT_EXAMPLES__BT_NODES__HAS_GOAL_CONDITION_HPP_
#define BT_EXAMPLES__BT_NODES__HAS_GOAL_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class HasGoalCondition : public BT::ConditionNode {
public:
  HasGoalCondition(const std::string& name,
                   const BT::NodeConfiguration& config,
                   rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

#endif  // BT_EXAMPLES__BT_NODES__HAS_GOAL_CONDITION_HPP_
