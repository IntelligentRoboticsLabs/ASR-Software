// bt_nodes/abort_mission_action.hpp
#ifndef BT_EXAMPLES__BT_NODES__ABORT_MISSION_ACTION_HPP_
#define BT_EXAMPLES__BT_NODES__ABORT_MISSION_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

class AbortMissionAction : public BT::SyncActionNode {
public:
  AbortMissionAction(const std::string& name,
                     const BT::NodeConfiguration& config,
                     rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

#endif  // BT_EXAMPLES__BT_NODES__ABORT_MISSION_ACTION_HPP_
