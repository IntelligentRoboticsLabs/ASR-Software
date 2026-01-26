#include "bt_examples/abort_mission_action.hpp"

AbortMissionAction::AbortMissionAction(const std::string& name,
                                       const BT::NodeConfiguration& config,
                                       rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node) {}

BT::PortsList AbortMissionAction::providedPorts() {
  return {};
}

BT::NodeStatus AbortMissionAction::tick() {
  RCLCPP_ERROR(node_->get_logger(), "Mission aborted - all strategies failed");
  return BT::NodeStatus::FAILURE;
}
