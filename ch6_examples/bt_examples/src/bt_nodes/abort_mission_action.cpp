#include "bt_examples/bt_nodes/abort_mission_action.hpp"

AbortMissionAction::AbortMissionAction(const std::string& name,
                                       const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("Missing required node in blackboard");
  }
}

BT::PortsList AbortMissionAction::providedPorts() {
  return {};
}

BT::NodeStatus AbortMissionAction::tick() {
  RCLCPP_ERROR(node_->get_logger(), "Mission aborted - all strategies failed");
  return BT::NodeStatus::FAILURE;
}
