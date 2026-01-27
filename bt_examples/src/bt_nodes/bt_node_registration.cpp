#include <behaviortree_cpp_v3/bt_factory.h>
#include "bt_examples/is_obstacle_near_condition.hpp"
#include "bt_examples/is_battery_low_condition.hpp"
#include "bt_examples/backup_action.hpp"
#include "bt_examples/spin_action.hpp"
#include "bt_examples/move_forward_action.hpp"
#include "bt_examples/return_to_charger_action.hpp"
#include "bt_examples/abort_mission_action.hpp"

void register_bt_nodes(BT::BehaviorTreeFactory& factory, rclcpp::Node::SharedPtr node) {
  factory.registerBuilder<IsObstacleNearCondition>(
    "IsObstacleNear",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<IsObstacleNearCondition>(name, config, node);
    });

  factory.registerBuilder<IsBatteryLowCondition>(
    "IsBatteryLow",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<IsBatteryLowCondition>(name, config, node);
    });

  factory.registerBuilder<BackUpAction>(
    "BackUp",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<BackUpAction>(name, config, node);
    });

  factory.registerBuilder<SpinAction>(
    "Spin",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SpinAction>(name, config, node);
    });

  factory.registerBuilder<MoveForwardAction>(
    "MoveForward",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<MoveForwardAction>(name, config, node);
    });

  factory.registerBuilder<ReturnToChargerAction>(
    "ReturnToCharger",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<ReturnToChargerAction>(name, config, node);
    });

  factory.registerBuilder<AbortMissionAction>(
    "AbortMission",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<AbortMissionAction>(name, config, node);
    });

    // Registrar decorador RetryNode (Timeout ya está built-in)
  factory.registerBuilder<BT::RetryNode>(
    "RetryNode",
    [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<BT::RetryNode>(name, config);
    });
}
