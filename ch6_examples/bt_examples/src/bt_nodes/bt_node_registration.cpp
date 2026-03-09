#include <behaviortree_cpp/bt_factory.h>
#include "bt_examples/bt_nodes/is_obstacle_near_condition.hpp"
#include "bt_examples/bt_nodes/is_battery_low_condition.hpp"
#include "bt_examples/bt_nodes/backup_action.hpp"
#include "bt_examples/bt_nodes/spin_action.hpp"
#include "bt_examples/bt_nodes/move_forward_action.hpp"
#include "bt_examples/bt_nodes/return_to_charger_action.hpp"
#include "bt_examples/bt_nodes/abort_mission_action.hpp"
#include "bt_examples/bt_nodes/say_text_action.hpp"
#include "bt_examples/bt_nodes/listen_text_action.hpp"
#include "bt_examples/bt_nodes/extract_info_action.hpp"
#include "bt_examples/bt_nodes/say_text_client_action.hpp"
#include "bt_examples/bt_nodes/listen_text_client_action.hpp"
#include "bt_examples/bt_nodes/extract_info_client_action.hpp"

void register_bt_nodes(
  BT::BehaviorTreeFactory& factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<HRIClient> hri_client) {
  // Registro de IsObstacleNear:
  // - Tipo de nodo C++: IsObstacleNearCondition
  // - Nombre en XML: "IsObstacleNear" -> se usa como <IsObstacleNear/>
  // - Lambda captura 'node' para inyectarlo al constructor
  // - BehaviorTree.CPP proporciona 'name' y 'config' al crear el nodo desde el XML
  // - La lambda actúa como "factory function" que añade dependencias extra (node)
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

  factory.registerBuilder<SayTextAction>(
    "SayText",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SayTextAction>(name, config, node);
    });

  factory.registerBuilder<ListenTextAction>(
    "ListenText",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<ListenTextAction>(name, config, node);
    });

  factory.registerBuilder<ExtractInfoAction>(
    "ExtractInfo",
    [node](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<ExtractInfoAction>(name, config, node);
    });

  // Registrar nodos que usan HRIClient (solo si está disponible)
  if (hri_client) {
    factory.registerBuilder<SayTextClientAction>(
      "SayTextClient",
      [hri_client](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<SayTextClientAction>(name, config, hri_client);
      });

    factory.registerBuilder<ListenTextClientAction>(
      "ListenTextClient",
      [hri_client](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<ListenTextClientAction>(name, config, hri_client);
      });

    factory.registerBuilder<ExtractInfoClientAction>(
      "ExtractInfoClient",
      [hri_client](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<ExtractInfoClientAction>(name, config, hri_client);
      });
  }

    // Registrar decorador RetryNode (Timeout ya está built-in)
  factory.registerBuilder<BT::RetryNode>(
    "RetryNode",
    [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<BT::RetryNode>(name, config);
    });
}
