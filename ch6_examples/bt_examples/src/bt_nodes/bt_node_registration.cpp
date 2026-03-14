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

void register_bt_nodes(BT::BehaviorTreeFactory& factory) {
  // Registrar nodos que obtienen 'node' y 'hri_client' de la blackboard
  // Ya no necesitamos lambdas - registerNodeType lo hace automáticamente
  factory.registerNodeType<IsObstacleNearCondition>("IsObstacleNear");
  factory.registerNodeType<IsBatteryLowCondition>("IsBatteryLow");
  factory.registerNodeType<BackUpAction>("BackUp");
  factory.registerNodeType<SpinAction>("Spin");
  factory.registerNodeType<MoveForwardAction>("MoveForward");
  factory.registerNodeType<ReturnToChargerAction>("ReturnToCharger");
  factory.registerNodeType<AbortMissionAction>("AbortMission");
  factory.registerNodeType<SayTextAction>("SayText");
  factory.registerNodeType<ListenTextAction>("ListenText");
  factory.registerNodeType<ExtractInfoAction>("ExtractInfo");
  factory.registerNodeType<SayTextClientAction>("SayTextClient");
  factory.registerNodeType<ListenTextClientAction>("ListenTextClient");
  factory.registerNodeType<ExtractInfoClientAction>("ExtractInfoClient");

  // Registrar decorador RetryNode (Timeout ya está built-in)
  factory.registerBuilder<BT::RetryNode>(
    "RetryNode",
    [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<BT::RetryNode>(name, config);
    });
}
