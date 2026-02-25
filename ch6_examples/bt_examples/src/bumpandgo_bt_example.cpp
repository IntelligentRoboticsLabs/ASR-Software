#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/decorators/retry_node.h>
#include <behaviortree_cpp/decorators/timeout_node.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bt_examples/bt_node_registration.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bumpandgo_bt");
  
  // Factory para registrar nodos personalizados
  BT::BehaviorTreeFactory factory;
  
  // Registrar nodos personalizados
  register_bt_nodes(factory, node);
  
  // Obtener path al archivo XML
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("bt_examples");
  std::string tree_path = package_share_dir + "/config/bumpandgo_tree.xml";
  
  // Cargar árbol desde XML
  auto tree = factory.createTreeFromFile(tree_path);
  
  // Leer parámetros ROS y establecer en el blackboard GLOBAL
  node->declare_parameter("charger_x", 0.0);
  node->declare_parameter("charger_y", 0.0);
  
  double charger_x = node->get_parameter("charger_x").as_double();
  double charger_y = node->get_parameter("charger_y").as_double();
  
  tree.subtrees[0]->blackboard->set("charger_x", charger_x);
  tree.subtrees[0]->blackboard->set("charger_y", charger_y);
  tree.subtrees[0]->blackboard->set("node", node);
  
  // Logger para depuración
  BT::StdCoutLogger logger(tree);
  
  RCLCPP_INFO(node->get_logger(), "Bump-and-go behavior tree started");
  RCLCPP_INFO(node->get_logger(), "Charger position: (%.2f, %.2f)", 
              charger_x, charger_y);
  
  // Bucle principal: tick del árbol a 10 Hz
  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  
  while (rclcpp::ok() && status != BT::NodeStatus::FAILURE) {
    RCLCPP_DEBUG(node->get_logger(), "Ticking the behavior tree...");
    
    // Procesar callbacks de ROS
    rclcpp::spin_some(node);
    
    // Evaluar árbol
    status = tree.tickOnce();
    rate.sleep();
    
  }
  
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Mission completed successfully");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Mission failed");
  }
  
  rclcpp::shutdown();
  return 0;
}
