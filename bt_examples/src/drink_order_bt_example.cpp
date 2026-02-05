// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bt_examples/bt_node_registration.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("drink_order_bt");
  
  // Factory para registrar nodos personalizados
  BT::BehaviorTreeFactory factory;
  
  // Registrar nodos personalizados
  register_bt_nodes(factory, node);
  
  // Obtener path al archivo XML
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("bt_examples");
  std::string tree_path = package_share_dir + "/config/drink_order_tree.xml";
  
  // Cargar árbol desde XML
  auto tree = factory.createTreeFromFile(tree_path);
  
  // Logger para depuración
  BT::StdCoutLogger logger(tree);
  
  RCLCPP_INFO(node->get_logger(), "Drink order behavior tree started");
  RCLCPP_INFO(node->get_logger(), "The robot will ask what you want to drink, listen, and repeat it back");
  
  // Bucle principal: tick del árbol a 10 Hz
  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  
  while (rclcpp::ok() && status == BT::NodeStatus::IDLE) {
    // Procesar callbacks de ROS
    rclcpp::spin_some(node);
    
    // Evaluar árbol
    status = tree.tickOnce();
    
    if (status == BT::NodeStatus::RUNNING) {
      // El árbol todavía está ejecutándose
      rate.sleep();
      continue;
    } else if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Interaction completed successfully!");
      break;
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "Interaction failed");
      break;
    }
  }
  
  rclcpp::shutdown();
  return 0;
}
