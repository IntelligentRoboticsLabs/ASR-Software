// Copyright 2026 Intelligent Robotics Lab
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

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mission_task_example/mission_executor.hpp"
#include "mission_task_example/bt_task_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("mission_fsm_node");
  RCLCPP_INFO(node->get_logger(), "Initializing Mission FSM Node");
  
  // Get package share directory for BT XML files
  std::string config_dir = 
    ament_index_cpp::get_package_share_directory("mission_task_example") + "/config";
  
  // Create lifecycle nodes for each task with BT XML file parameter
  RCLCPP_INFO(node->get_logger(), "Creating BT task lifecycle nodes...");
  
  rclcpp::NodeOptions options_a;
  options_a.parameter_overrides({
    {"bt_xml_file", config_dir + "/task_a.xml"}
  });
  auto task_a_node = std::make_shared<mission_task_example::BTTaskNode>(
    "task_a_node", options_a);
  
  rclcpp::NodeOptions options_b;
  options_b.parameter_overrides({
    {"bt_xml_file", config_dir + "/task_b.xml"}
  });
  auto task_b_node = std::make_shared<mission_task_example::BTTaskNode>(
    "task_b_node", options_b);
  
  rclcpp::NodeOptions options_c;
  options_c.parameter_overrides({
    {"bt_xml_file", config_dir + "/task_c.xml"}
  });
  auto task_c_node = std::make_shared<mission_task_example::BTTaskNode>(
    "task_c_node", options_c);
  
  RCLCPP_INFO(node->get_logger(), "Creating Mission Executor");
  auto mission_executor = std::make_unique<mission_task_example::MissionExecutor>(
    node, task_a_node, task_b_node, task_c_node);
  
  RCLCPP_INFO(node->get_logger(), "Initializing Mission Executor");
  mission_executor->initialize();
  
  // Timer to execute FSM update at 10 Hz
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [&mission_executor]() { mission_executor->update(); });
  
  RCLCPP_INFO(node->get_logger(), "Mission FSM Node ready");
  
  // Create executor for spinning multiple nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(task_a_node->get_node_base_interface());
  executor.add_node(task_b_node->get_node_base_interface());
  executor.add_node(task_c_node->get_node_base_interface());
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
