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

#include "mission_task_example/bt_task_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>

namespace mission_task_example
{

BTTaskNode::BTTaskNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: TaskLifecycleNode(node_name, options),
  last_bt_status_(BT::NodeStatus::IDLE)
{
  // Declare parameter for BT XML file
  this->declare_parameter("bt_xml_file", "");
}

bool BTTaskNode::do_task_work()
{
  // Load BT on first execution (lazy loading in activate)
  if (!tree_) {
    bt_xml_file_ = this->get_parameter("bt_xml_file").as_string();
    
    if (bt_xml_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "[%s] Parameter 'bt_xml_file' not set!", get_name());
      set_task_complete(false);
      return false;
    }
    
    try {
      RCLCPP_INFO(get_logger(), "[%s] Loading BT from: %s", get_name(), bt_xml_file_.c_str());
      
      // Load and create tree from XML file (BT v3 API)
      tree_ = std::make_unique<BT::Tree>(
        factory_.createTreeFromFile(bt_xml_file_));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to load BT: %s", get_name(), e.what());
      set_task_complete(false);
      return false;
    }
  }
  
  // Tick the behavior tree
  last_bt_status_ = tree_->tickRoot();
  
  // Check if BT completed
  if (last_bt_status_ == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(get_logger(), "[%s] BT completed successfully", get_name());
    set_task_complete(true);
    return true;
  } else if (last_bt_status_ == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(get_logger(), "[%s] BT failed", get_name());
    set_task_complete(false);
    return false;
  }
  
  // Still running
  return false;
}

void BTTaskNode::do_task_reset()
{
  RCLCPP_INFO(get_logger(), "[%s] Resetting BT task", get_name());
  
  if (tree_) {
    tree_->haltTree();
    tree_.reset();
  }
  
  last_bt_status_ = BT::NodeStatus::IDLE;
}

}  // namespace mission_task_example
