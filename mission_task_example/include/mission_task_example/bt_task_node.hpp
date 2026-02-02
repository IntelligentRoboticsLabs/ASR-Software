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

#ifndef MISSION_TASK_EXAMPLE__BT_TASK_NODE_HPP_
#define MISSION_TASK_EXAMPLE__BT_TASK_NODE_HPP_

#include <string>
#include <memory>
#include "mission_task_example/task_lifecycle_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace mission_task_example
{

/**
 * @brief Generic Behavior Tree task node
 * 
 * This lifecycle node loads and executes a Behavior Tree from an XML file.
 * The XML file path is specified via the "bt_xml_file" ROS parameter.
 */
class BTTaskNode : public TaskLifecycleNode
{
public:
  explicit BTTaskNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  bool do_task_work() override;
  void do_task_reset() override;

private:
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::string bt_xml_file_;
  BT::NodeStatus last_bt_status_;
};

}  // namespace mission_task_example

#endif  // MISSION_TASK_EXAMPLE__BT_TASK_NODE_HPP_
