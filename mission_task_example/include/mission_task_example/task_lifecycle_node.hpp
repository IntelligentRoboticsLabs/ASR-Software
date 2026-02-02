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

#ifndef MISSION_TASK_EXAMPLE__TASK_LIFECYCLE_NODE_HPP_
#define MISSION_TASK_EXAMPLE__TASK_LIFECYCLE_NODE_HPP_

#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mission_task_example
{

/**
 * @brief Base class for task lifecycle nodes
 * 
 * Lifecycle states:
 * - Unconfigured: Initial state
 * - Inactive: Configured but not active
 * - Active: Executing the task
 * - Finalized: Cleaned up
 */
class TaskLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit TaskLifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle transition callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  // Task execution
  bool is_task_complete() const { return task_complete_; }
  bool is_task_success() const { return task_success_; }
  void reset_task();

protected:
  // To be implemented by derived classes
  virtual bool do_task_work() = 0;
  virtual void do_task_reset() = 0;

  void set_task_complete(bool success);

  rclcpp::TimerBase::SharedPtr timer_;
  bool task_complete_;
  bool task_success_;
  int work_counter_;
};

}  // namespace mission_task_example

#endif  // MISSION_TASK_EXAMPLE__TASK_LIFECYCLE_NODE_HPP_
