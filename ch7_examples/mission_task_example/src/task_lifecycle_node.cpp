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

#include "mission_task_example/task_lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mission_task_example
{

TaskLifecycleNode::TaskLifecycleNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options),
  task_complete_(false),
  task_success_(false),
  work_counter_(0)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TaskLifecycleNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());
  
  // Reset task state
  task_complete_ = false;
  task_success_ = false;
  work_counter_ = 0;
  
  RCLCPP_INFO(get_logger(), "[%s] Successfully configured", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TaskLifecycleNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  
  // Create timer to execute task work
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      if (!task_complete_) {
        bool success = do_task_work();
        if (task_complete_) {
          task_success_ = success;
          RCLCPP_INFO(
            get_logger(), "[%s] Task completed with %s",
            get_name(), success ? "SUCCESS" : "FAILURE");
        }
      }
    });
  
  RCLCPP_INFO(get_logger(), "[%s] Successfully activated", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TaskLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  
  // Stop timer
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  
  RCLCPP_INFO(get_logger(), "[%s] Successfully deactivated", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TaskLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  
  // Clean up resources
  do_task_reset();
  task_complete_ = false;
  task_success_ = false;
  work_counter_ = 0;
  
  RCLCPP_INFO(get_logger(), "[%s] Successfully cleaned up", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TaskLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  
  RCLCPP_INFO(get_logger(), "[%s] Successfully shut down", get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void TaskLifecycleNode::reset_task()
{
  task_complete_ = false;
  task_success_ = false;
  work_counter_ = 0;
  do_task_reset();
}

void TaskLifecycleNode::set_task_complete(bool success)
{
  task_complete_ = true;
  task_success_ = success;
}

}  // namespace mission_task_example
