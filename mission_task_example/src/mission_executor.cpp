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

#include "mission_task_example/mission_executor.hpp"
#include "mission_task_example/task_lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <string>

namespace mission_task_example
{

MissionExecutor::MissionExecutor(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_a_node,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_b_node,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_c_node)
: node_(node),
  task_a_node_(task_a_node),
  task_b_node_(task_b_node),
  task_c_node_(task_c_node),
  current_state_(MissionState::IDLE),
  idle_ticks_(0),
  completed_ticks_(0)
{
  state_pub_ = node_->create_publisher<std_msgs::msg::String>("/mission_state", 10);
}

void MissionExecutor::initialize()
{
  RCLCPP_INFO(node_->get_logger(), "Initializing Mission Executor with lifecycle tasks");

  // Configure all task nodes
  RCLCPP_INFO(node_->get_logger(), "Configuring task nodes...");
  task_a_node_->configure();
  task_b_node_->configure();
  task_c_node_->configure();

  RCLCPP_INFO(node_->get_logger(), "All tasks configured successfully");

  // Start FSM
  transition_to(MissionState::IDLE);
}

void MissionExecutor::update()
{
  switch (current_state_) {
    case MissionState::IDLE:
      handle_idle_state();
      break;
    case MissionState::TASK_A:
      handle_task_a_state();
      break;
    case MissionState::TASK_B:
      handle_task_b_state();
      break;
    case MissionState::TASK_C:
      handle_task_c_state();
      break;
    case MissionState::COMPLETED:
      handle_completed_state();
      break;
  }
}

void MissionExecutor::handle_idle_state()
{
  idle_ticks_++;

  if (idle_ticks_ >= IDLE_WAIT_TICKS) {
    idle_ticks_ = 0;
    transition_to(MissionState::TASK_A);
  }
}

void MissionExecutor::handle_task_a_state()
{
  auto task_node = std::dynamic_pointer_cast<TaskLifecycleNode>(task_a_node_);
  
  if (task_node && task_node->is_task_complete()) {
    if (task_node->is_task_success()) {
      RCLCPP_INFO(node_->get_logger(), "Task A completed successfully");
      task_a_node_->deactivate();
      task_a_node_->cleanup();
      transition_to(MissionState::TASK_B);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Task A failed, switching to Task C");
      task_a_node_->deactivate();
      task_a_node_->cleanup();
      transition_to(MissionState::TASK_C);
    }
  }
  // Otherwise: task still running
}

void MissionExecutor::handle_task_b_state()
{
  auto task_node = std::dynamic_pointer_cast<TaskLifecycleNode>(task_b_node_);
  
  if (task_node && task_node->is_task_complete()) {
    if (task_node->is_task_success()) {
      RCLCPP_INFO(node_->get_logger(), "Task B completed successfully");
      task_b_node_->deactivate();
      task_b_node_->cleanup();
      transition_to(MissionState::COMPLETED);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Task B failed, retrying from Task A");
      task_b_node_->deactivate();
      task_b_node_->cleanup();
      transition_to(MissionState::TASK_A);
    }
  }
  // Otherwise: task still running
}

void MissionExecutor::handle_task_c_state()
{
  auto task_node = std::dynamic_pointer_cast<TaskLifecycleNode>(task_c_node_);
  
  if (task_node && task_node->is_task_complete()) {
    RCLCPP_INFO(node_->get_logger(), "Task C (recovery) completed");
    task_c_node_->deactivate();
    task_c_node_->cleanup();
    transition_to(MissionState::COMPLETED);
  }
  // Otherwise: task still running
}

void MissionExecutor::handle_completed_state()
{
  completed_ticks_++;

  if (completed_ticks_ >= COMPLETED_WAIT_TICKS) {
    RCLCPP_INFO(node_->get_logger(), "Restarting mission...");
    completed_ticks_ = 0;
    transition_to(MissionState::IDLE);
  }
}

std::string state_to_string(MissionState state)
{
  switch (state) {
    case MissionState::IDLE:
      return "IDLE";
    case MissionState::TASK_A:
      return "TASK_A";
    case MissionState::TASK_B:
      return "TASK_B";
    case MissionState::TASK_C:
      return "TASK_C";
    case MissionState::COMPLETED:
      return "COMPLETED";
    default:
      return "UNKNOWN";
  }
}

void MissionExecutor::transition_to(MissionState new_state)
{
  RCLCPP_INFO(node_->get_logger(), "FSM Transition: %s -> %s",
    state_to_string(current_state_).c_str(), state_to_string(new_state).c_str());

  current_state_ = new_state;
  
  // Activate lifecycle node when entering task state
  if (new_state == MissionState::TASK_A) {
    task_a_node_->configure();
    task_a_node_->activate();
  } else if (new_state == MissionState::TASK_B) {
    task_b_node_->configure();
    task_b_node_->activate();
  } else if (new_state == MissionState::TASK_C) {
    task_c_node_->configure();
    task_c_node_->activate();
  }

  // Publish state
  std_msgs::msg::String msg;
  msg.data = state_to_string(new_state);
  state_pub_->publish(msg);
}

}  // namespace mission_task_example
