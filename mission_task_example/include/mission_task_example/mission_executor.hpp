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

#ifndef MISSION_TASK_EXAMPLE__MISSION_EXECUTOR_HPP_
#define MISSION_TASK_EXAMPLE__MISSION_EXECUTOR_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace mission_task_example
{

enum class MissionState
{
  IDLE,
  TASK_A,
  TASK_B,
  TASK_C,
  COMPLETED
};

class MissionExecutor
{
public:
  explicit MissionExecutor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_a_node,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_b_node,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_c_node);

  void initialize();
  void update();
  MissionState get_current_state() const { return current_state_; }

private:
  void transition_to(MissionState new_state);
  void handle_idle_state();
  void handle_task_a_state();
  void handle_task_b_state();
  void handle_task_c_state();
  void handle_completed_state();

  rclcpp::Node::SharedPtr node_;
  MissionState current_state_;
  
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_a_node_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_b_node_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> task_c_node_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  int idle_ticks_;
  int completed_ticks_;
  static const int IDLE_WAIT_TICKS = 30;
  static const int COMPLETED_WAIT_TICKS = 50;
};

}  // namespace mission_task_example

#endif  // MISSION_TASK_EXAMPLE__MISSION_EXECUTOR_HPP_
