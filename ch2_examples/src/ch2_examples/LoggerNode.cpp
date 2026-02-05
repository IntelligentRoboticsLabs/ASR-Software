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

#include "rclcpp/rclcpp.hpp"
#include "ch2_examples/LoggerNode.hpp"

using namespace std::chrono_literals;

LoggerNode::LoggerNode()
: Node("logger_node")
{
  counter_ = 0;
  timer_ = create_wall_timer(
    500ms, std::bind(&LoggerNode::timer_callback, this));
}

void
LoggerNode::timer_callback()
{
  RCLCPP_INFO(get_logger(), "Hello %d", counter_++);
}
