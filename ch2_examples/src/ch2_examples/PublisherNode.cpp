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
#include "ch2_examples/PublisherNode.hpp"

using namespace std::chrono_literals;

PublisherNode::PublisherNode()
: Node("publisher_node")
{
  publisher_ = create_publisher<std_msgs::msg::Int32>("counter", 10);
  timer_ = create_wall_timer(
    100ms, std::bind(&PublisherNode::timer_callback, this));

  counter_message_.data = 0;
}

void
PublisherNode::timer_callback()
{
  RCLCPP_INFO(get_logger(), "Publishing %d", counter_message_.data++);
  publisher_->publish(counter_message_);
}
