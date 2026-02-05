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
#include "ch2_examples/SubscriberNode.hpp"

using namespace std::chrono_literals;

SubscriberNode::SubscriberNode()
: Node("subscriber_node")
{
  counter_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "counter", 10, std::bind(&SubscriberNode::subscription_callback, this, std::placeholders::_1));
}

void
SubscriberNode::subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received %d", msg->data);
}
