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

#ifndef CH2_EXAMPLES__SUBSCRIBER_NODE_HPP_
#define CH2_EXAMPLES__SUBSCRIBER_NODE_HPP_

#include "std_msgs/msg/int32.hpp"

#include "rclcpp/rclcpp.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode();

  void subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr counter_subscription_;
};

#endif // CH2_EXAMPLES__SUBSCRIBER_NODE_HPP_
