// Copyright 2025 Intelligent Robotics Lab
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

#ifndef BT_EXAMPLES__LISTEN_TEXT_ACTION_HPP_
#define BT_EXAMPLES__LISTEN_TEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ListenTextAction : public BT::SyncActionNode
{
public:
  using SetBool = std_srvs::srv::SetBool;

  ListenTextAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("recognized_text", "Text recognized by ASR")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SetBool>::SharedPtr stt_client_;
};

#endif  // BT_EXAMPLES__LISTEN_TEXT_ACTION_HPP_
