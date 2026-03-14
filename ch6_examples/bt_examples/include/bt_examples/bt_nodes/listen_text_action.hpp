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
#include <optional>

class ListenTextAction : public BT::StatefulActionNode
{
public:
  using SetBool = std_srvs::srv::SetBool;

  ListenTextAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("recognized_text", "Text recognized by ASR")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SetBool>::SharedPtr stt_client_;
  std::optional<rclcpp::Client<SetBool>::FutureAndRequestId> future_;
  std::chrono::steady_clock::time_point start_time_;
};

#endif  // BT_EXAMPLES__LISTEN_TEXT_ACTION_HPP_
