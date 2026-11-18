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


#ifndef BT_EXAMPLES__SAY_TEXT_CLIENT_ACTION_HPP_
#define BT_EXAMPLES__SAY_TEXT_CLIENT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "hri_client/hri_client.hpp"

// SayTextClient: como SayText, pero a través de HRIClient (compartido en la blackboard).
// Termina cuando el audio termina de verdad; halt -> cancel_speaking() (corta el audio).
class SayTextClientAction : public BT::StatefulActionNode
{
public:
  SayTextClientAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("text", "Text to say"),
      BT::InputPort<double>("timeout", 0.0, "Seconds to finish speaking (0 = no limit)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string formatText(const std::string & text);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<HRIClient> hri_client_;
};

#endif  // BT_EXAMPLES__SAY_TEXT_CLIENT_ACTION_HPP_
