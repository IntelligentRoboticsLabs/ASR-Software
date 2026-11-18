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


#ifndef BT_EXAMPLES__SAY_TEXT_ACTION_HPP_
#define BT_EXAMPLES__SAY_TEXT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <simple_hri_interfaces/action/say.hpp>

#include <memory>
#include <string>

#include "hri_client/action_call.hpp"

// SayText: el robot dice un texto con la ACCIÓN /tts_action de simple_hri.
// La acción termina cuando el audio termina de verdad; si el nodo se detiene (halt),
// se cancela el goal y el servidor corta el audio.
class SayTextAction : public BT::StatefulActionNode
{
public:
  using Say = simple_hri_interfaces::action::Say;

  SayTextAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("text", "Text to say")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string formatText(const std::string & text);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Say>::SharedPtr tts_client_;
  std::unique_ptr<hri_client::ActionCall<Say>> call_;
};

#endif  // BT_EXAMPLES__SAY_TEXT_ACTION_HPP_
