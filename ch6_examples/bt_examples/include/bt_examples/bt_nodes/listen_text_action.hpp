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
#include <rclcpp_action/rclcpp_action.hpp>
#include <simple_hri_interfaces/action/listen.hpp>

#include <memory>
#include <string>

#include "hri_client/action_call.hpp"

// ListenText: escucha a la persona y transcribe con la ACCIÓN /stt_action de simple_hri.
// FAILURE si nadie empieza a hablar en 'max_wait' segundos. Si el nodo se detiene (halt),
// se cancela el goal y el servidor deja de grabar.
class ListenTextAction : public BT::StatefulActionNode
{
public:
  using Listen = simple_hri_interfaces::action::Listen;

  ListenTextAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "max_wait", 10.0, "Seconds for the person to start speaking (0 = server default)"),
      BT::OutputPort<std::string>("recognized_text", "Text recognized by STT")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Listen>::SharedPtr stt_client_;
  std::unique_ptr<hri_client::ActionCall<Listen>> call_;
};

#endif  // BT_EXAMPLES__LISTEN_TEXT_ACTION_HPP_
