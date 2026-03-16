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

#include "bt_examples/bt_nodes/say_text_client_action.hpp"
#include "bt_examples/text_utils.hpp"

SayTextClientAction::SayTextClientAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("Missing required node in blackboard");
  }
  if (!config.blackboard->get("hri_client", hri_client_)) {
    throw BT::RuntimeError("Missing required hri_client in blackboard");
  }
}

std::string SayTextClientAction::formatText(const std::string & text)
{
  return bt_examples::formatText(text, config().blackboard);
}

BT::NodeStatus SayTextClientAction::onStart()
{
  // Obtener el texto a decir del puerto de entrada
  std::string text;
  if (!getInput("text", text)) {
    RCLCPP_ERROR(node_->get_logger(), "SayTextClientAction: 'text' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  // Formatear el texto (expandir variables del blackboard y formatear listas)
  text = formatText(text);

  RCLCPP_INFO(node_->get_logger(), "Saying: '%s'", text.c_str());

  // Usar HRIClient para iniciar TTS
  hri_client_->start_speaking(text);
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SayTextClientAction::onRunning()
{
  // Verificar timeout (30 segundos)
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(30)) {
    RCLCPP_ERROR(node_->get_logger(), "TTS operation timeout");
    return BT::NodeStatus::FAILURE;
  }

  // Verificar si HRIClient terminó de hablar
  if (hri_client_->is_speaking_done()) {
    if (hri_client_->get_speaking_result()) {
      RCLCPP_INFO(node_->get_logger(), "TTS completed successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "TTS operation failed");
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void SayTextClientAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "TTS action halted");
}
