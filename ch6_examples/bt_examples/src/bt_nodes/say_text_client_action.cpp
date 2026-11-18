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

  // Usar HRIClient para iniciar TTS (el timeout lo gestiona el cliente)
  hri_client_->start_speaking(text, getInput<double>("timeout").value_or(0.0));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SayTextClientAction::onRunning()
{
  if (!hri_client_->is_speaking_done()) {
    return BT::NodeStatus::RUNNING;
  }

  // Cómo terminó: COMPLETED, ERROR, TIMEOUT o CANCELED
  auto state = hri_client_->get_speaking_state();
  if (state == OperationState::COMPLETED) {
    RCLCPP_INFO(node_->get_logger(), "TTS completed successfully");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(node_->get_logger(), "TTS operation failed (%s)", to_string(state));
  return BT::NodeStatus::FAILURE;
}

void SayTextClientAction::onHalted()
{
  // Si el árbol interrumpe este nodo, el robot deja de hablar
  RCLCPP_WARN(node_->get_logger(), "TTS action halted");
  hri_client_->cancel_speaking();
}
