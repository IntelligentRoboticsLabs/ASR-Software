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


#include "bt_examples/bt_nodes/listen_text_client_action.hpp"

ListenTextClientAction::ListenTextClientAction(
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

BT::NodeStatus ListenTextClientAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "Listening for speech...");

  // Usar HRIClient para iniciar STT: el servidor deja de grabar si nadie EMPIEZA a
  // hablar en max_wait segundos (TIMEOUT)
  hri_client_->start_listening(getInput<double>("max_wait").value_or(10.0));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenTextClientAction::onRunning()
{
  if (!hri_client_->is_listening_done()) {
    return BT::NodeStatus::RUNNING;
  }

  // Cómo terminó: COMPLETED, ERROR, TIMEOUT (nadie habló) o CANCELED
  auto state = hri_client_->get_listening_state();
  std::string recognized_text = hri_client_->get_listened_text();
  if (state != OperationState::COMPLETED || recognized_text.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "STT operation failed (%s)", to_string(state));
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Recognized: '%s'", recognized_text.c_str());

  // Escribir el texto reconocido en el puerto de salida
  setOutput("recognized_text", recognized_text);
  return BT::NodeStatus::SUCCESS;
}

void ListenTextClientAction::onHalted()
{
  // Si el árbol interrumpe este nodo, el robot deja de grabar
  RCLCPP_WARN(node_->get_logger(), "STT action halted");
  hri_client_->cancel_listening();
}
