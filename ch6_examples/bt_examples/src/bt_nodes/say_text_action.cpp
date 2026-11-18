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


#include "bt_examples/bt_nodes/say_text_action.hpp"
#include "bt_examples/text_utils.hpp"

SayTextAction::SayTextAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("Missing required node in blackboard");
  }
  tts_client_ = rclcpp_action::create_client<Say>(node_, "/tts_action");
}

std::string SayTextAction::formatText(const std::string & text)
{
  return bt_examples::formatText(text, config().blackboard);
}

BT::NodeStatus SayTextAction::onStart()
{
  // Obtener el texto a decir del puerto de entrada
  std::string text;
  if (!getInput("text", text)) {
    RCLCPP_ERROR(node_->get_logger(), "SayTextAction: 'text' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  // Formatear el texto (expandir variables del blackboard y formatear listas)
  text = formatText(text);

  // Esperar a que el servidor de la acción esté disponible
  if (!tts_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "TTS action (/tts_action) not available");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Saying: '%s'", text.c_str());

  // Enviar el goal SIN BLOQUEAR: onRunning() consulta si ha terminado
  Say::Goal goal;
  goal.text = text;
  call_ = std::make_unique<hri_client::ActionCall<Say>>(tts_client_, goal);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SayTextAction::onRunning()
{
  auto finished = call_->poll();
  if (!finished) {
    return BT::NodeStatus::RUNNING;  // Todavía hablando
  }
  call_.reset();

  // El resultado llega cuando termina la reproducción: no hay que estimar la duración
  const auto & result = finished->result;
  if (finished->code == rclcpp_action::ResultCode::SUCCEEDED && result && result->success) {
    RCLCPP_INFO(node_->get_logger(), "TTS completed");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    node_->get_logger(), "TTS failed: %s", result ? result->message.c_str() : "goal rejected");
  return BT::NodeStatus::FAILURE;
}

void SayTextAction::onHalted()
{
  // Si el árbol interrumpe este nodo, el robot deja de hablar
  RCLCPP_WARN(node_->get_logger(), "TTS action halted");
  if (call_) {
    call_->cancel();
    call_.reset();
  }
}
