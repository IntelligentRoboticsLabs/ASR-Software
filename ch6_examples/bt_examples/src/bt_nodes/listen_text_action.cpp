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


#include "bt_examples/bt_nodes/listen_text_action.hpp"

ListenTextAction::ListenTextAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("Missing required node in blackboard");
  }
  stt_client_ = rclcpp_action::create_client<Listen>(node_, "/stt_action");
}

BT::NodeStatus ListenTextAction::onStart()
{
  // Esperar a que el servidor de la acción esté disponible
  if (!stt_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "STT action (/stt_action) not available");
    return BT::NodeStatus::FAILURE;
  }

  // El servidor deja de grabar si nadie EMPIEZA a hablar en max_wait segundos
  Listen::Goal goal;
  goal.max_wait = static_cast<float>(getInput<double>("max_wait").value_or(10.0));

  RCLCPP_INFO(node_->get_logger(), "Listening for speech...");

  // Enviar el goal SIN BLOQUEAR: onRunning() consulta si ha terminado
  call_ = std::make_unique<hri_client::ActionCall<Listen>>(stt_client_, goal);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenTextAction::onRunning()
{
  auto finished = call_->poll();
  if (!finished) {
    return BT::NodeStatus::RUNNING;  // Escuchando o transcribiendo
  }
  call_.reset();

  const auto & result = finished->result;
  if (finished->code != rclcpp_action::ResultCode::SUCCEEDED || !result || !result->success) {
    if (result && result->timed_out) {
      RCLCPP_WARN(node_->get_logger(), "STT timed out: nobody spoke");
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "STT failed: %s", result ? result->message.c_str() : "goal rejected");
    }
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Recognized: '%s'", result->text.c_str());

  // Escribir el texto reconocido en el puerto de salida
  setOutput("recognized_text", result->text);
  return BT::NodeStatus::SUCCESS;
}

void ListenTextAction::onHalted()
{
  // Si el árbol interrumpe este nodo, el robot deja de grabar
  RCLCPP_WARN(node_->get_logger(), "STT action halted");
  if (call_) {
    call_->cancel();
    call_.reset();
  }
}
