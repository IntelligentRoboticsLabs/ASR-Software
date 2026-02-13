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

#include "bt_examples/listen_text_action.hpp"

ListenTextAction::ListenTextAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  stt_client_ = node_->create_client<SetBool>("/stt_service");
}

BT::NodeStatus ListenTextAction::onStart()
{
  // Esperar a que el servidor esté disponible
  if (!stt_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "STT service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Crear la petición (true para iniciar el reconocimiento)
  auto request = std::make_shared<SetBool::Request>();
  request->data = true;

  RCLCPP_INFO(node_->get_logger(), "Listening for speech...");

  // Enviar petición asíncrona
  future_ = stt_client_->async_send_request(request);
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenTextAction::onRunning()
{
  // Verificar timeout (60 segundos, el STT puede tardar más)
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(60)) {
    RCLCPP_ERROR(node_->get_logger(), "STT service timeout");
    future_.reset();
    return BT::NodeStatus::FAILURE;
  }

  // Verificar si el future está listo
  if (future_ && future_->future.valid() && 
      future_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    auto response = future_->future.get();
    future_.reset();
    
    if (!response->success) {
      RCLCPP_ERROR(node_->get_logger(), "STT service failed: %s", response->message.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // El texto reconocido viene en response->message
    std::string recognized_text = response->message;
    RCLCPP_INFO(node_->get_logger(), "Recognized: '%s'", recognized_text.c_str());
    
    // Escribir el texto reconocido en el puerto de salida
    setOutput("recognized_text", recognized_text);
    
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ListenTextAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "STT action halted");
  future_.reset();
}
