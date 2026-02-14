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

#include "bt_examples/say_text_action.hpp"
#include "bt_examples/text_utils.hpp"

SayTextAction::SayTextAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  tts_client_ = node_->create_client<Speech>("/tts_service");
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

  // Esperar a que el servidor esté disponible
  if (!tts_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "TTS service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Crear la petición
  auto request = std::make_shared<Speech::Request>();
  request->text = text;

  RCLCPP_INFO(node_->get_logger(), "Saying: '%s'", text.c_str());

  // Estimar duración basada en el tamaño del texto
  // Aproximadamente 10 caracteres por segundo (incluye pausas por puntuación)
  int text_length = text.length();
  int duration_ms = (text_length * 100) + 500; // +500ms de margen
  tts_expected_duration_ = std::chrono::milliseconds(duration_ms);
  
  RCLCPP_DEBUG(node_->get_logger(), "Duración estimada de TTS: %d ms", duration_ms);

  // Enviar petición asíncrona
  future_ = tts_client_->async_send_request(request);
  start_time_ = std::chrono::steady_clock::now();
  service_responded_ = false;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SayTextAction::onRunning()
{
  // Verificar si el servicio ha respondido (generación del audio)
  if (!service_responded_ && future_ && future_->future.valid() && 
      future_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    auto response = future_->future.get();
    future_.reset();
    
    if (!response->success) {
      RCLCPP_ERROR(node_->get_logger(), "TTS service failed: %s", response->debug.c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    service_responded_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "TTS service responded, waiting for audio playback...");
  }
  
  // Verificar si ha pasado el tiempo estimado de habla
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  
  // Timeout de seguridad (respuesta del servicio + reproducción)
  auto max_timeout = tts_expected_duration_ + std::chrono::seconds(5);
  if (elapsed > max_timeout) {
    RCLCPP_ERROR(node_->get_logger(), "TTS operation timeout");
    return BT::NodeStatus::FAILURE;
  }
  
  // Solo completar si el servicio respondió Y pasó el tiempo estimado
  if (service_responded_ && elapsed >= tts_expected_duration_) {
    RCLCPP_INFO(node_->get_logger(), "TTS completed (duration: %ld ms)", 
                std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void SayTextAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "TTS action halted");
  future_.reset();
  service_responded_ = false;
}
