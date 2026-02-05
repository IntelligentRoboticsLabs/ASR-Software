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
#include <vector>

SayTextAction::SayTextAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
  tts_client_ = node_->create_client<Speech>("/tts_service");
}

std::string SayTextAction::formatText(const std::string & text)
{
  std::string formatted_text = text;

  // Expandir manualmente las referencias del blackboard en el texto
  // BehaviorTree.CPP no expande {variables} dentro de strings literales automáticamente
  size_t pos = 0;
  while ((pos = formatted_text.find('{', pos)) != std::string::npos) {
    size_t end_pos = formatted_text.find('}', pos);
    if (end_pos == std::string::npos) break;
    
    std::string var_name = formatted_text.substr(pos + 1, end_pos - pos - 1);
    BT::Expected<std::string> var_value = config().blackboard->get<std::string>(var_name);
    
    if (var_value) {
      formatted_text.replace(pos, end_pos - pos + 1, var_value.value());
      pos += var_value.value().length();
    } else {
      pos = end_pos + 1;
    }
  }

  // Formatear listas separadas por punto y coma (ej: "agua;café;té" -> "agua, café y té")
  if (formatted_text.find(';') != std::string::npos) {
    std::vector<std::string> items;
    size_t start = 0;
    size_t end;
    
    while ((end = formatted_text.find(';', start)) != std::string::npos) {
      items.push_back(formatted_text.substr(start, end - start));
      start = end + 1;
    }
    items.push_back(formatted_text.substr(start));
    
    if (items.size() > 1) {
      formatted_text.clear();
      for (size_t i = 0; i < items.size(); ++i) {
        if (i > 0) {
          if (i == items.size() - 1) {
            formatted_text += " y ";
          } else {
            formatted_text += ", ";
          }
        }
        formatted_text += items[i];
      }
    }
  }

  return formatted_text;
}

BT::NodeStatus SayTextAction::tick()
{
  // Obtener el texto a decir del puerto de entrada
  std::string text;
  if (!getInput("text", text)) {
    RCLCPP_ERROR(node_->get_logger(), "SayTextAction: 'text' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  // Formatear el texto (expandir variables del blackboard y formatear listas)
  text = formatText(text);

  // Esperar a que el servidor esté disponible con reintentos
  const int max_wait_attempts = 3;
  bool service_available = false;
  
  for (int attempt = 1; attempt <= max_wait_attempts; ++attempt) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for TTS service... (attempt %d/%d)", attempt, max_wait_attempts);
    if (tts_client_->wait_for_service(std::chrono::seconds(5))) {
      service_available = true;
      break;
    }
    if (attempt < max_wait_attempts) {
      RCLCPP_WARN(node_->get_logger(), "TTS service not available yet, retrying...");
    }
  }
  
  if (!service_available) {
    RCLCPP_ERROR(node_->get_logger(), "TTS service not available after %d attempts", max_wait_attempts);
    return BT::NodeStatus::FAILURE;
  }

  // Crear la petición
  auto request = std::make_shared<Speech::Request>();
  request->text = text;

  RCLCPP_INFO(node_->get_logger(), "Saying: '%s'", text.c_str());

  // Llamar al servicio con reintentos
  const int max_call_attempts = 3;
  std::shared_ptr<Speech::Response> response;
  bool call_successful = false;
  
  for (int attempt = 1; attempt <= max_call_attempts; ++attempt) {
    auto future = tts_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(30)) 
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      response = future.get();
      call_successful = true;
      break;
    } else {
      if (attempt < max_call_attempts) {
        RCLCPP_WARN(node_->get_logger(), "Failed to call TTS service (attempt %d/%d), retrying...", 
                    attempt, max_call_attempts);
      }
    }
  }
  
  if (!call_successful) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call TTS service after %d attempts", max_call_attempts);
    return BT::NodeStatus::FAILURE;
  }
  
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "TTS service failed: %s", response->debug.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "TTS completed successfully");
  return BT::NodeStatus::SUCCESS;
}
