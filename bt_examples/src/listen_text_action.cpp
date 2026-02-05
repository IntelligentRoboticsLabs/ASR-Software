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
: BT::SyncActionNode(name, config),
  node_(node)
{
  stt_client_ = node_->create_client<SetBool>("/stt_service");
}

BT::NodeStatus ListenTextAction::tick()
{
  // Esperar a que el servidor esté disponible con reintentos
  const int max_wait_attempts = 3;
  bool service_available = false;
  
  for (int attempt = 1; attempt <= max_wait_attempts; ++attempt) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for STT service... (attempt %d/%d)", attempt, max_wait_attempts);
    if (stt_client_->wait_for_service(std::chrono::seconds(5))) {
      service_available = true;
      break;
    }
    if (attempt < max_wait_attempts) {
      RCLCPP_WARN(node_->get_logger(), "STT service not available yet, retrying...");
    }
  }
  
  if (!service_available) {
    RCLCPP_ERROR(node_->get_logger(), "STT service not available after %d attempts", max_wait_attempts);
    return BT::NodeStatus::FAILURE;
  }

  // Crear la petición (true para iniciar el reconocimiento)
  auto request = std::make_shared<SetBool::Request>();
  request->data = true;

  RCLCPP_INFO(node_->get_logger(), "Listening for speech...");

  // Llamar al servicio con reintentos (puede tardar varios segundos)
  const int max_call_attempts = 2;
  std::shared_ptr<SetBool::Response> response;
  bool call_successful = false;
  
  for (int attempt = 1; attempt <= max_call_attempts; ++attempt) {
    auto future = stt_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(60)) 
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      response = future.get();
      call_successful = true;
      break;
    } else {
      if (attempt < max_call_attempts) {
        RCLCPP_WARN(node_->get_logger(), "Failed to call STT service (timeout, attempt %d/%d), retrying...", 
                    attempt, max_call_attempts);
      }
    }
  }
  
  if (!call_successful) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call STT service after %d attempts", max_call_attempts);
    return BT::NodeStatus::FAILURE;
  }
  
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
