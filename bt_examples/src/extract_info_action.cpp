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

#include "bt_examples/extract_info_action.hpp"

ExtractInfoAction::ExtractInfoAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
  extract_client_ = node_->create_client<Extract>("/extract_service");
}

BT::NodeStatus ExtractInfoAction::tick()
{
  // Obtener el interés del puerto de entrada
  std::string interest;
  if (!getInput("interest", interest)) {
    RCLCPP_ERROR(node_->get_logger(), "ExtractInfoAction: 'interest' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  // Obtener el texto completo del puerto de entrada
  std::string full_text;
  if (!getInput("full_text", full_text)) {
    RCLCPP_ERROR(node_->get_logger(), "ExtractInfoAction: 'full_text' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  // Esperar a que el servidor de servicio esté disponible con reintentos
  const int max_wait_attempts = 3;
  bool service_available = false;
  
  for (int attempt = 1; attempt <= max_wait_attempts; ++attempt) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for Extract service... (attempt %d/%d)", attempt, max_wait_attempts);
    if (extract_client_->wait_for_service(std::chrono::seconds(5))) {
      service_available = true;
      break;
    }
    if (attempt < max_wait_attempts) {
      RCLCPP_WARN(node_->get_logger(), "Extract service not available yet, retrying...");
    }
  }
  
  if (!service_available) {
    RCLCPP_ERROR(node_->get_logger(), "Extract service not available after %d attempts", max_wait_attempts);
    return BT::NodeStatus::FAILURE;
  }

  // Crear la petición del servicio
  auto request = std::make_shared<Extract::Request>();
  request->interest = interest;
  request->text = full_text;

  RCLCPP_INFO(node_->get_logger(), "Extracting '%s' from: '%s'", interest.c_str(), full_text.c_str());

  // Llamar al servicio con reintentos
  const int max_call_attempts = 3;
  std::shared_ptr<Extract::Response> response;
  bool call_successful = false;
  
  for (int attempt = 1; attempt <= max_call_attempts; ++attempt) {
    auto future = extract_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(10)) 
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      response = future.get();
      call_successful = true;
      break;
    } else {
      if (attempt < max_call_attempts) {
        RCLCPP_WARN(node_->get_logger(), "Failed to call extract service (attempt %d/%d), retrying...", 
                    attempt, max_call_attempts);
      }
    }
  }
  
  if (!call_successful) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call extract service after %d attempts", max_call_attempts);
    return BT::NodeStatus::FAILURE;
  }
  
  if (response->result.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Extract service returned empty result");
    // Si no se extrajo nada útil, usar el texto original
    setOutput("extracted_info", full_text);
  } else {
    RCLCPP_INFO(node_->get_logger(), "Extracted: '%s'", response->result.c_str());
    setOutput("extracted_info", response->result);
  }

  return BT::NodeStatus::SUCCESS;
}
