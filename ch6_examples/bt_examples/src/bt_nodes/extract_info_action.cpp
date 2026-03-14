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

#include "bt_examples/bt_nodes/extract_info_action.hpp"

ExtractInfoAction::ExtractInfoAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("Missing required node in blackboard");
  }
  extract_client_ = node_->create_client<Extract>("/extract_service");
}

BT::NodeStatus ExtractInfoAction::onStart()
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

  // Esperar a que el servidor de servicio esté disponible
  if (!extract_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Extract service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Crear la petición del servicio
  auto request = std::make_shared<Extract::Request>();
  request->interest = interest;
  request->text = full_text;

  RCLCPP_INFO(node_->get_logger(), "Extracting '%s' from: '%s'", interest.c_str(), full_text.c_str());

  // Enviar petición asíncrona
  future_ = extract_client_->async_send_request(request);
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExtractInfoAction::onRunning()
{
  // Verificar timeout (10 segundos)
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(10)) {
    RCLCPP_ERROR(node_->get_logger(), "Extract service timeout");
    future_.reset();
    return BT::NodeStatus::FAILURE;
  }

  // Verificar si el future está listo
  if (future_ && future_->future.valid() && 
      future_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    auto response = future_->future.get();
    future_.reset();
    
    if (response->result.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Extract service returned empty result");
      // Si no se extrajo nada útil, marcar como fallo
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Extracted: '%s'", response->result.c_str());
    setOutput("extracted_info", response->result);
    
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ExtractInfoAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "Extract action halted");
  future_.reset();
}
