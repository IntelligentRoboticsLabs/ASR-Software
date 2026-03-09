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

#include "bt_examples/bt_nodes/extract_info_client_action.hpp"

ExtractInfoClientAction::ExtractInfoClientAction(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<HRIClient> hri_client)
: BT::StatefulActionNode(name, config),
  hri_client_(hri_client)
{
}

BT::NodeStatus ExtractInfoClientAction::onStart()
{
  // Obtener el interés del puerto de entrada
  std::string interest;
  if (!getInput("interest", interest)) {
    RCLCPP_ERROR(hri_client_->get_logger(), "ExtractInfoClientAction: 'interest' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  // Obtener el texto completo del puerto de entrada
  std::string full_text;
  if (!getInput("full_text", full_text)) {
    RCLCPP_ERROR(hri_client_->get_logger(), "ExtractInfoClientAction: 'full_text' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(hri_client_->get_logger(), "Extracting '%s' from: '%s'", interest.c_str(), full_text.c_str());

  // Usar HRIClient para iniciar extracción
  hri_client_->start_extract(interest, full_text);
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExtractInfoClientAction::onRunning()
{
  // Verificar timeout (10 segundos)
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(10)) {
    RCLCPP_ERROR(hri_client_->get_logger(), "Extract operation timeout");
    return BT::NodeStatus::FAILURE;
  }

  // Verificar si HRIClient terminó la extracción
  if (hri_client_->is_extract_done()) {
    std::string extracted_info = hri_client_->get_extracted_info();
    
    if (extracted_info.empty()) {
      RCLCPP_WARN(hri_client_->get_logger(), "Extract operation returned empty result");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(hri_client_->get_logger(), "Extracted: '%s'", extracted_info.c_str());
    setOutput("extracted_info", extracted_info);
    
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ExtractInfoClientAction::onHalted()
{
  RCLCPP_WARN(hri_client_->get_logger(), "Extract action halted");
}
