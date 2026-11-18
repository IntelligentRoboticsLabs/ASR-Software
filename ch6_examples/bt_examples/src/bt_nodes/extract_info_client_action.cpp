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

BT::NodeStatus ExtractInfoClientAction::onStart()
{
  // Obtener el interés del puerto de entrada
  std::string interest;
  if (!getInput("interest", interest)) {
    RCLCPP_ERROR(
      node_->get_logger(), "ExtractInfoClientAction: 'interest' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  // Obtener el texto completo del puerto de entrada (Extract no graba audio)
  std::string full_text;
  if (!getInput("full_text", full_text)) {
    RCLCPP_ERROR(
      node_->get_logger(), "ExtractInfoClientAction: 'full_text' input port is required");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Extracting '%s' from: '%s'", interest.c_str(), full_text.c_str());

  // Usar HRIClient para iniciar extracción (el timeout lo gestiona el cliente)
  hri_client_->start_extract(interest, full_text, getInput<double>("timeout").value_or(30.0));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExtractInfoClientAction::onRunning()
{
  if (!hri_client_->is_extract_done()) {
    return BT::NodeStatus::RUNNING;
  }

  // COMPLETED incluye "NONE" (no había nada relevante): para este nodo es un fallo
  auto state = hri_client_->get_extract_state();
  std::string extracted_info = hri_client_->get_extracted_info();
  if (state != OperationState::COMPLETED || extracted_info == "NONE") {
    RCLCPP_WARN(
      node_->get_logger(), "Extract found nothing (%s): '%s'",
      to_string(state), extracted_info.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Extracted: '%s'", extracted_info.c_str());
  setOutput("extracted_info", extracted_info);
  return BT::NodeStatus::SUCCESS;
}

void ExtractInfoClientAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "Extract action halted");
  hri_client_->cancel_extract();
}
