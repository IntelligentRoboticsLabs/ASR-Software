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

#include "bt_examples/listen_text_client_action.hpp"

ListenTextClientAction::ListenTextClientAction(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<HRIClient> hri_client)
: BT::StatefulActionNode(name, config),
  hri_client_(hri_client)
{
}

BT::NodeStatus ListenTextClientAction::onStart()
{
  RCLCPP_INFO(hri_client_->get_logger(), "Listening for speech...");

  // Usar HRIClient para iniciar STT
  hri_client_->start_listen();
  start_time_ = std::chrono::steady_clock::now();
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenTextClientAction::onRunning()
{
  // Verificar timeout (60 segundos, el STT puede tardar más)
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(60)) {
    RCLCPP_ERROR(hri_client_->get_logger(), "STT operation timeout");
    return BT::NodeStatus::FAILURE;
  }

  // Verificar si HRIClient terminó de escuchar
  if (hri_client_->is_listen_done()) {
    std::string recognized_text = hri_client_->get_listened_text();
    
    if (recognized_text.empty()) {
      RCLCPP_ERROR(hri_client_->get_logger(), "STT operation returned empty text");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(hri_client_->get_logger(), "Recognized: '%s'", recognized_text.c_str());
    
    // Escribir el texto reconocido en el puerto de salida
    setOutput("recognized_text", recognized_text);
    
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ListenTextClientAction::onHalted()
{
  RCLCPP_WARN(hri_client_->get_logger(), "STT action halted");
}
