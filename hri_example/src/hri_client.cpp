// Copyright 2024 Intelligent Robotics Lab
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

#include "hri_example/hri_client.hpp"

HRIClient::HRIClient()
: Node("hri_client")
{
  // Crear los clientes de servicios
  stt_client_ = create_client<std_srvs::srv::SetBool>("/stt_service");
  tts_client_ = create_client<simple_hri_interfaces::srv::Speech>("/tts_service");
  extract_client_ = create_client<simple_hri_interfaces::srv::Extract>("/extract_service");
  yesno_client_ = create_client<std_srvs::srv::SetBool>("/yesno_service");

  // Suscribirse al topic de texto escuchado
  listened_text_sub_ = create_subscription<std_msgs::msg::String>(
    "/listened_text",
    10,
    std::bind(&HRIClient::listened_text_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(get_logger(), "Cliente HRI inicializado");
}

bool HRIClient::wait_for_services(std::chrono::seconds timeout)
{
  bool all_ready = true;

  if (!stt_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(get_logger(), "Servicio STT no disponible");
    all_ready = false;
  }

  if (!tts_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(get_logger(), "Servicio TTS no disponible");
    all_ready = false;
  }

  if (!extract_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(get_logger(), "Servicio Extract no disponible");
    all_ready = false;
  }

  if (!yesno_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(get_logger(), "Servicio YesNo no disponible");
    all_ready = false;
  }

  if (all_ready) {
    RCLCPP_DEBUG(get_logger(), "Todos los servicios HRI disponibles");
  }

  return all_ready;
}

bool HRIClient::call_stt_service(std::string & transcribed_text)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  RCLCPP_DEBUG(get_logger(), "Llamando al servicio STT...");

  auto future = stt_client_->async_send_request(request);

  // Esperar la respuesta (bloqueante)
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      transcribed_text = response->message;
      RCLCPP_DEBUG(get_logger(), "STT exitoso: %s", transcribed_text.c_str());
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "STT falló: %s", response->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Error al llamar al servicio STT");
    return false;
  }
}

bool HRIClient::call_tts_service(const std::string & text)
{
  auto request = std::make_shared<simple_hri_interfaces::srv::Speech::Request>();
  request->text = text;

  RCLCPP_DEBUG(get_logger(), "Llamando al servicio TTS con: '%s'", text.c_str());

  auto future = tts_client_->async_send_request(request);

  // Esperar la respuesta (bloqueante)
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_DEBUG(get_logger(), "TTS exitoso");
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "TTS falló: %s", response->debug.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Error al llamar al servicio TTS");
    return false;
  }
}

bool HRIClient::call_extract_service(const std::string & interest, std::string & extracted_info)
{
  auto request = std::make_shared<simple_hri_interfaces::srv::Extract::Request>();
  request->interest = interest;
  request->text = "";  // Dejarlo vacío para que grabe audio

  RCLCPP_DEBUG(get_logger(), "Llamando al servicio Extract para obtener: %s", interest.c_str());

  auto future = extract_client_->async_send_request(request);

  // Esperar la respuesta (bloqueante)
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    extracted_info = response->result;
    
    if (!extracted_info.empty()) {
      RCLCPP_DEBUG(get_logger(), "Extract exitoso: %s", extracted_info.c_str());
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Extract no pudo extraer información");
      return false;
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Error al llamar al servicio Extract");
    return false;
  }
}

bool HRIClient::call_yesno_service(bool & user_said_yes)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  RCLCPP_DEBUG(get_logger(), "Llamando al servicio YesNo...");

  auto future = yesno_client_->async_send_request(request);

  // Esperar la respuesta (bloqueante)
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      std::string answer = response->message;
      // Convertir a minúsculas para comparar
      std::transform(answer.begin(), answer.end(), answer.begin(), ::tolower);
      
      user_said_yes = (answer.find("yes") != std::string::npos || 
                       answer.find("sí") != std::string::npos ||
                       answer.find("si") != std::string::npos);
      
      RCLCPP_DEBUG(get_logger(), "YesNo exitoso: %s", answer.c_str());
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "YesNo falló: %s", response->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Error al llamar al servicio YesNo");
    return false;
  }
}

void HRIClient::listened_text_callback(const std_msgs::msg::String::SharedPtr msg)
{
  last_listened_text_ = msg->data;
  RCLCPP_DEBUG(get_logger(), "Texto escuchado recibido: %s", last_listened_text_.c_str());
}
