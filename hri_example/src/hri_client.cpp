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
  yesno_client_ = create_client<simple_hri_interfaces::srv::YesNo>("/yesno_service");

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

// ============ IMPLEMENTACIÓN MÉTODOS ASÍNCRONOS ============

void HRIClient::start_listen()
{
  // Evitar reiniciar si ya hay una operación en progreso
  if (stt_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(get_logger(), "STT ya está en progreso, ignorando nueva petición");
    return;
  }
  
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  
  RCLCPP_INFO(get_logger(), "Iniciando escucha (STT)...");
  
  stt_state_ = OperationState::IN_PROGRESS;
  stt_text_.clear();
  stt_future_ = stt_client_->async_send_request(request);
}

bool HRIClient::is_listen_done()
{
  if (stt_state_ != OperationState::IN_PROGRESS) {
    // Devolver true si está completado O en error (ya terminó)
    return stt_state_ == OperationState::COMPLETED || 
           stt_state_ == OperationState::ERROR;
  }
  
  // Verificar si el future está listo
  if (stt_future_ && stt_future_->future.valid() && 
      stt_future_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    // Pro cesar resultado
    auto response = stt_future_->future.get();
    if (response->success) {
      stt_text_ = response->message;
      stt_state_ = OperationState::COMPLETED;
      RCLCPP_INFO(get_logger(), "STT completado: '%s'", stt_text_.c_str());
    } else {
      stt_state_ = OperationState::ERROR;
      RCLCPP_WARN(get_logger(), "STT falló: %s", response->message.c_str());
    }
    return true;
  }
  
  return false;
}

std::string HRIClient::get_listened_text() const
{
  return stt_text_;
}

void HRIClient::start_speaking(const std::string & text)
{
  // Evitar reiniciar si ya hay una operación en progreso
  if (tts_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(get_logger(), "TTS ya está en progreso, ignorando nueva petición");
    return;
  }
  
  auto request = std::make_shared<simple_hri_interfaces::srv::Speech::Request>();
  request->text = text;
  
  RCLCPP_INFO(get_logger(), "Iniciando TTS: '%s'", text.c_str());
  
  tts_state_ = OperationState::IN_PROGRESS;
  tts_start_time_ = std::chrono::steady_clock::now();
  
  // Estimar duración basada en el tamaño del texto
  // Aproximadamente 10 caracteres por segundo (incluye pausas por puntuación)
  int text_length = text.length();
  int duration_ms = (text_length * 100) + 500; // +500ms de margen
  tts_expected_duration_ = std::chrono::milliseconds(duration_ms);
  
  RCLCPP_DEBUG(get_logger(), "Duración estimada de TTS: %d ms", duration_ms);
  
  tts_future_ = tts_client_->async_send_request(request);
}

bool HRIClient::is_speaking_done()
{
  if (tts_state_ != OperationState::IN_PROGRESS) {
    // Devolver true si está completado O en error (ya terminó)
    return tts_state_ == OperationState::COMPLETED || 
           tts_state_ == OperationState::ERROR;
  }
  
  // Primero verificar si el servicio ha respondido
  bool service_responded = false;
  if (tts_future_ && tts_future_->future.valid() && 
      tts_future_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    auto response = tts_future_->future.get();
    if (!response->success) {
      // Si el servicio falló, marcar como error inmediatamente
      tts_result_ = false;
      tts_state_ = OperationState::ERROR;
      RCLCPP_ERROR(get_logger(), "TTS falló");
      return true;
    }
    tts_result_ = true;
    service_responded = true;
  }
  
  // Verificar si ha pasado el tiempo estimado de habla
  auto elapsed = std::chrono::steady_clock::now() - tts_start_time_;
  
  if (elapsed >= tts_expected_duration_ && service_responded) {
    // El servicio respondió Y ha pasado el tiempo estimado
    tts_state_ = OperationState::COMPLETED;
    RCLCPP_INFO(get_logger(), "TTS completado (duración: %ld ms)", 
                std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
    return true;
  }
  
  return false;
}

bool HRIClient::get_speaking_result() const
{
  return tts_result_;
}

void HRIClient::start_extract(const std::string & interest, const std::string & text)
{
  // Evitar reiniciar si ya hay una operación en progreso
  if (extract_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(get_logger(), "Extract ya está en progreso, ignorando nueva petición");
    return;
  }
  
  auto request = std::make_shared<simple_hri_interfaces::srv::Extract::Request>();
  request->interest = interest;
  request->text = text;  // Si está vacío, el servicio grabará audio; si no, usa este texto
  
  if (text.empty()) {
    RCLCPP_INFO(get_logger(), "Iniciando extracción con audio: %s", interest.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Iniciando extracción de texto '%s': %s", text.c_str(), interest.c_str());
  }
  
  extract_state_ = OperationState::IN_PROGRESS;
  extracted_info_.clear();
  extract_future_ = extract_client_->async_send_request(request);
}

bool HRIClient::is_extract_done()
{
  if (extract_state_ != OperationState::IN_PROGRESS) {
    // Devolver true si está completado O en error (ya terminó)
    return extract_state_ == OperationState::COMPLETED || 
           extract_state_ == OperationState::ERROR;
  }
  
  // Verificar si el future está listo
  if (extract_future_ && extract_future_->future.valid() && 
      extract_future_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    // Procesar resultado
    auto response = extract_future_->future.get();
    extracted_info_ = response->result;
    
    if (!response->result.empty()) {
      extract_state_ = OperationState::COMPLETED;
      RCLCPP_INFO(get_logger(), "Extracción completada: %s", extracted_info_.c_str());
    } else {
      extract_state_ = OperationState::ERROR;
      RCLCPP_WARN(get_logger(), "Extracción no pudo obtener información");
    }
    return true;
  }
  
  return false;
}

std::string HRIClient::get_extracted_info() const
{
  return extracted_info_;
}

void HRIClient::start_yesno(const std::string & text)
{
  // Evitar reiniciar si ya hay una operación en progreso
  if (yesno_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(get_logger(), "YesNo ya está en progreso, ignorando nueva petición");
    return;
  }
  
  auto request = std::make_shared<simple_hri_interfaces::srv::YesNo::Request>();
  request->text = text;  // Si está vacío, el servicio grabará audio; si no, usa este texto
  
  if (text.empty()) {
    RCLCPP_INFO(get_logger(), "Iniciando detección yes/no con audio...");
  } else {
    RCLCPP_INFO(get_logger(), "Iniciando detección yes/no de texto '%s'", text.c_str());
  }
  
  yesno_state_ = OperationState::IN_PROGRESS;
  yesno_future_ = yesno_client_->async_send_request(request);
}

bool HRIClient::is_yesno_done()
{
  if (yesno_state_ != OperationState::IN_PROGRESS) {
    // Devolver true si está completado O en error (ya terminó)
    return yesno_state_ == OperationState::COMPLETED || 
           yesno_state_ == OperationState::ERROR;
  }
  
  // Verificar si el future está listo
  if (yesno_future_ && yesno_future_->future.valid() && 
      yesno_future_->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    // Procesar resultado
    auto response = yesno_future_->future.get();
    yesno_result_ = response->result;
    std::string answer_lower = response->result;
    std::transform(answer_lower.begin(), answer_lower.end(), answer_lower.begin(), ::tolower);
    if (answer_lower == "yes" || answer_lower == "no") {
      yesno_state_ = OperationState::COMPLETED;
      RCLCPP_INFO(get_logger(), "YesNo completado: %s", response->result.c_str());
    } else {
      yesno_state_ = OperationState::ERROR;
      RCLCPP_WARN(get_logger(), "YesNo no pudo obtener respuesta válida: %s", response->result.c_str());
    }
    return true;
  }
  
  return false;
}

std::string HRIClient::get_yesno_result() const
{
  return yesno_result_;
}

void HRIClient::listened_text_callback(const std_msgs::msg::String::SharedPtr msg)
{
  last_listened_text_ = msg->data;
  RCLCPP_DEBUG(get_logger(), "Texto escuchado recibido: %s", last_listened_text_.c_str());
}
