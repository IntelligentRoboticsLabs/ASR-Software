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

#ifndef HRI_EXAMPLE__HRI_CLIENT_HPP_
#define HRI_EXAMPLE__HRI_CLIENT_HPP_

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "simple_hri_interfaces/srv/speech.hpp"
#include "simple_hri_interfaces/srv/extract.hpp"
#include "simple_hri_interfaces/srv/yes_no.hpp"

class HRIClient : public rclcpp::Node
{
public:
  HRIClient();

  // Método para verificar disponibilidad de los servicios
  bool wait_for_services(std::chrono::seconds timeout = std::chrono::seconds(5));

  // ============ MÉTODOS ASÍNCRONOS ============
  
  // Iniciar Speech-to-Text (STT) - Escuchar y transcribir audio
  void start_listen();
  
  // Verificar si terminó de escuchar
  bool is_listen_done() const;
  
  // Obtener texto transcrito
  std::string get_listened_text() const;
  
  // Iniciar Text-to-Speech (TTS) - Convierte texto en audio y lo reproduce
  void start_speaking(const std::string & text);
  
  // Verificar si terminó de hablar
  bool is_speaking_done() const;
  
  // Obtener resultado de TTS (true si exitoso)
  bool get_speaking_result() const;
  
  // Iniciar extracción de información del audio
  void start_extract(const std::string & interest, const std::string & text = "");
  
  // Verificar si la extracción está lista
  bool is_extract_done() const;
  
  // Obtener información extraída
  std::string get_extracted_info() const;
  
  // Iniciar detección de yes/no
  void start_yesno(const std::string & text = "");
  
  // Verificar si la detección está lista
  bool is_yesno_done() const;
  
  // Obtener resultado yes/no
  std::string get_yesno_result() const;

  // Callback para el topic de texto escuchado (publicado por STT)
  void listened_text_callback(const std_msgs::msg::String::SharedPtr msg);

  // Obtener el último texto escuchado del topic
  std::string get_last_listened_text() const { return last_listened_text_; }

private:
  // Clientes de servicios
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stt_client_;
  rclcpp::Client<simple_hri_interfaces::srv::Speech>::SharedPtr tts_client_;
  rclcpp::Client<simple_hri_interfaces::srv::Extract>::SharedPtr extract_client_;
  rclcpp::Client<simple_hri_interfaces::srv::YesNo>::SharedPtr yesno_client_;

  // Suscriptor al topic de texto escuchado
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listened_text_sub_;

  // Último texto escuchado
  std::string last_listened_text_;
  
  // Estados de operaciones asíncronas
  enum class OperationState {
    IDLE,
    IN_PROGRESS,
    COMPLETED,
    ERROR
  };
  
  OperationState stt_state_ = OperationState::IDLE;
  std::string stt_text_;
  
  OperationState tts_state_ = OperationState::IDLE;
  bool tts_result_ = false;
  std::chrono::steady_clock::time_point tts_start_time_;
  std::chrono::milliseconds tts_expected_duration_{0};
  
  OperationState extract_state_ = OperationState::IDLE;
  std::string extracted_info_;
  
  OperationState yesno_state_ = OperationState::IDLE;
  std::string yesno_result_;
  
  // Futures para operaciones asíncronas
  std::shared_future<std_srvs::srv::SetBool::Response::SharedPtr> stt_future_;
  std::shared_future<simple_hri_interfaces::srv::Speech::Response::SharedPtr> tts_future_;
  std::shared_future<simple_hri_interfaces::srv::Extract::Response::SharedPtr> extract_future_;
  std::shared_future<simple_hri_interfaces::srv::YesNo::Response::SharedPtr> yesno_future_;
};

#endif  // HRI_EXAMPLE__HRI_CLIENT_HPP_
