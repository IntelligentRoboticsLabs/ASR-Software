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

#ifndef HRI_CLIENT__HRI_CLIENT_HPP_
#define HRI_CLIENT__HRI_CLIENT_HPP_

#include <chrono>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "simple_hri_interfaces/srv/speech.hpp"
#include "simple_hri_interfaces/srv/extract.hpp"
#include "simple_hri_interfaces/srv/yes_no.hpp"

// HRIClient: Cliente simplificado para interacciones de HRI (TTS, STT, etc.)
//
// DECISIÓN DE DISEÑO - Composición con puntero raw:
// - Usa puntero raw (rclcpp::Node*) en lugar de shared_ptr por las siguientes razones:
//   1. El cliente siempre será un miembro de un nodo (nunca vive más que el nodo)
//   2. Permite pasar 'this' directamente en el constructor del nodo padre
//   3. Evita problemas con shared_from_this() que no puede llamarse en constructores
//   4. No hay riesgo de dangling pointer porque el ciclo de vida está garantizado
class HRIClient
{
public:
  explicit HRIClient(rclcpp::Node* node);

  // Método para verificar disponibilidad de los servicios
  bool wait_for_services(std::chrono::seconds timeout = std::chrono::seconds(5));

  // ============ MÉTODOS ASÍNCRONOS ============
  
  // Iniciar Speech-to-Text (STT) - Escuchar y transcribir audio
  void start_listen();
  
  // Verificar si terminó de escuchar
  bool is_listen_done();
  
  // Obtener texto transcrito
  std::string get_listened_text() const;
  
  // Iniciar Text-to-Speech (TTS) - Convierte texto en audio y lo reproduce
  void start_speaking(const std::string & text);
  
  // Verificar si terminó de hablar
  bool is_speaking_done();
  
  // Obtener resultado de TTS (true si exitoso)
  bool get_speaking_result() const;
  
  // Iniciar extracción de información del audio
  void start_extract(const std::string & interest, const std::string & text = "");
  
  // Verificar si la extracción está lista
  bool is_extract_done();
  
  // Obtener información extraída
  std::string get_extracted_info() const;
  
  // Iniciar detección de yes/no
  void start_yesno(const std::string & text = "");
  
  // Verificar si la detección está lista
  bool is_yesno_done();
  
  // Obtener resultado yes/no
  std::string get_yesno_result() const;

  // Callback para el topic de texto escuchado (publicado por STT)
  void listened_text_callback(const std_msgs::msg::String::SharedPtr msg);

  // Obtener el último texto escuchado del topic
  std::string get_last_listened_text() const { return last_listened_text_; }

private:
  // Nodo de ROS2 que gestiona la comunicación
  // NOTA: Puntero raw porque el cliente siempre es miembro del nodo (lifetime garantizado)
  rclcpp::Node* node_;

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
  bool tts_service_responded_ = false;
  
  OperationState extract_state_ = OperationState::IDLE;
  std::string extracted_info_;
  
  OperationState yesno_state_ = OperationState::IDLE;
  std::string yesno_result_;
  
  // Futures para operaciones asíncronas
  std::optional<rclcpp::Client<std_srvs::srv::SetBool>::FutureAndRequestId> stt_future_;
  std::optional<rclcpp::Client<simple_hri_interfaces::srv::Speech>::FutureAndRequestId> tts_future_;
  std::optional<rclcpp::Client<simple_hri_interfaces::srv::Extract>::FutureAndRequestId> extract_future_;
  std::optional<rclcpp::Client<simple_hri_interfaces::srv::YesNo>::FutureAndRequestId> yesno_future_;
};

#endif  // HRI_CLIENT__HRI_CLIENT_HPP_
