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

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "simple_hri_interfaces/srv/speech.hpp"
#include "simple_hri_interfaces/srv/extract.hpp"

class HRIClient : public rclcpp::Node
{
public:
  HRIClient();

  // Método para verificar disponibilidad de los servicios
  bool wait_for_services(std::chrono::seconds timeout = std::chrono::seconds(5));

  // Speech-to-Text (STT) - Convierte audio en texto
  // Retorna true si el servicio tuvo éxito, el texto está en el parámetro transcribed_text
  bool call_stt_service(std::string & transcribed_text);

  // Text-to-Speech (TTS) - Convierte texto en audio y lo reproduce
  bool call_tts_service(const std::string & text);

  // Extract - Extrae información específica del audio usando LLM
  bool call_extract_service(const std::string & interest, std::string & extracted_info);

  // Yes/No - Detecta si el usuario dice "yes" o "no"
  bool call_yesno_service(bool & user_said_yes);

  // Callback para el topic de texto escuchado (publicado por STT)
  void listened_text_callback(const std_msgs::msg::String::SharedPtr msg);

  // Obtener el último texto escuchado del topic
  std::string get_last_listened_text() const { return last_listened_text_; }

private:
  // Clientes de servicios
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stt_client_;
  rclcpp::Client<simple_hri_interfaces::srv::Speech>::SharedPtr tts_client_;
  rclcpp::Client<simple_hri_interfaces::srv::Extract>::SharedPtr extract_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr yesno_client_;

  // Suscriptor al topic de texto escuchado
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listened_text_sub_;

  // Último texto escuchado
  std::string last_listened_text_;
};

#endif  // HRI_EXAMPLE__HRI_CLIENT_HPP_
