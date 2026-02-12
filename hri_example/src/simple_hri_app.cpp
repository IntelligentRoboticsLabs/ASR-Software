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

#include "hri_example/simple_hri_app.hpp"

SimpleHRIApp::SimpleHRIApp()
: Node("simple_hri_app")
{
  // Crear el cliente HRI como componente
  hri_client_ = std::make_shared<HRIClient>();
  
  RCLCPP_INFO(get_logger(), "Aplicación HRI iniciada");
  
  // Timer para gestionar el flujo de la aplicación
  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&SimpleHRIApp::control_cycle, this));
}

void SimpleHRIApp::control_cycle()
{
  switch (current_state_) {
    case State::WAITING_FOR_SERVICES:
      {
        if (hri_client_->wait_for_services(std::chrono::seconds(1))) {
          RCLCPP_INFO(get_logger(), "Servicios HRI disponibles");
          current_state_ = State::GREETING;
        }
      }
      break;

    case State::GREETING:
      {
        RCLCPP_INFO(get_logger(), "Saludando al usuario...");
        if (hri_client_->call_tts_service("Hola, ¿cómo te llamas?")) {
          current_state_ = State::ASKING_NAME;
        } else {
          RCLCPP_ERROR(get_logger(), "Error en saludo");
          current_state_ = State::DONE;
        }
      }
      break;

    case State::ASKING_NAME:
      {
        RCLCPP_INFO(get_logger(), "Esperando nombre...");
        // Pasar directo a extraer (el servicio ya pedirá el audio)
        current_state_ = State::EXTRACTING_NAME;
      }
      break;

    case State::EXTRACTING_NAME:
      {
        RCLCPP_INFO(get_logger(), "Extrayendo nombre del usuario...");
        if (hri_client_->call_extract_service("person's name", user_name_)) {
          RCLCPP_INFO(get_logger(), "Nombre extraído: %s", user_name_.c_str());
          current_state_ = State::CONFIRMING;
        } else {
          RCLCPP_ERROR(get_logger(), "Error extrayendo nombre");
          current_state_ = State::DONE;
        }
      }
      break;

    case State::CONFIRMING:
      {
        std::string confirmation_msg = "Encantado de conocerte, " + user_name_ + 
                                       ". ¿He entendido bien tu nombre?";
        RCLCPP_INFO(get_logger(), "Confirmando nombre...");
        
        if (hri_client_->call_tts_service(confirmation_msg)) {
          current_state_ = State::CHECKING_CONFIRMATION;
        } else {
          RCLCPP_ERROR(get_logger(), "Error en confirmación");
          current_state_ = State::DONE;
        }
      }
      break;

    case State::CHECKING_CONFIRMATION:
      {
        RCLCPP_INFO(get_logger(), "Esperando confirmación...");
        bool confirmed = false;
        
        if (hri_client_->call_yesno_service(confirmed)) {
          if (confirmed) {
            RCLCPP_INFO(get_logger(), "Usuario confirmó el nombre");
            current_state_ = State::FAREWELL;
          } else {
            RCLCPP_INFO(get_logger(), "Usuario rechazó el nombre, pidiendo de nuevo...");
            current_state_ = State::ASKING_NAME;
          }
        } else {
          RCLCPP_ERROR(get_logger(), "Error verificando confirmación");
          current_state_ = State::DONE;
        }
      }
      break;

    case State::FAREWELL:
      {
        std::string farewell_msg = "¡Genial! Ha sido un placer hablar contigo, " + user_name_ + 
                                   ". ¡Que tengas un día maravilloso!";
        RCLCPP_INFO(get_logger(), "Despidiéndose...");
        
        if (hri_client_->call_tts_service(farewell_msg)) {
          RCLCPP_INFO(get_logger(), "Conversación completada exitosamente");
        } else {
          RCLCPP_ERROR(get_logger(), "Error en despedida");
        }
        
        current_state_ = State::DONE;
      }
      break;

    case State::DONE:
      {
        RCLCPP_INFO(get_logger(), "Aplicación finalizada");
        timer_->cancel();
      }
      break;
  }
}
