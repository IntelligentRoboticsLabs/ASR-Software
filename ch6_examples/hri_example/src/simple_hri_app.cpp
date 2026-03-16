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
  // Crear el cliente HRI como componente (patrón de composición)
  // Úso de 'this': Pasamos el puntero del nodo directamente
  // - Es seguro porque el cliente es miembro del nodo (mismo ciclo de vida)
  // - No podemos usar shared_from_this() en el constructor
  // - El cliente usará este nodo para crear clientes, suscriptores, logging, etc.
  hri_client_ = std::make_shared<HRIClient>(this);
  
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
        if (!on_duty_) {
          hri_client_->start_speaking("Hola, ¿cómo te llamas?");
          on_duty_ = true;
        } else {
          if (hri_client_->is_speaking_done()) {
            if (hri_client_->get_speaking_result()) {
              RCLCPP_INFO(get_logger(), "Saludo completado");
              on_duty_ = false;
              current_state_ = State::LISTENING_NAME;
            } else {
              RCLCPP_ERROR(get_logger(), "Error en saludo");
              current_state_ = State::DONE;
            }
          }
        }        
      }
      break;

    case State::LISTENING_NAME:
      {
        if (!on_duty_) {
          RCLCPP_INFO(get_logger(), "Iniciando escucha...");
          hri_client_->start_listen();
          on_duty_ = true;
        } else {
          RCLCPP_INFO(get_logger(), "Escuchando...");
          if (hri_client_->is_listen_done()) {
            listened_text_ = hri_client_->get_listened_text();
            RCLCPP_INFO(get_logger(), "Texto escuchado: '%s'", listened_text_.c_str());
            on_duty_ = false;
            current_state_ = State::EXTRACTING;
          }
        }
      }
      break;
    
    case State::EXTRACTING:
      {
        if (!on_duty_) {
          RCLCPP_INFO(get_logger(), "Iniciando extracción de nombre del texto: '%s'", listened_text_.c_str());
          hri_client_->start_extract("nombre de la persona", listened_text_);
          on_duty_ = true;
        } else {
          RCLCPP_INFO(get_logger(), "Extrayendo información...");
          if (hri_client_->is_extract_done()) {
            user_name_ = hri_client_->get_extracted_info();
            if (!user_name_.empty()) {
              RCLCPP_INFO(get_logger(), "Nombre extraído: %s", user_name_.c_str());
              current_state_ = State::REPEATING;
            } else {
              RCLCPP_ERROR(get_logger(), "Error extrayendo nombre");
              current_state_ = State::DONE;
            }
            on_duty_ = false;
          }
        }
      }
      break;

    case State::REPEATING:
      {
        if (!on_duty_) {
          RCLCPP_INFO(get_logger(), "Repetiendo nombre para confirmación...");
          std::string confirm_text = "Entonces, ¿tu nombre es " + user_name_ + "?";
          hri_client_->start_speaking(confirm_text);
          on_duty_ = true;
        } else {
          if (hri_client_->is_speaking_done()) {
            if (hri_client_->get_speaking_result()) {
              RCLCPP_INFO(get_logger(), "Pregunta de confirmación completada");
              current_state_ = State::LISTENING_CONFIRMATION;
            } else {
              RCLCPP_ERROR(get_logger(), "Error en confirmación");
              current_state_ = State::DONE;
            }
            on_duty_ = false;
          }
        }
      }
      break;

    case State::LISTENING_CONFIRMATION:
       {
        if (!on_duty_) {
          RCLCPP_INFO(get_logger(), "Escuchando confirmación...");
          hri_client_->start_listen();
          on_duty_ = true;
        } else {
          RCLCPP_INFO(get_logger(), "Escuchando...");
          if (hri_client_->is_listen_done()) {
            listened_text_ = hri_client_->get_listened_text();
            RCLCPP_INFO(get_logger(), "Texto escuchado para confirmación: '%s'", listened_text_.c_str());
            on_duty_ = false;
            current_state_ = State::CONFIRMING;
          }
        }
      }
      break;

    case State::CONFIRMING:
      {
        if (!on_duty_) {
          RCLCPP_INFO(get_logger(), "Esperando confirmación del usuario...");
          hri_client_->start_yesno(listened_text_);
          on_duty_ = true;
        } else {
          if (hri_client_->is_yesno_done()) {
            yes_no_ = hri_client_->get_yesno_result();
            RCLCPP_INFO(get_logger(), "Resultado de confirmación: '%s'", yes_no_.c_str());
            if (yes_no_ == "YES") {
              RCLCPP_INFO(get_logger(), "Usuario confirmó que su nombre es %s", user_name_.c_str());
              current_state_ = State::BYE_SUCCESS;
            } else {
              RCLCPP_INFO(get_logger(), "Usuario negó que su nombre sea %s", user_name_.c_str());
              current_state_ = State::BYE_FAILURE;
            }
            on_duty_ = false;
          } else {
            RCLCPP_INFO(get_logger(), "Esperando respuesta de confirmación...");
          }
        }
      }
      break;

    case State::BYE_SUCCESS:
      {
        if (!on_duty_) {
          std::string bye_text = "Encantado de conocerte, " + user_name_ + ". ¡Adiós!";
          hri_client_->start_speaking(bye_text);
          on_duty_ = true;
        } else {
          if (hri_client_->is_speaking_done()) {
            RCLCPP_INFO(get_logger(), "Despedida completada");
            current_state_ = State::DONE;
            on_duty_ = false;
          }
        }
      }
      break;
    
    case State::BYE_FAILURE:
      {
        if (!on_duty_) {
          std::string bye_text = "Lo siento, no pude confirmar tu nombre. ¡Adiós!";
          hri_client_->start_speaking(bye_text);
          on_duty_ = true;
        } else {
          if (hri_client_->is_speaking_done()) {
            RCLCPP_INFO(get_logger(), "Despedida por fallo completada");
            current_state_ = State::DONE;
            on_duty_ = false;
          }
        }
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
