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

// Aplicación HRI modelada como FSM: pregunta el nombre, lo extrae del texto escuchado y
// pide confirmación (sí/no).
//
// Cada estado usa una operación de HRIClient con el patrón NO BLOQUEANTE:
//   - primera vez en el estado (!on_duty_): start_*() y on_duty_ = true
//   - siguientes ciclos: is_*_done(); al terminar, get_*_state() decide la transición
//
// Escuchar y hablar son acciones de simple_hri; extraer y sí/no son servicios.

namespace
{

const char * state_name(int state)
{
  static const char * names[] = {
    "WAITING_FOR_SERVICES", "GREETING", "LISTENING_NAME", "LISTENING_CONFIRMATION",
    "EXTRACTING", "REPEATING", "CONFIRMING", "BYE_SUCCESS", "BYE_FAILURE", "DONE"};
  return names[state];
}

}  // namespace

SimpleHRIApp::SimpleHRIApp()
: Node("simple_hri_app")
{
  // Crear el cliente HRI como componente (patrón de composición)
  // Uso de 'this': Pasamos el puntero del nodo directamente
  // - Es seguro porque el cliente es miembro del nodo (mismo ciclo de vida)
  // - No podemos usar shared_from_this() en el constructor
  // - El cliente usará este nodo para crear clientes, suscriptores, logging, etc.
  hri_client_ = std::make_shared<HRIClient>(this);

  RCLCPP_INFO(get_logger(), "Aplicación HRI iniciada");

  // Timer para gestionar el flujo de la aplicación (10 Hz)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SimpleHRIApp::control_cycle, this));
}

void SimpleHRIApp::go_to_state(State new_state)
{
  RCLCPP_INFO(
    get_logger(), "Estado: %s -> %s",
    state_name(static_cast<int>(current_state_)), state_name(static_cast<int>(new_state)));
  current_state_ = new_state;
  on_duty_ = false;
}

void SimpleHRIApp::control_cycle()
{
  switch (current_state_) {
    case State::WAITING_FOR_SERVICES:
      {
        if (hri_client_->wait_until_ready(std::chrono::seconds(1))) {
          RCLCPP_INFO(get_logger(), "Servicios HRI disponibles");
          go_to_state(State::GREETING);
        }
      }
      break;

    case State::GREETING:
      {
        if (!on_duty_) {
          hri_client_->start_speaking(question_);
          on_duty_ = true;
        } else if (hri_client_->is_speaking_done()) {
          // Escuchar SOLO cuando el robot ha terminado de hablar: si no, el micrófono
          // grabaría la voz del propio robot
          if (hri_client_->get_speaking_state() == OperationState::COMPLETED) {
            go_to_state(State::LISTENING_NAME);
          } else {
            RCLCPP_ERROR(get_logger(), "Error en el saludo");
            go_to_state(State::DONE);
          }
        }
      }
      break;

    case State::LISTENING_NAME:
      {
        if (!on_duty_) {
          listen_attempts_++;
          hri_client_->start_listening(LISTEN_TIMEOUT);
          on_duty_ = true;
        } else if (hri_client_->is_listening_done()) {
          if (hri_client_->get_listening_state() == OperationState::COMPLETED) {
            listened_text_ = hri_client_->get_listened_text();
            RCLCPP_INFO(get_logger(), "Texto escuchado: '%s'", listened_text_.c_str());
            go_to_state(State::EXTRACTING);
          } else if (listen_attempts_ < MAX_LISTEN_ATTEMPTS) {
            // TIMEOUT (nadie habló) o ERROR: reformular la pregunta y volver a escuchar
            RCLCPP_WARN(
              get_logger(), "No se ha oído nada (%s), se repite la pregunta",
              to_string(hri_client_->get_listening_state()));
            question_ = "No te he oído. ¿Me dices tu nombre?";
            go_to_state(State::GREETING);
          } else {
            go_to_state(State::BYE_FAILURE);
          }
        } else {
          // Feedback de la acción: "listening", "speech_detected", "transcribing"
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000, "Escuchando... (%s)",
            hri_client_->get_listening_feedback().c_str());
        }
      }
      break;

    case State::EXTRACTING:
      {
        if (!on_duty_) {
          hri_client_->start_extract("nombre", listened_text_);
          on_duty_ = true;
        } else if (hri_client_->is_extract_done()) {
          user_name_ = hri_client_->get_extracted_info();
          // Extract devuelve "NONE" si el texto no contiene un nombre
          if (hri_client_->get_extract_state() == OperationState::COMPLETED &&
            user_name_ != "NONE")
          {
            RCLCPP_INFO(get_logger(), "Nombre extraído: %s", user_name_.c_str());
            go_to_state(State::REPEATING);
          } else {
            RCLCPP_ERROR(get_logger(), "No se ha podido extraer el nombre");
            go_to_state(State::BYE_FAILURE);
          }
        }
      }
      break;

    case State::REPEATING:
      {
        if (!on_duty_) {
          hri_client_->start_speaking("Entonces, ¿tu nombre es " + user_name_ + "?");
          on_duty_ = true;
        } else if (hri_client_->is_speaking_done()) {
          if (hri_client_->get_speaking_state() == OperationState::COMPLETED) {
            go_to_state(State::LISTENING_CONFIRMATION);
          } else {
            RCLCPP_ERROR(get_logger(), "Error en la pregunta de confirmación");
            go_to_state(State::DONE);
          }
        }
      }
      break;

    case State::LISTENING_CONFIRMATION:
      {
        if (!on_duty_) {
          hri_client_->start_listening(LISTEN_TIMEOUT);
          on_duty_ = true;
        } else if (hri_client_->is_listening_done()) {
          if (hri_client_->get_listening_state() == OperationState::COMPLETED) {
            listened_text_ = hri_client_->get_listened_text();
            RCLCPP_INFO(get_logger(), "Respuesta escuchada: '%s'", listened_text_.c_str());
            go_to_state(State::CONFIRMING);
          } else {
            RCLCPP_WARN(get_logger(), "No se ha oído la confirmación");
            go_to_state(State::BYE_FAILURE);
          }
        }
      }
      break;

    case State::CONFIRMING:
      {
        if (!on_duty_) {
          hri_client_->start_yesno(listened_text_);
          on_duty_ = true;
        } else if (hri_client_->is_yesno_done()) {
          // COMPLETED con "YES" o "NO"; ERROR con "UNKNOWN" (duda) o "ERROR..."
          yes_no_ = hri_client_->get_yesno_result();
          RCLCPP_INFO(get_logger(), "Resultado de confirmación: '%s'", yes_no_.c_str());
          if (hri_client_->get_yesno_state() == OperationState::COMPLETED && yes_no_ == "YES") {
            RCLCPP_INFO(get_logger(), "Confirmado: el nombre es %s", user_name_.c_str());
            go_to_state(State::BYE_SUCCESS);
          } else {
            RCLCPP_INFO(get_logger(), "No confirmado: el nombre no es %s", user_name_.c_str());
            go_to_state(State::BYE_FAILURE);
          }
        }
      }
      break;

    case State::BYE_SUCCESS:
      {
        if (!on_duty_) {
          hri_client_->start_speaking("Encantado de conocerte, " + user_name_ + ". ¡Adiós!");
          on_duty_ = true;
        } else if (hri_client_->is_speaking_done()) {
          go_to_state(State::DONE);
        }
      }
      break;

    case State::BYE_FAILURE:
      {
        if (!on_duty_) {
          hri_client_->start_speaking("Lo siento, no he podido saber tu nombre. ¡Adiós!");
          on_duty_ = true;
        } else if (hri_client_->is_speaking_done()) {
          go_to_state(State::DONE);
        }
      }
      break;

    case State::DONE:
      {
        RCLCPP_INFO(get_logger(), "Aplicación finalizada");
        timer_->cancel();
        rclcpp::shutdown();  // rclcpp::spin() retorna en main
      }
      break;
  }
}
