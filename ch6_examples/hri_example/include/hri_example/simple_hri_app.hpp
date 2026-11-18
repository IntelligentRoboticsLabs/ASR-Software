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

#ifndef HRI_EXAMPLE__SIMPLE_HRI_APP_HPP_
#define HRI_EXAMPLE__SIMPLE_HRI_APP_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "hri_client/hri_client.hpp"

class SimpleHRIApp : public rclcpp::Node
{
public:
  SimpleHRIApp();

private:
  void control_cycle();

  std::shared_ptr<HRIClient> hri_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Variables de control de la FSM
  enum class State {
    WAITING_FOR_SERVICES,
    GREETING,
    LISTENING_NAME,
    LISTENING_CONFIRMATION,
    EXTRACTING,
    REPEATING,
    CONFIRMING,
    BYE_SUCCESS,
    BYE_FAILURE,
    DONE
  };
  
  // Cambia de estado y lo registra en el log
  void go_to_state(State new_state);

  State current_state_ = State::WAITING_FOR_SERVICES;
  std::string user_name_, yes_no_;
  std::string listened_text_;
  bool on_duty_ = false;  // true: la operación del estado actual ya está lanzada

  // Pregunta del saludo y reintentos si no se oye a la persona
  std::string question_ = "Hola, ¿cómo te llamas?";
  int listen_attempts_ = 0;
  static constexpr int MAX_LISTEN_ATTEMPTS = 3;
  // Segundos para que la persona EMPIECE a hablar (lo aplica el servidor de STT)
  static constexpr double LISTEN_TIMEOUT = 8.0;
};

#endif  // HRI_EXAMPLE__SIMPLE_HRI_APP_HPP_
