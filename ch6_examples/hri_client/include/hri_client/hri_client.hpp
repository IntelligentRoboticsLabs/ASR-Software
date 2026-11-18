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
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "simple_hri_interfaces/action/listen.hpp"
#include "simple_hri_interfaces/action/say.hpp"
#include "simple_hri_interfaces/srv/extract.hpp"
#include "simple_hri_interfaces/srv/yes_no.hpp"

#include "hri_client/action_call.hpp"

// Estado de cada operación. Mismos estados que NavigationState (navigation_client):
// todas las capacidades cumplen el mismo contrato.
enum class OperationState
{
  IDLE,         // Nunca se ha lanzado
  IN_PROGRESS,  // Lanzada, sin terminar
  COMPLETED,    // Terminó bien
  ERROR,        // Terminó mal (fallo del servidor, respuesta no válida, goal rechazado)
  TIMEOUT,      // Se agotó el tiempo
  CANCELED      // La canceló el cliente (cancel_*()) o el servidor
};

// Nombre del estado, para los logs
const char * to_string(OperationState state);

// HRIClient: cliente de las capacidades de interacción de simple_hri
//
// - Escuchar (STT) y hablar (TTS) son ACCIONES (/stt_action, /tts_action): duran
//   segundos, dan feedback, se pueden cancelar y hablar termina cuando el audio
//   termina de verdad (no hay que estimar la duración).
// - Extraer información (Extract) y sí/no (YesNo) son SERVICIOS (/extract_service,
//   /yesno_service): consultas sobre un texto, normalmente el que devuelve STT.
//
// CONTRATO DE LA CAPACIDAD (igual que NavigationClient), NO BLOQUEANTE:
//   start_*()       lanza la operación y retorna inmediatamente
//   is_*_done()     se consulta en cada ciclo de control; true cuando ha terminado
//   get_*_state()   cómo terminó: COMPLETED, ERROR, TIMEOUT o CANCELED
//   get_*()         el resultado (texto escuchado, información extraída, sí/no)
//   cancel_*()      la termina con el estado CANCELED
//
// DECISIÓN DE DISEÑO - Composición con puntero raw:
// - Usa puntero raw (rclcpp::Node*) en lugar de shared_ptr por las siguientes razones:
//   1. El cliente siempre será un miembro de un nodo (nunca vive más que el nodo)
//   2. Permite pasar 'this' directamente en el constructor del nodo padre
//   3. Evita problemas con shared_from_this() que no puede llamarse en constructores
//   4. No hay riesgo de dangling pointer porque el ciclo de vida está garantizado
//
// Se asume que el nodo se ejecuta con un executor de un hilo (rclcpp::spin o spin_some):
// los callbacks y el ciclo de control nunca se ejecutan a la vez.
class HRIClient
{
public:
  using Listen = simple_hri_interfaces::action::Listen;
  using Say = simple_hri_interfaces::action::Say;
  using Extract = simple_hri_interfaces::srv::Extract;
  using YesNo = simple_hri_interfaces::srv::YesNo;

  explicit HRIClient(rclcpp::Node * node);

  // Espera a las acciones (escuchar, hablar) y a los servicios (extraer, sí/no)
  bool wait_until_ready(std::chrono::seconds timeout = std::chrono::seconds(5));

  // ============ ESCUCHAR (STT, acción /stt_action) ============

  // timeout_sec: tiempo para que la persona EMPIECE a hablar (el servidor deja de grabar
  // si nadie habla -> TIMEOUT); si habla, espera a que termine. 0 = valor por defecto
  // del servidor (10 s)
  void start_listening(double timeout_sec = 0.0);
  bool is_listening_done();
  std::string get_listened_text() const {return stt_text_;}
  // Fase de la escucha: "listening", "speech_detected", "transcribing" ("" sin feedback)
  std::string get_listening_feedback() const;

  // ============ HABLAR (TTS, acción /tts_action) ============

  // Termina cuando acaba de verdad la reproducción. timeout_sec: si tarda más, se
  // cancela (se corta el audio) -> TIMEOUT. 0 = sin límite
  void start_speaking(const std::string & text, double timeout_sec = 0.0);
  bool is_speaking_done();
  // Segundos de reproducción que faltan (-1.0 antes del primer feedback)
  float get_speaking_feedback() const;

  // ============ EXTRAER INFORMACIÓN (servicio /extract_service) ============

  // No graba audio: el texto se obtiene antes con start_listening()/get_listened_text().
  // timeout_sec: del cliente (un servicio no se puede cancelar: la respuesta tardía se
  // descarta). 0 = sin límite
  void start_extract(const std::string & interest, const std::string & text,
    double timeout_sec = 0.0);
  bool is_extract_done();
  // Valores separados por ';' (ej. "agua;cerveza") o "NONE" si no hay nada relevante
  std::string get_extracted_info() const {return extracted_info_;}

  // ============ SÍ/NO (servicio /yesno_service) ============

  // No graba audio: se le pasa el texto escuchado. 0 = sin límite
  void start_yesno(const std::string & text, double timeout_sec = 0.0);
  bool is_yesno_done();
  // "YES" o "NO" (COMPLETED); "UNKNOWN" o "ERROR..." (ERROR)
  std::string get_yesno_result() const {return yesno_result_;}

  // ============ CANCELACIÓN Y ESTADO ============

  void cancel_listening();   // El servidor deja de grabar
  void cancel_speaking();    // El servidor corta el audio
  void cancel_extract();     // Se descarta la respuesta
  void cancel_yesno();       // Se descarta la respuesta

  OperationState get_listening_state() const {return stt_state_;}
  OperationState get_speaking_state() const {return tts_state_;}
  OperationState get_extract_state() const {return extract_state_;}
  OperationState get_yesno_state() const {return yesno_state_;}

  // Último texto publicado en /listened_text (lo publica STT con cada transcripción)
  std::string get_last_listened_text() const {return last_listened_text_;}

private:
  using Clock = std::chrono::steady_clock;

  // Plazos en tiempo de RELOJ (steady_clock), también en simulación (a diferencia de
  // NavigationClient, que cuenta tiempo del robot): escuchar y hablar ocurren en tiempo
  // real, 8 s para que alguien empiece a hablar deben ser 8 s reales
  static std::optional<Clock::time_point> deadline(double timeout_sec);
  static bool expired(const std::optional<Clock::time_point> & deadline);
  static bool is_finished(OperationState state);

  void listened_text_callback(const std_msgs::msg::String::SharedPtr msg);

  // Nodo de ROS 2 que gestiona la comunicación
  // NOTA: Puntero raw porque el cliente siempre es miembro del nodo (lifetime garantizado)
  rclcpp::Node * node_;

  // Clientes de las acciones y de los servicios
  rclcpp_action::Client<Listen>::SharedPtr listen_client_;
  rclcpp_action::Client<Say>::SharedPtr say_client_;
  rclcpp::Client<Extract>::SharedPtr extract_client_;
  rclcpp::Client<YesNo>::SharedPtr yesno_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listened_text_sub_;
  std::string last_listened_text_;

  // STT
  OperationState stt_state_ = OperationState::IDLE;
  std::string stt_text_;
  std::unique_ptr<hri_client::ActionCall<Listen>> stt_call_;

  // TTS
  OperationState tts_state_ = OperationState::IDLE;
  std::optional<Clock::time_point> tts_deadline_;
  std::unique_ptr<hri_client::ActionCall<Say>> tts_call_;

  // Extract
  OperationState extract_state_ = OperationState::IDLE;
  std::string extracted_info_;
  std::optional<rclcpp::Client<Extract>::FutureAndRequestId> extract_future_;
  std::optional<Clock::time_point> extract_deadline_;

  // YesNo
  OperationState yesno_state_ = OperationState::IDLE;
  std::string yesno_result_;
  std::optional<rclcpp::Client<YesNo>::FutureAndRequestId> yesno_future_;
  std::optional<Clock::time_point> yesno_deadline_;
};

#endif  // HRI_CLIENT__HRI_CLIENT_HPP_
