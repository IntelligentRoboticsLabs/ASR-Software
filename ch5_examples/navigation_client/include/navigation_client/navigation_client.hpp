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

#ifndef NAVIGATION_CLIENT__NAVIGATION_CLIENT_HPP_
#define NAVIGATION_CLIENT__NAVIGATION_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Estado del objetivo de navegación. Mismos estados que OperationState (hri_client):
// todas las capacidades cumplen el mismo contrato.
enum class NavigationState
{
  IDLE,         // Nunca se ha enviado un objetivo
  IN_PROGRESS,  // Objetivo enviado, sin terminar
  COMPLETED,    // Nav2 llegó al objetivo
  ERROR,        // Nav2 rechazó o abortó el objetivo
  TIMEOUT,      // Se agotó el tiempo (el cliente canceló el objetivo)
  CANCELED      // Cancelado por cancel_navigation() o por el servidor
};

// Nombre del estado, para los logs
const char * to_string(NavigationState state);

// NavigationClient: Cliente simplificado para interactuar con Nav2 (NavigateToPose)
//
// CONTRATO DE LA CAPACIDAD (igual que HRIClient), NO BLOQUEANTE:
//   wait_until_ready()          ¿está disponible Nav2?
//   start_navigation(pose, t)   envía el objetivo y retorna inmediatamente
//   is_navigation_done()        se consulta en cada ciclo de control
//   get_navigation_state()      cómo terminó: COMPLETED, ERROR, TIMEOUT o CANCELED
//   get_navigation_feedback()   progreso (distancia restante, tiempo...)
//   cancel_navigation()         lo termina con el estado CANCELED
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
class NavigationClient
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigationClient(rclcpp::Node * node);

  // Verifica la disponibilidad del servidor de navegación
  bool wait_until_ready(std::chrono::seconds timeout = std::chrono::seconds(5));

  // Envía un objetivo de forma ASÍNCRONA (retorna inmediatamente).
  // timeout_sec: si no ha terminado en ese tiempo, is_navigation_done() lo cancela y el
  // estado pasa a TIMEOUT. Cuenta tiempo del ROBOT (reloj del nodo: con use_sim_time es
  // el del simulador). 0 = sin límite
  void start_navigation(const geometry_msgs::msg::PoseStamped & target_pose,
    double timeout_sec = 0.0);

  // true cuando el objetivo ha terminado (por cualquier motivo). Comprueba el timeout
  bool is_navigation_done();

  // Cómo terminó el objetivo: COMPLETED, ERROR, TIMEOUT o CANCELED
  NavigationState get_navigation_state() const {return state_;}

  // Último feedback recibido (nullptr antes del primero)
  std::shared_ptr<const NavigateToPose::Feedback> get_navigation_feedback() const
  {
    return last_feedback_;
  }

  // Cancela el objetivo en progreso (también si Nav2 aún no lo ha aceptado)
  void cancel_navigation();

  // Espera BLOQUEANDO hasta que el objetivo actual termine; true si llegó (COMPLETED).
  // Hace spin_some() del nodo: solo se puede usar si el nodo NO está ya en un executor
  // (p. ej. en el constructor, antes de rclcpp::spin). Útil en secuencias simples.
  bool wait_for_navigation_result();

  // Método auxiliar para crear poses en el frame "map"
  geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double yaw);

private:
  // Callbacks. 'seq' identifica el objetivo al que pertenecen: los callbacks de un
  // objetivo anterior (p. ej. su resultado, que llega tras un timeout) no deben cambiar
  // el estado del objetivo nuevo
  void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle, uint64_t seq);
  void feedback_callback(
    const std::shared_ptr<const NavigateToPose::Feedback> feedback, uint64_t seq);
  void result_callback(const GoalHandleNav::WrappedResult & result, uint64_t seq);

  // Pide a Nav2 cancelar (aunque aún no haya aceptado el objetivo) y lo da por terminado
  void finish(NavigationState state);
  void cancel_goal(const GoalHandleNav::SharedPtr & goal_handle);

  // Nodo de ROS 2 que gestiona la comunicación
  // NOTA: Puntero raw porque el cliente siempre es miembro del nodo (lifetime garantizado)
  rclcpp::Node * node_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr goal_handle_;

  NavigationState state_ = NavigationState::IDLE;
  std::optional<rclcpp::Time> deadline_;   // Plazo en tiempo del robot
  bool cancel_requested_ = false;          // Cancelación pedida antes de la aceptación
  uint64_t goal_seq_ = 0;                  // Identifica el objetivo actual

  std::shared_ptr<const NavigateToPose::Feedback> last_feedback_;
};

#endif  // NAVIGATION_CLIENT__NAVIGATION_CLIENT_HPP_
