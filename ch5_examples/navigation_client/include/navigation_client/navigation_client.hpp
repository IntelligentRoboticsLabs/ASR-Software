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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

// NavigationClient: Cliente simplificado para interactuar con Nav2
//
// DECISIÓN DE DISEÑO - Composición con puntero raw:
// - Usa puntero raw (rclcpp::Node*) en lugar de shared_ptr por las siguientes razones:
//   1. El cliente siempre será un miembro de un nodo (nunca vive más que el nodo)
//   2. Permite pasar 'this' directamente en el constructor del nodo padre
//   3. Evita problemas con shared_from_this() que no puede llamarse en constructores
//   4. No hay riesgo de dangling pointer porque el ciclo de vida está garantizado
class NavigationClient
{
public:
  // Alias para simplificar tipos
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigationClient(rclcpp::Node* node);

  // Método para verificar disponibilidad del servidor
  bool wait_for_action_server(std::chrono::seconds timeout = std::chrono::seconds(5));

  // Envío de objetivo de navegación
  void send_goal(const geometry_msgs::msg::PoseStamped& target_pose);

  // Consultar estado del objetivo actual
  bool is_goal_active() const { return goal_active_; }
  bool is_goal_done() const { return goal_done_; }
  bool was_goal_successful() const { return goal_success_; }

  // Obtener el último feedback recibido
  std::shared_ptr<const NavigateToPose::Feedback> get_feedback() const { return last_feedback_; }

  // Cancelar el objetivo en progreso
  void cancel_goal();
  
  // Esperar BLOQUEANDO hasta que el goal actual termine
  // Retorna true si fue exitoso, false si falló/fue cancelado
  // Útil para secuencias de navegación donde necesitas esperar entre goals
  bool wait_for_result(std::chrono::seconds timeout = std::chrono::seconds(300));

  // Método auxiliar para crear poses
  geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double yaw);

private:
  // Callbacks
  void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle);
  
  void feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  
  void result_callback(const GoalHandleNav::WrappedResult & result);

  // Nodo de ROS2 que gestiona la comunicación
  // NOTA: Puntero raw porque el cliente siempre es miembro del nodo (lifetime garantizado)
  rclcpp::Node* node_;

  // Cliente de acción
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  
  // Goal handle (se actualiza al enviar objetivo)
  std::shared_ptr<GoalHandleNav> goal_handle_;
  
  // Variables de control para la FSM
  bool goal_active_ = false;     // Objetivo en progreso
  bool goal_done_ = false;       // Objetivo completado (éxito o fallo)
  bool goal_success_ = false;    // true si terminó con éxito
  
  // Último feedback recibido
  std::shared_ptr<const NavigateToPose::Feedback> last_feedback_;
};

#endif  // NAVIGATION_CLIENT__NAVIGATION_CLIENT_HPP_
