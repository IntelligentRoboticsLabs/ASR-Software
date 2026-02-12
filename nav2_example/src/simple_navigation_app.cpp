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

#include "nav2_example/simple_navigation_app.hpp"

SimpleNavigationApp::SimpleNavigationApp()
: Node("simple_navigation_app")
{
  // Crear el cliente de navegación como componente
  nav_client_ = std::make_shared<NavigationClient>();
  
  // Definir objetivo de navegación
  target_pose_ = nav_client_->create_pose_stamped(6.0, -2.0, 0.0);
  
  RCLCPP_INFO(get_logger(), "Aplicación de navegación iniciada");
  
  // Timer para gestionar el flujo de la aplicación
  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&SimpleNavigationApp::control_cycle, this));
}

void SimpleNavigationApp::control_cycle()
{
  // Fase 1: Esperar disponibilidad del servidor
  if (!server_ready_) {
    if (nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Servidor disponible, preparado para navegar");
      server_ready_ = true;
    }
    return;  // Seguir esperando en próxima iteración
  }
  
  // Fase 2: Enviar objetivo (solo una vez)
  if (!goal_sent_) {
    RCLCPP_INFO(get_logger(), "Enviando objetivo de navegación...");
    nav_client_->send_goal(target_pose_);
    goal_sent_ = true;
    return;
  }

  // Fase 3: Monitorizar progreso
  if (!nav_client_->is_goal_done()) {
    // Objetivo en progreso (esperando confirmación o navegando)
    auto feedback = nav_client_->get_feedback();
    if (feedback) {
      RCLCPP_INFO(get_logger(), 
                  "\t-Distancia restante: %.2f m | Tiempo: %.1f s",
                  feedback->distance_remaining,
                  rclcpp::Duration(feedback->navigation_time).seconds());
    }
    return;
  }
  
  // Fase 4: Procesar resultado (solo se alcanza si is_goal_done() es true)
  if (nav_client_->was_goal_successful()) {
    RCLCPP_INFO(get_logger(), "Navegación completada con éxito");
  } else {
    RCLCPP_WARN(get_logger(), "Navegación fallida");
  }
  
  // Detener el timer, tarea completada
  timer_->cancel();
  RCLCPP_INFO(get_logger(), "Aplicación finalizada");
}
