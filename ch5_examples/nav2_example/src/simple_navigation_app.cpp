// Copyright 2026 Intelligent Robotics Lab
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
  // 1. Composición: La aplicación TIENE UN NavigationClient (no hereda)
  //    Patrón: Usar capacidades como componentes en lugar de herencias múltiples
  //    Úso de 'this': Pasamos el puntero del nodo directamente al cliente
  //    - Es seguro porque el cliente es miembro del nodo (mismo ciclo de vida)
  //    - No podemos usar shared_from_this() en el constructor
  //    - El cliente usará este nodo para crear clientes, timers, logging, etc.
  nav_client_ = std::make_shared<NavigationClient>(this);
  
  // 2. Usar create_pose_stamped() para construir el objetivo
  //    Método auxiliar: simplifica crear poses sin quaterniones manuales
  target_pose_ = nav_client_->create_pose_stamped(6.0, -2.0, 0.0);
  
  RCLCPP_INFO(get_logger(), "Aplicación de navegación iniciada");
  
  // 3. Timer periódico: patron NO BLOQUEANTE
  //    Permite que el nodo siga procesando callbacks mientras consulta estado
  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&SimpleNavigationApp::control_cycle, this));
}

void SimpleNavigationApp::control_cycle()
{
  // FASE 1: Verificar disponibilidad de la CAPACIDAD
  // Usa: wait_for_action_server(timeout)
  // Propósito: Asegurar que Nav2 está listo antes de enviar objetivos
  if (!server_ready_) {
    if (nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Servidor disponible, preparado para navegar");
      server_ready_ = true;
    }
    return;  // Seguir esperando en próxima iteración
  }
  
  // FASE 2: Activar la CAPACIDAD (enviar objetivo)
  // Usa: send_goal(target_pose)
  // Propósito: Solicitar navegación de forma asíncrona (no bloquea)
  if (!goal_sent_) {
    RCLCPP_INFO(get_logger(), "Enviando objetivo de navegación...");
    nav_client_->send_goal(target_pose_);
    goal_sent_ = true;
    return;
  }

  // FASE 3: Monitorizar PROGRESO de la capacidad
  // Usa: is_goal_done() para verificar si la capacidad terminó
  // Usa: get_feedback() para obtener información de progreso
  if (!nav_client_->is_goal_done()) {
    // Objetivo en progreso: consultar feedback (opcional pero útil)
    auto feedback = nav_client_->get_feedback();
    if (feedback) {
      RCLCPP_INFO(get_logger(), 
                  "\t-Distancia restante: %.2f m | Tiempo: %.1f s",
                  feedback->distance_remaining,
                  rclcpp::Duration(feedback->navigation_time).seconds());
    }
    return;
  }
  
  // FASE 4: Procesar RESULTADO de la capacidad
  // Usa: was_goal_successful() para distinguir éxito/fallo
  // Propósito: Tomar decisiones según el resultado (continuar, reintentar, etc.)
  if (nav_client_->was_goal_successful()) {
    RCLCPP_INFO(get_logger(), "Navegación completada con éxito");
  } else {
    RCLCPP_WARN(get_logger(), "Navegación fallida");
  }
  
  // Detener el timer, tarea completada
  timer_->cancel();
  RCLCPP_INFO(get_logger(), "Aplicación finalizada");
}
