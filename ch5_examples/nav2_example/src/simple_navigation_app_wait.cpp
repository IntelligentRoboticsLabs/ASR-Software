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

#include "nav2_example/simple_navigation_app_wait.hpp"

SimpleNavigationAppWait::SimpleNavigationAppWait()
: Node("simple_navigation_app_wait")
{
    // 1. Composición: La aplicación TIENE UN NavigationClient
    //    Úso de 'this': Pasamos el puntero del nodo directamente
    //    - Seguro porque el cliente es miembro del nodo (mismo ciclo de vida)
    //    - No podemos usar shared_from_this() en el constructor
    nav_client_ = std::make_shared<NavigationClient>(this);
    
    RCLCPP_INFO(get_logger(), "=== Aplicación de navegación BLOQUEANTE iniciada ===");;
    
    // 2. FASE 1: Verificar disponibilidad de Nav2
    RCLCPP_INFO(get_logger(), "Esperando a que Nav2 esté disponible...");
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Nav2 no disponible tras 10 segundos");
      return;
    }
    RCLCPP_INFO(get_logger(), "✓ Nav2 disponible");
    
    // 3. Definir secuencia de waypoints
    // Patrón: Lista de poses que el robot visitará en orden
    std::vector<std::tuple<double, double, double>> waypoints = {
      {6.0, -2.0, 0.0},      // Waypoint 1: (x, y, yaw)
      {3.7, 0.7, 1.57},      // Waypoint 2
      {-5.5, 0.0, 3.14},      // Waypoint 3
      {0.0, 0.0, 0.0}        // Waypoint 4: volver al origen
    };
    
    RCLCPP_INFO(get_logger(), "Navegando por %zu waypoints...", waypoints.size());
    
    // 4. FASE 2-4: Iterar por waypoints usando wait_for_result()
    // Patrón BLOQUEANTE SIMPLE: enviar -> esperar -> procesar -> siguiente
    for (size_t i = 0; i < waypoints.size(); i++) {
      auto [x, y, yaw] = waypoints[i];
      
      // Crear objetivo
      auto goal = nav_client_->create_pose_stamped(x, y, yaw);
      
      RCLCPP_INFO(get_logger(), 
                  "\n[Waypoint %zu/%zu] Navegando a (%.2f, %.2f, %.2f rad)...",
                  i + 1, waypoints.size(), x, y, yaw);
      
      // Enviar objetivo (se cancela automáticamente cualquier goal anterior)
      nav_client_->send_goal(goal);
      
      // CLAVE: Esperar BLOQUEANDO hasta que termine
      // wait_for_result() internamente procesa callbacks y verifica estado
      // Retorna: true si éxito, false si falló/timeout/cancelado
      bool success = nav_client_->wait_for_result(std::chrono::seconds(300));
      
      // Procesar resultado
      if (success) {
        RCLCPP_INFO(get_logger(), 
                    "✓ Waypoint %zu completado con ÉXITO",
                    i + 1);
      } else {
        RCLCPP_ERROR(get_logger(), 
                     "✗ Waypoint %zu FALLÓ (abortado, timeout o cancelado)",
                     i + 1);
        RCLCPP_ERROR(get_logger(), "Abortando secuencia de navegación");
        return;  // Salir si falla algún waypoint
      }
    }
    
    // 5. Secuencia completada exitosamente
    RCLCPP_INFO(get_logger(), 
                "\n=== ✓ Secuencia completa exitosa: %zu/%zu waypoints ===",
                waypoints.size(), waypoints.size());
}
