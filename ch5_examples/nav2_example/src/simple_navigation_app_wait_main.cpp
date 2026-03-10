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

#include "rclcpp/rclcpp.hpp"
#include "nav2_example/simple_navigation_app_wait.hpp"

int main(int argc, char** argv)
{
  // Inicializar ROS 2
  rclcpp::init(argc, argv);
  
  // Crear la aplicación de navegación (la navegación se ejecuta en el constructor)
  // Patrón: Código bloqueante en constructor es aceptable para apps simples
  auto app = std::make_shared<SimpleNavigationAppWait>();
  
  // Crear executor para manejar múltiples nodos
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(app);
  executor.add_node(app->get_navigation_client());
  
  // Spin para mantener el nodo vivo y procesar callbacks residuales
  // En este caso no es estrictamente necesario porque wait_for_result()
  // ya procesa callbacks internamente, pero es buena práctica
  executor.spin();
  
  // Cleanup
  rclcpp::shutdown();
  return 0;
}
