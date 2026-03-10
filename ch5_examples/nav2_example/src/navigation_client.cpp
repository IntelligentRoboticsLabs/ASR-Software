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

#include "nav2_example/navigation_client.hpp"

NavigationClient::NavigationClient()
: Node("navigation_client")
{
  // Crear el cliente de acción para comunicarse con Nav2
  // Este cliente encapsula toda la complejidad de comunicación asíncrona
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");
  
  RCLCPP_DEBUG(get_logger(), "Cliente de navegación inicializado");
}

bool NavigationClient::wait_for_action_server(std::chrono::seconds timeout)
{
  // Método de verificación: asegura que Nav2 está operativo
  // FASE 1: Antes de enviar objetivos, verificar disponibilidad de la capacidad
  if (!nav_client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(get_logger(), 
                 "Servidor de navegación no disponible tras espera");
    return false;
  }
  RCLCPP_DEBUG(get_logger(), "Servidor de navegación disponible");
  return true;
}

void NavigationClient::send_goal(const geometry_msgs::msg::PoseStamped& target_pose)
{
  // Método de activación: solicita navegación de forma ASÍNCRONA
  // FASE 2: Enviar objetivo sin bloquear, retorna inmediatamente
  // Los callbacks gestionarán la respuesta, feedback y resultado
  
  // // IMPORTANTE: Cancelar goal anterior si existe
  // // Esto evita que goals abortados interfieran con nuevos goals
  // if (goal_handle_ && (goal_active_ || !goal_done_)) {
  //   RCLCPP_DEBUG(get_logger(), "Cancelando goal anterior antes de enviar nuevo");
  //   nav_client_->async_cancel_goal(goal_handle_);
  //   // Pequeña espera para que Nav2 procese la cancelación
  //   rclcpp::sleep_for(std::chrono::milliseconds(100));
  // }
  
  // // Resetear flags de control y limpiar handle anterior
  // goal_active_ = false;
  // goal_done_ = false;
  // goal_success_ = false;
  // goal_handle_.reset();  // Limpiar referencia al goal anterior
  // last_feedback_.reset();  // Limpiar feedback anterior

  // Construir el mensaje de objetivo
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = target_pose;
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = "map";

  RCLCPP_DEBUG(get_logger(), 
               "Enviando objetivo: (%.2f, %.2f)",
               target_pose.pose.position.x,
               target_pose.pose.position.y);

  // Configurar opciones con callbacks
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  // Callback 1: Respuesta al envío (objetivo aceptado/rechazado)
  send_goal_options.goal_response_callback = 
    std::bind(&NavigationClient::goal_response_callback, this, 
              std::placeholders::_1);
  
  // Callback 2: Feedback periódico durante la navegación
  send_goal_options.feedback_callback = 
    std::bind(&NavigationClient::feedback_callback, this,
              std::placeholders::_1, std::placeholders::_2);
  
  // Callback 3: Resultado final de la navegación
  send_goal_options.result_callback = 
    std::bind(&NavigationClient::result_callback, this,
              std::placeholders::_1);

  // Enviar el objetivo de forma asíncrona
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void NavigationClient::goal_response_callback(
  const GoalHandleNav::SharedPtr & goal_handle)
{
  // Callback ejecutado cuando Nav2 responde si acepta o rechaza el objetivo
  // Actualiza flags para que la aplicación pueda consultar el estado
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Objetivo rechazado por el servidor");
    goal_done_ = true;
    goal_success_ = false;
    return;
  }
  
  RCLCPP_DEBUG(get_logger(), "Objetivo aceptado, navegación iniciada");
  goal_handle_ = goal_handle;
  goal_active_ = true;  // Señaliza que la navegación está en progreso
}

void NavigationClient::feedback_callback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // Callback ejecutado periódicamente durante la navegación
  // FASE 3: Provee información de progreso (distancia, tiempo) para monitorizar
  
  // Almacenar el último feedback recibido para consultas no bloqueantes
  last_feedback_ = feedback;
  
  // Feedback periódico: distancia restante, tiempo, etc.
  RCLCPP_DEBUG(get_logger(), 
               "Distancia restante: %.2f m | Tiempo: %.1f s",
               feedback->distance_remaining,
               rclcpp::Duration(feedback->navigation_time).seconds());
  
  // Aquí se puede implementar lógica adicional:
  // - Detectar robot atascado (distancia no disminuye)
  // - Timeout por exceso de tiempo
  // - Actualizar UI con progreso
}

void NavigationClient::result_callback(const GoalHandleNav::WrappedResult & result)
{
  // Callback ejecutado cuando la navegación finaliza (sea cual sea el resultado)
  // FASE 4: Actualiza flags para permitir procesar éxito/fallo/cancelación
  goal_active_ = false;
  goal_done_ = true;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(get_logger(), "Navegación completada con ÉXITO");
      goal_success_ = true;
      break;
      
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "Navegación ABORTADA (obstáculo o timeout)");
      goal_success_ = false;
      break;
      
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Navegación CANCELADA");
      goal_success_ = false;
      break;
      
    default:
      RCLCPP_ERROR(get_logger(), "Estado desconocido: %d", 
                   static_cast<int>(result.code));
      goal_success_ = false;
      break;
  }
}

void NavigationClient::cancel_goal()
{
  // Método de control: permite abortar una navegación en progreso
  // Útil cuando la aplicación decide cambiar de plan (emergencia, timeout, nueva misión)
  if (goal_handle_ && goal_active_) {
    RCLCPP_DEBUG(get_logger(), "Cancelando objetivo de navegación");
    nav_client_->async_cancel_goal(goal_handle_);
    goal_active_ = false;
  }
}

bool NavigationClient::wait_for_result(std::chrono::seconds timeout)
{
  // Método BLOQUEANTE: espera hasta que el goal actual termine
  // Útil para secuencias de navegación: enviar goal -> esperar -> enviar siguiente
  // Retorna: true si el goal fue exitoso, false si falló o timeout
  
  if (!goal_active_ && goal_done_) {
    // El goal ya terminó, retornar resultado inmediatamente
    return goal_success_;
  }
  
  auto start_time = std::chrono::steady_clock::now();
  rclcpp::Rate rate(10);  // Verificar cada 100ms
  
  while (rclcpp::ok()) {
    // Procesar callbacks para recibir actualizaciones de estado
    rclcpp::spin_some(this->get_node_base_interface());
    
    // Verificar si el goal terminó
    if (goal_done_) {
      RCLCPP_DEBUG(get_logger(), 
                   "Goal finalizado: %s", 
                   goal_success_ ? "ÉXITO" : "FALLO");
      return goal_success_;
    }
    
    // Verificar timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > timeout) {
      RCLCPP_WARN(get_logger(), 
                  "Timeout esperando resultado del goal (%.1f s)", 
                  std::chrono::duration<double>(timeout).count());
      cancel_goal();  // Cancelar el goal si hay timeout
      return false;
    }
    
    rate.sleep();
  }
  
  return false;
}

geometry_msgs::msg::PoseStamped NavigationClient::create_pose_stamped(
  double x, double y, double yaw)
{
  // Método auxiliar: simplifica la creación de poses sin gestionar quaterniones
  // manualmente. Evita repetir código en aplicaciones con múltiples waypoints.
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  
  // Convertir yaw a quaternion usando TF2
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  
  return pose;
}
