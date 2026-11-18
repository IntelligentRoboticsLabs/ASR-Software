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

#include "navigation_client/navigation_client.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

const char * to_string(NavigationState state)
{
  switch (state) {
    case NavigationState::IDLE: return "IDLE";
    case NavigationState::IN_PROGRESS: return "IN_PROGRESS";
    case NavigationState::COMPLETED: return "COMPLETED";
    case NavigationState::ERROR: return "ERROR";
    case NavigationState::TIMEOUT: return "TIMEOUT";
    case NavigationState::CANCELED: return "CANCELED";
  }
  return "UNKNOWN";
}

NavigationClient::NavigationClient(rclcpp::Node * node)
: node_(node)
{
  // Cliente de la acción de Nav2: encapsula la comunicación asíncrona
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  RCLCPP_DEBUG(node_->get_logger(), "Cliente de navegación inicializado");
}

bool NavigationClient::wait_until_ready(std::chrono::seconds timeout)
{
  // FASE 1: antes de enviar objetivos, verificar que la capacidad está disponible
  if (!nav_client_->wait_for_action_server(timeout)) {
    RCLCPP_WARN(node_->get_logger(), "Acción de navegación (navigate_to_pose) no disponible");
    return false;
  }
  RCLCPP_DEBUG(node_->get_logger(), "Acción de navegación disponible");
  return true;
}

void NavigationClient::start_navigation(
  const geometry_msgs::msg::PoseStamped & target_pose, double timeout_sec)
{
  // FASE 2: enviar el objetivo SIN BLOQUEAR. Los callbacks gestionan la respuesta,
  // el feedback y el resultado; la aplicación consulta is_navigation_done()

  // Si había un objetivo en curso, se cancela: el nuevo lo sustituye
  if (state_ == NavigationState::IN_PROGRESS) {
    RCLCPP_DEBUG(node_->get_logger(), "Cancelando el objetivo anterior");
    finish(NavigationState::CANCELED);
  }

  goal_handle_.reset();
  last_feedback_.reset();
  cancel_requested_ = false;
  state_ = NavigationState::IN_PROGRESS;

  // El plazo cuenta tiempo del ROBOT (reloj del nodo): con use_sim_time es el tiempo del
  // simulador, así un objetivo no expira antes porque el simulador vaya lento
  deadline_.reset();
  if (timeout_sec > 0.0) {
    deadline_ = node_->now() + rclcpp::Duration::from_seconds(timeout_sec);
  }

  const uint64_t seq = ++goal_seq_;

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = target_pose;  // Copia: no se modifica la pose de quien llama
  goal_msg.pose.header.stamp = node_->now();
  if (goal_msg.pose.header.frame_id.empty()) {
    goal_msg.pose.header.frame_id = "map";
  }

  RCLCPP_DEBUG(
    node_->get_logger(), "Enviando objetivo: (%.2f, %.2f)",
    target_pose.pose.position.x, target_pose.pose.position.y);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // Callback 1: respuesta al envío (objetivo aceptado/rechazado)
  send_goal_options.goal_response_callback =
    [this, seq](const GoalHandleNav::SharedPtr & goal_handle) {
      goal_response_callback(goal_handle, seq);
    };

  // Callback 2: feedback periódico durante la navegación
  send_goal_options.feedback_callback =
    [this, seq](GoalHandleNav::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      feedback_callback(feedback, seq);
    };

  // Callback 3: resultado final de la navegación
  send_goal_options.result_callback =
    [this, seq](const GoalHandleNav::WrappedResult & result) {
      result_callback(result, seq);
    };

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void NavigationClient::goal_response_callback(
  const GoalHandleNav::SharedPtr & goal_handle, uint64_t seq)
{
  if (seq != goal_seq_) {
    // Objetivo de un start_navigation() anterior: si se aceptó, se cancela y se ignora
    if (goal_handle) {
      cancel_goal(goal_handle);
    }
    return;
  }

  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Objetivo rechazado por el servidor");
    state_ = NavigationState::ERROR;
    return;
  }

  goal_handle_ = goal_handle;
  if (cancel_requested_) {
    // cancel_navigation() o un timeout ocurrieron antes de que Nav2 aceptara el objetivo
    cancel_goal(goal_handle);
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Objetivo aceptado, navegación iniciada");
}

void NavigationClient::feedback_callback(
  const std::shared_ptr<const NavigateToPose::Feedback> feedback, uint64_t seq)
{
  // FASE 3: información de progreso para monitorizar (distancia, tiempo...)
  if (seq != goal_seq_) {
    return;
  }
  last_feedback_ = feedback;
  RCLCPP_DEBUG(
    node_->get_logger(), "Distancia restante: %.2f m | Tiempo: %.1f s",
    feedback->distance_remaining, rclcpp::Duration(feedback->navigation_time).seconds());
}

void NavigationClient::result_callback(const GoalHandleNav::WrappedResult & result, uint64_t seq)
{
  // FASE 4: el objetivo ha terminado en el servidor
  if (seq != goal_seq_ || state_ != NavigationState::IN_PROGRESS) {
    // Objetivo anterior, o el cliente ya lo dio por terminado (timeout o cancel_navigation())
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(node_->get_logger(), "Navegación completada con ÉXITO");
      state_ = NavigationState::COMPLETED;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node_->get_logger(), "Navegación ABORTADA (obstáculo, sin plan...)");
      state_ = NavigationState::ERROR;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "Navegación CANCELADA");
      state_ = NavigationState::CANCELED;
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Estado desconocido: %d", static_cast<int>(result.code));
      state_ = NavigationState::ERROR;
      break;
  }
}

void NavigationClient::cancel_goal(const GoalHandleNav::SharedPtr & goal_handle)
{
  try {
    nav_client_->async_cancel_goal(goal_handle);
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    // El objetivo ya había terminado: no hay nada que cancelar
  }
}

void NavigationClient::finish(NavigationState state)
{
  cancel_requested_ = true;  // Si aún no está aceptado, se cancela al aceptarse
  if (goal_handle_) {
    cancel_goal(goal_handle_);
  }
  state_ = state;
}

void NavigationClient::cancel_navigation()
{
  // Útil cuando la aplicación cambia de plan (emergencia, nueva misión...)
  if (state_ == NavigationState::IN_PROGRESS) {
    RCLCPP_INFO(node_->get_logger(), "Navegación cancelada");
    finish(NavigationState::CANCELED);
  }
}

bool NavigationClient::is_navigation_done()
{
  if (state_ == NavigationState::IN_PROGRESS && deadline_ && node_->now() > *deadline_) {
    RCLCPP_WARN(node_->get_logger(), "Navegación: timeout, cancelando el objetivo");
    finish(NavigationState::TIMEOUT);
  }
  return state_ != NavigationState::IDLE && state_ != NavigationState::IN_PROGRESS;
}

bool NavigationClient::wait_for_navigation_result()
{
  // Método BLOQUEANTE: procesa callbacks hasta que el objetivo termine.
  // El timeout es el de start_navigation() (lo comprueba is_navigation_done())
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_->get_node_base_interface());
  rclcpp::WallRate rate(10);
  while (rclcpp::ok() && !is_navigation_done()) {
    // spin_all: todos los eventos pendientes (spin_some ejecuta uno por cliente de acción)
    executor.spin_all(std::chrono::milliseconds(10));
    rate.sleep();
  }
  executor.remove_node(node_->get_node_base_interface());
  RCLCPP_DEBUG(node_->get_logger(), "Objetivo finalizado: %s", to_string(state_));
  return state_ == NavigationState::COMPLETED;
}

geometry_msgs::msg::PoseStamped NavigationClient::create_pose_stamped(
  double x, double y, double yaw)
{
  // Simplifica crear poses sin gestionar cuaterniones a mano
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  // Rotación solo en yaw (roll = pitch = 0)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  return pose;
}
