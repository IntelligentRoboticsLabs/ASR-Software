/**
 * @file fsm_basic_example.cpp
 * @brief Ejemplo 1: FSM básica con patrón switch-case - Implementación
 * 
 * Este ejemplo ilustra la implementación más simple de una FSM en ROS 2.
 * Ventajas: Directo, fácil de entender
 * Limitaciones: No implementa on_entry/on_exit, acciones pueden repetirse
 * 
 * Basado en el Capítulo 7 del libro ASR - Sección "Patrón básico: switch-case"
 */

#include "fsm_examples/fsm_basic_example.hpp"

namespace fsm_examples
{

// ============================================================================
// IMPLEMENTACIÓN DE BasicFSMRobot
// ============================================================================

BasicFSMRobot::BasicFSMRobot() : Node("basic_fsm_robot"), current_state_(State::IDLE)
  {
    // Suscripciones
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&BasicFSMRobot::laser_callback, this, std::placeholders::_1));
    
    // Publicadores
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Timer para el ciclo de control (100ms = 10Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BasicFSMRobot::control_cycle, this));
    
    RCLCPP_INFO(this->get_logger(), "FSM Básica iniciada en estado IDLE");
    RCLCPP_INFO(this->get_logger(), "Presiona Space para iniciar...");
  }

void BasicFSMRobot::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Callback de sensor: SOLO actualiza memoria, NO toma decisiones
  if (!msg->ranges.empty()) {
    min_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  }
}

void BasicFSMRobot::control_cycle()
{
  // Este método se llama periódicamente (cada 100ms)
  // AQUÍ es donde la FSM toma decisiones
  
  switch (current_state_) {
      case State::IDLE:
        // Acción: detener motores (se ejecuta en cada ciclo - no ideal)
        publish_velocity(0.0, 0.0);
        
        // Transición: Si se presiona start, pasar a MOVING
        if (start_button_pressed_) {
          RCLCPP_INFO(this->get_logger(), "Transición: IDLE -> MOVING");
          current_state_ = State::MOVING;
          start_button_pressed_ = false;
        }
        break;

      case State::MOVING:
        // Acción: avanzar (se ejecuta en cada ciclo)
        publish_velocity(0.3, 0.0);
        
        // Transición: Si detecta obstáculo, pasar a OBSTACLE_DETECTED
        if (min_distance_ < OBSTACLE_THRESHOLD) {
          RCLCPP_WARN(this->get_logger(), 
                      "¡Obstáculo detectado a %.2f m! Transición: MOVING -> OBSTACLE_DETECTED",
                      min_distance_);
          current_state_ = State::OBSTACLE_DETECTED;
        }
        break;

      case State::OBSTACLE_DETECTED:
        // Acción: frenar (se ejecuta en cada ciclo - no ideal)
        publish_velocity(0.0, 0.0);
        
        // Transición inmediata a STOPPED
        RCLCPP_INFO(this->get_logger(), "Transición: OBSTACLE_DETECTED -> STOPPED");
        current_state_ = State::STOPPED;
        stopped_time_ = this->now();
        break;

      case State::STOPPED:
        // Acción: mantenerse detenido
        publish_velocity(0.0, 0.0);
        
        // Transición: Si el obstáculo desaparece después de 2 segundos, volver a MOVING
        auto elapsed = (this->now() - stopped_time_).seconds();
        if (min_distance_ > OBSTACLE_THRESHOLD && elapsed > 2.0) {
          RCLCPP_INFO(this->get_logger(), 
                      "Obstáculo despejado. Transición: STOPPED -> MOVING");
          current_state_ = State::MOVING;
        }
        break;
    }
  }

void BasicFSMRobot::publish_velocity(double linear, double angular)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_pub_->publish(msg);
}

void BasicFSMRobot::simulate_start_button()
{
  start_button_pressed_ = true;
}

// ============================================================================
// IMPLEMENTACIÓN DE SimulatedInput
// ============================================================================

SimulatedInput::SimulatedInput(std::shared_ptr<BasicFSMRobot> robot) 
  : Node("simulated_input"), robot_(robot)
{
  timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() { 
      RCLCPP_INFO(this->get_logger(), "Simulando presión de botón START...");
      robot_->simulate_start_button();
      timer_->cancel();
    });
}

}  // namespace fsm_examples

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto robot_node = std::make_shared<fsm_examples::BasicFSMRobot>();
  auto input_node = std::make_shared<fsm_examples::SimulatedInput>(robot_node);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_node);
  executor.add_node(input_node);
  
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  RCLCPP_INFO(robot_node->get_logger(), 
              "EJEMPLO 1: FSM con patrón switch-case");
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  RCLCPP_INFO(robot_node->get_logger(), 
              "Este nodo simula un robot que:");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  - Comienza en IDLE");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  - Avanza al detectar 'start'");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  - Se detiene ante obstáculos");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  - Retoma movimiento cuando se despeja");
  RCLCPP_INFO(robot_node->get_logger(), " ");
  RCLCPP_INFO(robot_node->get_logger(), 
              "Limitación: Las acciones se repiten en cada ciclo");
  RCLCPP_INFO(robot_node->get_logger(), 
              "(no hay on_entry/on_exit)");
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
