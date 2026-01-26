/**
 * @file fsm_oop_example.cpp
 * @brief Ejemplo 2: FSM con patrón orientado a objetos - Implementación
 * 
 * Este ejemplo ilustra una arquitectura más robusta donde cada estado es una clase
 * que implementa correctamente on_entry, on_do y on_exit.
 * 
 * Basado en el Capítulo 7 del libro ASR - Sección "Patrón orientado a objetos"
 */

#include "fsm_examples/fsm_oop_example.hpp"

namespace fsm_examples
{

// ============================================================================
// IMPLEMENTACIÓN DE StateMachine
// ============================================================================

StateMachine::StateMachine(State* initial_state, rclcpp::Logger logger)
  : current_state_(initial_state), logger_(logger)
{
  RCLCPP_INFO(logger_, "FSM iniciada en estado: %s", 
              current_state_->get_name().c_str());
  current_state_->on_entry();
}

StateMachine::~StateMachine()
{
  delete current_state_;
}

void StateMachine::step()
{
  // 1. Ejecutar lógica del estado actual
  current_state_->on_do();
  
  // 2. Verificar transiciones
  State* next_state = current_state_->check_transitions();
  
  // 3. Si hay transición, ejecutar salida, cambiar y ejecutar entrada
  if (next_state != nullptr) {
    RCLCPP_INFO(logger_, "Transición: %s -> %s",
                current_state_->get_name().c_str(),
                next_state->get_name().c_str());
    
    current_state_->on_exit();
    delete current_state_;
    current_state_ = next_state;
    current_state_->on_entry();
  }
}

// ============================================================================
// IMPLEMENTACIÓN DE OOPFSMRobot
// ============================================================================

OOPFSMRobot::OOPFSMRobot() : Node("oop_fsm_robot")
  {
    // Suscripciones
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&OOPFSMRobot::laser_callback, this, std::placeholders::_1));
    
    // Publicadores
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Timer para el ciclo de control
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&OOPFSMRobot::control_cycle, this));
    
    // Inicializar FSM en estado IDLE
    fsm_ = new StateMachine(new IdleState(this), this->get_logger());
  }

OOPFSMRobot::~OOPFSMRobot()
{
  delete fsm_;
}

void OOPFSMRobot::publish_velocity(double linear, double angular)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_pub_->publish(msg);
}

void OOPFSMRobot::simulate_start_button()
{
  start_button_pressed_ = true;
}

void OOPFSMRobot::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg->ranges.empty()) {
    min_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  }
}

void OOPFSMRobot::control_cycle()
{
  // La FSM avanza un paso. Debe ser rápido y retornar.
  fsm_->step();
}

// ============================================================================
// IMPLEMENTACIÓN DE LOS ESTADOS
// ============================================================================

// Estado IDLE
IdleState::IdleState(OOPFSMRobot* r) : robot_(r) {}

void IdleState::on_entry()
{
  RCLCPP_INFO(robot_->get_logger(), "[IDLE] Entrando al estado - Deteniendo motores");
  robot_->publish_velocity(0.0, 0.0);
}

void IdleState::on_do()
{
  // No hace nada, simplemente espera
}

State* IdleState::check_transitions()
{
  if (robot_->is_start_pressed()) {
    robot_->clear_start_button();
    return new MovingState(robot_);
  }
  return nullptr;  // Sin transición
}

void IdleState::on_exit()
{
  RCLCPP_INFO(robot_->get_logger(), "[IDLE] Saliendo del estado - Preparando para mover");
}

// Estado MOVING
MovingState::MovingState(OOPFSMRobot* r) : robot_(r) {}

void MovingState::on_entry()
{
  RCLCPP_INFO(robot_->get_logger(), "[MOVING] Entrando al estado - Iniciando movimiento");
}

void MovingState::on_do()
{
  // Ejecutar acción continua: avanzar
  robot_->publish_velocity(0.3, 0.0);
}

State* MovingState::check_transitions()
{
  if (robot_->get_min_distance() < robot_->get_obstacle_threshold()) {
    RCLCPP_WARN(robot_->get_logger(), 
                "[MOVING] ¡Obstáculo detectado a %.2f m!",
                robot_->get_min_distance());
    return new StoppedState(robot_);
  }
  return nullptr;
}

void MovingState::on_exit()
{
  RCLCPP_INFO(robot_->get_logger(), "[MOVING] Saliendo del estado - Iniciando frenado");
  // CRÍTICO: Detener motores antes de cambiar de estado
  robot_->publish_velocity(0.0, 0.0);
}

// Estado STOPPED
StoppedState::StoppedState(OOPFSMRobot* r) : robot_(r) {}

void StoppedState::on_entry()
{
  RCLCPP_INFO(robot_->get_logger(), "[STOPPED] Entrando al estado - Robot detenido por seguridad");
  entry_time_ = robot_->get_clock()->now();
  robot_->publish_velocity(0.0, 0.0);
}

void StoppedState::on_do()
{
  // Mantener velocidad cero
  robot_->publish_velocity(0.0, 0.0);
}

State* StoppedState::check_transitions()
{
  auto elapsed = (robot_->get_clock()->now() - entry_time_).seconds();
  
  // Si el obstáculo desaparece después de 2 segundos, volver a moverse
  if (robot_->get_min_distance() > robot_->get_obstacle_threshold() && elapsed > 2.0) {
    RCLCPP_INFO(robot_->get_logger(), 
                "[STOPPED] Obstáculo despejado después de %.1f segundos", elapsed);
    return new MovingState(robot_);
  }
  return nullptr;
}

void StoppedState::on_exit()
{
  RCLCPP_INFO(robot_->get_logger(), "[STOPPED] Saliendo del estado - Retomando operación");
}

// ============================================================================
// IMPLEMENTACIÓN DE SimulatedInputOOP
// ============================================================================

SimulatedInputOOP::SimulatedInputOOP(std::shared_ptr<OOPFSMRobot> robot) 
  : Node("simulated_input_oop"), robot_(robot)
{
  timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() { 
      RCLCPP_INFO(this->get_logger(), ">>> Simulando presión de botón START...");
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
  
  auto robot_node = std::make_shared<fsm_examples::OOPFSMRobot>();
  auto input_node = std::make_shared<fsm_examples::SimulatedInputOOP>(robot_node);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_node);
  executor.add_node(input_node);
  
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  RCLCPP_INFO(robot_node->get_logger(), 
              "EJEMPLO 2: FSM con patrón orientado a objetos");
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  RCLCPP_INFO(robot_node->get_logger(), 
              "Este nodo implementa el mismo comportamiento");
  RCLCPP_INFO(robot_node->get_logger(), 
              "pero con arquitectura OOP robusta:");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  - on_entry: se ejecuta UNA VEZ al entrar");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  - on_do: se ejecuta cada ciclo");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  - on_exit: se ejecuta UNA VEZ al salir");
  RCLCPP_INFO(robot_node->get_logger(), " ");
  RCLCPP_INFO(robot_node->get_logger(), 
              "Observe cómo las transiciones son más limpias");
  RCLCPP_INFO(robot_node->get_logger(), 
              "y cada estado gestiona sus propios recursos");
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
