/**
 * @file fsm_orthogonal_example.cpp
 * @brief Ejemplo 3: FSM con descomposición ortogonal - Implementación
 * 
 * Este ejemplo ilustra cómo evitar la explosión combinatoria de estados
 * mediante la descomposición ortogonal propuesta por Harel (Statecharts).
 * 
 * Basado en el Capítulo 7 del libro ASR - Sección "El problema de la concurrencia"
 */

#include "fsm_examples/fsm_orthogonal_example.hpp"

namespace fsm_examples
{

// ============================================================================
// IMPLEMENTACIÓN DE OrthogonalRobot
// ============================================================================

OrthogonalRobot::OrthogonalRobot() : Node("orthogonal_robot")
  {
    // Suscripciones
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&OrthogonalRobot::laser_callback, this, std::placeholders::_1));
    
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10,
      std::bind(&OrthogonalRobot::battery_callback, this, std::placeholders::_1));
    
    // Publicadores
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Timer para el ciclo de control
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&OrthogonalRobot::control_cycle, this));
    
    // Timer para mostrar estado combinado
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&OrthogonalRobot::print_status, this));
    
    // Inicializar ambas FSM en sus estados iniciales
    nav_fsm_ = new GenericStateMachine<NavState>(
      new NavIdleState(this), this->get_logger(), "NAVEGACIÓN");
    
    battery_fsm_ = new GenericStateMachine<BatteryState>(
      new BatteryOKState(this), this->get_logger(), "BATERÍA");
    
    RCLCPP_INFO(this->get_logger(), 
                "FSM Ortogonal iniciada: %s + %s",
                nav_fsm_->get_current_state_name().c_str(),
                battery_fsm_->get_current_state_name().c_str());
  }

OrthogonalRobot::~OrthogonalRobot()
{
  delete nav_fsm_;
  delete battery_fsm_;
}

void OrthogonalRobot::publish_velocity(double linear, double angular)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_pub_->publish(msg);
}

void OrthogonalRobot::simulate_start_button()
{
  start_button_pressed_ = true;
}

void OrthogonalRobot::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg->ranges.empty()) {
    min_distance_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  }
}

void OrthogonalRobot::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  battery_percentage_ = msg->percentage * 100.0;
}

void OrthogonalRobot::control_cycle()
{
  // CLAVE ARQUITECTÓNICA: Ambas FSM se ejecutan en el mismo ciclo
  // Cada una gestiona su dimensión ortogonal del comportamiento
  nav_fsm_->step();
  battery_fsm_->step();
}

void OrthogonalRobot::print_status()
{
  RCLCPP_INFO(this->get_logger(), 
              "Estado combinado: [%s] + [%s] | Batería: %.1f%% | Obstáculo: %.2fm",
              nav_fsm_->get_current_state_name().c_str(),
              battery_fsm_->get_current_state_name().c_str(),
              battery_percentage_,
              min_distance_);
}

// ============================================================================
// IMPLEMENTACIÓN DE ESTADOS DE NAVEGACIÓN
// ============================================================================

NavIdleState::NavIdleState(OrthogonalRobot* r) : robot_(r) {}

void NavIdleState::on_entry()
{
  RCLCPP_INFO(robot_->get_logger(), "[NAV] Entrando en IDLE - Deteniendo motores");
  robot_->publish_velocity(0.0, 0.0);
}

void NavIdleState::on_do()
{
  // Mantener detenido
  robot_->publish_velocity(0.0, 0.0);
}

NavState* NavIdleState::check_transitions()
{
  if (robot_->is_start_pressed()) {
    robot_->clear_start_button();
    return new NavMovingState(robot_);
  }
  return nullptr;
}

NavMovingState::NavMovingState(OrthogonalRobot* r) : robot_(r) {}

void NavMovingState::on_entry()
{
  RCLCPP_INFO(robot_->get_logger(), "[NAV] Entrando en MOVING - Iniciando movimiento");
}

void NavMovingState::on_do()
{
  robot_->publish_velocity(0.3, 0.0);
}

NavState* NavMovingState::check_transitions()
{
  if (robot_->get_min_distance() < robot_->get_obstacle_threshold()) {
    return new NavStoppedState(robot_);
  }
  return nullptr;
}

void NavMovingState::on_exit()
{
  RCLCPP_INFO(robot_->get_logger(), "[NAV] Saliendo de MOVING - Frenando");
  robot_->publish_velocity(0.0, 0.0);
}

NavStoppedState::NavStoppedState(OrthogonalRobot* r) : robot_(r) {}

void NavStoppedState::on_entry()
{
  RCLCPP_WARN(robot_->get_logger(), "[NAV] Entrando en STOPPED - Obstáculo detectado");
  entry_time_ = robot_->get_clock()->now();
  robot_->publish_velocity(0.0, 0.0);
}

void NavStoppedState::on_do()
{
  robot_->publish_velocity(0.0, 0.0);
}

NavState* NavStoppedState::check_transitions()
{
  auto elapsed = (robot_->get_clock()->now() - entry_time_).seconds();
  
  if (robot_->get_min_distance() > robot_->get_obstacle_threshold() && elapsed > 2.0) {
    return new NavMovingState(robot_);
  }
  return nullptr;
}

// ============================================================================
// IMPLEMENTACIÓN DE ESTADOS DE BATERÍA
// ============================================================================

BatteryOKState::BatteryOKState(OrthogonalRobot* r) : robot_(r) {}

void BatteryOKState::on_entry()
{
  RCLCPP_INFO(robot_->get_logger(), "[BAT] Batería en nivel óptimo");
}

void BatteryOKState::on_do()
{
  // No hace nada, solo monitoriza
}

BatteryState* BatteryOKState::check_transitions()
{
  if (robot_->get_battery_percentage() < robot_->get_battery_low_threshold()) {
    return new BatteryLowState(robot_);
  }
  return nullptr;
}

BatteryLowState::BatteryLowState(OrthogonalRobot* r) : robot_(r) {}

void BatteryLowState::on_entry()
{
  RCLCPP_WARN(robot_->get_logger(), "[BAT] ⚠️  Batería baja - Considerar recarga pronto");
}

void BatteryLowState::on_do()
{
  // Podría reducir velocidad máxima, activar modo ahorro, etc.
}

BatteryState* BatteryLowState::check_transitions()
{
  if (robot_->get_battery_percentage() > robot_->get_battery_low_threshold()) {
    return new BatteryOKState(robot_);
  }
  if (robot_->get_battery_percentage() < robot_->get_battery_critical_threshold()) {
    return new BatteryCriticalState(robot_);
  }
  return nullptr;
}

BatteryCriticalState::BatteryCriticalState(OrthogonalRobot* r) : robot_(r) {}

void BatteryCriticalState::on_entry()
{
  RCLCPP_ERROR(robot_->get_logger(), "[BAT] 🔴 BATERÍA CRÍTICA - Deteniendo operaciones");
  // Forzar detención de motores por seguridad
  robot_->publish_velocity(0.0, 0.0);
}

void BatteryCriticalState::on_do()
{
  // Mantener motores apagados
  robot_->publish_velocity(0.0, 0.0);
}

BatteryState* BatteryCriticalState::check_transitions()
{
  if (robot_->get_battery_percentage() > robot_->get_battery_critical_threshold() + 5.0) {
    return new BatteryLowState(robot_);
  }
  return nullptr;
}

// ============================================================================
// IMPLEMENTACIÓN DE SimulatedInputOrthogonal Y BatterySimulator
// ============================================================================

SimulatedInputOrthogonal::SimulatedInputOrthogonal(std::shared_ptr<OrthogonalRobot> robot) 
  : Node("simulated_input_orthogonal"), robot_(robot)
{
  timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() { 
      RCLCPP_INFO(this->get_logger(), ">>> Simulando presión de botón START...");
      robot_->simulate_start_button();
      timer_->cancel();
    });
}

BatterySimulator::BatterySimulator() : Node("battery_simulator")
{
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10);
  
  timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&BatterySimulator::publish_battery, this));
  
  RCLCPP_INFO(this->get_logger(), "Simulador de batería iniciado - Descarga gradual");
}

void BatterySimulator::publish_battery()
{
  auto msg = sensor_msgs::msg::BatteryState();
  msg.percentage = battery_level_ / 100.0;
  battery_pub_->publish(msg);
  
  // Simular descarga gradual
  battery_level_ -= 2.0;
  if (battery_level_ < 0.0) {
    battery_level_ = 100.0;  // Reiniciar para demostración continua
    RCLCPP_INFO(this->get_logger(), "🔋 Batería recargada al 100%% (simulación)");
  }
}

}  // namespace fsm_examples

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto robot_node = std::make_shared<fsm_examples::OrthogonalRobot>();
  auto input_node = std::make_shared<fsm_examples::SimulatedInputOrthogonal>(robot_node);
  auto battery_sim_node = std::make_shared<fsm_examples::BatterySimulator>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_node);
  executor.add_node(input_node);
  executor.add_node(battery_sim_node);
  
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  RCLCPP_INFO(robot_node->get_logger(), 
              "EJEMPLO 3: FSM con descomposición ortogonal");
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  RCLCPP_INFO(robot_node->get_logger(), 
              "Este nodo implementa DOS FSM independientes:");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  1. NavigationFSM (IDLE/MOVING/STOPPED)");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  2. BatteryFSM (OK/LOW/CRITICAL)");
  RCLCPP_INFO(robot_node->get_logger(), " ");
  RCLCPP_INFO(robot_node->get_logger(), 
              "Ventaja arquitectónica:");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  Sin ortogonalidad: 3×3 = 9 estados combinados");
  RCLCPP_INFO(robot_node->get_logger(), 
              "  Con ortogonalidad: 3+3 = 6 estados independientes");
  RCLCPP_INFO(robot_node->get_logger(), " ");
  RCLCPP_INFO(robot_node->get_logger(), 
              "Observe cómo ambas FSM operan simultáneamente");
  RCLCPP_INFO(robot_node->get_logger(), 
              "sin interferir entre sí");
  RCLCPP_INFO(robot_node->get_logger(), 
              "===========================================");
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
