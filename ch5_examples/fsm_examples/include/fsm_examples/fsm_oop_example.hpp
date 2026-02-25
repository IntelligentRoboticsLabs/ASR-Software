/**
 * @file fsm_oop_example.hpp
 * @brief Declaraciones para el ejemplo de FSM con patrón orientado a objetos
 */

#ifndef FSM_EXAMPLES__FSM_OOP_EXAMPLE_HPP_
#define FSM_EXAMPLES__FSM_OOP_EXAMPLE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <string>

namespace fsm_examples
{

// Forward declarations
class OOPFSMRobot;

// ============================================================================
// INTERFAZ BASE PARA TODOS LOS ESTADOS
// ============================================================================
class State
{
public:
  virtual void on_entry() {}
  virtual void on_do() = 0;
  virtual void on_exit() {}
  virtual State* check_transitions() = 0;
  virtual ~State() = default;
  
  virtual std::string get_name() const = 0;
};

// ============================================================================
// ESTADOS CONCRETOS
// ============================================================================
// Cada estado concreto mantiene un puntero al robot (contexto) para:
//   1. Leer sensores y tomar decisiones en check_transitions()
//   2. Ejecutar acciones (publicar velocidades) en on_entry/on_do/on_exit
//   3. Acceder al logger para depuración
// Esto sigue el patrón State: los estados tienen acceso al contexto.
// ============================================================================

class IdleState : public State
{
  OOPFSMRobot* robot_;  // Contexto: referencia al robot para acceder a sensores/actuadores

public:
  explicit IdleState(OOPFSMRobot* r);
  
  void on_entry() override;
  void on_do() override;
  State* check_transitions() override;
  void on_exit() override;
  std::string get_name() const override { return "IDLE"; }
};

class MovingState : public State
{
  OOPFSMRobot* robot_;  // Contexto: referencia al robot para acceder a sensores/actuadores

public:
  explicit MovingState(OOPFSMRobot* r);
  
  void on_entry() override;
  void on_do() override;
  State* check_transitions() override;
  void on_exit() override;
  std::string get_name() const override { return "MOVING"; }
};

class StoppedState : public State
{
  OOPFSMRobot* robot_;  // Contexto: referencia al robot para acceder a sensores/actuadores
  rclcpp::Time entry_time_;

public:
  explicit StoppedState(OOPFSMRobot* r);
  
  void on_entry() override;
  void on_do() override;
  State* check_transitions() override;
  void on_exit() override;
  std::string get_name() const override { return "STOPPED"; }
};

// ============================================================================
// MÁQUINA DE ESTADOS
// ============================================================================

class StateMachine
{
  State* current_state_;
  rclcpp::Logger logger_;

public:
  StateMachine(State* initial_state, rclcpp::Logger logger);
  ~StateMachine();

  void step();
};

// ============================================================================
// NODO ROBOT CON FSM ORIENTADA A OBJETOS
// ============================================================================
// OOPFSMRobot actúa como CONTEXTO en el patrón State:
//   - Contiene la FSM (StateMachine) que gestiona los estados
//   - Proporciona acceso a sensores, actuadores y configuración
//   - Los estados concretos reciben un puntero a este contexto
//   - Los callbacks de sensores solo actualizan variables (min_distance_)
//   - El timer llama a fsm_->step() para que la FSM tome decisiones
// ============================================================================

class OOPFSMRobot : public rclcpp::Node
{
  StateMachine* fsm_;
  
  // Variables de estado (memoria compartida entre estados)
  double min_distance_ = 10.0;
  const double OBSTACLE_THRESHOLD = 0.5;
  bool start_button_pressed_ = false;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  OOPFSMRobot();
  ~OOPFSMRobot();

  // Interfaz pública para que los estados accedan a la información del robot
  double get_min_distance() const { return min_distance_; }
  double get_obstacle_threshold() const { return OBSTACLE_THRESHOLD; }
  bool is_start_pressed() const { return start_button_pressed_; }
  void clear_start_button() { start_button_pressed_ = false; }
  
  void publish_velocity(double linear, double angular);
  void simulate_start_button();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_cycle();

  friend class SimulatedInputOOP;
};

// ============================================================================
// SIMULACIÓN DE ENTRADA
// ============================================================================

class SimulatedInputOOP : public rclcpp::Node
{
public:
  explicit SimulatedInputOOP(std::shared_ptr<OOPFSMRobot> robot);

private:
  std::shared_ptr<OOPFSMRobot> robot_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fsm_examples

#endif  // FSM_EXAMPLES__FSM_OOP_EXAMPLE_HPP_
