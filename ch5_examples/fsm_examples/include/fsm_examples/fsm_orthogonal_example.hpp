/**
 * @file fsm_orthogonal_example.hpp
 * @brief Declaraciones para el ejemplo de FSM con descomposición ortogonal
 */

#ifndef FSM_EXAMPLES__FSM_ORTHOGONAL_EXAMPLE_HPP_
#define FSM_EXAMPLES__FSM_ORTHOGONAL_EXAMPLE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <string>

namespace fsm_examples
{

// Forward declarations
class OrthogonalRobot;

// ============================================================================
// REGIÓN 1: FSM DE NAVEGACIÓN
// ============================================================================

class NavState
{
public:
  virtual void on_entry() {}
  virtual void on_do() = 0;
  virtual NavState* check_transitions() = 0;
  virtual void on_exit() {}
  virtual ~NavState() = default;
  virtual std::string get_name() const = 0;
};

class NavIdleState : public NavState
{
  OrthogonalRobot* robot_;
public:
  explicit NavIdleState(OrthogonalRobot* r);
  void on_entry() override;
  void on_do() override;
  NavState* check_transitions() override;
  std::string get_name() const override { return "NAV:IDLE"; }
};

class NavMovingState : public NavState
{
  OrthogonalRobot* robot_;
public:
  explicit NavMovingState(OrthogonalRobot* r);
  void on_entry() override;
  void on_do() override;
  NavState* check_transitions() override;
  void on_exit() override;
  std::string get_name() const override { return "NAV:MOVING"; }
};

class NavStoppedState : public NavState
{
  OrthogonalRobot* robot_;
  rclcpp::Time entry_time_;
public:
  explicit NavStoppedState(OrthogonalRobot* r);
  void on_entry() override;
  void on_do() override;
  NavState* check_transitions() override;
  std::string get_name() const override { return "NAV:STOPPED"; }
};

// ============================================================================
// REGIÓN 2: FSM DE BATERÍA
// ============================================================================

class BatteryState
{
public:
  virtual void on_entry() {}
  virtual void on_do() = 0;
  virtual BatteryState* check_transitions() = 0;
  virtual void on_exit() {}
  virtual ~BatteryState() = default;
  virtual std::string get_name() const = 0;
};

class BatteryOKState : public BatteryState
{
  OrthogonalRobot* robot_;
public:
  explicit BatteryOKState(OrthogonalRobot* r);
  void on_entry() override;
  void on_do() override;
  BatteryState* check_transitions() override;
  std::string get_name() const override { return "BAT:OK"; }
};

class BatteryLowState : public BatteryState
{
  OrthogonalRobot* robot_;
public:
  explicit BatteryLowState(OrthogonalRobot* r);
  void on_entry() override;
  void on_do() override;
  BatteryState* check_transitions() override;
  std::string get_name() const override { return "BAT:LOW"; }
};

class BatteryCriticalState : public BatteryState
{
  OrthogonalRobot* robot_;
public:
  explicit BatteryCriticalState(OrthogonalRobot* r);
  void on_entry() override;
  void on_do() override;
  BatteryState* check_transitions() override;
  std::string get_name() const override { return "BAT:CRITICAL"; }
};

// ============================================================================
// MÁQUINAS DE ESTADOS GENÉRICAS
// ============================================================================

template<typename T>
class GenericStateMachine
{
  T* current_state_;
  rclcpp::Logger logger_;
  std::string region_name_;

public:
  GenericStateMachine(T* initial_state, rclcpp::Logger logger, const std::string& region_name)
    : current_state_(initial_state), logger_(logger), region_name_(region_name)
  {
    current_state_->on_entry();
  }

  void step()
  {
    current_state_->on_do();
    
    T* next_state = current_state_->check_transitions();
    
    if (next_state != nullptr) {
      RCLCPP_INFO(logger_, "[%s] Transición: %s -> %s",
                  region_name_.c_str(),
                  current_state_->get_name().c_str(),
                  next_state->get_name().c_str());
      
      current_state_->on_exit();
      delete current_state_;
      current_state_ = next_state;
      current_state_->on_entry();
    }
  }

  std::string get_current_state_name() const
  {
    return current_state_->get_name();
  }

  ~GenericStateMachine()
  {
    delete current_state_;
  }
};

// ============================================================================
// NODO ROBOT CON DOS FSM ORTOGONALES
// ============================================================================

class OrthogonalRobot : public rclcpp::Node
{
  // Dos FSM independientes (regiones ortogonales)
  GenericStateMachine<NavState>* nav_fsm_;
  GenericStateMachine<BatteryState>* battery_fsm_;
  
  // Variables de estado compartidas
  double min_distance_ = 10.0;
  double battery_percentage_ = 100.0;
  bool start_button_pressed_ = false;
  
  const double OBSTACLE_THRESHOLD = 0.5;
  const double BATTERY_LOW_THRESHOLD = 30.0;
  const double BATTERY_CRITICAL_THRESHOLD = 10.0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

public:
  OrthogonalRobot();
  ~OrthogonalRobot();

  // Interfaz pública para que los estados accedan a la información
  double get_min_distance() const { return min_distance_; }
  double get_battery_percentage() const { return battery_percentage_; }
  double get_obstacle_threshold() const { return OBSTACLE_THRESHOLD; }
  double get_battery_low_threshold() const { return BATTERY_LOW_THRESHOLD; }
  double get_battery_critical_threshold() const { return BATTERY_CRITICAL_THRESHOLD; }
  bool is_start_pressed() const { return start_button_pressed_; }
  void clear_start_button() { start_button_pressed_ = false; }
  
  void publish_velocity(double linear, double angular);
  void simulate_start_button();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void control_cycle();
  void print_status();

  friend class SimulatedInputOrthogonal;
  friend class BatterySimulator;
};

// ============================================================================
// SIMULACIÓN DE ENTRADA Y BATERÍA
// ============================================================================

class SimulatedInputOrthogonal : public rclcpp::Node
{
public:
  explicit SimulatedInputOrthogonal(std::shared_ptr<OrthogonalRobot> robot);

private:
  std::shared_ptr<OrthogonalRobot> robot_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class BatterySimulator : public rclcpp::Node
{
public:
  BatterySimulator();

private:
  void publish_battery();

  double battery_level_ = 100.0;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fsm_examples

#endif  // FSM_EXAMPLES__FSM_ORTHOGONAL_EXAMPLE_HPP_
