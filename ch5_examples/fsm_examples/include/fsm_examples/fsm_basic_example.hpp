/**
 * @file fsm_basic_example.hpp
 * @brief Declaraciones para el ejemplo básico de FSM con switch-case
 */

#ifndef FSM_EXAMPLES__FSM_BASIC_EXAMPLE_HPP_
#define FSM_EXAMPLES__FSM_BASIC_EXAMPLE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace fsm_examples
{

/**
 * @class BasicFSMRobot
 * @brief Robot con FSM básica usando patrón switch-case
 * 
 * Este ejemplo ilustra la implementación más simple de una FSM en ROS 2.
 * Ventajas: Directo, fácil de entender
 * Limitaciones: No implementa on_entry/on_exit, acciones pueden repetirse
 */
class BasicFSMRobot : public rclcpp::Node
{
public:
  enum class State { IDLE, MOVING, OBSTACLE_DETECTED, STOPPED };

  BasicFSMRobot();

private:
  State current_state_;
  double min_distance_ = 10.0;
  const double OBSTACLE_THRESHOLD = 0.5;  // 0.5 metros
  bool start_button_pressed_ = false;
  rclcpp::Time stopped_time_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_cycle();
  void publish_velocity(double linear, double angular);

public:
  void simulate_start_button();

  friend class SimulatedInput;
};

/**
 * @class SimulatedInput
 * @brief Simula entrada de usuario para iniciar el robot
 */
class SimulatedInput : public rclcpp::Node
{
public:
  SimulatedInput(std::shared_ptr<BasicFSMRobot> robot);

private:
  std::shared_ptr<BasicFSMRobot> robot_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fsm_examples

#endif  // FSM_EXAMPLES__FSM_BASIC_EXAMPLE_HPP_
