// Copyright 2026 Intelligent Robotics Lab
// Licensed under the Apache License, Version 2.0
// Simulador de batería para BT bump-and-go (idéntico a FSM)

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "bt_examples/battery_simulator.hpp"

class BatterySimulator : public rclcpp::Node
{
public:
  BatterySimulator()
  : Node("battery_simulator"),
    battery_percentage_(100.0)
  {
    this->declare_parameter<double>("drain_time", 60.0);
    drain_time_ = this->get_parameter("drain_time").as_double();
    double publish_rate = 10.0;
    drain_rate_ = 100.0 / (drain_time_ * publish_rate);
    RCLCPP_INFO(this->get_logger(),
      "Simulador de batería iniciado - Tiempo de drenaje: %.1f s (%.4f%%/ciclo)",
      drain_time_, drain_rate_);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      std::bind(&BatterySimulator::publish_battery_state, this));
  }
private:
  void publish_battery_state()
  {
    battery_percentage_ -= drain_rate_;
    if (battery_percentage_ < 0.0) battery_percentage_ = 0.0;
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "battery";
    msg.percentage = battery_percentage_ / 100.0;
    msg.voltage = 12.0 * (battery_percentage_ / 100.0);
    msg.current = -0.5;
    msg.charge = battery_percentage_;
    msg.capacity = 100.0;
    msg.design_capacity = 100.0;
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    battery_pub_->publish(msg);
    static int last_log_percentage = 100;
    int current_percentage_int = static_cast<int>(battery_percentage_);
    if (current_percentage_int % 10 == 0 && current_percentage_int != last_log_percentage) {
      RCLCPP_INFO(this->get_logger(), "Batería: %.1f%% (%.2fV)", battery_percentage_, msg.voltage);
      last_log_percentage = current_percentage_int;
    }
    if (battery_percentage_ <= 20.0 && battery_percentage_ > 19.5) {
      RCLCPP_WARN(this->get_logger(), "⚠️  Batería BAJA: %.1f%%", battery_percentage_);
    }
    if (battery_percentage_ <= 10.0 && battery_percentage_ > 9.5) {
      RCLCPP_ERROR(this->get_logger(), "🔴 Batería CRÍTICA: %.1f%%", battery_percentage_);
    }
    if (battery_percentage_ <= 0.01) {
      RCLCPP_FATAL(this->get_logger(), "💀 Batería AGOTADA - Robot detenido");
    }
  }
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double battery_percentage_;
  double drain_time_;
  double drain_rate_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatterySimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
