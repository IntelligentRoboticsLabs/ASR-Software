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
#include "sensor_msgs/msg/battery_state.hpp"

/**
 * Nodo simulador de batería para ejemplos FSM
 * 
 * Publica el estado de batería en /battery_state con drenaje configurable.
 * El parámetro 'drain_time' controla cuántos segundos tarda en drenar desde 100% a 0%.
 */
class BatterySimulator : public rclcpp::Node
{
public:
  BatterySimulator()
  : Node("battery_simulator"),
    battery_percentage_(100.0)
  {
    // Declarar parámetro de tiempo de drenaje (segundos)
    this->declare_parameter<double>("drain_time", 60.0);
    drain_time_ = this->get_parameter("drain_time").as_double();
    
    // Frecuencia de publicación (Hz)
    double publish_rate = 10.0;  // 10 Hz
    
    // Calcular cuánto porcentaje drenar por ciclo
    // drain_rate = 100% / (drain_time * publish_rate)
    drain_rate_ = 100.0 / (drain_time_ * publish_rate);
    
    RCLCPP_INFO(
      this->get_logger(),
      "Simulador de batería iniciado - Tiempo de drenaje: %.1f segundos (%.4f%%/ciclo)",
      drain_time_, drain_rate_
    );
    
    // Publisher de estado de batería
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10
    );
    
    // Timer para publicar periódicamente
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      std::bind(&BatterySimulator::publish_battery_state, this)
    );
  }

private:
  void publish_battery_state()
  {
    // Drenar batería gradualmente
    battery_percentage_ -= drain_rate_;
    
    // Limitar al rango [0, 100]
    if (battery_percentage_ < 0.0) {
      battery_percentage_ = 0.0;
    }
    
    // Crear mensaje de estado de batería
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "battery";
    
    // Datos principales
    msg.percentage = battery_percentage_ / 100.0;  // Normalizado [0.0, 1.0]
    msg.voltage = 12.0 * (battery_percentage_ / 100.0);  // Voltaje simulado
    msg.current = -0.5;  // Corriente negativa = drenando
    msg.charge = battery_percentage_;  // En este simulador, charge = porcentaje
    msg.capacity = 100.0;  // Capacidad máxima
    msg.design_capacity = 100.0;
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    
    // Publicar
    battery_pub_->publish(msg);
    
    // Log cada 10% de cambio
    static int last_log_percentage = 100;
    int current_percentage_int = static_cast<int>(battery_percentage_);
    if (current_percentage_int % 10 == 0 && current_percentage_int != last_log_percentage) {
      RCLCPP_INFO(
        this->get_logger(),
        "Batería: %.1f%% (%.2fV)",
        battery_percentage_, msg.voltage
      );
      last_log_percentage = current_percentage_int;
    }
    
    // Advertencia cuando llega a niveles críticos
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
  
  double battery_percentage_;  // Porcentaje actual de batería [0, 100]
  double drain_time_;          // Tiempo total de drenaje en segundos
  double drain_rate_;          // Porcentaje a drenar por ciclo
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatterySimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
