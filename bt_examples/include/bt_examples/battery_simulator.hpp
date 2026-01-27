#pragma once
// Copyright 2026 Intelligent Robotics Lab
// Licensed under the Apache License, Version 2.0
// Declaración del simulador de batería para BT bump-and-go


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

class BatterySimulator : public rclcpp::Node
{
public:
	BatterySimulator();

private:
	void publish_battery_state();
	void charger_callback(const std_msgs::msg::Bool::SharedPtr msg);
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charger_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
	double battery_percentage_;
	double drain_time_;
	double drain_rate_;
};
