#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

void register_bt_nodes(BT::BehaviorTreeFactory& factory, rclcpp::Node::SharedPtr node);
