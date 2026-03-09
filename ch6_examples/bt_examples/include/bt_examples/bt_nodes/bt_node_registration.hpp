#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class HRIClient;

void register_bt_nodes(
  BT::BehaviorTreeFactory& factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<HRIClient> hri_client = nullptr);
