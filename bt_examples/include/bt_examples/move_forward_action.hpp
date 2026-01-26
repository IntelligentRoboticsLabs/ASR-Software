// bt_nodes/move_forward_action.hpp
#ifndef BT_EXAMPLES__BT_NODES__MOVE_FORWARD_ACTION_HPP_
#define BT_EXAMPLES__BT_NODES__MOVE_FORWARD_ACTION_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MoveForwardAction : public BT::SyncActionNode {
public:
  MoveForwardAction(const std::string& name,
                    const BT::NodeConfiguration& config,
                    rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

#endif  // BT_EXAMPLES__BT_NODES__MOVE_FORWARD_ACTION_HPP_
