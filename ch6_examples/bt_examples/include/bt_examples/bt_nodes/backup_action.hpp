// bt_nodes/backup_action.hpp
#ifndef BT_EXAMPLES__BT_NODES__BACKUP_ACTION_HPP_
#define BT_EXAMPLES__BT_NODES__BACKUP_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class BackUpAction : public BT::StatefulActionNode {
public:
  BackUpAction(const std::string& name,
               const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  double distance_;
  rclcpp::Time start_time_;
  rclcpp::Duration duration_;
};

#endif  // BT_EXAMPLES__BT_NODES__BACKUP_ACTION_HPP_
