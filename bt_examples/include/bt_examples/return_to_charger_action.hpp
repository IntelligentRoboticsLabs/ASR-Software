// bt_nodes/return_to_charger_action.hpp
#ifndef BT_EXAMPLES__BT_NODES__RETURN_TO_CHARGER_ACTION_HPP_
#define BT_EXAMPLES__BT_NODES__RETURN_TO_CHARGER_ACTION_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ReturnToChargerAction : public BT::StatefulActionNode {
public:
  ReturnToChargerAction(const std::string& name,
                        const BT::NodeConfiguration& config,
                        rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double charger_x_;
  double charger_y_;
  // nav2 action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> goal_handle_future_;
  bool goal_sent_;
  bool result_received_;
  rclcpp_action::ResultCode result_code_;
  void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
};

#endif  // BT_EXAMPLES__BT_NODES__RETURN_TO_CHARGER_ACTION_HPP_
