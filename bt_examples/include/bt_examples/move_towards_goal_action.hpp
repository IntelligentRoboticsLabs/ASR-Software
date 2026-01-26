// bt_nodes/move_towards_goal_action.hpp
#ifndef BT_EXAMPLES__BT_NODES__MOVE_TOWARDS_GOAL_ACTION_HPP_
#define BT_EXAMPLES__BT_NODES__MOVE_TOWARDS_GOAL_ACTION_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MoveTowardsGoalAction : public BT::StatefulActionNode {
public:
  MoveTowardsGoalAction(const std::string& name,
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
  double goal_x_, goal_y_, goal_theta_;
};

#endif  // BT_EXAMPLES__BT_NODES__MOVE_TOWARDS_GOAL_ACTION_HPP_
