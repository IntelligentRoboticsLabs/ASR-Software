// bt_nodes/is_obstacle_near_condition.hpp
#ifndef BT_EXAMPLES__BT_NODES__IS_OBSTACLE_NEAR_CONDITION_HPP_
#define BT_EXAMPLES__BT_NODES__IS_OBSTACLE_NEAR_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class IsObstacleNearCondition : public BT::ConditionNode {
public:
  IsObstacleNearCondition(const std::string& name,
                         const BT::NodeConfiguration& config,
                         rclcpp::Node::SharedPtr node);
  
  static BT::PortsList providedPorts();
  
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

#endif  // BT_EXAMPLES__BT_NODES__IS_OBSTACLE_NEAR_CONDITION_HPP_
