  #include <std_msgs/msg/bool.hpp>
// bt_nodes/return_to_charger_action.hpp
#ifndef BT_EXAMPLES__BT_NODES__RETURN_TO_CHARGER_ACTION_HPP_
#define BT_EXAMPLES__BT_NODES__RETURN_TO_CHARGER_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ReturnToChargerAction : public BT::StatefulActionNode {
public:
  ReturnToChargerAction(const std::string& name,
                        const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_in_charger_pub_;
  
  // tf_buffer_ y tf_listener_ se declaran como unique_ptr porque:
  // - tf2_ros::Buffer requiere un clock válido en su construcción (no tiene constructor por defecto)
  // - El node_ (que proporciona el clock) solo está disponible después de obtenerlo
  //   de la blackboard en el cuerpo del constructor
  // - No podemos usar la lista de inicialización (:) porque config.blackboard no es
  //   accesible hasta dentro del cuerpo del constructor
  // - unique_ptr permite construir el objeto más tarde con make_unique
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  double charger_x_;
  double charger_y_;
  // nav2 action client (no se utiliza navigation_client para tener ejemplos de ambos tipos llamadas a la acción)
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> goal_handle_future_;
  bool goal_sent_;
  bool result_received_;
  rclcpp_action::ResultCode result_code_;
  void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
};

#endif  // BT_EXAMPLES__BT_NODES__RETURN_TO_CHARGER_ACTION_HPP_
