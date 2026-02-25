#include "bt_examples/return_to_charger_action.hpp"
#include <cmath>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

ReturnToChargerAction::ReturnToChargerAction(const std::string& name,
                                             const BT::NodeConfiguration& config,
                                             rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node),

    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_),
    goal_sent_(false),
    result_received_(false) {
  robot_in_charger_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot_in_charger", 1);
  using namespace std::placeholders;
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "navigate_to_pose");
}

BT::PortsList ReturnToChargerAction::providedPorts() {
  return {
    BT::InputPort<double>("charger_x", 0.0, "Charger X coordinate"),
    BT::InputPort<double>("charger_y", 0.0, "Charger Y coordinate")
  };
}

BT::NodeStatus ReturnToChargerAction::onStart() {
  if (!getInput("charger_x", charger_x_) ||
      !getInput("charger_y", charger_y_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing charger position parameters");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
              "Returning to charger at (%.2f, %.2f)",
              charger_x_, charger_y_);
  goal_sent_ = false;
  result_received_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToChargerAction::onRunning() {
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for navigate_to_pose action server...");
    return BT::NodeStatus::RUNNING;
  }

  if (!goal_sent_) {
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = charger_x_;
    goal.pose.pose.position.y = charger_y_;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&ReturnToChargerAction::resultCallback, this, std::placeholders::_1);

    auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
    goal_handle_future_ = future_goal_handle;
    goal_sent_ = true;
    RCLCPP_INFO(node_->get_logger(), "Sent NavigateToPose goal to (%.2f, %.2f)", charger_x_, charger_y_);
    return BT::NodeStatus::RUNNING;
  }

  if (result_received_) {
    if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "Reached charger! Ready to recharge.");
      std_msgs::msg::Bool msg;
      msg.data = true;
      robot_in_charger_pub_->publish(msg);
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to reach charger (NavigateToPose result: %d)", static_cast<int>(result_code_));
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void ReturnToChargerAction::onHalted() {
  if (goal_sent_ && !result_received_) {
    if (goal_handle_future_.valid()) {
      try {
        auto goal_handle = goal_handle_future_.get();
        if (goal_handle) {
          action_client_->async_cancel_goal(goal_handle);
        }
      } catch (...) {}
    }
  }
}

void ReturnToChargerAction::resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
  result_code_ = result.code;
  result_received_ = true;
}
// Add to class definition in return_to_charger_action.hpp:
//
// rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
// std::future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> goal_handle_future_;
// bool goal_sent_;
// bool result_received_;
// rclcpp_action::ResultCode result_code_;
// void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
