// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BT_EXAMPLES__EXTRACT_INFO_ACTION_HPP_
#define BT_EXAMPLES__EXTRACT_INFO_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <simple_hri_interfaces/srv/extract.hpp>
#include <optional>

class ExtractInfoAction : public BT::StatefulActionNode
{
public:
  using Extract = simple_hri_interfaces::srv::Extract;

  ExtractInfoAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("interest", "Interest/category to extract"),
      BT::InputPort<std::string>("full_text", "Full text from ASR"),
      BT::OutputPort<std::string>("extracted_info", "Extracted useful information")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Extract>::SharedPtr extract_client_;
  std::optional<rclcpp::Client<Extract>::FutureAndRequestId> future_;
  std::chrono::steady_clock::time_point start_time_;
  std::string interest_;
  std::string full_text_;
};

#endif  // BT_EXAMPLES__EXTRACT_INFO_ACTION_HPP_
