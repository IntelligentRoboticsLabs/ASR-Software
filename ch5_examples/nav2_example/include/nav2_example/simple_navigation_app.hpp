// Copyright 2024 Intelligent Robotics Lab
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

#ifndef NAV2_EXAMPLE__SIMPLE_NAVIGATION_APP_HPP_
#define NAV2_EXAMPLE__SIMPLE_NAVIGATION_APP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navigation_client/navigation_client.hpp"

class SimpleNavigationApp : public rclcpp::Node
{
public:
  SimpleNavigationApp();

private:
  void control_cycle();

  std::shared_ptr<NavigationClient> nav_client_;
  geometry_msgs::msg::PoseStamped target_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool server_ready_ = false;
  bool goal_sent_ = false;
};

#endif  // NAV2_EXAMPLE__SIMPLE_NAVIGATION_APP_HPP_
