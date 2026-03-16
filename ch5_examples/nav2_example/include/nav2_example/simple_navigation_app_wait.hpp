// Copyright 2026 Intelligent Robotics Lab
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

#ifndef NAV2_EXAMPLE__SIMPLE_NAVIGATION_APP_WAIT_HPP_
#define NAV2_EXAMPLE__SIMPLE_NAVIGATION_APP_WAIT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "navigation_client/navigation_client.hpp"

class SimpleNavigationAppWait : public rclcpp::Node
{
public:
  SimpleNavigationAppWait();

private:
  std::shared_ptr<NavigationClient> nav_client_;
};

#endif  // NAV2_EXAMPLE__SIMPLE_NAVIGATION_APP_WAIT_HPP_
