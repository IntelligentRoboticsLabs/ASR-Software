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

#include <memory>

#include "laser/ObstacleDetectorNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"

namespace laser
{

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node")
{
  declare_parameter("min_distance", min_distance_);
  get_parameter("min_distance", min_distance_);

  RCLCPP_INFO(get_logger(), "ObstacleDetectorNode set to %f m", min_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&ObstacleDetectorNode::laser_callback, this, _1));
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(
    "obstacle", 100);
}

bool
ObstacleDetectorNode::is_obstacle(const sensor_msgs::msg::LaserScan & scan, float dist_thrld)
{
  std::cerr << "Checking for obstacles within " << dist_thrld << " m..." << std::endl;
  int min_idx = std::min_element(scan.ranges.begin(), scan.ranges.end()) - scan.ranges.begin();
  float distance_min = scan.ranges[min_idx];

  return distance_min < dist_thrld;
}

void ObstacleDetectorNode::print_obstacle_info(const sensor_msgs::msg::LaserScan & scan, float dist_thrld)
{
  int min_idx = std::min_element(scan.ranges.begin(), scan.ranges.end()) - scan.ranges.begin();
  float distance_min = scan.ranges[min_idx];

  float obstacle_angle = scan.angle_min + min_idx * scan.angle_increment;
  RCLCPP_INFO(get_logger(), "Min distance: %f m at angle %f deg", distance_min, obstacle_angle * 180.0f / M_PI);

  float obstacle_x = distance_min * std::cos(obstacle_angle);
  float obstacle_y = distance_min * std::sin(obstacle_angle);
  RCLCPP_INFO(get_logger(), "Obstacle position: x = %f m, y = %f m", obstacle_x, obstacle_y);

  if (distance_min < dist_thrld) {
    RCLCPP_WARN(get_logger(), "Obstacle detected within threshold of %f m!", dist_thrld);
  }
}

void
ObstacleDetectorNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  std_msgs::msg::Bool obstacle_msg;
  obstacle_msg.data = is_obstacle(*scan, min_distance_);

  if (obstacle_msg.data) {
    print_obstacle_info(*scan, min_distance_);
  }

  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace laser
