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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "laser/ObstacleDetectorNode.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace laser
{

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
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
  RCLCPP_DEBUG(get_logger(), "Checking for obstacles within %f m...", dist_thrld);

  float distance_min = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }
    distance_min = std::min(distance_min, range);
  }

  if (!std::isfinite(distance_min)) {
    return false;
  }

  return distance_min < dist_thrld;
}

void ObstacleDetectorNode::print_obstacle_info(
  const sensor_msgs::msg::LaserScan & scan,
  float dist_thrld)
{
  if (scan.ranges.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty LaserScan ranges");
    return;
  }

  size_t min_idx = 0;
  float distance_min = std::numeric_limits<float>::infinity();
  bool found_valid = false;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }

    if (range < distance_min) {
      distance_min = range;
      min_idx = i;
      found_valid = true;
    }
  }

  if (!found_valid) {
    RCLCPP_WARN(get_logger(), "No valid LaserScan ranges (all inf/nan/out-of-range)");
    return;
  }

  float obstacle_angle = scan.angle_min + static_cast<float>(min_idx) * scan.angle_increment;
  RCLCPP_INFO(get_logger(), "Min distance: %f m at angle %f deg", distance_min,
      obstacle_angle * 180.0f / M_PI);

  float obstacle_x = distance_min * std::cos(obstacle_angle);
  float obstacle_y = distance_min * std::sin(obstacle_angle);
  RCLCPP_INFO(get_logger(), "Obstacle position: x = %f m, y = %f m", obstacle_x, obstacle_y);

  if (distance_min < dist_thrld) {
    RCLCPP_WARN(get_logger(), "Obstacle detected within threshold of %f m!", dist_thrld);
  }

  // Also report the obstacle position in the robot frame (base_link) using TF.
  // The point is initially expressed in the LaserScan frame (scan.header.frame_id).
  try {
    geometry_msgs::msg::PointStamped obstacle_point;
    obstacle_point.header = scan.header;
    obstacle_point.point.x = obstacle_x;
    obstacle_point.point.y = obstacle_y;
    obstacle_point.point.z = 0.0;

    auto tf = tf_buffer_.lookupTransform(
      "base_link",
      obstacle_point.header.frame_id,
      obstacle_point.header.stamp,
      tf2::durationFromSec(0.1));

    geometry_msgs::msg::PointStamped obstacle_point_base;
    tf2::doTransform(obstacle_point, obstacle_point_base, tf);

    RCLCPP_INFO(
      get_logger(),
      "Obstacle position in %s: x = %f m, y = %f m, z = %f m",
      "base_link",
      obstacle_point_base.point.x,
      obstacle_point_base.point.y,
      obstacle_point_base.point.z);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF transform to base_link failed: %s", ex.what());
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
