// Copyright 2021 Intelligent Robotics Lab
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

#include <limits>
#include <vector>
#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser/ObstacleDetectorNode.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;

class ObstacleDetectorNodeTest : public laser::ObstacleDetectorNode
{
public:
  bool is_obstacle_test(const sensor_msgs::msg::LaserScan & scan, float dist_thrld)
  {
    return is_obstacle(scan, dist_thrld);
  }
};

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[2] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[6] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_5(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[10] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_6(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.5);
  ret.ranges[10] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_7(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[14] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_8(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[8] = 0.8;

  return ret;
}

TEST(laser_tests, is_obstacle)
{
  auto laser_node = ObstacleDetectorNodeTest();
  rclcpp::Time ts = laser_node.now();

  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_1(ts), 0.5));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_2(ts), 0.5));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_3(ts), 0.5));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_4(ts), 0.5));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_5(ts), 0.5));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_6(ts), 0.5));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_7(ts), 0.5));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_8(ts), 0.5));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_1(ts), 0.1));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_2(ts), 0.1));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_3(ts), 0.1));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_4(ts), 0.1));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_5(ts), 0.1));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_6(ts), 0.1));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_7(ts), 0.1));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_8(ts), 0.1));
  ASSERT_FALSE(laser_node.is_obstacle_test(get_scan_test_1(ts), 1.0));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_2(ts), 1.0));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_3(ts), 1.0));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_4(ts), 1.0));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_5(ts), 1.0));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_6(ts), 1.0));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_7(ts), 1.0));
  ASSERT_TRUE(laser_node.is_obstacle_test(get_scan_test_8(ts), 1.0));
}

TEST(laser_tests, ouput_obs)
{
  auto laser_node = std::make_shared<ObstacleDetectorNodeTest>();

  // Create a testing node with a scan publisher and a speed subscriber
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto scan_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>("input_laser", 100);

  std_msgs::msg::Bool last_obs;
  auto obs_sub = test_node->create_subscription<std_msgs::msg::Bool>(
    "obstacle", 1, [&last_obs](std_msgs::msg::Bool::SharedPtr msg) {
      last_obs = *msg;
    });

  ASSERT_EQ(obs_sub->get_publisher_count(), 1);
  ASSERT_EQ(scan_pub->get_subscription_count(), 1);

  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(laser_node->get_node_base_interface());
  executor.add_node(test_node);

  // Test for scan test #1
  auto start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_1(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_FALSE(last_obs.data);

  // Test for scan test #2
  start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_2(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(last_obs.data);

  // Test for scan test #3
  start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_3(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(last_obs.data);

  // Test for scan test #4
  start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_4(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(last_obs.data);

  // Test for scan test #5
  start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_5(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(last_obs.data);

  // Test for scan test #6
  start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_6(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(last_obs.data);

  // Test for scan test #7
  start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_7(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(last_obs.data);

  // Test for scan test #8
  start = laser_node->now();
  while (rclcpp::ok() && (laser_node->now() - start) < 0.5s) {
    scan_pub->publish(get_scan_test_8(laser_node->now()));
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_FALSE(last_obs.data);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
