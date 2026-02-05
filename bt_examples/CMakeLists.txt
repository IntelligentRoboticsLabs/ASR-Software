cmake_minimum_required(VERSION 3.8)
project(bt_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(simple_hri_interfaces REQUIRED)

# Include directories
include_directories(include)

 # BT node implementations library
add_library(bt_nodes SHARED
  src/is_obstacle_near_condition.cpp
  src/is_battery_low_condition.cpp
  src/backup_action.cpp
  src/spin_action.cpp
  src/move_forward_action.cpp
  src/return_to_charger_action.cpp
  src/abort_mission_action.cpp
  src/say_text_action.cpp
  src/listen_text_action.cpp
  src/extract_info_action.cpp
  src/bt_nodes/bt_node_registration.cpp
)

ament_target_dependencies(bt_nodes
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  nav2_msgs
  behaviortree_cpp
  simple_hri_interfaces
)

include_directories(include/bt_examples/bt_nodes)
# Bump-and-go BT example
add_executable(bumpandgo_bt_example src/bumpandgo_bt_example.cpp)
target_link_libraries(bumpandgo_bt_example bt_nodes)
ament_target_dependencies(bumpandgo_bt_example
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  ament_index_cpp
  behaviortree_cpp
)

# Drink order BT example
add_executable(drink_order_bt_example src/drink_order_bt_example.cpp)
target_link_libraries(drink_order_bt_example bt_nodes)
ament_target_dependencies(drink_order_bt_example
  rclcpp
  rclcpp_action
  std_msgs
  ament_index_cpp
  behaviortree_cpp
  simple_hri_interfaces
)

# Battery simulator node (for BT)
add_executable(battery_simulator src/battery_simulator.cpp)
ament_target_dependencies(battery_simulator
  rclcpp
  sensor_msgs
)

# Install library
install(TARGETS bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  bumpandgo_bt_example
  drink_order_bt_example
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  battery_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
