cmake_minimum_required(VERSION 3.8)
project(nav2_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# NavigationClient library
add_library(navigation_client SHARED
  src/navigation_client.cpp
)

ament_target_dependencies(navigation_client
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Simple navigation app executable
add_executable(simple_navigation_app
  src/simple_navigation_app.cpp
  src/simple_navigation_app_main.cpp
)

target_link_libraries(simple_navigation_app
  navigation_client
)

ament_target_dependencies(simple_navigation_app
  rclcpp
  geometry_msgs
)

# Install library
install(TARGETS
  navigation_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  simple_navigation_app
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(navigation_client)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
