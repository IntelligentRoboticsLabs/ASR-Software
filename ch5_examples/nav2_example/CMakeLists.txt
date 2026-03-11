cmake_minimum_required(VERSION 3.8)
project(nav2_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigation_client REQUIRED)

# Simple navigation app executable
add_executable(simple_navigation_app
  src/simple_navigation_app.cpp
  src/simple_navigation_app_main.cpp
)

target_include_directories(simple_navigation_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(simple_navigation_app
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  navigation_client::navigation_client
)

add_executable(simple_navigation_app_wait
  src/simple_navigation_app_wait.cpp
  src/simple_navigation_app_wait_main.cpp
)

target_include_directories(simple_navigation_app_wait PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(simple_navigation_app_wait
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  navigation_client::navigation_client
)

# Install executables
install(TARGETS
  simple_navigation_app
  simple_navigation_app_wait
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
