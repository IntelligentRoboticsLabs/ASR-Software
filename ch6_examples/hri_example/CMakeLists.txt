cmake_minimum_required(VERSION 3.8)
project(hri_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hri_client REQUIRED)

# Simple HRI app executable
add_executable(simple_hri_app
  src/simple_hri_app.cpp
  src/simple_hri_app_main.cpp
)

target_include_directories(simple_hri_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(simple_hri_app
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  hri_client::hri_client
)

# Install executables
install(TARGETS
  simple_hri_app
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
