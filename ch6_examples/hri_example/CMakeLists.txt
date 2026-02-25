cmake_minimum_required(VERSION 3.8)
project(hri_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simple_hri_interfaces REQUIRED)

# Include directories
include_directories(include)

# HRIClient library
add_library(hri_client SHARED
  src/hri_client.cpp
)

ament_target_dependencies(hri_client
  rclcpp
  std_srvs
  std_msgs
  simple_hri_interfaces
)

# Simple HRI app executable
add_executable(simple_hri_app
  src/simple_hri_app.cpp
  src/simple_hri_app_main.cpp
)

target_link_libraries(simple_hri_app
  hri_client
)

ament_target_dependencies(simple_hri_app
  rclcpp
  std_msgs
)

# Install library
install(TARGETS
  hri_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  simple_hri_app
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(hri_client)
ament_export_dependencies(
  rclcpp
  std_srvs
  std_msgs
  simple_hri_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
