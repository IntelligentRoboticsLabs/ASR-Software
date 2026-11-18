cmake_minimum_required(VERSION 3.8)
project(kobuki)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  kobuki_ros_interfaces
)

# Targets de CMake de las dependencias (ament_target_dependencies no existe desde Lyrical)
set(dependency_targets
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${kobuki_ros_interfaces_TARGETS}
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/kobuki/FakeBumper.cpp
)
target_link_libraries(${PROJECT_NAME} ${dependency_targets})

add_executable(fake_bumer_node src/fake_bumper_node.cpp)
target_link_libraries(fake_bumer_node ${dependency_targets})
target_link_libraries(fake_bumer_node ${PROJECT_NAME})

install(TARGETS
  fake_bumer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config maps worlds rviz models
        DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
