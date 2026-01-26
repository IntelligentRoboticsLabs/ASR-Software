cmake_minimum_required(VERSION 3.5)

project(kobuki_random_walker)

find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(kobuki_random_walker src/random_walker.cpp)
target_include_directories(kobuki_random_walker PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(kobuki_random_walker PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${kobuki_ros_interfaces_TARGETS}
  ${std_msgs_TARGETS}
)
target_include_directories(kobuki_random_walker PUBLIC
  ${rclcpp_components_INCLUDE_DIRS}
)

rclcpp_components_register_node(kobuki_random_walker
  PLUGIN "kobuki_random_walker::RandomWalkerNode"
  EXECUTABLE kobuki_random_walker_node)

install(TARGETS kobuki_random_walker
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(kobuki_random_walker)
ament_export_targets(${PROJECT_NAME})

ament_package()
