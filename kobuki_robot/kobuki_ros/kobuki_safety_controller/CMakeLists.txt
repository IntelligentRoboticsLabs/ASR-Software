cmake_minimum_required(VERSION 3.5)
project(kobuki_safety_controller)

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_build REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(kobuki_safety_controller SHARED src/safety_controller.cpp)
target_include_directories(kobuki_safety_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(kobuki_safety_controller
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${kobuki_ros_interfaces_TARGETS}
  ${std_msgs_TARGETS}
)
target_include_directories(kobuki_safety_controller PUBLIC
  ${rclcpp_components_INCLUDE_DIRS}
)

rclcpp_components_register_node(kobuki_safety_controller
  PLUGIN "kobuki_safety_controller::SafetyController"
  EXECUTABLE kobuki_safety_controller_node)

install(TARGETS kobuki_safety_controller
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package()
