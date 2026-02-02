cmake_minimum_required(VERSION 3.8)
project(mission_task_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  rclcpp_components
  behaviortree_cpp_v3
  ament_index_cpp
)

include_directories(include)

# TaskLifecycleNode library
add_library(task_lifecycle_node SHARED
  src/task_lifecycle_node.cpp
)
ament_target_dependencies(task_lifecycle_node ${dependencies})

# BTTaskNode library (generic BT task node)
add_library(bt_task_node SHARED
  src/bt_task_node.cpp
)
ament_target_dependencies(bt_task_node ${dependencies})
target_link_libraries(bt_task_node task_lifecycle_node)

# MissionExecutor library
add_library(mission_executor SHARED
  src/mission_executor.cpp
)
ament_target_dependencies(mission_executor ${dependencies})
target_link_libraries(mission_executor task_lifecycle_node)

# Mission FSM executable
add_executable(mission_fsm_example
  src/mission_fsm_example.cpp
)
ament_target_dependencies(mission_fsm_example ${dependencies})
target_link_libraries(mission_fsm_example mission_executor task_lifecycle_node bt_task_node)

# Install
install(TARGETS
  task_lifecycle_node
  bt_task_node
  mission_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mission_fsm_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(task_lifecycle_node bt_task_node mission_executor)
ament_export_dependencies(${dependencies})

ament_package()
