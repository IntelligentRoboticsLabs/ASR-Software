cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/camera/HSVFilterNode.cpp
  src/camera/YoloDetectionNode.cpp
  src/camera/DetectionTo3DfromDepthNode.cpp
  src/camera/DetectionTo3DfromPCNode.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${cv_bridge_INCLUDE_DIRS} 
  ${pcl_ros_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
  ${depth_image_proc_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  message_filters::message_filters
  image_geometry::image_geometry
  tf2::tf2
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${yolo_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS} )

add_executable(hsv_filter src/hsv_filter_main.cpp)
target_link_libraries(hsv_filter ${PROJECT_NAME})

add_executable(yolo_detection src/yolo_detection_main.cpp)
target_link_libraries(yolo_detection ${PROJECT_NAME})

add_executable(detection_2d_to_3d_pc2 src/detection_2d_to_3d_pc2_main.cpp)
target_link_libraries(detection_2d_to_3d_pc2 ${PROJECT_NAME})

add_executable(detection_2d_to_3d_depth src/detection_2d_to_3d_depth_main.cpp)
target_link_libraries(detection_2d_to_3d_depth ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  hsv_filter
  yolo_detection
  detection_2d_to_3d_pc2
  detection_2d_to_3d_depth
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY worlds launch config models DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
  vision_msgs
  image_transport
  cv_bridge
  OpenCV
  message_filters
  pcl_conversions
  pcl_ros
  image_geometry
  depth_image_proc
  yolo_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)
ament_package()
