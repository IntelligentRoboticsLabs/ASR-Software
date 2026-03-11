cmake_minimum_required(VERSION 3.8)
project(fsm_examples)

# Configuración del compilador
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Ejemplo 1: FSM básica con switch-case
add_executable(fsm_basic_example src/fsm_basic_example.cpp)

target_include_directories(fsm_basic_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fsm_basic_example
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Ejemplo 2: FSM con patrón orientado a objetos (lifecycle completo)
add_executable(fsm_oop_example src/fsm_oop_example.cpp)

target_include_directories(fsm_oop_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fsm_oop_example
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Ejemplo 3: FSM con regiones ortogonales (navegación + batería)
add_executable(fsm_orthogonal_example src/fsm_orthogonal_example.cpp)

target_include_directories(fsm_orthogonal_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fsm_orthogonal_example
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Simulador de batería (utilidad para ejemplos)
add_executable(battery_simulator src/battery_simulator.cpp)

target_link_libraries(battery_simulator
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Instalación de ejecutables
install(TARGETS
  fsm_basic_example
  fsm_oop_example
  fsm_orthogonal_example
  battery_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Instalación de directorios adicionales
install(DIRECTORY
  launch
  include/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
