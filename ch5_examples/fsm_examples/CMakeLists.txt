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

# Incluir directorios de cabeceras
include_directories(include)

# Ejemplo 1: FSM básica con switch-case
add_executable(fsm_basic_example src/fsm_basic_example.cpp)
ament_target_dependencies(fsm_basic_example 
  rclcpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
)

# Ejemplo 2: FSM con patrón orientado a objetos (lifecycle completo)
add_executable(fsm_oop_example src/fsm_oop_example.cpp)
ament_target_dependencies(fsm_oop_example 
  rclcpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
)

# Ejemplo 3: FSM con regiones ortogonales (navegación + batería)
add_executable(fsm_orthogonal_example src/fsm_orthogonal_example.cpp)
ament_target_dependencies(fsm_orthogonal_example 
  rclcpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
)

# Simulador de batería (utilidad para ejemplos)
add_executable(battery_simulator src/battery_simulator.cpp)
ament_target_dependencies(battery_simulator 
  rclcpp 
  sensor_msgs
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
