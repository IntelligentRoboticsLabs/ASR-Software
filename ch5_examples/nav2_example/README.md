# nav2_example

Ejemplos de uso del cliente de acciones NavigateToPose de Nav2.

## Descripción

Este paquete contiene código de ejemplo que demuestra cómo usar la acción NavigateToPose de Nav2 para enviar objetivos de navegación a un robot móvil.

### Contenido

**Biblioteca NavigationClient** (`libnavigation_client.so`):
- Clase reutilizable que encapsula el cliente de acciones de Nav2
- Proporciona métodos para:
  - Enviar objetivos de navegación
  - Monitorizar el progreso mediante feedback
  - Consultar el estado de la navegación
  - Cancelar objetivos en progreso
  - Crear mensajes de pose con conversión de Euler a quaternion
  - Esperar bloqueando hasta que termine un objetivo

**Ejecutable simple_navigation_app** (patrón NO BLOQUEANTE):
- Aplicación de ejemplo que utiliza NavigationClient con timers
- Navega a una pose objetivo mediante una máquina de estados simple con 4 fases:
  1. Esperar disponibilidad del servidor
  2. Enviar objetivo de navegación
  3. Monitorizar progreso
  4. Procesar resultado
- **Ventaja**: No bloquea, permite procesar otros eventos
- **Uso**: Un solo objetivo de navegación

**Ejecutable simple_navigation_app_wait** (patrón BLOQUEANTE):
- Aplicación de ejemplo que utiliza `wait_for_result()` para navegación secuencial
- Navega por múltiples waypoints en secuencia esperando bloqueando entre cada uno
- **Ventaja**: Código más simple y directo para secuencias
- **Uso**: Múltiples waypoints consecutivos con espera automática

## Estructura del paquete

```
nav2_example/
├── include/nav2_example/
│   ├── navigation_client.hpp              # Header de NavigationClient
│   ├── simple_navigation_app.hpp          # Header de SimpleNavigationApp
│   └── simple_navigation_app_wait.hpp     # Header de SimpleNavigationAppWait
├── src/
│   ├── navigation_client.cpp              # Implementación de NavigationClient
│   ├── simple_navigation_app.cpp          # Implementación de SimpleNavigationApp
│   ├── simple_navigation_app_main.cpp     # Main de simple_navigation_app
│   ├── simple_navigation_app_wait.cpp     # Implementación de SimpleNavigationAppWait
│   └── simple_navigation_app_wait_main.cpp # Main de simple_navigation_app_wait
├── CMakeLists.txt
├── package.xml
├── README.md
```

## Compilación

```bash
cd ~/asr_ws
colcon build --packages-select nav2_example --symlink-install
source install/setup.bash
```

## Uso

### Ejecutar los ejemplos

Asegúrate de tener Nav2 ejecutándose con un mapa cargado y el robot localizado:

```bash
# En una terminal, lanzar Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/path/to/map.yaml

# En otra terminal, ejecutar el ejemplo NO BLOQUEANTE (un solo objetivo)
ros2 run nav2_example simple_navigation_app

# O ejecutar el ejemplo BLOQUEANTE (múltiples waypoints en secuencia)
ros2 run nav2_example simple_navigation_app_wait
```

**Diferencias entre los ejemplos:**

- `simple_navigation_app`: Navega a **un solo objetivo** usando timers (no bloqueante)
  - Ideal para aplicaciones que necesitan seguir procesando otros eventos
  - Patrón: Máquina de estados con callbacks

- `simple_navigation_app_wait`: Navega por **4 waypoints consecutivos** usando `wait_for_result()` (bloqueante)
  - Ideal para secuencias simples de navegación punto a punto
  - Patrón: Bucle secuencial con esperas automáticas
  - Demuestra cómo enviar múltiples goals sin conflictos

### Usar NavigationClient en tu propio paquete

#### 1. Añadir dependencia en `package.xml`:

```xml
<depend>nav2_example</depend>
```

#### 2. Configurar `CMakeLists.txt`:

```cmake
find_package(nav2_example REQUIRED)

add_executable(mi_aplicacion src/mi_aplicacion.cpp)

target_link_libraries(mi_aplicacion
  ${nav2_example_LIBRARIES}
)

ament_target_dependencies(mi_aplicacion
  rclcpp
  nav2_example
  geometry_msgs
)
```

#### 3. Incluir el header en tu código:

```cpp
#include "nav2_example/navigation_client.hpp"

class MiAplicacion : public rclcpp::Node
{
public:
  MiAplicacion() : Node("mi_aplicacion")
  {
    // Crear el cliente de navegación como componente
    nav_client_ = std::make_shared<NavigationClient>();
    
    // Usar el cliente...
    auto pose = nav_client_->create_pose_stamped(1.0, 2.0, 0.0);
    nav_client_->send_goal(pose);
  }

private:
  std::shared_ptr<NavigationClient> nav_client_;
};
```

## Personalización

Para modificar el objetivo de navegación en el ejemplo, edita las coordenadas en el constructor de `SimpleNavigationApp`:

```cpp
target_pose_ = nav_client_->create_pose_stamped(2.0, 1.5, 0.0);  // x, y, yaw
```