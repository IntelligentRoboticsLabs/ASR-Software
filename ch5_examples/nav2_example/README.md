# nav2_example

Ejemplos de uso de la acción `NavigateToPose` de Nav2 a través de `NavigationClient`
(paquete `navigation_client`).

## Contenido

**`simple_navigation_app`** (patrón NO BLOQUEANTE):

- Usa `NavigationClient` desde un timer, con 4 fases:
  1. Verificar la disponibilidad de Nav2 (`wait_until_ready`)
  2. Enviar el objetivo (`start_navigation`, con *timeout*)
  3. Monitorizar el progreso (`is_navigation_done`, `get_navigation_feedback`)
  4. Procesar el resultado (`get_navigation_state`: `COMPLETED`, `ERROR`, `TIMEOUT` o
     `CANCELED`)
- El objetivo y el *timeout* son parámetros: `x`, `y`, `yaw` (frame `map`) y
  `goal_timeout` (segundos de tiempo del robot; por defecto 120).
- Termina al acabar la navegación.

**`simple_navigation_app_wait`** (patrón BLOQUEANTE):

- Recorre 4 waypoints en secuencia con `wait_for_navigation_result()`.
- Código más simple para secuencias, pero el nodo no atiende nada más mientras espera.

## Estructura del paquete

```text
nav2_example/
├── include/nav2_example/
│   ├── simple_navigation_app.hpp
│   └── simple_navigation_app_wait.hpp
├── src/
│   ├── simple_navigation_app.cpp
│   ├── simple_navigation_app_main.cpp
│   ├── simple_navigation_app_wait.cpp
│   └── simple_navigation_app_wait_main.cpp
├── CMakeLists.txt
└── package.xml
```

`NavigationClient` está en el paquete `navigation_client` (ver su README).

## Compilación

```bash
cd ~/asr_ws
python3 -m colcon build --symlink-install --packages-up-to nav2_example
source install/setup.bash
```

## Uso

Con Nav2 en marcha, un mapa cargado y el robot localizado:

```bash
# Un objetivo (no bloqueante); el objetivo y el timeout se pueden cambiar
ros2 run nav2_example simple_navigation_app
ros2 run nav2_example simple_navigation_app --ros-args -p x:=2.0 -p y:=1.5 -p goal_timeout:=60.0

# Varios waypoints en secuencia (bloqueante)
ros2 run nav2_example simple_navigation_app_wait
```

En simulación, añadir `-p use_sim_time:=true`: el *timeout* cuenta tiempo del robot.

## Usar NavigationClient en tu propio paquete

`package.xml`:

```xml
<depend>navigation_client</depend>
```

`CMakeLists.txt`:

```cmake
find_package(navigation_client REQUIRED)
target_link_libraries(mi_aplicacion navigation_client::navigation_client)
```

Código:

```cpp
#include "navigation_client/navigation_client.hpp"

// En el constructor del nodo (composición: el cliente usa este nodo)
nav_client_ = std::make_shared<NavigationClient>(this);
```
