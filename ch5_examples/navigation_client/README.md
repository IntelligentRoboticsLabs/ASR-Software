# navigation_client

Cliente reutilizable para Nav2 NavigateToPose action.

## Descripción

Este paquete proporciona una clase `NavigationClient` que encapsula la funcionalidad de navegación de Nav2, facilitando su uso en aplicaciones de navegación autónoma.

## Características

- Cliente de acción asíncrono para `NavigateToPose`
- Soporte para callbacks de feedback, respuesta y resultado
- Método bloqueante para esperar resultados (`wait_for_result`)
- Método auxiliar para crear poses (`create_pose_stamped`)
- Cancelación de objetivos en progreso

## Uso

```cpp
#include "navigation_client/navigation_client.hpp"

auto nav_client = std::make_shared<NavigationClient>();

// Esperar a que el servidor esté disponible
if (!nav_client->wait_for_action_server()) {
  // Servidor no disponible
  return;
}

// Crear y enviar objetivo
auto pose = nav_client->create_pose_stamped(1.0, 2.0, 0.0);
nav_client->send_goal(pose);
```

## Dependencias

- rclcpp
- rclcpp_action
- nav2_msgs
- geometry_msgs
- tf2
- tf2_geometry_msgs
