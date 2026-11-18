# navigation_client

Cliente reutilizable para la acción `NavigateToPose` de Nav2.

## Descripción

La clase `NavigationClient` encapsula la navegación de Nav2 para usarla en aplicaciones
(FSM) y behavior trees. Cumple el mismo **contrato de capacidad** que `HRIClient`
(`hri_client`): `start_*()`, `is_*_done()`, `get_*_state()`, `cancel_*()`.

## Características

- Cliente de acción asíncrono (no bloqueante) para `NavigateToPose`.
- Estado del objetivo (`NavigationState`): `IDLE`, `IN_PROGRESS`, `COMPLETED`, `ERROR`,
  `TIMEOUT`, `CANCELED`. `to_string(state)` devuelve el nombre, para los logs.
- *Timeout* por objetivo, en tiempo del **robot** (reloj del nodo: con `use_sim_time` es el
  del simulador, así un objetivo no expira antes porque el simulador vaya lento).
- Cancelación también antes de que Nav2 acepte el objetivo.
- Cada objetivo tiene un número de secuencia: los callbacks de un objetivo anterior (p. ej.
  su resultado, que llega tras un *timeout*) no cambian el estado del nuevo.
- Método bloqueante para secuencias simples (`wait_for_navigation_result`).
- Método auxiliar para crear poses (`create_pose_stamped`).

## Uso

```cpp
#include "navigation_client/navigation_client.hpp"

// En el constructor del nodo (composición: el cliente usa este nodo)
nav_client_ = std::make_shared<NavigationClient>(this);

// En el ciclo de control (timer), sin bloquear:
if (!nav_client_->wait_until_ready(std::chrono::seconds(1))) {
  return;  // Nav2 aún no está disponible
}
nav_client_->start_navigation(nav_client_->create_pose_stamped(1.0, 2.0, 0.0), 120.0);
// ... ciclos siguientes:
if (nav_client_->is_navigation_done()) {
  if (nav_client_->get_navigation_state() == NavigationState::COMPLETED) {
    // Llegó
  } else {
    // ERROR (Nav2 lo abortó o rechazó), TIMEOUT o CANCELED
  }
}
```

| Método | Descripción |
| --- | --- |
| `wait_until_ready(timeout)` | ¿Está disponible el servidor de navegación? |
| `start_navigation(pose, timeout_sec)` | Envía el objetivo (asíncrono). `timeout_sec` en tiempo del robot; `0` = sin límite. Si había otro objetivo en curso, lo cancela |
| `is_navigation_done()` | `true` cuando el objetivo ha terminado. Si se agota el *timeout*, cancela el objetivo (`TIMEOUT`) |
| `get_navigation_state()` | Cómo terminó: `COMPLETED`, `ERROR`, `TIMEOUT` o `CANCELED` |
| `get_navigation_feedback()` | Último *feedback* (distancia restante, tiempo...) o `nullptr` |
| `cancel_navigation()` | Cancela el objetivo → `CANCELED` |
| `wait_for_navigation_result()` | **Bloqueante**: procesa callbacks hasta que el objetivo termine; `true` si `COMPLETED`. Solo si el nodo no está ya en un executor (p. ej. en el constructor) |
| `create_pose_stamped(x, y, yaw)` | Pose en el frame `map` |

## Dependencias

- rclcpp
- rclcpp_action
- nav2_msgs
- geometry_msgs
- tf2
- tf2_geometry_msgs
