# Mission-Task Integration Example

Este paquete demuestra la integración entre **FSM (Finite State Machine)** para coordinación de misión y **Behavior Trees** para implementación de tareas, ilustrando la arquitectura Misión-Tarea-Capacidad.

## Concepto Arquitectónico

El ejemplo implementa la separación clara entre:
- **Misión (FSM)**: Coordina el flujo de alto nivel entre tareas
- **Tareas (BT)**: Implementan comportamientos específicos con lógica jerárquica
- **Ejecutor Genérico**: Abstrae la ejecución de BTs mediante una interfaz reutilizable

## Componentes

### 1. BTTaskExecutor (Ejecutor Genérico de Tareas BT)

Clase reutilizable que:
- Carga un Behavior Tree desde un archivo XML
- Proporciona interfaz `tick()` para ejecutar el árbol
- Gestiona inicialización, reset y estado
- Permite múltiples instancias para diferentes tareas

**Ventaja clave**: Una única implementación sirve para todas las tareas. Solo se necesitan diferentes archivos XML.

### 2. Mission FSM

Máquina de estados finitos con 5 estados:

```
IDLE → TASK_A → TASK_B → COMPLETED → IDLE (ciclo)
          ↓        ↓
        TASK_C → COMPLETED
```

**Transiciones**:
- `TASK_A → TASK_B`: Si Task A retorna SUCCESS
- `TASK_A → TASK_C`: Si Task A retorna FAILURE (recuperación)
- `TASK_B → COMPLETED`: Si Task B retorna SUCCESS
- `TASK_B → TASK_A`: Si Task B retorna FAILURE (reintentar)
- `TASK_C → COMPLETED`: Siempre (tarea de recuperación)
- `COMPLETED → IDLE`: Después de 5 segundos
- `IDLE → TASK_A`: Después de 3 segundos (reiniciar misión)

### 3. Tareas BT (Archivos XML)

Tres tareas simples implementadas como Behavior Trees usando nodos built-in:

- **task_a.xml**: Secuencia que usa `SetBlackboard` para registrar progreso (simula procesamiento de datos). Siempre tiene éxito.
- **task_b.xml**: Incluye lógica de fallback con nodos `AlwaysSuccess` (simula intentar acción primaria con alternativa). Siempre tiene éxito.
- **task_c.xml**: Tarea de recuperación que simula procedimiento de reset. Siempre tiene éxito.

**Nota sobre implementación**: Los XMLs usan nodos built-in de BehaviorTree.CPP (`SetBlackboard`, `AlwaysSuccess`, `Fallback`, `Sequence`) que no requieren registro adicional. En un sistema real, estos se reemplazarían con nodos personalizados que implementen funcionalidad específica del dominio.

## Estructura del Paquete

```
mission_task_example/
├── include/mission_task_example/
│   └── bt_task_executor.hpp          # Header del ejecutor genérico
├── src/
│   ├── bt_task_executor.cpp          # Implementación del ejecutor
│   └── mission_fsm_example.cpp       # FSM principal
├── config/
│   ├── task_a.xml                    # Definición BT de Task A
│   ├── task_b.xml                    # Definición BT de Task B
│   └── task_c.xml                    # Definición BT de Task C
├── launch/
│   └── mission_task_example.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Compilación

```bash
cd ~/asr_ws
colcon build --packages-select mission_task_example
source install/setup.bash
```

## Ejecución

### Ejecutar el ejemplo completo

```bash
ros2 launch mission_task_example mission_task_example.launch.py
```

### Ejecutar con nivel de log detallado

```bash
ros2 launch mission_task_example mission_task_example.launch.py log_level:=debug
```

### Monitorizar el estado de la misión

En otra terminal:

```bash
ros2 topic echo /mission_state
```

## Salida Esperada

El sistema ejecutará ciclos completos de la misión. Verás logs como:

```
[INFO] [mission_fsm_node]: Initializing Mission FSM Node
[INFO] [mission_fsm_node]: Successfully initialized BT task: task_a
[INFO] [mission_fsm_node]: Successfully initialized BT task: task_b
[INFO] [mission_fsm_node]: Successfully initialized BT task: task_c
[INFO] [mission_fsm_node]: FSM Transition: IDLE -> TASK_A
[INFO] [mission_fsm_node]: Mission FSM Node ready
[INFO] [mission_fsm_node]: Task 'task_a' status: SUCCESS
[INFO] [mission_fsm_node]: Task A completed successfully
[INFO] [mission_fsm_node]: Reset BT task: task_a
[INFO] [mission_fsm_node]: FSM Transition: TASK_A -> TASK_B
[INFO] [mission_fsm_node]: Task B completed successfully
[INFO] [mission_fsm_node]: Reset BT task: task_b
[INFO] [mission_fsm_node]: FSM Transition: TASK_B -> COMPLETED
[INFO] [mission_fsm_node]: Restarting mission...
[INFO] [mission_fsm_node]: FSM Transition: COMPLETED -> IDLE
```

El ciclo completo se repite indefinidamente:
- IDLE (3 segundos) → TASK_A (instantáneo) → TASK_B (instantáneo) → COMPLETED (5 segundos) → IDLE...

## Modificar el Comportamiento

### Cambiar las transiciones de la FSM

Edita `src/mission_fsm_example.cpp` en los métodos `handleTask*State()` para modificar las condiciones de transición.

### Hacer que Task A falle para probar la recuperación

Modifica `config/task_a.xml` cambiando el último nodo:

```xml
<AlwaysFailure />  <!-- En lugar de AlwaysSuccess -->
```

Recompila y ejecuta. Verás:
```
[INFO] [mission_fsm_node]: FSM Transition: IDLE -> TASK_A
[WARN] [mission_fsm_node]: Task A failed, going to recovery task C
[INFO] [mission_fsm_node]: FSM Transition: TASK_A -> TASK_C
[INFO] [mission_fsm_node]: Task C completed (recovery task), mission completed
[INFO] [mission_fsm_node]: FSM Transition: TASK_C -> COMPLETED
```

### Implementar nodos BT personalizados

Para tareas más realistas, crea nodos personalizados. Ejemplo de un nodo de log simple:

```cpp
// En include/mission_task_example/log_action.hpp
class LogAction : public BT::SyncActionNode {
public:
  LogAction(const std::string& name, const BT::NodeConfiguration& config,
            rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }
  
  BT::NodeStatus tick() override {
    std::string msg;
    getInput("message", msg);
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};
```

Regístralo en el factory dentro de `BTTaskExecutor::initialize()` y úsalo en los XMLs:

```xml
<LogAction message="Task A: Starting execution" />
```

## Puntos Clave de Diseño

1. **Separación de Responsabilidades**: 
   - FSM decide QUÉ hacer y CUÁNDO
   - BT decide CÓMO hacer cada tarea

2. **Reutilización**: 
   - Un solo ejecutor sirve para todas las tareas
   - Solo se necesitan diferentes XMLs

3. **Extensibilidad**: 
   - Añadir tareas = añadir XML + instancia del ejecutor
   - No requiere recompilar para cambiar lógica de tareas

4. **Observabilidad**: 
   - Estado de misión publicado en topic
   - Logs detallados de transiciones y ejecución

## Relación con la Teoría

Este ejemplo ilustra:
- **Capítulo 7**: FSM como arquitectura de decisión de misión
- **Capítulo 8**: Behavior Trees para implementación de tareas
- **Capítulo 1**: Modelo Misión-Tarea-Capacidad aplicado explícitamente
- **Capítulo 5**: Sistema observable con publicación de estado

## Ejercicios Propuestos

1. Modifica `task_b.xml` para que falle aleatoriamente y observa cómo la FSM reintenta desde Task A
2. Añade una cuarta tarea que se ejecute en paralelo con Task B
3. Implementa un nodo custom de BT que haga algo más complejo que un log
4. Añade parámetros configurables a las tareas mediante el blackboard
5. Implementa un mecanismo de cancelación externa de la misión

## Referencias

- BehaviorTree.CPP: https://www.behaviortree.dev/
- ROS 2 Lifecycle Nodes: https://design.ros2.org/articles/node_lifecycle.html
