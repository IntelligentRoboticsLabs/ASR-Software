# BT Examples - Ejemplos de Behavior Trees para ASR

Este paquete contiene ejemplos progresivos de implementación de Behavior Trees en ROS 2, diseñados para complementar el **Capítulo 8** del libro ASR y la **Práctica 5**.

> **📌 Nota importante**: El código de estos ejemplos está alineado con los fragmentos de código que aparecen en el libro (Capítulo 8 y Práctica 5). Los patrones, comentarios y estructura siguen exactamente los presentados en el material teórico para facilitar el seguimiento en clase:
> - **Bump-and-Go**: Práctica 5, secciones 5.3.6 y 5.3.7 (Ejemplo completo con puertos y blackboard)

## 🚀 Quick Start

```bash
# Compilar
cd ~/UNI/docencia/repos/ASR/asr_ws
colcon build --packages-select bt_examples
source install/setup.bash

# Ejecutar con launch file
ros2 launch bt_examples bumpandgo_bt_example.launch.py

# Ver argumentos disponibles
ros2 launch bt_examples bumpandgo_bt_example.launch.py --show-args
```

---

## 📚 Relación con el material teórico

Este ejemplo ilustra conceptos específicos del Capítulo 8:

- **Blackboard y puertos**: Comunicación entre nodos sin acoplamiento directo
- **Nodos de control**: Fallback, Sequence, Retry, Timeout
- **Nodos de acción**: StatefulActionNode con ciclo de vida onStart/onRunning/onHalted
- **Nodos de condición**: ConditionNode para verificar precondiciones
- **Tres patrones de comunicación**: datos globales, conexión directa entre nodos, parámetros configurables

## 🎯 Objetivo pedagógico

Mostrar un **Behavior Tree completo** que combina:

1. Navegación reactiva hacia un objetivo
2. Detección de obstáculos mediante sensores
3. Recuperación automática (backup + spin)
4. Uso de puertos para comunicación explícita entre nodos
5. Parámetros configurables con valores por defecto

---

## 📦 Contenido del paquete

### Estructura del código

```
bt_examples/
├── include/bt_examples/         # Headers de nodos personalizados
│   ├── is_obstacle_near_condition.hpp    # Condición: detecta obstáculos
│   ├── has_goal_condition.hpp            # Condición: verifica objetivo
│   ├── backup_action.hpp                 # Acción: retroceder
│   ├── spin_action.hpp                   # Acción: girar
│   ├── move_towards_goal_action.hpp      # Acción: navegar
│   └── abort_mission_action.hpp          # Acción: abortar misión
├── src/
│   └── bumpandgo_bt_example.cpp          # Programa principal
├── config/
│   └── bumpandgo_tree.xml                # Definición del árbol BT
└── launch/
    └── bumpandgo_bt_example.launch.py    # Launch file
```

### Ejemplo: Bump-and-Go

**Archivo**: `bumpandgo_bt_example.cpp`

**Descripción**: Implementación completa de bump-and-go usando Behavior Trees. El robot navega hacia un objetivo mientras monitoriza obstáculos. Si detecta un obstáculo cercano, ejecuta una maniobra de recuperación (retroceder + girar) antes de reintentar la navegación.

**Arquitectura del árbol**:
```
Fallback (SafetyPriority)
├─ Sequence (ObstacleRecovery)
│  ├─ IsObstacleNear (threshold=0.5) → {obstacle_dist}
│  └─ Sequence (RecoveryManeuver)
│     ├─ BackUp (← {obstacle_dist})
│     └─ Spin (← {obstacle_dist})
├─ Retry(3)
│  └─ Sequence (Navigation)
│     ├─ HasGoal
│     └─ Timeout(120s)
│        └─ MoveTowardsGoal
└─ AbortMission
```

**Patrones de comunicación mediante blackboard**:

1. **Datos globales de misión**: 
   - El programa principal establece `goal_x`, `goal_y`, `goal_theta` en el blackboard global
   - `MoveTowardsGoal` lee estos valores para navegar

2. **Conexión directa entre nodos**:
   - `IsObstacleNear` escribe `obstacle_distance` (OutputPort)
   - `BackUp` y `Spin` leen `obstacle_distance` (InputPort)
   - Comunicación explícita: distancia fluye desde sensor a actuadores

3. **Parámetros configurables**:
   - `threshold=0.5`: distancia para considerar obstáculo cercano
   - `base_distance=0.3`: distancia mínima de retroceso
   - `base_angle=1.57`: ángulo base de giro (90°)

**Ejecución**:
```bash
ros2 launch bt_examples bumpandgo_bt_example.launch.py
```

**Comportamiento observable**:
- El robot se mueve hacia el objetivo (5, 3) en el mapa
- Si detecta un obstáculo a menos de 0.5m:
  - Retrocede proporcionalmente a la distancia
  - Gira 180° si obstáculo < 0.3m, sino 90°
  - Reintenta la navegación
- Si falla 3 veces, aborta la misión

---

## 🔍 Detalles técnicos

### Nodos personalizados

#### IsObstacleNear (Condición)
- **Entrada**: topic `/scan` (LaserScan)
- **Puerto de salida**: `obstacle_distance` (double)
- **Puerto de entrada**: `threshold` (double, default=0.5)
- **Lógica**: Busca distancia mínima en el scan, escribe al blackboard, retorna SUCCESS si distancia < threshold

#### BackUp (Acción)
- **Puertos de entrada**: `obstacle_distance`, `base_distance`
- **Salida**: `/cmd_vel` (Twist)
- **Lógica**: Retrocede `max(base_distance, obstacle_dist + 0.2)` metros a -0.2 m/s

#### Spin (Acción)
- **Puertos de entrada**: `obstacle_distance`, `base_angle`
- **Salida**: `/cmd_vel` (Twist)
- **Lógica**: Gira 180° si obstáculo < 0.3m, sino usa `base_angle`

#### MoveTowardsGoal (Acción)
- **Puertos de entrada**: `goal_x`, `goal_y`, `goal_theta`
- **TF**: lee transform `map → base_link`
- **Salida**: `/cmd_vel` (Twist)
- **Lógica**: Control proporcional simple hacia el objetivo

### Ciclo de vida de nodos

**StatefulActionNode** (BackUp, Spin, MoveTowardsGoal):
- `onStart()`: Inicializa, lee parámetros del blackboard, retorna RUNNING
- `onRunning()`: Ejecuta lógica, retorna RUNNING o SUCCESS/FAILURE
- `onHalted()`: Limpieza si el nodo es interrumpido

**ConditionNode** (IsObstacleNear, HasGoal):
- `tick()`: Evalúa condición, retorna SUCCESS/FAILURE inmediatamente

---

## 🔧 Configuración

### Argumentos del launch file

```bash
ros2 launch bt_examples bumpandgo_bt_example.launch.py \
  use_sim_time:=true \
  goal_x:=5.0 \
  goal_y:=3.0
```

### Modificar el árbol BT

Edita `config/bumpandgo_tree.xml` para:
- Cambiar umbrales: `threshold="0.5"`
- Ajustar reintentos: `num_attempts="3"`
- Modificar timeout: `msec="120000"`

---

## 📖 Referencias

- **Capítulo 8**: Behavior Trees (teoría completa)
- **Práctica 5, sección 5.3**: Implementación con BehaviorTree.CPP
- **BehaviorTree.CPP**: https://www.behaviortree.dev/

---

## ⚠️ Requisitos

- ROS 2 Humble o superior
- BehaviorTree.CPP v3
- TF2
- Sensor Laser (`/scan` topic)
- Simulador o robot real con TF configurado

## 🐛 Troubleshooting

**Error: "No transform map → base_link"**
```bash
# Verificar que TF esté publicando
ros2 run tf2_ros tf2_echo map base_link
```

**El árbol no se carga**
```bash
# Verificar path al XML
ros2 run bt_examples bumpandgo_bt_example
# Error esperado: "Cannot find file: ..."
```

**No detecta obstáculos**
```bash
# Verificar que el topic scan exista
ros2 topic echo /scan --once
```

---

## 🎓 Notas pedagógicas

Este ejemplo demuestra:

1. **Separación de responsabilidades**: Cada nodo tiene una única función clara
2. **Reutilización**: Los nodos pueden usarse en diferentes árboles
3. **Composición jerárquica**: El árbol expresa prioridades de forma legible
4. **Comunicación explícita**: Los puertos hacen visible el flujo de datos
5. **Validación temprana**: BehaviorTree.CPP verifica tipos y conexiones al cargar el XML

Comparar con una FSM equivalente (práctica 4) permite apreciar las ventajas de los BT en tareas complejas con múltiples condiciones concurrentes.
