# FSM Examples - Ejemplos de Máquinas de Estados Finitos para ASR

Este paquete contiene ejemplos progresivos de implementación de FSM (Finite State Machines) en ROS 2, diseñados para complementar el **Capítulo 7** del libro ASR y la **Práctica 4**.

> **📌 Nota importante**: El código de estos ejemplos está alineado con los fragmentos de código que aparecen en el libro (Capítulo 7 y Práctica 4). Los patrones, comentarios y estructura siguen exactamente los presentados en el material teórico para facilitar el seguimiento en clase:
> - **Ejemplo 1** (switch-case): Práctica 4, sección 4.2.1 (Patrón básico: switch-case)
> - **Ejemplo 2** (OOP): Práctica 4, sección 4.2.2 (Patrón orientado a objetos)
> - **Ejemplo 3** (ortogonal): Capítulo 7, sección 7.5 (Descomposición ortogonal)

## � Quick Start

```bash
# Compilar
cd ~/UNI/docencia/repos/ASR/asr_ws
colcon build --packages-select fsm_examples
source install/setup.bash

# Ejecutar con launch files (incluye remappings configurables)
ros2 launch fsm_examples fsm_oop_example.launch.py

# Ver argumentos disponibles
ros2 launch fsm_examples fsm_oop_example.launch.py --show-args
```

**📖 Para más información sobre launch files y remappings**: Ver [QUICK_REFERENCE.md](QUICK_REFERENCE.md)

---

## �📚 Relación con el material teórico

Cada ejemplo ilustra conceptos específicos del Capítulo 7:

- **Ejemplo 1**: Sección "Patrón básico: switch-case"
- **Ejemplo 2**: Sección "Patrón orientado a objetos (recomendado)"
- **Ejemplo 3**: Sección "El problema de la concurrencia" y "Statecharts"

## 🎯 Objetivo pedagógico

Mostrar la **evolución arquitectónica** desde una FSM simple hasta una implementación robusta con descomposición ortogonal, permitiendo al alumno comprender:

1. La diferencia entre código funcional y código mantenible
2. La importancia del ciclo de vida (on_entry/on_do/on_exit)
3. Cómo evitar la explosión combinatoria de estados

---

## 📦 Contenido del paquete

### Estructura del código

El paquete sigue las mejores prácticas de C++ y ROS 2, separando cabeceras de implementaciones:

```
fsm_examples/
├── include/fsm_examples/     # Cabeceras (.hpp)
│   ├── fsm_basic_example.hpp
│   ├── fsm_oop_example.hpp
│   └── fsm_orthogonal_example.hpp
├── src/                      # Implementaciones (.cpp)
│   ├── fsm_basic_example.cpp
│   ├── fsm_oop_example.cpp
│   └── fsm_orthogonal_example.cpp
├── launch/                   # Launch files (opcional)
├── CMakeLists.txt
├── package.xml
└── README.md
```

Esta organización permite:
- **Reutilización**: Las clases pueden incluirse en otros paquetes
- **Claridad**: Interfaz (`.hpp`) separada de implementación (`.cpp`)
- **Compilación modular**: Cambios en implementación no requieren recompilar dependientes

### Ejemplo 1: `fsm_basic_example`
**Patrón**: Switch-case simple

**Comportamiento**: Robot que avanza y se detiene ante obstáculos

**Estados**:
- `IDLE`: En reposo esperando orden
- `MOVING`: Avanzando hacia adelante
- `OBSTACLE_DETECTED`: Transitorio al detectar obstáculo
- `STOPPED`: Detenido por seguridad

**Ventajas**:
- ✅ Fácil de entender
- ✅ Implementación directa

**Limitaciones**:
- ❌ Acciones se ejecutan repetidamente cada ciclo
- ❌ No hay on_entry/on_exit diferenciados
- ❌ Difícil de escalar

**Ejecutar**:
```bash
ros2 run fsm_examples fsm_basic_example
```

---

### Ejemplo 2: `fsm_oop_example`
**Patrón**: Orientado a objetos con lifecycle completo

**Comportamiento**: El mismo robot, pero con arquitectura robusta

**Estados**: Mismos que el Ejemplo 1, pero implementados como clases

**Ventajas**:
- ✅ on_entry se ejecuta UNA VEZ al entrar
- ✅ on_exit se ejecuta UNA VEZ al salir (limpieza segura)
- ✅ Cada estado encapsula su lógica
- ✅ Fácil de mantener y extender

**Principio arquitectónico clave**:
```cpp
// Callbacks de sensores: SOLO actualizan memoria (NO toman decisiones)
void laser_callback(...) {
  min_distance_ = ...;  // Solo guardar dato
}

// Timer de control: AQUÍ se toman decisiones
void control_cycle() {
  fsm_->step();  // La FSM decide usando la memoria actualizada
}
```

**Ejecutar**:
```bash
ros2 run fsm_examples fsm_oop_example
```

---

### Ejemplo 3: `fsm_orthogonal_example`
**Patrón**: Descomposición ortogonal (Statecharts de Harel)

**Comportamiento**: Robot que navega Y monitoriza batería simultáneamente

**Arquitectura**:
- **Región 1 - NavigationFSM**: IDLE, MOVING, STOPPED
- **Región 2 - BatteryFSM**: OK, LOW, CRITICAL

**Problema que resuelve**:
Sin ortogonalidad (FSM plana):
```
3 estados navegación × 3 estados batería = 9 estados combinados
Añadir luces (ON/OFF) → 18 estados
Añadir modo (MANUAL/AUTO) → 36 estados 🤯
```

Con ortogonalidad (Statecharts):
```
3 + 3 + 2 + 2 = 10 estados independientes
Complejidad lineal en lugar de exponencial ✅
```

**Implementación clave**:
```cpp
void control_cycle() {
  nav_fsm_->step();      // Región 1
  battery_fsm_->step();  // Región 2
}
// Ambas FSM activas simultáneamente en cada ciclo
```

**Ejecutar con simulador de batería** (recomendado):
```bash
# Drenaje por defecto (60 segundos)
ros2 launch fsm_examples fsm_orthogonal_example.launch.py

# Drenaje rápido para demos (10 segundos)
ros2 launch fsm_examples fsm_orthogonal_example.launch.py drain_time:=10.0

# Drenaje muy lento (300 segundos = 5 minutos)
ros2 launch fsm_examples fsm_orthogonal_example.launch.py drain_time:=300.0
```

**Ejecutar manualmente** (sin simulador):
```bash
# Terminal 1: Simulador de batería
ros2 run fsm_examples battery_simulator --ros-args -p drain_time:=30.0

# Terminal 2: FSM ortogonal
ros2 run fsm_examples fsm_orthogonal_example
```

---

### Utilidad: `battery_simulator`
**Propósito**: Simular drenaje de batería para ejemplos FSM

**Parámetros**:
- `drain_time` (double): Segundos para drenar de 100% a 0% (default: 60.0)

**Topic publicado**:
- `/battery_state` (sensor_msgs/BatteryState): Estado de batería con drenaje gradual

**Características**:
- ✅ Publica a 10 Hz
- ✅ Logs cada 10% de cambio
- ⚠️ Advertencia al 20% (LOW)
- 🔴 Error al 10% (CRITICAL)
- 💀 Fatal al 0% (AGOTADA)

**Ejecutar standalone**:
```bash
ros2 run fsm_examples battery_simulator --ros-args -p drain_time:=20.0
```

**Observar**: El robot está siempre en un estado de navegación Y un estado de batería simultáneamente. La batería puede llegar a CRITICAL y forzar la detención independientemente del estado de navegación.

---

## 🛠️ Compilación

### Prerrequisitos
```bash
# Asegúrate de tener ROS 2 instalado (Humble, Iron o Rolling)
# y el workspace configurado
```

### Compilar el paquete
```bash
cd ~/UNI/docencia/repos/ASR/asr_ws
colcon build --packages-select fsm_examples
source install/setup.bash
```

### Verificar compilación
```bash
ros2 pkg list | grep fsm_examples
# Debería mostrar: fsm_examples
```

---

## 🚀 Uso en clase

### Opción 1: Ejecutar directamente (Simple)
```bash
ros2 run fsm_examples fsm_basic_example
ros2 run fsm_examples fsm_oop_example
ros2 run fsm_examples fsm_orthogonal_example
```

### Opción 2: Usar launch files con remappings (Recomendado)

Los launch files incluyen **remappings configurables** para adaptar los ejemplos a diferentes robots:

#### Lanzar ejemplo individual con remappings personalizados:
```bash
# Ejemplo básico con topics customizados
ros2 launch fsm_examples fsm_basic_example.launch.py \
  cmd_vel_topic:=/robot/cmd_vel \
  scan_topic:=/robot/scan

# Ejemplo OOP con TurtleBot3 en Gazebo
ros2 launch fsm_examples fsm_oop_example.launch.py \
  use_sim_time:=true

# Ejemplo ortogonal con todos los remappings
ros2 launch fsm_examples fsm_orthogonal_example.launch.py \
  cmd_vel_topic:=/mobile_base/commands/velocity \
  scan_topic:=/base_scan \
  battery_topic:=/sensors/battery \
  use_sim_time:=true
```

#### Lanzar con el launcher unificado:
```bash
# Por defecto lanza el ejemplo OOP
ros2 launch fsm_examples demo.launch.py

# Seleccionar ejemplo específico
ros2 launch fsm_examples demo.launch.py example:=basic
ros2 launch fsm_examples demo.launch.py example:=orthogonal

# Con remappings para robot específico
ros2 launch fsm_examples demo.launch.py \
  example:=oop \
  cmd_vel_topic:=/tb3/cmd_vel \
  scan_topic:=/tb3/scan
```

### Uso de remappings: ¿Por qué son importantes?

Los remappings permiten **adaptar el código a diferentes robots sin modificarlo**. Cada robot puede tener nombres de topics diferentes:

| Robot | cmd_vel | scan | Uso |
|-------|---------|------|-----|
| **TurtleBot3** | `/cmd_vel` | `/scan` | Simulación Gazebo |
| **Kobuki** | `/mobile_base/commands/velocity` | `/base_scan` | Robot real |
| **Custom** | `/robot/cmd_vel` | `/robot/lidar` | Tu robot |

**Ventaja educativa**: Los alumnos ven cómo ROS 2 abstrae la comunicación hardware/software.

**Ejemplo práctico**:
```bash
# Mismo código FSM, diferentes robots
ros2 launch fsm_examples fsm_oop_example.launch.py  # TurtleBot3 por defecto

ros2 launch fsm_examples fsm_oop_example.launch.py \
  cmd_vel_topic:=/mobile_base/commands/velocity \
  scan_topic:=/base_scan  # Kobuki

ros2 launch fsm_examples fsm_oop_example.launch.py \
  cmd_vel_topic:=/robot1/cmd_vel \
  scan_topic:=/robot1/scan  # Multi-robot
```

### Orden recomendado de presentación

1. **Ejemplo 1**: Mostrar implementación básica y sus limitaciones
   - Ejecutar y observar cómo las acciones se repiten
   - Discutir: "¿Qué pasa si queremos inicializar un timer al entrar?"

2. **Ejemplo 2**: Mostrar arquitectura robusta
   - Comparar logs: on_entry se ejecuta UNA VEZ
   - Destacar la limpieza en on_exit (crítico para seguridad)

3. **Ejemplo 3**: Mostrar descomposición ortogonal
   - Calcular con los alumnos: 3×3 vs 3+3
   - Añadir hipotéticamente más dimensiones (luces, modos...)
   - Observar cómo batería CRITICAL detiene el robot

### Demostraciones interactivas

**Simulación de sensores**:
```bash
# Terminal 1: Ejecutar ejemplo con launcher (incluye remappings)
ros2 launch fsm_examples fsm_oop_example.launch.py

# Terminal 2: Publicar datos de láser simulados
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{ranges: [0.3]}" --once
# Debería provocar transición a STOPPED

ros2 topic pub /scan sensor_msgs/msg/LaserScan "{ranges: [2.0]}" --once
# Después de 2 segundos, vuelve a MOVING
```

**Visualización de estados** (Ejemplo 3):
```bash
# Los logs muestran estado combinado cada 3 segundos:
# Estado combinado: [NAV:MOVING] + [BAT:LOW] | Batería: 25.0% | Obstáculo: 1.50m
```

---

## 🎓 Preguntas para reflexión en clase

1. **Ejemplo 1 vs 2**: ¿Por qué es importante que on_entry se ejecute una sola vez?
   - Respuesta: Inicialización de temporizadores, apertura de recursos, configuración de parámetros

2. **Ejemplo 2**: ¿Qué pasaría si no ejecutamos on_exit antes de cambiar de estado?
   - Respuesta: Motores podrían quedar encendidos, recursos sin liberar, estado inconsistente

3. **Ejemplo 3**: Si añadimos un sistema de luces (ON/OFF), ¿cuántos estados tenemos?
   - Sin ortogonalidad: 9 × 2 = 18
   - Con ortogonalidad: 6 + 2 = 8

4. **Arquitectura**: ¿Por qué los callbacks de sensores solo actualizan variables?
   - Respuesta: Separación de responsabilidades, reactividad del nodo, decisiones centralizadas

---

## 📖 Relación con la Práctica 4

Estos ejemplos son **preparación** para la Práctica 4 (Navegación con Nav2 y patrullaje mediante FSM), donde:

- La FSM de patrullaje será similar al **Ejemplo 2** (patrón OOP)
- Nav2 actuará como una **capacidad** (no como un estado)
- El patrullaje es la **tarea** (orquestada por la FSM)

**Diferencia clave con la práctica**:
- Aquí: Estados ejecutan acciones simples (publicar velocidad)
- Práctica 4: Estados invocan capacidades externas (enviar goal a Nav2)

---

## 🐛 Troubleshooting

### Error: `No executable found`
```bash
# Recompilar y source
cd ~/UNI/docencia/repos/ASR/asr_ws
colcon build --packages-select fsm_examples
source install/setup.bash
```

### Advertencia: `[WARN] No laser data received`
Es normal si no hay un simulador ejecutándose. Los ejemplos funcionan sin sensores reales:
- Ejemplo 1 y 2: Simulan detección de obstáculo con valor fijo
- Ejemplo 3: Simula batería con descarga gradual

### Para usar con simulador real:
```bash
# Terminal 1: Simulador (ejemplo con Gazebo)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Ejemplo
ros2 run fsm_examples fsm_oop_example
```

---

## 📝 Modificaciones sugeridas para ejercicios

### Ejercicio 1: Añadir nuevo estado
Modificar `fsm_oop_example.cpp` para añadir estado `REVERSING` que retrocede ante obstáculo antes de detenerse.

### Ejercicio 2: Implementar jerarquía
Crear un superestado `OPERATIONAL` que contenga `MOVING` y `STOPPED`, con transición global a `IDLE` ante botón de parada.

### Ejercicio 3: Añadir tercera región ortogonal
En `fsm_orthogonal_example.cpp`, añadir `LightsFSM` con estados `ON`/`OFF` que parpadeen cuando la batería esté LOW.

---

## 📚 Referencias y Documentación Adicional

### Documentación del paquete
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Referencia rápida de launch files y remappings 🚀
- **[GUIA_PROFESOR.md](GUIA_PROFESOR.md)** - Guía pedagógica con timing de clase
- **[EJEMPLOS_USO.md](EJEMPLOS_USO.md)** - Casos de uso detallados con Gazebo y múltiples robots
- **[ALINEACION_LIBRO.md](ALINEACION_LIBRO.md)** - Correspondencia exacta con capítulos del libro
- **[ARQUITECTURA_CODIGO.md](ARQUITECTURA_CODIGO.md)** - Explicación de la separación headers/implementation

### Material teórico
- **Libro ASR**: Capítulo 7 - Generación de comportamientos I: máquinas de estados finitos
- **Práctica 4**: Navegación con Nav2 y patrullaje mediante FSM
- **Harel, D. (1987)**: "Statecharts: A visual formalism for complex systems"

---

## 👨‍🏫 Para el profesor

### Material complementario

Estos ejemplos se pueden complementar con:

1. **Diagramas de estados**: Dibuje en pizarra la FSM mientras ejecuta el código
2. **Comparación temporal**: Muestre logs lado a lado de Ejemplo 1 vs 2
3. **Cálculo de complejidad**: Haga que los alumnos calculen estados combinados

### Tiempos recomendados

- Ejemplo 1: 10-15 minutos (introducción)
- Ejemplo 2: 20-25 minutos (concepto clave)
- Ejemplo 3: 15-20 minutos (avanzado)

**Total**: ~50 minutos de sesión práctica

---

## 📧 Contacto

Para preguntas sobre estos ejemplos, contactar al equipo docente de ASR.
