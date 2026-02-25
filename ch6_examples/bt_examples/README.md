# BT Examples - Ejemplos de Behavior Trees para ASR

Este paquete contiene ejemplos progresivos de implementación de Behavior Trees en ROS 2, diseñados para complementar el **Capítulo 8** del libro ASR y la **Práctica 5**.

> **📌 Nota importante**: El código de estos ejemplos está alineado con los fragmentos de código que aparecen en el libro (Capítulo 8 y Práctica 5). Los patrones, comentarios y estructura siguen exactamente los presentados en el material teórico para facilitar el seguimiento en clase:
> - **Bump-and-Go**: Práctica 5, secciones 5.3.6 y 5.3.7 (Ejemplo completo con puertos y blackboard)
> - **Drink Order**: Ejemplo básico de HRI con TTS y ASR usando simple_hri

## � Requisitos previos

Este paquete requiere:
- **BehaviorTree.CPP** (rama master)
- **simple_hri** - Servicios de TTS y ASR para HRI

Las dependencias de terceros se gestionan mediante el archivo `thirdparty.repos`.

### Instalación de dependencias

```bash
# Navegar al workspace
cd ~/UNI/docencia/repos/ASR/asr_ws

# Instalar dependencias de terceros usando vcstool
vcs import src/thirdparty < src/bt_examples/thirdparty.repos

# Compilar las dependencias
colcon build --packages-up-to behaviortree_cpp simple_hri

# Compilar el paquete bt_examples
colcon build --packages-select bt_examples

# Cargar el entorno
source install/setup.bash
```

> **💡 Nota**: Si `vcstool` no está instalado, instálalo con:
> ```bash
> sudo apt install python3-vcstool
> ```

## 🚀 Quick Start

### Bump-and-Go (Navegación reactiva con obstáculos)

```bash
# Ejecutar con launch file
ros2 launch bt_examples bumpandgo_bt_example.launch.py

# Ver argumentos disponibles
ros2 launch bt_examples bumpandgo_bt_example.launch.py --show-args
```

### Drink Order (Interacción básica HRI)

```bash
# Asegúrate de que los servicios de simple_hri estén ejecutándose
ros2 launch simple_hri tts_listen.launch.py

# En otra terminal, ejecuta el ejemplo
ros2 launch bt_examples drink_order_bt_example.launch.py
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
│   ├── abort_mission_action.hpp          # Acción: abortar misión
│   ├── say_text_action.hpp               # Acción: TTS (HRI)
│   └── listen_text_action.hpp            # Acción: ASR (HRI)
├── src/
│   ├── bumpandgo_bt_example.cpp          # Programa principal bump-and-go
│   └── drink_order_bt_example.cpp        # Programa principal HRI
├── config/
│   ├── bumpandgo_tree.xml                # Definición del árbol bump-and-go
│   └── drink_order_tree.xml              # Definición del árbol HRI
└── launch/
    ├── bumpandgo_bt_example.launch.py    # Launch file bump-and-go
    └── drink_order_bt_example.launch.py  # Launch file HRI
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

### Ejemplo: Drink Order (HRI Básico)

**Archivo**: `drink_order_bt_example.cpp`

**Descripción**: Ejemplo simple de interacción humano-robot usando Behavior Trees con capacidades de TTS (Text-To-Speech) y ASR (Automatic Speech Recognition). El robot pregunta a la persona qué quiere beber, escucha la respuesta, y repite lo que ha entendido.

**Arquitectura del árbol**:
```
Sequence (DrinkOrderSequence)
├─ SayText (text="¿Qué quieres beber?")
├─ ListenText → {full_text}
├─ ExtractInfo (interest="bebida", full_text={full_text}) → {drink_order}
└─ SayText (text="Has dicho que quieres {drink_order}. Perfecto.")
```

**Comunicación mediante blackboard**:
- Los textos y el interés están definidos directamente en el XML para facilitar su modificación
- Primer `SayText` dice el texto especificado en el XML
- `ListenText` escribe el texto completo reconocido en `full_text` (OutputPort)
- `ExtractInfo` lee el interés ("bebida") y `full_text`, extrae la información relevante escribiéndola en `drink_order` (InputPorts → OutputPort)
- Segundo `SayText` usa una plantilla con `{drink_order}` que se sustituye automáticamente con el valor del blackboard

**Ejecución**:
```bash
# Terminal 1: Iniciar servicios de HRI (incluye extract)
ros2 launch simple_hri simple_hri.launch.py

# Terminal 2: Configurar API key de OpenAI
export OPENAI_API_KEY="tu-clave-api"

# Terminal 3: Ejecutar el ejemplo
ros2 launch bt_examples drink_order_bt_example.launch.py
```

**Comportamiento observable**:
1. El robot dice: "¿Qué quieres beber?"
2. El robot activa el micrófono y espera que hables
3. Dices, por ejemplo: "Quiero un café con leche, por favor"
4. El sistema extrae la información relevante: "café con leche"
5. El robot responde: "Has dicho que quieres café con leche. Perfecto."

**Nodos HRI personalizados**:

#### SayText (Acción)
- **Puerto de entrada**: `text` (string) - Texto a decir (puede contener plantillas con {variable})
- **Service client**: `/tts_service` (simple_hri_interfaces/srv/Speech)
- **Lógica**: 
  - Lee el texto del puerto de entrada
  - Envía el texto al servicio TTS
  - Espera a que termine de hablar (síncrono)
  - Retorna SUCCESS cuando termina

#### ListenText (Acción)
- **Puerto de salida**: `recognized_text` (string) - Texto reconocido por ASR
- **Service client**: `/stt_service` (std_srvs/srv/SetBool)
- **Lógica**:
  - Llama al servicio STT con request.data = true
  - Espera a que el usuario hable y el sistema transcriba (síncrono, puede tardar)
  - Recibe el texto transcrito en response.message
  - Escribe el texto reconocido en el blackboard
  - Retorna SUCCESS con el texto reconocido

#### ExtractInfo (Acción)
- **Puerto de entrada**: 
  - `interest` (string) - Interés/categoría a extraer (ej: "bebida", "lugar", "persona")
  - `full_text` (string) - Texto completo del ASR
- **Puerto de salida**: `extracted_info` (string) - Información útil extraída
- **Service client**: `/extract` (simple_hri_interfaces/srv/Extract)
- **Lógica**:
  - Recibe el interés (categoría) y el texto completo del reconocimiento de voz
  - Llama al servicio Extract con ambos parámetros
  - Usa IA (OpenAI GPT) para extraer solo la información relevante según el interés
  - Escribe la información extraída en el blackboard
  - Si no se extrae nada, usa el texto original
  - Retorna SUCCESS con la información extraída

**Por qué usar ExtractInfo:**
En una conversación real, el usuario no dice solo "agua", sino frases como:
- "Quiero un café con leche, por favor"
- "Me gustaría tomar un té verde"
- "Pues no sé, quizás agua con gas"

ExtractInfo procesa estas frases complejas y extrae únicamente la información relevante según el **interest** especificado (en este caso "bebida"), haciendo la interacción más natural y robusta.

**Requisitos adicionales**:
- Paquete `simple_hri` instalado y ejecutándose
- Micrófono y altavoces funcionales
- Modelos de ASR/TTS configurados en simple_hri
- **Variable de entorno `OPENAI_API_KEY` configurada** (para ExtractInfo)
  ```bash
  export OPENAI_API_KEY="tu-clave-api"
  ```

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
- Sensor Laser (`/scan` topic) - para bump-and-go
- simple_hri - para el ejemplo de HRI

### Troubleshooting

**Error: "Extract service not available"** (drink order)
```bash
# Verificar que el servicio extract esté ejecutándose
ros2 service list | grep extract

# Verificar variable de entorno
echo $OPENAI_API_KEY

# Iniciar servicios de HRI con extract si no están activos
ros2 launch simple_hri simple_hri.launch.py
```

**Error: "TF transform not available"** (bump-and-go)
```bash
# Verificar que el transform map → base_link está disponible  
ros2 run tf2_ros tf2_echo map base_link
```

**El árbol no se carga**
```bash
# Verificar path al XML
ros2 run bt_examples bumpandgo_bt_example
# Error esperado: "Cannot find file: ..."
```

**No detecta obstáculos** (bump-and-go)
```bash
# Verificar que el topic scan exista
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

**Error: "TTS/Listen action server not available"** (drink order)
```bash
# Verificar que simple_hri esté ejecutándose
ros2 action list | grep -E "(say|listen)"

# Iniciar servicios de HRI si no están activos
ros2 launch simple_hri tts_listen.launch.py
```

**No se reconoce la voz** (drink order)
```bash
# Verificar micrófono
arecord -l

# Probar ASR directamente
ros2 action send_goal /listen simple_hri_interfaces/action/Listen "{}"
```
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
