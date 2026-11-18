# ASR-Software

Workspace de ROS 2 Jazzy de la asignatura Arquitecturas Software para Robots: nodos,
sensores y TF, máquinas de estados, navegación, behavior trees, interacción
humano-robot y misiones, con el robot Kobuki (real o simulado).

| Directorio | Contenido |
| --- | --- |
| `ch2_examples` | Programación de nodos |
| `ch3_examples` | Sensores (cámara, láser), TF y detección de objetos con YOLO (`yolo_ros`) |
| `ch5_examples` | Máquinas de estados y navegación (`navigation_client`, `nav2_example`) |
| `ch6_examples` | Behavior trees e interacción humano-robot (`hri_client`, `hri_example`, `bt_examples`) |
| `ch7_examples` | Misiones y tareas |
| `kobuki_robot` | Kobuki (robot real y simulador), sus drivers y los mundos de Gazebo |
| `thirdparty` | Paquetes de terceros que se descargan con `vcs` (no están en el repositorio) |

Los comandos usan `~/asr_ws` como directorio del workspace; puedes usar otro sustituyendo
esa ruta. Este repositorio es el directorio `src` del workspace.

## Instalación

### Requisitos

- Ubuntu 24.04 con [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
  (variante `desktop`).
- Unos 20 GB libres: los modelos de voz y de visión usan PyTorch.
- Opcional, para ejecutar YOLO y Whisper en la GPU: tarjeta NVIDIA con su driver
  (`nvidia-smi` lo muestra). Sin ella funcionan en la CPU, más despacio.

### 1. Herramientas

```bash
sudo apt update
sudo apt install -y git python3-venv python3-pip python3-rosdep python3-vcstool \
  python3-colcon-common-extensions
[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init  # solo la primera vez
rosdep update
```

### 2. Descargar el código

El repositorio va en `~/asr_ws/src`. Los paquetes de terceros (`simple_hri`) se descargan
con `vcs` en `src/thirdparty`, que Git ignora:

```bash
mkdir -p ~/asr_ws
git clone https://github.com/IntelligentRoboticsLabs/ASR-Software.git ~/asr_ws/src
cd ~/asr_ws/src
vcs import < thirdparty.repos
```

### 3. Dependencias del sistema

```bash
cd ~/asr_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="gazebo gazebo_ros gazebo_plugins astra_camera aws_robomaker_bookstore_world aws_robomaker_racetrack_world aws_robomaker_small_warehouse_world"
sudo apt install -y libusb-1.0-0-dev libftdi1-dev libuvc-dev libportaudio2 alsa-utils
```

- `rosdep` instala con `apt` las dependencias que declaran los `package.xml`: Nav2,
  Gazebo, BehaviorTree.CPP (`ros-jazzy-behaviortree-cpp`), PCL, OpenCV...
- El `sudo apt install` instala lo que ningún paquete declara: las librerías USB del
  Kobuki, PortAudio (para grabar audio) y `alsa-utils` (`aplay`, para reproducirlo).
- `--skip-keys` indica a `rosdep` qué dependencias no debe intentar instalar:

  | Clave | Quién la declara | Por qué se omite |
  | --- | --- | --- |
  | `gazebo`, `gazebo_ros`, `gazebo_plugins` | `aws-robomaker-small-house-world` | Son de Gazebo clásico, que no existe en Jazzy; el mundo usa el Gazebo actual (`ros_gz_sim`), que sí se instala |
  | `astra_camera`, `aws_robomaker_bookstore_world`, `aws_robomaker_racetrack_world`, `aws_robomaker_small_warehouse_world` | `kobuki` | Cámara y mundos opcionales que no usa la asignatura (el almacén ni siquiera existe como paquete de Jazzy) |

### 4. Entorno virtual de Python

Las dependencias Python de la interacción humano-robot (PyTorch, Whisper,
Transformers...) y de YOLO (Ultralytics) se instalan en un entorno virtual, para no
interferir con las del sistema:

```bash
cd ~/asr_ws
python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate
python3 -m pip install -r src/requirements.txt
```

- `--system-site-packages` permite ver los paquetes Python de ROS 2 y los que instala
  `apt` (por ejemplo, los que acaba de instalar `rosdep`).
- `COLCON_IGNORE` evita que `colcon` busque paquetes dentro del entorno.
- `src/requirements.txt` reúne las dependencias de `simple_hri` y de `yolo_ros`, y
  `colcon`: se instala dentro del entorno para que los nodos compilados usen su Python y
  encuentren sus dependencias.

### 5. Compilar

```bash
cd ~/asr_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 -m colcon build --symlink-install
```

Si el equipo se queda sin memoria, compila con `--parallel-workers 1`. Para recompilar
solo un paquete y los que necesita: `python3 -m colcon build --symlink-install
--packages-up-to bt_examples`.

### 6. Usar el workspace

En cada terminal nueva, carga ROS 2, el entorno virtual y el workspace, en este orden:

```bash
source /opt/ros/jazzy/setup.bash
source ~/asr_ws/.venv/bin/activate
source ~/asr_ws/install/setup.bash
```

Para no repetirlo, puedes añadir esas tres líneas al final de `~/.bashrc`. Comprueba que
todo funciona con el simulador del Kobuki (ciérralo con Ctrl+C):

```bash
ros2 launch kobuki simulation.launch.py
```

## Kobuki real

Ver con extremo cuidado [kobuki_robot/kobuki/README.md](kobuki_robot/kobuki/README.md).

Para conectar el Kobuki y su láser por USB hacen falta reglas udev que den permisos a los
dispositivos (una sola vez por equipo):

```bash
cd ~/asr_ws/src/kobuki_robot
sudo cp kobuki_ros/60-kobuki.rules rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Varios equipos en la misma red

ROS 2 descubre automáticamente los nodos de otros equipos de la red. En un laboratorio eso
hace que los robots y simuladores de distintas personas se mezclen. Para evitarlo, cada
persona puede limitar ROS 2 a su propio equipo, o usar un dominio distinto (un número
entre 1 y 101). Añade una de estas líneas a `~/.bashrc`:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST  # solo este equipo
export ROS_DOMAIN_ID=42                         # o un número distinto por persona
```

Para comunicarse con el robot real desde otro equipo, ambos deben tener el mismo
`ROS_DOMAIN_ID` y no usar `LOCALHOST`.

## Interacción humano-robot

Los ejemplos de interacción (`ch6_examples`) usan
[`simple_hri`](https://github.com/rodperex/simple_hri): escuchar y hablar (acciones),
extraer información y sí/no con modelos de lenguaje (servicios). Se lanza con:

```bash
ros2 launch simple_hri simple_hri.launch.py   # todo local y gratis, en español
```

Los modelos locales se descargan la primera vez (varios GB, en `models/` del directorio
desde el que se lanza), así que esa vez hace falta Internet. Otros *backends* (Ollama,
Hugging Face, nube) en el README de `simple_hri`.

Hacen falta micrófono y altavoces. Si no funciona, comprueba los dispositivos de audio y
las dependencias desde una terminal con el workspace cargado:

```bash
python3 -c "import sounddevice as sd; print(sd.query_devices())"
python3 -c "import torch, whisper, transformers; print('OK, GPU:', torch.cuda.is_available())"
```

## Actualizar el workspace

```bash
cd ~/asr_ws/src
git pull
vcs import < thirdparty.repos   # paquetes de terceros nuevos
vcs pull thirdparty
cd ~/asr_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 -m pip install -r src/requirements.txt
python3 -m colcon build --symlink-install
```

Si tras actualizar la compilación falla por restos antiguos, borra `build install log` y
vuelve a compilar.
