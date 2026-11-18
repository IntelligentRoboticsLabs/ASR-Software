# hri_example

Ejemplo de interacción humano-robot (HRI) con `simple_hri` a través de `HRIClient`
(paquete `hri_client`).

## Descripción

`simple_hri_app` es una aplicación modelada como **máquina de estados** que conversa con
una persona:

1. Espera a que `simple_hri` esté disponible.
2. Pregunta el nombre (TTS) y escucha la respuesta (STT). Si nadie habla en 8 s
   (`TIMEOUT`), reformula la pregunta (hasta 3 intentos).
3. Extrae el nombre del texto escuchado (Extract). Si no hay nombre (`NONE`), se despide.
4. Pide confirmación ("Entonces, ¿tu nombre es ...?"), escucha y la interpreta (YesNo).
5. Se despide y termina.

Cada estado usa una operación de `HRIClient` con el patrón **no bloqueante**: la primera
vez que entra en el estado llama a `start_*()`; en los ciclos siguientes consulta
`is_*_done()` y, al terminar, `get_*_state()` decide la transición.

## Estructura del paquete

```text
hri_example/
├── include/hri_example/simple_hri_app.hpp
├── src/
│   ├── simple_hri_app.cpp        # FSM de la conversación
│   └── simple_hri_app_main.cpp   # main
├── CMakeLists.txt
├── package.xml
└── thirdparty.repos              # simple_hri
```

`HRIClient` está en el paquete `hri_client` (ver su README).

## Dependencias

- `hri_client`
- `simple_hri` y `simple_hri_interfaces`

La instalación (rosdep, `vcs import < thirdparty.repos`, entorno virtual de Python con
las dependencias de `simple_hri`) está en el [README del repositorio](../../README.md).

## Compilación

```bash
cd ~/asr_ws
python3 -m colcon build --symlink-install --packages-up-to hri_example simple_hri
source install/setup.bash
```

## Uso

```bash
# Terminal 1: lanzar simple_hri (todo local y gratis, en español)
ros2 launch simple_hri simple_hri.launch.py

# Terminal 2: ejecutar la aplicación
ros2 run hri_example simple_hri_app
```

La primera vez `simple_hri` descarga los modelos (varios GB) en `models/` del directorio
desde el que se lanza. Otros *backends*: `backend:=ollama`, `backend:=hf`, `backend:=cloud`
(ver el README de `simple_hri`).

## Capacidades de simple_hri

| Capacidad | Interfaz | Tipo |
| --- | --- | --- |
| Escuchar (STT) | `/stt_action` | `simple_hri_interfaces/action/Listen` |
| Hablar (TTS) | `/tts_action` | `simple_hri_interfaces/action/Say` |
| Extraer información | `/extract_service` | `simple_hri_interfaces/srv/Extract` |
| Sí/No | `/yesno_service` | `simple_hri_interfaces/srv/YesNo` |
| Texto escuchado | `/listened_text` (topic) | `std_msgs/msg/String` |

## Notas

- Escuchar **después** de que el robot termine de hablar: si no, el micrófono graba la voz
  del propio robot.
- Extract y YesNo no graban audio: trabajan sobre el texto que devuelve STT.
- La aplicación termina (`rclcpp::shutdown()`) al llegar al estado `DONE`.
