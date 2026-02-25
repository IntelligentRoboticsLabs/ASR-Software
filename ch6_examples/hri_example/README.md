# hri_example

Ejemplos de uso de los servicios de Human-Robot Interaction (HRI) proporcionados por el paquete simple_hri.

## Descripción

Este paquete contiene código de ejemplo que demuestra cómo usar los servicios de simple_hri para realizar interacciones de voz con usuarios, incluyendo Speech-to-Text (STT), Text-to-Speech (TTS), extracción de información y detección de respuestas Yes/No.

### Contenido

**Biblioteca HRIClient** (`libhri_client.so`):
- Clase reutilizable que encapsula los servicios de simple_hri
- Proporciona métodos para:
  - Speech-to-Text: Convertir audio a texto
  - Text-to-Speech: Reproducir texto como audio
  - Extract: Extraer información específica del audio usando LLM
  - Yes/No: Detectar respuestas de confirmación
  - Suscripción al topic `/listened_text`

**Ejecutable simple_hri_app**:
- Aplicación de ejemplo que utiliza HRIClient
- Implementa una conversación simple mediante una máquina de estados:
  1. Saludo al usuario
  2. Pregunta por el nombre
  3. Extracción del nombre del audio
  4. Confirmación del nombre con el usuario
  5. Despedida

## Estructura del paquete

```
hri_example/
├── include/hri_example/
│   ├── hri_client.hpp            # Header de HRIClient
│   └── simple_hri_app.hpp        # Header de SimpleHRIApp
├── src/
│   ├── hri_client.cpp            # Implementación de HRIClient
│   ├── simple_hri_app.cpp        # Implementación de SimpleHRIApp
│   └── simple_hri_app_main.cpp   # Punto de entrada (main)
├── CMakeLists.txt
├── package.xml
├── thirdparty.repos              # Dependencias de terceros
└── README.md
```

## Dependencias

Este paquete depende de:
- **simple_hri**: Paquete que proporciona los servicios de HRI
- **simple_hri_interfaces**: Definiciones de mensajes y servicios
- **audio_common**: Paquete para captura y reproducción de audio

### Instalación de dependencias

```bash
cd ~/asr_ws
vcs import src < src/hri_example/thirdparty.repos
```

### Configuración de API keys (para versión cloud)

Para usar las versiones en la nube de los servicios, necesitas configurar:

**OpenAI API** (para STT con Whisper y Extract):
```bash
export OPENAI_API_KEY="your_openai_api_key"
```

**Google Cloud TTS** (para TTS):
```bash
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/your/google_credentials.json"
```

> **Nota:** También puedes usar las versiones locales de los servicios que no requieren conexión a Internet. Ver sección de uso.

## Compilación

```bash
cd ~/asr_ws
colcon build --packages-select hri_example simple_hri simple_hri_interfaces audio_common --symlink-install
source install/setup.bash
```

## Uso

### Ejecutar el ejemplo (versión cloud)

```bash
# Terminal 1: Lanzar los servicios de simple_hri
ros2 launch simple_hri simple_hri.launch.py

# Terminal 2: Ejecutar la aplicación de ejemplo
ros2 run hri_example simple_hri_app
```

### Ejecutar el ejemplo (versión local - sin Internet)

```bash
# Terminal 1: Lanzar los servicios locales de simple_hri
ros2 launch simple_hri local_simple_hri.launch.py

# Terminal 2: Ejecutar la aplicación de ejemplo
ros2 run hri_example simple_hri_app
```

### Flujo de la conversación

1. La aplicación espera a que los servicios estén disponibles
2. Saluda al usuario: "Hello! I am a robot. What is your name?"
3. Escucha y extrae el nombre del usuario
4. Confirma: "Nice to meet you, [nombre]. Is that correct?"
5. Si el usuario dice "yes", se despide: "Great! It was nice talking to you, [nombre]."
6. Si el usuario dice "no", vuelve a preguntar el nombre

## Usar HRIClient en tu propio paquete

### 1. Añadir dependencia en `package.xml`:

```xml
<depend>hri_example</depend>
<depend>simple_hri_interfaces</depend>
```

### 2. Configurar `CMakeLists.txt`:

```cmake
find_package(hri_example REQUIRED)
find_package(simple_hri_interfaces REQUIRED)

add_executable(mi_aplicacion src/mi_aplicacion.cpp)

target_link_libraries(mi_aplicacion
  ${hri_example_LIBRARIES}
)

ament_target_dependencies(mi_aplicacion
  rclcpp
  hri_example
  simple_hri_interfaces
  std_msgs
  std_srvs
)
```

### 3. Incluir el header en tu código:

```cpp
#include "hri_example/hri_client.hpp"

class MiAplicacion : public rclcpp::Node
{
public:
  MiAplicacion() : Node("mi_aplicacion")
  {
    hri_client_ = std::make_shared<HRIClient>();
    
    // Esperar servicios
    if (hri_client_->wait_for_services()) {
      // Usar TTS
      hri_client_->call_tts_service("Hello World");
      
      // Usar STT
      std::string text;
      if (hri_client_->call_stt_service(text)) {
        RCLCPP_INFO(get_logger(), "Usuario dijo: %s", text.c_str());
      }
      
      // Extraer información
      std::string name;
      if (hri_client_->call_extract_service("person's name", name)) {
        RCLCPP_INFO(get_logger(), "Nombre: %s", name.c_str());
      }
      
      // Detectar Yes/No
      bool said_yes;
      if (hri_client_->call_yesno_service(said_yes)) {
        RCLCPP_INFO(get_logger(), "Usuario dijo: %s", said_yes ? "Yes" : "No");
      }
    }
  }
  
private:
  std::shared_ptr<HRIClient> hri_client_;
};
```

## API de HRIClient

### Métodos principales

- `bool wait_for_services(std::chrono::seconds timeout)`: Espera a que todos los servicios estén disponibles
- `bool call_stt_service(std::string & transcribed_text)`: Convierte audio a texto
- `bool call_tts_service(const std::string & text)`: Reproduce texto como audio
- `bool call_extract_service(const std::string & interest, std::string & extracted_info)`: Extrae información específica
- `bool call_yesno_service(bool & user_said_yes)`: Detecta respuestas Yes/No
- `std::string get_last_listened_text()`: Obtiene el último texto del topic `/listened_text`

### Servicios disponibles

| Servicio | Tipo | Descripción |
|----------|------|-------------|
| `/stt_service` | `std_srvs/SetBool` | Speech-to-Text |
| `/tts_service` | `simple_hri_interfaces/Speech` | Text-to-Speech |
| `/extract_service` | `simple_hri_interfaces/Extract` | Extracción con LLM |
| `/yesno_service` | `std_srvs/SetBool` | Detección Yes/No |

### Topics

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/listened_text` | `std_msgs/String` | Texto transcrito por STT |

## Notas

- Los servicios son **bloqueantes**: la llamada espera hasta que el servicio complete
- El servicio STT graba automáticamente cuando detecta voz (VAD)
- El servicio Extract puede recibir texto directamente o grabar audio si se deja vacío
- Los servicios locales son más lentos pero no requieren conexión a Internet
- Asegúrate de que el micrófono y los altavoces estén configurados correctamente

## Licencia

Apache-2.0

## Autor

ASR Course - Intelligent Robotics Lab
