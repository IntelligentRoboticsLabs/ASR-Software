# hri_client

Cliente reutilizable de `simple_hri`: escuchar (STT) y hablar (TTS) con sus **acciones**;
extraer información (Extract) y sí/no (YesNo) con sus **servicios**.

## Descripción

La clase `HRIClient` encapsula la interacción humano-robot de
[`simple_hri`](https://github.com/rodperex/simple_hri) para usarla en aplicaciones (FSM) y
behavior trees.

- Escuchar y hablar duran segundos, por eso son **acciones** (`/stt_action`, `/tts_action`):
  dan *feedback*, se pueden cancelar y hablar termina cuando el audio termina de verdad
  (no hay que estimar la duración).
- Extraer información y sí/no son consultas sobre un texto, por eso son **servicios**
  (`/extract_service`, `/yesno_service`). No graban audio: se les pasa el texto que
  devuelve STT.

`HRIClient` cumple el mismo **contrato de capacidad** que `NavigationClient`
(`navigation_client`): `start_*()`, `is_*_done()`, `get_*_state()`, `cancel_*()`.

## Uso

La API es asíncrona y **no bloqueante**: se lanza una operación con `start_*()` y en cada
ciclo de control se consulta `is_*_done()`. Así el nodo sigue procesando callbacks mientras
el robot habla o escucha. Ver `hri_example` (FSM) y `bt_examples` (behavior trees).

```cpp
#include "hri_client/hri_client.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    hri_ = std::make_shared<HRIClient>(this);   // composición: el cliente usa este nodo
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {control_cycle();});
  }

private:
  void control_cycle()
  {
    switch (state_) {
      case State::SAY:
        hri_->start_speaking("Hola, ¿cómo te llamas?");
        state_ = State::WAIT_SAY;
        break;
      case State::WAIT_SAY:
        // Escuchar SOLO cuando el robot ha terminado de hablar
        if (hri_->is_speaking_done()) {
          hri_->start_listening(8.0);  // 8 s para que la persona EMPIECE a hablar
          state_ = State::WAIT_LISTEN;
        }
        break;
      case State::WAIT_LISTEN:
        if (hri_->is_listening_done()) {
          if (hri_->get_listening_state() == OperationState::TIMEOUT) {
            hri_->start_speaking("No te he oído. ¿Cómo te llamas?");  // reformular
            state_ = State::WAIT_SAY;
          } else {
            hri_->start_extract("nombre", hri_->get_listened_text());
            state_ = State::WAIT_NAME;
          }
        }
        break;
      case State::WAIT_NAME:
        if (hri_->is_extract_done()) {
          RCLCPP_INFO(get_logger(), "Nombre: %s", hri_->get_extracted_info().c_str());
          state_ = State::DONE;
        }
        break;
      // ...
    }
  }
  // ...
};
```

| Operación | Iniciar | ¿Terminada? | Estado | Resultado |
| --- | --- | --- | --- | --- |
| STT | `start_listening(timeout_sec)` | `is_listening_done()` | `get_listening_state()` | `get_listened_text()` |
| TTS | `start_speaking(text, timeout_sec)` | `is_speaking_done()` | `get_speaking_state()` | — |
| Extract | `start_extract(interest, text, timeout_sec)` | `is_extract_done()` | `get_extract_state()` | `get_extracted_info()` |
| YesNo | `start_yesno(text, timeout_sec)` | `is_yesno_done()` | `get_yesno_state()` | `get_yesno_result()` |

### Estados (`OperationState`)

`IDLE`, `IN_PROGRESS`, `COMPLETED`, `ERROR`, `TIMEOUT`, `CANCELED`. `to_string(state)`
devuelve el nombre, para los logs.

- **Extract** devuelve los valores separados por `;` (`"agua;cerveza"`), o `"NONE"` si el
  texto no contiene nada de esa categoría (estado `COMPLETED`: la aplicación decide qué
  hacer). Si falla, el resultado empieza por `ERROR` y el estado es `ERROR`.
- **YesNo** devuelve `"YES"` o `"NO"` (`COMPLETED`), o `"UNKNOWN"` (duda, o ni sí ni no) o
  `"ERROR..."` (`ERROR`).

### Timeout, cancelación y *feedback*

| Método | Comportamiento |
| --- | --- |
| `start_listening(timeout_sec)` | El **servidor** deja de grabar si nadie empieza a hablar en `timeout_sec` → `TIMEOUT`; si habla, espera a que termine. `0` = valor por defecto del servidor (10 s) |
| `start_speaking(text, timeout_sec)` | Si tarda más, el cliente cancela el goal (se corta el audio) → `TIMEOUT`. `0` = sin límite |
| `start_extract/start_yesno(..., timeout_sec)` | *Timeout* del cliente: la respuesta que llegue tarde se descarta → `TIMEOUT`. `0` = sin límite |
| `cancel_listening()` / `cancel_speaking()` | El servidor deja de grabar / corta el audio → `CANCELED` (también si aún no había aceptado el goal) |
| `cancel_extract()` / `cancel_yesno()` | Un servicio no se puede cancelar: se descarta la respuesta → `CANCELED` |
| `get_listening_feedback()` | `"listening"`, `"speech_detected"`, `"transcribing"` |
| `get_speaking_feedback()` | Segundos de reproducción que faltan (`-1` antes del primer feedback) |

Los plazos de HRI cuentan tiempo de **reloj** (también en simulación: escuchar y hablar
ocurren en tiempo real), a diferencia de `NavigationClient`, que cuenta tiempo del robot.

### Detalles de implementación

- `ActionCall<ActionT>` (`action_call.hpp`) sigue un goal sin bloquear: `poll()` devuelve
  `std::nullopt` mientras se ejecuta y el resultado al terminar; `cancel()` funciona
  aunque el servidor aún no haya aceptado el goal (se cancela en cuanto lo acepta).
- Se asume un executor de un hilo. Si el nodo no usa `rclcpp::spin()` sino un bucle propio
  (p. ej. un behavior tree), procesar los eventos con `executor.spin_all(...)`:
  `spin_some()` ejecuta **un solo evento** por cliente de acción en cada llamada y, con el
  *feedback* a 10 Hz, la respuesta, el resultado o la cancelación llegarían con retraso.

## Dependencias

- rclcpp, rclcpp_action, std_msgs
- simple_hri_interfaces (acciones `Listen` y `Say`, servicios `Extract` y `YesNo`)

## Lanzar simple_hri

```bash
ros2 launch simple_hri simple_hri.launch.py                   # todo local y gratis (español)
ros2 launch simple_hri simple_hri.launch.py backend:=ollama   # Extract y YesNo con Ollama
```

Ver el README de `simple_hri` para los *backends*, idiomas y modelos.
