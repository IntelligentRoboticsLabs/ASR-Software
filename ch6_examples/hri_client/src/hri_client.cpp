// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hri_client/hri_client.hpp"

#include <algorithm>
#include <cctype>

namespace
{

// true si el future de un servicio ya tiene la respuesta (sin bloquear)
template<class FutureT>
bool future_ready(const FutureT & future)
{
  return future.valid() &&
         future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

std::string to_lower(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return std::tolower(c);});
  return text;
}

}  // namespace

const char * to_string(OperationState state)
{
  switch (state) {
    case OperationState::IDLE: return "IDLE";
    case OperationState::IN_PROGRESS: return "IN_PROGRESS";
    case OperationState::COMPLETED: return "COMPLETED";
    case OperationState::ERROR: return "ERROR";
    case OperationState::TIMEOUT: return "TIMEOUT";
    case OperationState::CANCELED: return "CANCELED";
  }
  return "UNKNOWN";
}

HRIClient::HRIClient(rclcpp::Node * node)
: node_(node)
{
  // Escuchar y hablar son acciones (feedback, cancelación, fin real de la reproducción);
  // extraer información y sí/no son servicios
  listen_client_ = rclcpp_action::create_client<Listen>(node_, "/stt_action");
  say_client_ = rclcpp_action::create_client<Say>(node_, "/tts_action");
  extract_client_ = node_->create_client<Extract>("/extract_service");
  yesno_client_ = node_->create_client<YesNo>("/yesno_service");

  listened_text_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/listened_text", 10,
    std::bind(&HRIClient::listened_text_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(node_->get_logger(), "Cliente HRI inicializado");
}

bool HRIClient::wait_until_ready(std::chrono::seconds timeout)
{
  bool all_ready = true;

  if (!listen_client_->wait_for_action_server(timeout)) {
    RCLCPP_WARN(node_->get_logger(), "Acción STT (/stt_action) no disponible");
    all_ready = false;
  }
  if (!say_client_->wait_for_action_server(timeout)) {
    RCLCPP_WARN(node_->get_logger(), "Acción TTS (/tts_action) no disponible");
    all_ready = false;
  }
  if (!extract_client_->wait_for_service(timeout)) {
    RCLCPP_WARN(node_->get_logger(), "Servicio Extract (/extract_service) no disponible");
    all_ready = false;
  }
  if (!yesno_client_->wait_for_service(timeout)) {
    RCLCPP_WARN(node_->get_logger(), "Servicio YesNo (/yesno_service) no disponible");
    all_ready = false;
  }

  if (all_ready) {
    RCLCPP_DEBUG(node_->get_logger(), "Todas las acciones y servicios HRI disponibles");
  }
  return all_ready;
}

// ============ PLAZOS ============

std::optional<HRIClient::Clock::time_point> HRIClient::deadline(double timeout_sec)
{
  if (timeout_sec <= 0.0) {
    return std::nullopt;  // Sin límite
  }
  return Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(timeout_sec));
}

bool HRIClient::expired(const std::optional<Clock::time_point> & deadline)
{
  return deadline.has_value() && Clock::now() > *deadline;
}

bool HRIClient::is_finished(OperationState state)
{
  return state != OperationState::IDLE && state != OperationState::IN_PROGRESS;
}

// ============ ESCUCHAR (STT) ============

void HRIClient::start_listening(double timeout_sec)
{
  if (stt_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(node_->get_logger(), "STT ya está en progreso, ignorando nueva petición");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Iniciando escucha (STT)...");

  stt_state_ = OperationState::IN_PROGRESS;
  stt_text_.clear();

  // El timeout de escuchar lo aplica el SERVIDOR (max_wait): deja de grabar si nadie
  // empieza a hablar, y devuelve timed_out = true
  Listen::Goal goal;
  goal.max_wait = static_cast<float>(std::max(0.0, timeout_sec));
  stt_call_ = std::make_unique<hri_client::ActionCall<Listen>>(listen_client_, goal);
}

bool HRIClient::is_listening_done()
{
  if (stt_state_ != OperationState::IN_PROGRESS) {
    return is_finished(stt_state_);
  }

  auto finished = stt_call_->poll();
  if (!finished) {
    return false;
  }

  const auto & result = finished->result;
  if (finished->code == rclcpp_action::ResultCode::SUCCEEDED && result && result->success) {
    stt_text_ = result->text;
    stt_state_ = OperationState::COMPLETED;
    RCLCPP_INFO(node_->get_logger(), "STT completado: '%s'", stt_text_.c_str());
  } else if (finished->code == rclcpp_action::ResultCode::CANCELED) {
    stt_state_ = OperationState::CANCELED;
  } else if (result && result->timed_out) {
    stt_state_ = OperationState::TIMEOUT;
    RCLCPP_WARN(node_->get_logger(), "STT: nadie ha hablado (timeout)");
  } else {
    stt_state_ = OperationState::ERROR;
    RCLCPP_WARN(
      node_->get_logger(), "STT falló: %s",
      result ? result->message.c_str() : "goal rechazado");
  }
  return true;
}

std::string HRIClient::get_listening_feedback() const
{
  if (!stt_call_) {
    return "";
  }
  auto feedback = stt_call_->feedback();
  return feedback ? feedback->status : "";
}

// ============ HABLAR (TTS) ============

void HRIClient::start_speaking(const std::string & text, double timeout_sec)
{
  if (tts_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(node_->get_logger(), "TTS ya está en progreso, ignorando nueva petición");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Iniciando TTS: '%s'", text.c_str());

  tts_state_ = OperationState::IN_PROGRESS;
  tts_deadline_ = deadline(timeout_sec);

  // La acción termina cuando acaba la reproducción: no hace falta estimar la duración
  Say::Goal goal;
  goal.text = text;
  tts_call_ = std::make_unique<hri_client::ActionCall<Say>>(say_client_, goal);
}

bool HRIClient::is_speaking_done()
{
  if (tts_state_ != OperationState::IN_PROGRESS) {
    return is_finished(tts_state_);
  }

  auto finished = tts_call_->poll();
  if (!finished) {
    if (expired(tts_deadline_)) {
      tts_call_->cancel();  // El servidor corta el audio
      tts_state_ = OperationState::TIMEOUT;
      RCLCPP_WARN(node_->get_logger(), "TTS: timeout");
      return true;
    }
    return false;
  }

  const auto & result = finished->result;
  if (finished->code == rclcpp_action::ResultCode::SUCCEEDED && result && result->success) {
    tts_state_ = OperationState::COMPLETED;
    RCLCPP_INFO(node_->get_logger(), "TTS completado");
  } else if (finished->code == rclcpp_action::ResultCode::CANCELED) {
    tts_state_ = OperationState::CANCELED;
  } else {
    tts_state_ = OperationState::ERROR;
    RCLCPP_ERROR(
      node_->get_logger(), "TTS falló: %s",
      result ? result->message.c_str() : "goal rechazado");
  }
  return true;
}

float HRIClient::get_speaking_feedback() const
{
  if (!tts_call_) {
    return -1.0f;
  }
  auto feedback = tts_call_->feedback();
  return feedback ? feedback->remaining : -1.0f;
}

// ============ EXTRAER INFORMACIÓN ============

void HRIClient::start_extract(
  const std::string & interest, const std::string & text, double timeout_sec)
{
  if (extract_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(node_->get_logger(), "Extract ya está en progreso, ignorando nueva petición");
    return;
  }

  if (text.empty()) {
    // simple_hri no graba audio en Extract: con un texto vacío devuelve "NONE".
    // El texto se obtiene antes con start_listening()/get_listened_text()
    RCLCPP_WARN(node_->get_logger(), "Extracción de '%s' con texto vacío", interest.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "Iniciando extracción de '%s' del texto '%s'",
      interest.c_str(), text.c_str());
  }

  auto request = std::make_shared<Extract::Request>();
  request->interest = interest;
  request->text = text;

  extract_state_ = OperationState::IN_PROGRESS;
  extracted_info_.clear();
  extract_deadline_ = deadline(timeout_sec);
  extract_future_ = extract_client_->async_send_request(request);
}

bool HRIClient::is_extract_done()
{
  if (extract_state_ != OperationState::IN_PROGRESS) {
    return is_finished(extract_state_);
  }

  if (extract_future_ && future_ready(extract_future_->future)) {
    auto response = extract_future_->future.get();
    extract_future_.reset();
    extracted_info_ = response->result;

    // simple_hri devuelve "ERROR..." si falla y "NONE" si no encuentra nada (eso NO es
    // un error: la aplicación decide qué hacer)
    if (!extracted_info_.empty() && extracted_info_.rfind("ERROR", 0) != 0) {
      extract_state_ = OperationState::COMPLETED;
      RCLCPP_INFO(node_->get_logger(), "Extracción completada: %s", extracted_info_.c_str());
    } else {
      extract_state_ = OperationState::ERROR;
      RCLCPP_WARN(
        node_->get_logger(), "La extracción no pudo obtener la información: '%s'",
        extracted_info_.c_str());
    }
    return true;
  }

  if (expired(extract_deadline_)) {
    cancel_extract();
    extract_state_ = OperationState::TIMEOUT;
    RCLCPP_WARN(node_->get_logger(), "Extract: timeout");
    return true;
  }

  return false;
}

// ============ SÍ/NO ============

void HRIClient::start_yesno(const std::string & text, double timeout_sec)
{
  if (yesno_state_ == OperationState::IN_PROGRESS) {
    RCLCPP_DEBUG(node_->get_logger(), "YesNo ya está en progreso, ignorando nueva petición");
    return;
  }

  if (text.empty()) {
    // simple_hri no graba audio en YesNo: con un texto vacío devuelve "ERROR: Empty text"
    RCLCPP_WARN(node_->get_logger(), "Detección sí/no con texto vacío");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Iniciando detección sí/no del texto '%s'", text.c_str());
  }

  auto request = std::make_shared<YesNo::Request>();
  request->text = text;

  yesno_state_ = OperationState::IN_PROGRESS;
  yesno_result_.clear();
  yesno_deadline_ = deadline(timeout_sec);
  yesno_future_ = yesno_client_->async_send_request(request);
}

bool HRIClient::is_yesno_done()
{
  if (yesno_state_ != OperationState::IN_PROGRESS) {
    return is_finished(yesno_state_);
  }

  if (yesno_future_ && future_ready(yesno_future_->future)) {
    auto response = yesno_future_->future.get();
    yesno_future_.reset();
    yesno_result_ = response->result;

    // simple_hri devuelve "YES", "NO", "UNKNOWN" (duda, o ni sí ni no) o "ERROR..."
    auto answer = to_lower(yesno_result_);
    if (answer == "yes" || answer == "no") {
      yesno_state_ = OperationState::COMPLETED;
      RCLCPP_INFO(node_->get_logger(), "YesNo completado: %s", yesno_result_.c_str());
    } else {
      yesno_state_ = OperationState::ERROR;
      RCLCPP_WARN(
        node_->get_logger(), "YesNo no obtuvo una respuesta válida: %s",
        yesno_result_.c_str());
    }
    return true;
  }

  if (expired(yesno_deadline_)) {
    cancel_yesno();
    yesno_state_ = OperationState::TIMEOUT;
    RCLCPP_WARN(node_->get_logger(), "YesNo: timeout");
    return true;
  }

  return false;
}

// ============ CANCELACIÓN ============

void HRIClient::cancel_listening()
{
  if (stt_state_ == OperationState::IN_PROGRESS) {
    stt_call_->cancel();  // El servidor deja de grabar
    stt_state_ = OperationState::CANCELED;
    RCLCPP_INFO(node_->get_logger(), "STT cancelado");
  }
}

void HRIClient::cancel_speaking()
{
  if (tts_state_ == OperationState::IN_PROGRESS) {
    tts_call_->cancel();  // El servidor corta el audio
    tts_state_ = OperationState::CANCELED;
    RCLCPP_INFO(node_->get_logger(), "TTS cancelado");
  }
}

void HRIClient::cancel_extract()
{
  // Una llamada a un servicio no se puede cancelar: se olvida la petición para que
  // una respuesta tardía se ignore
  if (extract_state_ == OperationState::IN_PROGRESS) {
    if (extract_future_) {
      extract_client_->remove_pending_request(*extract_future_);
      extract_future_.reset();
    }
    extract_state_ = OperationState::CANCELED;
    RCLCPP_DEBUG(node_->get_logger(), "Extract cancelado");
  }
}

void HRIClient::cancel_yesno()
{
  if (yesno_state_ == OperationState::IN_PROGRESS) {
    if (yesno_future_) {
      yesno_client_->remove_pending_request(*yesno_future_);
      yesno_future_.reset();
    }
    yesno_state_ = OperationState::CANCELED;
    RCLCPP_DEBUG(node_->get_logger(), "YesNo cancelado");
  }
}

void HRIClient::listened_text_callback(const std_msgs::msg::String::SharedPtr msg)
{
  last_listened_text_ = msg->data;
  RCLCPP_DEBUG(node_->get_logger(), "Texto escuchado recibido: %s", last_listened_text_.c_str());
}
