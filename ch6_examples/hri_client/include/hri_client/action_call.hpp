// Copyright 2026 Intelligent Robotics Lab
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

#ifndef HRI_CLIENT__ACTION_CALL_HPP_
#define HRI_CLIENT__ACTION_CALL_HPP_

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <optional>

#include "rclcpp_action/rclcpp_action.hpp"

namespace hri_client
{

// ActionCall: seguimiento NO BLOQUEANTE de un goal de una acción (Listen o Say).
//
// Escuchar y hablar duran segundos, por eso simple_hri los ofrece como acciones
// (/stt_action, /tts_action): el resultado llega cuando la persona ha terminado de
// hablar o cuando el audio ha terminado de verdad, y el goal se puede cancelar.
//
// Uso (en cada ciclo de control, sin bloquear):
//   send(goal)  -> envía el goal
//   poll()      -> std::nullopt mientras el goal se ejecuta; su resultado al terminar
//   cancel()    -> pide al servidor que lo cancele (para la grabación o el audio)
//   feedback()  -> último feedback recibido (nullptr antes del primero)
//
// Los futures los completa quien hace spin del nodo (el executor), igual que en los
// clientes de servicios. Se asume un executor de un hilo (rclcpp::spin, spin_some).
template<class ActionT>
class ActionCall
{
public:
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  ActionCall(typename Client::SharedPtr client, const typename ActionT::Goal & goal)
  : client_(client), shared_(std::make_shared<Shared>())
  {
    auto shared = shared_;
    std::weak_ptr<Client> weak_client = client_;

    typename Client::SendGoalOptions options;

    // Si se canceló ANTES de que el servidor aceptara el goal, se cancela en cuanto
    // lo acepta. Se capturan 'shared' y un weak_ptr al cliente (no 'this'): así funciona
    // aunque este ActionCall ya se haya destruido.
    options.goal_response_callback =
      [shared, weak_client](typename GoalHandle::SharedPtr handle) {
        auto client = weak_client.lock();
        if (handle && client && shared->cancel_requested) {
          cancel_goal(client, handle);
        }
      };

    options.feedback_callback =
      [shared](typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> feedback) {
        shared->last_feedback = feedback;
      };

    goal_future_ = client_->async_send_goal(goal, options);
  }

  // std::nullopt mientras el goal se ejecuta; su resultado cuando termina.
  // Un goal rechazado por el servidor termina con ResultCode::ABORTED y result == nullptr.
  std::optional<WrappedResult> poll()
  {
    if (!result_future_.valid()) {
      if (!ready(goal_future_)) {
        return std::nullopt;
      }
      auto handle = goal_future_.get();
      if (!handle) {
        WrappedResult rejected;
        rejected.code = rclcpp_action::ResultCode::ABORTED;
        return rejected;
      }
      result_future_ = client_->async_get_result(handle);
    }
    if (!ready(result_future_)) {
      return std::nullopt;
    }
    return result_future_.get();
  }

  // Pide al servidor que cancele el goal, aunque todavía no lo haya aceptado
  void cancel()
  {
    if (shared_->cancel_requested.exchange(true)) {
      return;  // Ya se pidió
    }
    if (ready(goal_future_)) {
      auto handle = goal_future_.get();
      if (handle) {
        cancel_goal(client_, handle);
      }
    }
    // Si aún no está aceptado, lo cancela goal_response_callback cuando llegue la respuesta
  }

  std::shared_ptr<const Feedback> feedback() const {return shared_->last_feedback;}

private:
  // Estado compartido con los callbacks (pueden ejecutarse cuando este objeto ya no existe)
  struct Shared
  {
    std::atomic<bool> cancel_requested{false};
    std::shared_ptr<const Feedback> last_feedback;
  };

  static void cancel_goal(
    const typename Client::SharedPtr & client, const typename GoalHandle::SharedPtr & handle)
  {
    try {
      client->async_cancel_goal(handle);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      // El goal ya había terminado: no hay nada que cancelar
    }
  }

  template<class FutureT>
  static bool ready(const FutureT & future)
  {
    return future.valid() &&
           future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  typename Client::SharedPtr client_;
  std::shared_ptr<Shared> shared_;
  std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
  std::shared_future<WrappedResult> result_future_;
};

}  // namespace hri_client

#endif  // HRI_CLIENT__ACTION_CALL_HPP_
