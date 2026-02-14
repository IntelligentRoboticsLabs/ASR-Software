// Copyright 2025 Intelligent Robotics Lab
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

#ifndef BT_EXAMPLES__TEXT_UTILS_HPP_
#define BT_EXAMPLES__TEXT_UTILS_HPP_

#include <string>
#include <behaviortree_cpp/bt_factory.h>

namespace bt_examples
{

/**
 * @brief Expande referencias del blackboard en un texto.
 * 
 * Reemplaza {variable} por el valor de la variable en el blackboard.
 * Ejemplo: "Hola {nombre}" -> "Hola Juan" si nombre="Juan"
 * 
 * @param text Texto con variables entre llaves
 * @param blackboard Blackboard del árbol de comportamiento
 * @return Texto con variables expandidas
 */
std::string expandBlackboardVariables(
  const std::string & text,
  const BT::Blackboard::Ptr & blackboard);

/**
 * @brief Formatea una lista separada por punto y coma en lenguaje natural.
 * 
 * Ejemplo: "agua;café;té" -> "agua, café y té"
 * 
 * @param text Texto con elementos separados por punto y coma
 * @return Texto formateado como lista en lenguaje natural
 */
std::string formatSemicolonList(const std::string & text);

/**
 * @brief Aplica todos los formateos de texto (expande variables y formatea listas).
 * 
 * @param text Texto a formatear
 * @param blackboard Blackboard del árbol de comportamiento
 * @return Texto completamente formateado
 */
std::string formatText(
  const std::string & text,
  const BT::Blackboard::Ptr & blackboard);

}  // namespace bt_examples

#endif  // BT_EXAMPLES__TEXT_UTILS_HPP_
