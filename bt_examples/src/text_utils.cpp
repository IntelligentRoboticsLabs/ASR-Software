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

#include "bt_examples/text_utils.hpp"
#include <vector>

namespace bt_examples
{

std::string expandBlackboardVariables(
  const std::string & text,
  const BT::Blackboard::Ptr & blackboard)
{
  std::string formatted_text = text;

  // Expandir manualmente las referencias del blackboard en el texto
  // BehaviorTree.CPP no expande {variables} dentro de strings literales automáticamente
  size_t pos = 0;
  while ((pos = formatted_text.find('{', pos)) != std::string::npos) {
    size_t end_pos = formatted_text.find('}', pos);
    if (end_pos == std::string::npos) break;
    
    std::string var_name = formatted_text.substr(pos + 1, end_pos - pos - 1);
    BT::Expected<std::string> var_value = blackboard->get<std::string>(var_name);
    
    if (var_value) {
      formatted_text.replace(pos, end_pos - pos + 1, var_value.value());
      pos += var_value.value().length();
    } else {
      pos = end_pos + 1;
    }
  }

  return formatted_text;
}

std::string formatSemicolonList(const std::string & text)
{
  std::string formatted_text = text;

  // Formatear listas separadas por punto y coma (ej: "agua;café;té" -> "agua, café y té")
  if (formatted_text.find(';') != std::string::npos) {
    std::vector<std::string> items;
    size_t start = 0;
    size_t end;
    
    while ((end = formatted_text.find(';', start)) != std::string::npos) {
      items.push_back(formatted_text.substr(start, end - start));
      start = end + 1;
    }
    items.push_back(formatted_text.substr(start));
    
    if (items.size() > 1) {
      formatted_text.clear();
      for (size_t i = 0; i < items.size(); ++i) {
        if (i > 0) {
          if (i == items.size() - 1) {
            formatted_text += " y ";
          } else {
            formatted_text += ", ";
          }
        }
        formatted_text += items[i];
      }
    }
  }

  return formatted_text;
}

std::string formatText(
  const std::string & text,
  const BT::Blackboard::Ptr & blackboard)
{
  // Primero expandir variables del blackboard
  std::string formatted = expandBlackboardVariables(text, blackboard);
  
  // Luego formatear listas
  formatted = formatSemicolonList(formatted);
  
  return formatted;
}

}  // namespace bt_examples
