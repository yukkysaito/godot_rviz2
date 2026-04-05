//
//  Copyright 2022 Yukihiro Saito. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "operation_mode_state.hpp"

void OperationModeState::_bind_methods()
{
  // Bind the is_autonomous_mode_available method to Godot
  ClassDB::bind_method(
    D_METHOD("is_autonomous_mode_available"), &OperationModeState::is_autonomous_mode_available);
  ClassDB::bind_method(D_METHOD("is_autonomous_mode"), &OperationModeState::is_autonomous_mode);
  TOPIC_SUBSCRIBER_BIND_METHODS(OperationModeState);
}

bool OperationModeState::is_autonomous_mode_available()
{
  bool operation_mode_state = false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return operation_mode_state;

  const bool is_autonomous_mode =
    (last_msg.value()->mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);

  return last_msg.value()->is_autonomous_mode_available && !is_autonomous_mode;
}

bool OperationModeState::is_autonomous_mode()
{
  bool operation_mode_state = false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return operation_mode_state;

  const bool is_autonomous_mode =
    (last_msg.value()->mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);
  const bool under_autoware_control = last_msg.value()->is_autoware_control_enabled;

  return is_autonomous_mode && under_autoware_control;
}
