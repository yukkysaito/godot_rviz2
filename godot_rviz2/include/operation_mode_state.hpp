//
//  Copyright 2025 Yukihiro Saito. All rights reserved.
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

#pragma once

#include "core/object/ref_counted.h"
#include "core/string/ustring.h"
#include "core/variant/variant.h"
#include "topic_subscriber.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
// #include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"

class OperationModeState : public RefCounted
{
  GDCLASS(OperationModeState, RefCounted);
  TOPIC_SUBSCRIBER(OperationModeState, autoware_adapi_v1_msgs::msg::OperationModeState);

public:
  /**
   * @brief Retrieves the operation mode state.
   *
   * @return boolean indicating if autonomous mode is available.
   */
  bool is_autonomous_mode_available();
  /**
   * @brief Checks if the current mode is autonomous.
   *
   * @return boolean indicating if the current mode is autonomous.
   */
  bool is_autonomous_mode();

  OperationModeState() = default;
  ~OperationModeState() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();
};
