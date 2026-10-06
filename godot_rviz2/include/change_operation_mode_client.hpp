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
#include "godot_rviz2.hpp"
#include "util.hpp"

#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"

#include <atomic>
#include <memory>

class OperationModeChanger : public RefCounted
{
  GDCLASS(OperationModeChanger, RefCounted);

public:
  bool create_client(const String & service_name);
  bool is_server_ready();

  /**
   * @brief Sends the request without waiting (the response is handled on the executor thread).
   * @return true if the request was sent (false: no client, service not ready or still pending)
   */
  bool change_to_autonomous_mode();

  /**
   * @brief State of the last request: "idle", "pending", "succeeded" or "failed".
   */
  String get_request_state();
  OperationModeChanger() = default;
  ~OperationModeChanger() = default;

private:
  enum class RequestState : int { Idle, Pending, Succeeded, Failed };

  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr client_;
  // Written from the executor thread when the response arrives
  std::shared_ptr<std::atomic<int>> request_state_ =
    std::make_shared<std::atomic<int>>(static_cast<int>(RequestState::Idle));

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();
};
