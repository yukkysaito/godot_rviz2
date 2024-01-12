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

#include "vehicle_status.hpp"

#include <string>

/**
 * @brief Binds methods of VehicleStatus class to the Godot system.
 */
void VehicleStatus::_bind_methods()
{
  // Bind the is_turn_on_right method to Godot
  ClassDB::bind_method(D_METHOD("is_turn_on_right"), &VehicleStatus::is_turn_on_right);
  // Bind the is_turn_on_left method to Godot
  ClassDB::bind_method(D_METHOD("is_turn_on_left"), &VehicleStatus::is_turn_on_left);
  // Additional method bindings for the TOPIC_SUBSCRIBER
  TOPIC_SUBSCRIBER_BIND_METHODS(VehicleStatus);
}

/**
 * @brief Checks if the right turn indicator is on.
 *
 * This method checks the latest TurnIndicatorsReport message and returns true if
 * the right turn indicator is enabled.
 *
 * @return true if the right turn indicator is active, false otherwise.
 */
bool VehicleStatus::is_turn_on_right()
{
  // Retrieve the last TurnIndicatorsReport message
  const auto last_msg = get_last_msg();
  // Return false if no message is found
  if (!last_msg) return false;

  // Check if the right turn indicator is enabled in the message
  return last_msg.value()->report ==
         autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
}

/**
 * @brief Checks if the left turn indicator is on.
 *
 * This method checks the latest TurnIndicatorsReport message and returns true if
 * the left turn indicator is enabled.
 *
 * @return true if the left turn indicator is active, false otherwise.
 */
bool VehicleStatus::is_turn_on_left()
{
  // Retrieve the last TurnIndicatorsReport message
  const auto last_msg = get_last_msg();
  // Return false if no message is found
  if (!last_msg) return false;

  // Check if the left turn indicator is enabled in the message
  return last_msg.value()->report ==
         autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
}
