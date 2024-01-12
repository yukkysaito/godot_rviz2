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

#include "velocity_report.hpp"

#include <string>

/**
 * @brief Binds methods of the VelocityReport class to the Godot system.
 */
void VelocityReport::_bind_methods()
{
  // Bind the get_velocity method to Godot
  ClassDB::bind_method(D_METHOD("get_velocity"), &VelocityReport::get_velocity);
  // Additional method bindings for the TOPIC_SUBSCRIBER
  TOPIC_SUBSCRIBER_BIND_METHODS(VelocityReport);
}

/**
 * @brief Retrieves the current velocity from the latest VelocityReport message.
 *
 * This method checks the latest VelocityReport message and returns the longitudinal
 * velocity of the vehicle. If no message is available, it returns 0.0.
 *
 * @return double Current longitudinal velocity of the vehicle, or 0.0 if no message is available.
 */
double VelocityReport::get_velocity()
{
  // Retrieve the last VelocityReport message
  const auto last_msg = get_last_msg();
  // Return 0.0 if no message is found
  if (!last_msg) return 0.0;

  // Return the longitudinal velocity from the message
  return last_msg.value()->longitudinal_velocity;
}
