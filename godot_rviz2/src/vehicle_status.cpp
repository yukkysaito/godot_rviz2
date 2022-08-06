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

void VehicleStatus::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("is_turn_on_right"), &VehicleStatus::is_turn_on_right);
  ClassDB::bind_method(D_METHOD("is_turn_on_left"), &VehicleStatus::is_turn_on_left);
  TOPIC_SUBSCRIBER_BIND_METHODS(VehicleStatus);
}

bool VehicleStatus::is_turn_on_right()
{
  const auto last_msg = get_last_msg();
  if (!last_msg) return false;

  return last_msg.value()->report ==
         autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
}

bool VehicleStatus::is_turn_on_left()
{
  const auto last_msg = get_last_msg();
  if (!last_msg) return false;

  return last_msg.value()->report ==
         autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
}
