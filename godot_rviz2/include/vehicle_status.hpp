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

#pragma once

#include "core/object/ref_counted.h"
#include "core/string/ustring.h"
#include "core/variant/variant.h"
#include "topic_subscriber.hpp"

#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"

class VehicleStatus : public RefCounted
{
  GDCLASS(VehicleStatus, RefCounted);
  TOPIC_SUBSCRIBER(VehicleStatus, autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport);

public:
  bool is_turn_on_right();
  bool is_turn_on_left();

  VehicleStatus() = default;
  ~VehicleStatus() = default;

protected:
  static void _bind_methods();
};
