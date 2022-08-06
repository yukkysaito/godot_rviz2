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

#include "steering_report.hpp"

#include <string>

void SteeringReport::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_angle"), &SteeringReport::get_angle);
  TOPIC_SUBSCRIBER_BIND_METHODS(SteeringReport);
}

double SteeringReport::get_angle()
{
  const auto last_msg = get_last_msg();
  if (!last_msg) return 0.0;

  return last_msg.value()->steering_tire_angle;
}
