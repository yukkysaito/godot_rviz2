//
//  Copyright 2024 Yukihiro Saito. All rights reserved.
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

#include "autoware_perception_msgs/msg/traffic_light_group_array.hpp"

#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <vector>

class TrafficLights : public RefCounted
{
  GDCLASS(TrafficLights, RefCounted);
  TOPIC_SUBSCRIBER(TrafficLights, autoware_perception_msgs::msg::TrafficLightGroupArray);

public:
  Array get_traffic_light_status();

  /**
   * @brief Returns only the groups whose status changed since the previous call.
   *
   * Groups that were not received for longer than stale_seconds are returned once with empty
   * status_elements, so the caller can turn them off. Cheap when nothing changed, unlike
   * get_traffic_light_status() which converts every group of the map on each call.
   */
  Array get_traffic_light_status_changes(double stale_seconds);

  TrafficLights() = default;
  ~TrafficLights() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();

private:
  struct GroupState
  {
    std::vector<uint32_t> elements;  // packed (color, shape, status) per element
    std::chrono::steady_clock::time_point last_seen;
  };

  std::unordered_map<int64_t, GroupState> group_states_;
  const void * last_processed_msg_ = nullptr;
};
