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

#include "traffic_lights.hpp"

#include <string>

void TrafficLights::_bind_methods()
{
  ClassDB::bind_method(
    D_METHOD("get_traffic_light_status"), &TrafficLights::get_traffic_light_status);
  ClassDB::bind_method(
    D_METHOD("get_traffic_light_status_changes", "stale_seconds"),
    &TrafficLights::get_traffic_light_status_changes);
  TOPIC_SUBSCRIBER_BIND_METHODS(TrafficLights);
}

namespace
{
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;

Dictionary to_status_element(const TrafficLightElement & element)
{
  Dictionary status_element;
  if (element.color == TrafficLightElement::GREEN) {
    status_element["color"] = "green";
  } else if (element.color == TrafficLightElement::RED) {
    status_element["color"] = "red";
  } else if (element.color == TrafficLightElement::AMBER) {
    status_element["color"] = "yellow";
  } else if (element.color == TrafficLightElement::WHITE) {
    status_element["color"] = "white";
  } else {
    status_element["color"] = "unknown";
  }

  if (element.shape == TrafficLightElement::CIRCLE) {
    status_element["shape"] = "circle";
    status_element["arrow"] = "none";
  } else if (element.shape == TrafficLightElement::LEFT_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "left";
  } else if (element.shape == TrafficLightElement::RIGHT_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "right";
  } else if (element.shape == TrafficLightElement::UP_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "up";
  } else if (element.shape == TrafficLightElement::UP_LEFT_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "up_left";
  } else if (element.shape == TrafficLightElement::UP_RIGHT_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "up_right";
  } else if (element.shape == TrafficLightElement::DOWN_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "down";
  } else if (element.shape == TrafficLightElement::DOWN_LEFT_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "down_left";
  } else if (element.shape == TrafficLightElement::DOWN_RIGHT_ARROW) {
    status_element["shape"] = "arrow";
    status_element["arrow"] = "down_right";
  } else {
    status_element["shape"] = "unknown";
    status_element["arrow"] = "unknown";
  }

  if (element.status == TrafficLightElement::SOLID_OFF) {
    status_element["status"] = "solid_off";
  } else if (element.status == TrafficLightElement::SOLID_ON) {
    status_element["status"] = "solid_on";
  } else if (element.status == TrafficLightElement::FLASHING) {
    status_element["status"] = "flashing";
  } else {
    status_element["status"] = "unknown";
  }
  return status_element;
}

Dictionary to_group_status(int64_t group_id, const Array & status_elements)
{
  Dictionary group_status;
  group_status["group_id"] = group_id;
  group_status["status_elements"] = status_elements;
  return group_status;
}

Array to_status_elements(const TrafficLightGroup & group)
{
  Array status_elements;
  for (const auto & element : group.elements) {
    status_elements.append(to_status_element(element));
  }
  return status_elements;
}

// Compact representation of a group's status, used to detect changes cheaply
std::vector<uint32_t> pack_elements(const TrafficLightGroup & group)
{
  std::vector<uint32_t> packed;
  packed.reserve(group.elements.size());
  for (const auto & element : group.elements) {
    packed.push_back(
      (static_cast<uint32_t>(element.color) << 16) | (static_cast<uint32_t>(element.shape) << 8) |
      static_cast<uint32_t>(element.status));
  }
  return packed;
}
}  // namespace

Array TrafficLights::get_traffic_light_status()
{
  const auto last_msg = get_last_msg();
  Array traffic_light_status_list;
  if (!last_msg) return traffic_light_status_list;

  for (const auto & group : last_msg.value()->traffic_light_groups) {
    traffic_light_status_list.append(
      to_group_status(group.traffic_light_group_id, to_status_elements(group)));
  }
  return traffic_light_status_list;
}

Array TrafficLights::get_traffic_light_status_changes(double stale_seconds)
{
  const auto now = std::chrono::steady_clock::now();
  Array changes;

  // New message: report groups whose status differs from what was last reported
  const auto last_msg = get_last_msg();
  if (last_msg && last_msg.value().get() != last_processed_msg_) {
    last_processed_msg_ = last_msg.value().get();
    for (const auto & group : last_msg.value()->traffic_light_groups) {
      auto packed = pack_elements(group);
      auto & state = group_states_[group.traffic_light_group_id];
      state.last_seen = now;
      if (state.elements != packed) {
        state.elements = std::move(packed);
        changes.append(to_group_status(group.traffic_light_group_id, to_status_elements(group)));
      }
    }
  }

  // Groups that have not been received for a while are reported once as "all off"
  const auto stale = std::chrono::duration<double>(stale_seconds);
  for (auto & [group_id, state] : group_states_) {
    if (!state.elements.empty() && now - state.last_seen > stale) {
      state.elements.clear();
      changes.append(to_group_status(group_id, Array()));
    }
  }
  return changes;
}
