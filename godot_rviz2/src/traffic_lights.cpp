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
  TOPIC_SUBSCRIBER_BIND_METHODS(TrafficLights);
}

Array TrafficLights::get_traffic_light_status()
{
  using namespace autoware_perception_msgs::msg;
  const auto last_msg = get_last_msg();
  Array traffic_light_status_list;
  if (!last_msg) return traffic_light_status_list;

  for (const auto & signal : last_msg.value()->signals) {
    Array traffic_light_status_elements;
    for (const auto & element : signal.elements) {
      Dictionary traffic_light_status_element;
      if (element.color == TrafficSignalElement::GREEN) {
        traffic_light_status_element["color"] = "green";
      } else if (element.color == TrafficSignalElement::RED) {
        traffic_light_status_element["color"] = "red";
      } else if (element.color == TrafficSignalElement::AMBER) {
        traffic_light_status_element["color"] = "yellow";
      } else {
        traffic_light_status_element["color"] = "unknown";
      }

      if (element.shape == TrafficSignalElement::LEFT_ARROW) {
        traffic_light_status_element["arrow"] = "left";
      } else if (element.shape == TrafficSignalElement::RIGHT_ARROW) {
        traffic_light_status_element["arrow"] = "right";
      } else if (element.shape == TrafficSignalElement::UP_ARROW) {
        traffic_light_status_element["arrow"] = "up";
      } else if (element.shape == TrafficSignalElement::DOWN_ARROW) {
        traffic_light_status_element["arrow"] = "down";
      } else {
        traffic_light_status_element["arrow"] = "none";
      }
      traffic_light_status_elements.append(traffic_light_status_element);
    }
    Dictionary traffic_light_group;
    traffic_light_group["group_id"] = signal.traffic_signal_id;
    traffic_light_group["status_elements"] = traffic_light_status_elements;
    traffic_light_status_list.append(traffic_light_group);
  }

  return traffic_light_status_list;
}
