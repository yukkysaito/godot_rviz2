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

  for (const auto & traffic_light_group : last_msg.value()->traffic_light_groups) {
    Array traffic_light_status_elements;
    for (const auto & element : traffic_light_group.elements) {
      Dictionary traffic_light_status_element;
      if (element.color == TrafficLightElement::GREEN) {
        traffic_light_status_element["color"] = "green";
      } else if (element.color == TrafficLightElement::RED) {
        traffic_light_status_element["color"] = "red";
      } else if (element.color == TrafficLightElement::AMBER) {
        traffic_light_status_element["color"] = "yellow";
      } else if (element.color == TrafficLightElement::WHITE) {
        traffic_light_status_element["color"] = "white";
      } else {
        traffic_light_status_element["color"] = "unknown";
      }

      if (element.shape == TrafficLightElement::CIRCLE) {
        traffic_light_status_element["shape"] = "circle";
        traffic_light_status_element["arrow"] = "none";
      } else if (element.shape == TrafficLightElement::LEFT_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "left";
      } else if (element.shape == TrafficLightElement::RIGHT_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "right";
      } else if (element.shape == TrafficLightElement::UP_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "up";
      } else if (element.shape == TrafficLightElement::UP_LEFT_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "up_left";
      } else if (element.shape == TrafficLightElement::UP_RIGHT_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "up_right";
      } else if (element.shape == TrafficLightElement::DOWN_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "down";
      } else if (element.shape == TrafficLightElement::DOWN_LEFT_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "down_left";
      } else if (element.shape == TrafficLightElement::DOWN_RIGHT_ARROW) {
        traffic_light_status_element["shape"] = "arrow";
        traffic_light_status_element["arrow"] = "down_right";
      } else {
        traffic_light_status_element["shape"] = "unknown";
        traffic_light_status_element["arrow"] = "unknown";
      }

      if (element.status == TrafficLightElement::SOLID_OFF) {
        traffic_light_status_element["status"] = "solid_off";
      } else if (element.status == TrafficLightElement::SOLID_ON) {
        traffic_light_status_element["status"] = "solid_on";
      } else if (element.status == TrafficLightElement::FLASHING) {
        traffic_light_status_element["status"] = "flashing";
      } else {
        traffic_light_status_element["status"] = "unknown";
      }
      traffic_light_status_elements.append(traffic_light_status_element);
    }
    Dictionary traffic_light_dict;
    traffic_light_dict["group_id"] = traffic_light_group.traffic_light_group_id;
    traffic_light_dict["status_elements"] = traffic_light_status_elements;
    traffic_light_status_list.append(traffic_light_dict);
  }

  return traffic_light_status_list;
}
