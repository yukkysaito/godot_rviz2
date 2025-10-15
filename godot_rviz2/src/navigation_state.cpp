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

#include "navigation_state.hpp"

void NavigationState::_bind_methods()
{
  // Bind the is_arrived method to Godot
  ClassDB::bind_method(D_METHOD("is_arrived"), &NavigationState::is_arrived);
  ClassDB::bind_method(D_METHOD("has_no_route"), &NavigationState::has_no_route);
  ClassDB::bind_method(D_METHOD("has_route"), &NavigationState::has_route);
  ClassDB::bind_method(D_METHOD("is_changing_route"), &NavigationState::is_changing_route);
  TOPIC_SUBSCRIBER_BIND_METHODS(NavigationState);
}

bool NavigationState::is_arrived()
{
  bool navigation_state = false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return navigation_state;

  return last_msg.value()->state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
}

bool NavigationState::has_no_route()
{
  bool navigation_state = false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return navigation_state;

  return last_msg.value()->state == autoware_adapi_v1_msgs::msg::RouteState::UNSET;
}

bool NavigationState::has_route()
{
  bool navigation_state = false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return navigation_state;

  return last_msg.value()->state == autoware_adapi_v1_msgs::msg::RouteState::SET;
}

bool NavigationState::is_changing_route()
{
  bool navigation_state = false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return navigation_state;

  return last_msg.value()->state == autoware_adapi_v1_msgs::msg::RouteState::CHANGING;
}
