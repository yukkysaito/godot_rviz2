//
//  Copyright 2026 Yukihiro Saito. All rights reserved.
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

#include "route.hpp"

void Route::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("has_route"), &Route::has_route);
  ClassDB::bind_method(D_METHOD("get_goal_position"), &Route::get_goal_position);
  ClassDB::bind_method(D_METHOD("get_preferred_lanelet_ids"), &Route::get_preferred_lanelet_ids);
  TOPIC_SUBSCRIBER_BIND_METHODS(Route);
}

bool Route::has_route() { return get_last_msg().has_value(); }

Vector3 Route::get_goal_position()
{
  const auto last_msg = get_last_msg();
  if (!last_msg) return Vector3(0, 0, 0);

  return ros2_to_godot(last_msg.value()->goal_pose.position);
}

PackedInt64Array Route::get_preferred_lanelet_ids()
{
  PackedInt64Array ids;
  const auto last_msg = get_last_msg();
  if (!last_msg) return ids;

  for (const auto & segment : last_msg.value()->segments) {
    ids.push_back(segment.preferred_primitive.id);
  }
  return ids;
}
