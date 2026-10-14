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

#pragma once

#include "core/object/ref_counted.h"
#include "core/variant/variant.h"
#include "topic_subscriber.hpp"

#include "autoware_planning_msgs/msg/lanelet_route.hpp"

/**
 * @class Route
 * @brief The route from the mission planner, for its goal pose.
 */
class Route : public RefCounted
{
  GDCLASS(Route, RefCounted);
  TOPIC_SUBSCRIBER(Route, autoware_planning_msgs::msg::LaneletRoute);

public:
  /**
   * @brief Whether a route was received.
   */
  bool has_route();

  /**
   * @brief Goal position in Godot's coordinate system (Vector3.ZERO without a route).
   */
  Vector3 get_goal_position();

  Route() = default;
  ~Route() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();
};
