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

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

/**
 * @class Trajectory
 * @brief The Trajectory class processes trajectory data for autonomous vehicles.
 *
 * This class converts Autoware trajectory messages into a format suitable for the Godot Engine.
 */
class Trajectory : public RefCounted
{
  GDCLASS(Trajectory, RefCounted);
  TOPIC_SUBSCRIBER(Trajectory, autoware_auto_planning_msgs::msg::Trajectory);

public:
  /**
   * @brief Generates a triangle strip for the trajectory.
   *
   * @param width Width of the trajectory
   * @return Array of triangle strips
   */
  Array get_trajectory_triangle_strip(const float width);

  /**
   * @brief Generates a triangle strip for the wall.
   *
   * @param width Width of the wall
   * @param height Height of the wall
   * @param length_offset Offset along the length of the wall
   * @param ignore_start_point Whether to ignore the start point
   * @param ignore_end_point Whether to ignore the end point
   * @return Array of triangle strips
   */
  Array get_wall_triangle_strip(
    const float width, const float height, const float length_offset, const bool ignore_start_point,
    const bool ignore_end_point);

  Trajectory() = default;
  ~Trajectory() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();

private:
  /**
   * @brief Creates a dictionary representing a point in the trajectory.
   *
   * This method calculates the rotated offset and normal for a given position and quaternion,
   * and then converts these to a Godot-friendly format.
   *
   * @param quat Quaternion representing the orientation
   * @param position Position vector
   * @param width_offset Offset applied to the width
   * @param velocity Velocity at this point
   * @return A Dictionary containing position, normal, and velocity
   */
  Dictionary create_point_dict(
    const Eigen::Quaternionf & quat, const Eigen::Vector3f & position, const float width_offset,
    const float velocity);
};
