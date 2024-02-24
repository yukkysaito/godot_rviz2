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

#include "behavior_path.hpp"

#include <string>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace
{
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

Dictionary create_offset_point_dict_with_velocity(
  const Eigen::Quaternionf & quat, const Eigen::Vector3f & position, const float width_offset,
  const float velocity)
{
  Dictionary point_dict = create_offset_point_dict(quat, position, width_offset);
  point_dict["velocity"] = velocity;
  return point_dict;
}
}  // namespace

void BehaviorPath::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_path_triangle_strip"), &BehaviorPath::get_path_triangle_strip);
  ClassDB::bind_method(
    D_METHOD("get_drivable_area_triangle_strip"), &BehaviorPath::get_drivable_area_triangle_strip);
  TOPIC_SUBSCRIBER_BIND_METHODS(BehaviorPath);
}

Array BehaviorPath::get_path_triangle_strip(const float width)
{
  // Initialize an empty array for the triangle strip
  Array triangle_strip;
  // Retrieve the last path message
  const auto last_msg = get_last_msg();
  // If no message is found, return the empty array
  if (!last_msg) return triangle_strip;

  // Iterate over each point in the path message
  for (const auto & point : last_msg.value()->points) {
    // Extract pose information from the point
    const auto & pose = point.pose;
    Eigen::Quaternionf quat(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3f position(pose.position.x, pose.position.y, pose.position.z);

    // Append two points for each path point to form a strip
    triangle_strip.append(create_offset_point_dict_with_velocity(
      quat, position, -(width / 2.0), point.longitudinal_velocity_mps));
    triangle_strip.append(create_offset_point_dict_with_velocity(
      quat, position, (width / 2.0), point.longitudinal_velocity_mps));
  }

  return triangle_strip;
}

Dictionary BehaviorPath::get_drivable_area_triangle_strip(const float width)
{
  Dictionary drivable_area_lines;
  const auto last_msg = get_last_msg();
  if (!last_msg) return drivable_area_lines;

  const auto & left_line = last_msg.value()->left_bound;
  const auto & right_line = last_msg.value()->right_bound;

  if (left_line.size() < 2 || right_line.size() < 2) {
    Array empty_array;
    drivable_area_lines["left_line"] = empty_array;
    drivable_area_lines["right_line"] = empty_array;
    return drivable_area_lines;
  }

  drivable_area_lines["left_line"] = calculate_line_as_triangle_strip(left_line, width);
  drivable_area_lines["right_line"] = calculate_line_as_triangle_strip(right_line, width);

  return drivable_area_lines;
}
