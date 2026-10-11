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

#include "trajectory.hpp"

#include "util.hpp"

#include <string>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/**
 * @brief Binds methods of Trajectory class to the Godot system.
 */
void Trajectory::_bind_methods()
{
  // Bind the get_trajectory_triangle_strip method to Godot
  ClassDB::bind_method(
    D_METHOD("get_trajectory_triangle_strip"), &Trajectory::get_trajectory_triangle_strip);
  // Additional method bindings for the TOPIC_SUBSCRIBER
  TOPIC_SUBSCRIBER_BIND_METHODS(Trajectory);
}

Dictionary Trajectory::create_offset_point_dict_with_velocity(
  const Eigen::Quaternionf & quat, const Eigen::Vector3f & position, const float width_offset,
  const float velocity)
{
  Dictionary point_dict = create_offset_point_dict(quat, position, width_offset);
  point_dict["velocity"] = velocity;
  return point_dict;
}

Array Trajectory::get_trajectory_triangle_strip(const float width)
{
  // Initialize an empty array for the triangle strip
  Array triangle_strip;
  // Retrieve the last trajectory message
  const auto last_msg = get_last_msg();
  // If no message is found, return the empty array
  if (!last_msg) return triangle_strip;

  // Iterate over each point in the trajectory message
  for (const auto & point : last_msg.value()->points) {
    // Extract pose information from the point
    const auto & pose = point.pose;
    Eigen::Quaternionf quat(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3f position(pose.position.x, pose.position.y, pose.position.z);

    // Append two points for each trajectory point to form a strip
    triangle_strip.append(create_offset_point_dict_with_velocity(
      quat, position, -(width / 2.0), point.longitudinal_velocity_mps));
    triangle_strip.append(create_offset_point_dict_with_velocity(
      quat, position, (width / 2.0), point.longitudinal_velocity_mps));
  }

  return triangle_strip;
}
