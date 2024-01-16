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
  // Bind the get_wall_triangle_strip method to Godot
  ClassDB::bind_method(D_METHOD("get_wall_triangle_strip"), &Trajectory::get_wall_triangle_strip);
  // Additional method bindings for the TOPIC_SUBSCRIBER
  TOPIC_SUBSCRIBER_BIND_METHODS(Trajectory);
}

Dictionary Trajectory::create_point_dict(
  const Eigen::Quaternionf & quat, const Eigen::Vector3f & position, const float width_offset,
  const float velocity)
{
  // Calculation of the rotated offset
  Eigen::Vector3f local_offset, rotated_offset;
  local_offset << 0, width_offset, 0;
  rotated_offset = quat * local_offset;

  // Conversion to Godot's coordinate system
  Vector3 godot_position = ros2_to_godot(
    position.x() + rotated_offset.x(), position.y() + rotated_offset.y(),
    position.z() + rotated_offset.z());

  // Calculation of the rotated normal
  Eigen::Vector3f local_normal, rotated_normal;
  local_normal << 0, 0, 1;
  rotated_normal = quat * local_normal;
  Vector3 godot_normal = ros2_to_godot(rotated_normal.x(), rotated_normal.y(), rotated_normal.z());

  // Creating the dictionary with calculated values
  Dictionary point_dict;
  point_dict["velocity"] = velocity;
  point_dict["position"] = godot_position;
  point_dict["normal"] = godot_normal;
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
    triangle_strip.append(
      create_point_dict(quat, position, -(width / 2.0), point.longitudinal_velocity_mps));
    triangle_strip.append(
      create_point_dict(quat, position, (width / 2.0), point.longitudinal_velocity_mps));
  }

  return triangle_strip;
}

Array Trajectory::get_wall_triangle_strip(
  const float width, const float height, const float length_offset, const bool ignore_start_point,
  const bool ignore_end_point)
{
  // Initialize an empty array for the wall triangle strip
  Array triangle_strip;
  // Retrieve the last trajectory message
  const auto last_msg = get_last_msg();
  // If no message is found, return the empty array
  if (!last_msg) return triangle_strip;

  // Iterate over each point in the trajectory message
  for (size_t i = 0; i < last_msg.value()->points.size(); i++) {
    // Skip the start or end point if specified
    if (
      (ignore_end_point && i == last_msg.value()->points.size() - 1) ||
      (ignore_start_point && i == 0)) {
      continue;
    }

    const auto & point = last_msg.value()->points[i];
    // A small epsilon to compare floating-point numbers
    constexpr float eps = 0.0001;
    // Only consider points with a velocity lower than the epsilon
    if (point.longitudinal_velocity_mps < eps) {
      const auto & pose = point.pose;
      Eigen::Quaternionf quat(
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
      Eigen::Vector3f position(pose.position.x, pose.position.y, pose.position.z);
      const float half_width = width / 2.0;

      // Define local points for the wall
      std::vector<Eigen::Vector3f> local_points = {
        Eigen::Vector3f(length_offset, -half_width, 0),
        Eigen::Vector3f(length_offset, half_width, 0),
        Eigen::Vector3f(length_offset, -half_width, height),
        Eigen::Vector3f(length_offset, half_width, height)};

      // Define the local normal
      Eigen::Vector3f local_normal(-1, 0, 0);

      // Transform and append each point to the triangle strip
      for (size_t j = 0; j < local_points.size(); j++) {
        const auto & local_point = local_points[j];
        Eigen::Vector3f rotated_point = quat * local_point;
        Eigen::Vector3f rotated_normal = quat * local_normal;

        Dictionary point_dict;
        point_dict["position"] = ros2_to_godot(
          position.x() + rotated_point.x(), position.y() + rotated_point.y(),
          position.z() + rotated_point.z());
        point_dict["normal"] =
          ros2_to_godot(rotated_normal.x(), rotated_normal.y(), rotated_normal.z());
        // Set color based on the position in the wall
        if (j == 0 || j == 1) {
          point_dict["color"] = Color(1.0, 0.0, 0.0, 1.0);  // Red color for the base
        } else {
          point_dict["color"] = Color(1.0, 0.0, 0.0, 0.0);  // Transparent red for the top
        }

        triangle_strip.append(point_dict);
      }

      // Break after the first point with velocity less than epsilon
      break;
    }
  }
  return triangle_strip;
}
