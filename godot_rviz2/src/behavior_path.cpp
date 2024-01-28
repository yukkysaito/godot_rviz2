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
 * @return A Dictionary containing position, normal, and velocity
 */
Dictionary create_point_dict(
  const Eigen::Quaternionf & quat, const Eigen::Vector3f & position, const float width_offset)
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
  point_dict["position"] = godot_position;
  point_dict["normal"] = godot_normal;
  return point_dict;
}

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
  const float velocity)
{
  Dictionary point_dict = create_point_dict(quat, position, width_offset);
  point_dict["velocity"] = velocity;
  return point_dict;
}

// Process a single line (either left or right) and return the triangle strip points
Array calculate_line(const std::vector<geometry_msgs::msg::Point> & line, float width)
{
  Array line_triangle_points;
  Eigen::Vector2f previous_front_vec;

  for (size_t i = 0; i < line.size(); ++i) {
    const Eigen::Vector3f position(line.at(i).x, line.at(i).y, line.at(i).z);

    float yaw;
    if (i == 0) {  // start point
      Eigen::Vector2f front_vec(line.at(i + 1).x - line.at(i).x, line.at(i + 1).y - line.at(i).y);
      front_vec.normalize();
      yaw = std::atan2(front_vec.y(), front_vec.x());
      previous_front_vec = front_vec;
    } else if (i == line.size() - 1) {  // end point
      Eigen::Vector2f & back_vec = previous_front_vec;
      yaw = std::atan2(back_vec.y(), back_vec.x());
    } else {  // middle points
      Eigen::Vector2f front_vec(line.at(i + 1).x - line.at(i).x, line.at(i + 1).y - line.at(i).y);
      Eigen::Vector2f & back_vec = previous_front_vec;
      front_vec.normalize();
      yaw = std::atan2(front_vec.y() + back_vec.y(), front_vec.x() + back_vec.x());
      previous_front_vec = front_vec;
    }

    Eigen::Quaternionf quat = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) *
                              Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
                              Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    // Append two points for each path point to form a strip
    line_triangle_points.append(create_point_dict(quat, position, -(width / 2.0)));
    line_triangle_points.append(create_point_dict(quat, position, (width / 2.0)));
  }

  return line_triangle_points;
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
    triangle_strip.append(
      create_point_dict(quat, position, -(width / 2.0), point.longitudinal_velocity_mps));
    triangle_strip.append(
      create_point_dict(quat, position, (width / 2.0), point.longitudinal_velocity_mps));
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

  drivable_area_lines["left_line"] = calculate_line(left_line, width);
  drivable_area_lines["right_line"] = calculate_line(right_line, width);

  return drivable_area_lines;
}
