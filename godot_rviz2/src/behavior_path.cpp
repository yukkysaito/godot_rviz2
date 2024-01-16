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

void BehaviorPath::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_path_triangle_strip"), &BehaviorPath::get_path_triangle_strip);
  ClassDB::bind_method(
    D_METHOD("get_drivable_area_triangle_strips"),
    &BehaviorPath::get_drivable_area_triangle_strips);
  TOPIC_SUBSCRIBER_BIND_METHODS(BehaviorPath);
}

Dictionary BehaviorPath::create_point_dict(
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

Array BehaviorPath::get_drivable_area_triangle_strips(const float width)
{
  Array drivable_area_lines;
  PackedVector3Array left_line_triangle_points, right_line_triangle_points;
  const auto last_msg = get_last_msg();
  if (!last_msg) return drivable_area_lines;

  const auto & left_line = last_msg.value()->left_bound;
  const auto & right_line = last_msg.value()->right_bound;

  if (left_line.size() < 2 || right_line.size() < 2) return drivable_area_lines;

  // TODO: refactor
  {
    float previous_yaw;
    for (size_t i = 0; i < left_line.size(); ++i) {
      const float yaw = i + 1 == left_line.size() ? previous_yaw
                                                  : std::atan2(
                                                      left_line.at(i + 1).y - left_line.at(i).y,
                                                      left_line.at(i + 1).x - left_line.at(i).x);
      Eigen::Quaternionf quat = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) *
                                Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
                                Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
      Eigen::Vector3f vec_in, vec_out;
      vec_in << 0, -(width / 2.0), 0;
      vec_out = quat * vec_in;
      left_line_triangle_points.append(ros2_to_godot(
        left_line.at(i).x + vec_out.x(), left_line.at(i).y + vec_out.y(),
        left_line.at(i).z + vec_out.z()));
      vec_in << 0, (width / 2.0), 0;
      vec_out = quat * vec_in;
      left_line_triangle_points.append(ros2_to_godot(
        left_line.at(i).x + vec_out.x(), left_line.at(i).y + vec_out.y(),
        left_line.at(i).z + vec_out.z()));
      previous_yaw = yaw;
    }
  }

  {
    float previous_yaw;
    for (size_t i = 0; i < right_line.size(); ++i) {
      const float yaw = i + 1 == right_line.size() ? previous_yaw
                                                   : std::atan2(
                                                       right_line.at(i + 1).y - right_line.at(i).y,
                                                       right_line.at(i + 1).x - right_line.at(i).x);
      Eigen::Quaternionf quat = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) *
                                Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
                                Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
      Eigen::Vector3f vec_in, vec_out;
      vec_in << 0, -(width / 2.0), 0;
      vec_out = quat * vec_in;
      right_line_triangle_points.append(ros2_to_godot(
        right_line.at(i).x + vec_out.x(), right_line.at(i).y + vec_out.y(),
        right_line.at(i).z + vec_out.z()));
      vec_in << 0, (width / 2.0), 0;
      vec_out = quat * vec_in;
      right_line_triangle_points.append(ros2_to_godot(
        right_line.at(i).x + vec_out.x(), right_line.at(i).y + vec_out.y(),
        right_line.at(i).z + vec_out.z()));
      previous_yaw = yaw;
    }
  }

  drivable_area_lines.append(left_line_triangle_points);
  drivable_area_lines.append(right_line_triangle_points);

  return drivable_area_lines;
}
