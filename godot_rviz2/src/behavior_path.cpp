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
  ClassDB::bind_method(
    D_METHOD("get_triangle_strip_with_velocity"), &BehaviorPath::get_triangle_strip_with_velocity);
  ClassDB::bind_method(
    D_METHOD("get_drivable_area_triangle_strips"),
    &BehaviorPath::get_drivable_area_triangle_strips);
  TOPIC_SUBSCRIBER_BIND_METHODS(BehaviorPath);
}

Array BehaviorPath::get_triangle_strip_with_velocity(const float width)
{
  Array triangle_strip_with_velocity;
  PoolVector3Array triangle_points;
  const auto last_msg = get_last_msg();
  if (!last_msg) return triangle_strip_with_velocity;

  for (const auto & point : last_msg.value()->points) {
    const auto & path_pose = point.pose;
    Eigen::Quaternionf quat(
      path_pose.orientation.w, path_pose.orientation.x, path_pose.orientation.y,
      path_pose.orientation.z);
    {
      Eigen::Vector3f vec_in, vec_out;
      vec_in << 0, -(width / 2.0), 0;
      vec_out = quat * vec_in;
      Array point_with_velocity;
      point_with_velocity.append(point.longitudinal_velocity_mps);
      point_with_velocity.append(ros2_to_godot(
        path_pose.position.x + vec_out.x(), path_pose.position.y + vec_out.y(),
        path_pose.position.z + vec_out.z()));
      triangle_strip_with_velocity.append(point_with_velocity);
    }
    {
      Eigen::Vector3f vec_in, vec_out;
      vec_in << 0, (width / 2.0), 0;
      vec_out = quat * vec_in;
      Array point_with_velocity;
      point_with_velocity.append(point.longitudinal_velocity_mps);
      point_with_velocity.append(ros2_to_godot(
        path_pose.position.x + vec_out.x(), path_pose.position.y + vec_out.y(),
        path_pose.position.z + vec_out.z()));
      triangle_strip_with_velocity.append(point_with_velocity);
    }
  }

  return triangle_strip_with_velocity;
}

Array BehaviorPath::get_drivable_area_triangle_strips(const float width)
{
  Array drivable_area_lines;
  PoolVector3Array left_line_triangle_points, right_line_triangle_points;
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
