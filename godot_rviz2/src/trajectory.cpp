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

void Trajectory::_bind_methods()
{
  ClassDB::bind_method(
    D_METHOD("get_triangle_strip_with_velocity"), &Trajectory::get_triangle_strip_with_velocity);
  TOPIC_SUBSCRIBER_BIND_METHODS(Trajectory);
}

Array Trajectory::get_triangle_strip_with_velocity(const float width)
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
