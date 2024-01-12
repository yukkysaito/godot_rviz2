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
#include "core/string/ustring.h"
#include "core/variant/variant.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_auto_perception_msgs/msg/shape.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <optional>
#include <string>

#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

std::optional<geometry_msgs::msg::Transform> get_transform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time);

void to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape,
  geometry_msgs::msg::Polygon & polygon);

inline std::string to_std(const String & godot_s) { return std::string(godot_s.utf8()); }

inline Vector3 ros2_to_godot(const double & x, const double & y, const double & z)
{
  return Vector3(x, z, -y);
}

inline Vector3 ros2_to_godot(const geometry_msgs::msg::Point & p)
{
  return ros2_to_godot(p.x, p.y, p.z);
}

inline Vector3 ros2_to_godot(const geometry_msgs::msg::Vector3 & p)
{
  return ros2_to_godot(p.x, p.y, p.z);
}

Eigen::Vector3f cross_product(const Eigen::Vector3f & a, const Eigen::Vector3f & b);
