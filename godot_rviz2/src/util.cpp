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

//
// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "util.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

std::optional<geometry_msgs::msg::Transform> get_transform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    if (!tf_buffer.canTransform(
          target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5)))
      return std::nullopt;
    transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
    return transform_stamped.transform;
  } catch (...) {
    return std::nullopt;
  }
}

geometry_msgs::msg::Polygon rotate_polygon(
  const geometry_msgs::msg::Polygon & polygon, const double angle)
{
  const double cos = std::cos(angle);
  const double sin = std::sin(angle);
  geometry_msgs::msg::Polygon rotated_polygon;
  for (const auto & point : polygon.points) {
    auto rotated_point = point;
    rotated_point.x = cos * point.x - sin * point.y;
    rotated_point.y = sin * point.x + cos * point.y;
    rotated_polygon.points.push_back(rotated_point);
  }
  return rotated_polygon;
}

geometry_msgs::msg::Polygon inverse_clock_wise(const geometry_msgs::msg::Polygon & polygon)
{
  geometry_msgs::msg::Polygon inverted_polygon;
  for (int i = polygon.points.size() - 1; 0 <= i; --i) {
    inverted_polygon.points.push_back(polygon.points.at(i));
  }
  return inverted_polygon;
}

bool is_clock_wise(const geometry_msgs::msg::Polygon & polygon)
{
  const int n = polygon.points.size();
  const double x_offset = polygon.points.at(0).x;
  const double y_offset = polygon.points.at(0).y;
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.points.size(); ++i) {
    sum += (polygon.points.at(i).x - x_offset) * (polygon.points.at((i + 1) % n).y - y_offset) -
           (polygon.points.at(i).y - y_offset) * (polygon.points.at((i + 1) % n).x - x_offset);
  }

  return sum < 0.0;
}

void to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape,
  geometry_msgs::msg::Polygon & polygon)
{
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const double yaw = tf2::getYaw(pose.orientation);
    Eigen::Matrix2d rotation;
    rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    {
      Eigen::Vector2d offset =
        rotation * Eigen::Vector2d(shape.dimensions.x * 0.5f, shape.dimensions.y * 0.5f);
      geometry_msgs::msg::Point32 point;
      point.x = pose.position.x + offset.x();
      point.y = pose.position.y + offset.y();
      point.z = pose.position.z;
      polygon.points.push_back(point);
    }
    {
      Eigen::Vector2d offset =
        rotation * Eigen::Vector2d(shape.dimensions.x * 0.5f, -shape.dimensions.y * 0.5f);
      geometry_msgs::msg::Point32 point;
      point.x = pose.position.x + offset.x();
      point.y = pose.position.y + offset.y();
      point.z = pose.position.z;
      polygon.points.push_back(point);
    }
    {
      Eigen::Vector2d offset =
        rotation * Eigen::Vector2d(-shape.dimensions.x * 0.5f, -shape.dimensions.y * 0.5f);
      geometry_msgs::msg::Point32 point;
      point.x = pose.position.x + offset.x();
      point.y = pose.position.y + offset.y();
      point.z = pose.position.z;
      polygon.points.push_back(point);
    }
    {
      Eigen::Vector2d offset =
        rotation * Eigen::Vector2d(-shape.dimensions.x * 0.5f, shape.dimensions.y * 0.5f);
      geometry_msgs::msg::Point32 point;
      point.x = pose.position.x + offset.x();
      point.y = pose.position.y + offset.y();
      point.z = pose.position.z;
      polygon.points.push_back(point);
    }
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    const auto & center = pose.position;
    const auto & radius = shape.dimensions.x * 0.5;
    constexpr int n = 12;
    for (int i = 0; i < n; ++i) {
      geometry_msgs::msg::Point32 point;

      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                  radius +
                center.x;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                  radius +
                center.y;
      point.z = pose.position.z;
      polygon.points.push_back(point);
    }
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const double yaw = tf2::getYaw(pose.orientation);
    const auto rotated_footprint = rotate_polygon(shape.footprint, yaw);
    for (const auto & footprint_point : rotated_footprint.points) {
      geometry_msgs::msg::Point32 point;

      point.x = pose.position.x + footprint_point.x;
      point.y = pose.position.y + footprint_point.y;
      point.z = pose.position.z;

      polygon.points.push_back(point);
    }
  }
  polygon = is_clock_wise(polygon) ? polygon : inverse_clock_wise(polygon);
}

Eigen::Vector3f cross_product(const Eigen::Vector3f & a, const Eigen::Vector3f & b)
{
  return Eigen::Vector3f(
    a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(), a.x() * b.y() - a.y() * b.x());
}
