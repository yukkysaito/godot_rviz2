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

/**
 * @brief Determines if a 2D polygon is defined in a clockwise manner.
 *
 * @param polygon_2d The 2D polygon to check.
 * @return True if the polygon is clockwise, false otherwise.
 */
bool is_clockwise(const std::vector<Vector2> & polygon_2d);

/**
 * @brief Inverts the order of vertices in a 2D polygon to change its winding.
 *
 * @param polygon_2d The 2D polygon to invert.
 * @return A new polygon with vertices in the inverse order.
 */
std::vector<Vector2> inverse_clockwise(const std::vector<Vector2> & polygon_2d);

/**
 * @brief Creates a dictionary representing a point.
 *
 * This method calculates the rotated offset and normal for a given position and quaternion,
 * and then converts these to a Godot-friendly format.
 *
 * @param quat Quaternion representing the orientation
 * @param position Position vector
 * @param width_offset Offset applied to the width
 * @return A Dictionary containing position and normal
 */
Dictionary create_offset_point_dict(
  const Eigen::Quaternionf & quat, const Eigen::Vector3f & position, const float width_offset);

/**
 * TODO
 */
Array calculate_line_as_triangle_strip(
  const std::vector<geometry_msgs::msg::Point> & line, float width);

/**
 * @brief Retrieves a transformation from the TF2 buffer.
 *
 * @param tf_buffer The TF2 buffer to retrieve the transformation from.
 * @param source_frame_id The source frame ID for the transformation.
 * @param target_frame_id The target frame ID for the transformation.
 * @param time The specific time at which to retrieve the transformation.
 * @return std::optional<geometry_msgs::msg::Transform> The transformation, if found.
 */
std::optional<geometry_msgs::msg::Transform> get_transform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time);

/**
 * @brief Converts a Godot String to a standard string.
 *
 * @param godot_s The Godot String.
 * @return std::string A standard string.
 */
inline std::string to_std(const String & godot_s) { return std::string(godot_s.utf8()); }

/**
 * @brief Converts ROS 2 coordinates to Godot's coordinate system.
 *
 * @param x The x-coordinate in ROS 2.
 * @param y The y-coordinate in ROS 2.
 * @param z The z-coordinate in ROS 2.
 * @return Vector3 The converted Vector3 in Godot's coordinate system.
 */
inline Vector3 ros2_to_godot(const float & x, const float & y, const float & z)
{
  return Vector3(x, z, -y);
}

inline Vector3 ros2_to_godot(const double & x, const double & y, const double & z)
{
  return ros2_to_godot(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

inline Vector3 ros2_to_godot(const geometry_msgs::msg::Point & p)
{
  return ros2_to_godot(p.x, p.y, p.z);
}

inline Vector3 ros2_to_godot(const geometry_msgs::msg::Vector3 & p)
{
  return ros2_to_godot(p.x, p.y, p.z);
}

inline Vector3 ros2_to_godot(const Eigen::Vector3f & p)
{
  return ros2_to_godot(p.x(), p.y(), p.z());
}

/**
 * @brief Calculates the cross product of two Eigen::Vector3f vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return Eigen::Vector3f The cross product.
 */
Eigen::Vector3f cross_product(const Eigen::Vector3f & a, const Eigen::Vector3f & b);

/**
 * @brief Generates a 3D bounding box.
 *
 * @param width The width of the bounding box.
 * @param height The height of the bounding box.
 * @param length The length of the bounding box.
 * @param translation The translation component of the bounding box's transformation.
 * @param quaternion The rotation component of the bounding box's transformation.
 * @param vertices Output vector of vertices defining the bounding box.
 * @param normals Output vector of normals for each vertex.
 */
void generate_boundingbox3d(
  float width, float length, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals);

/**
 * @brief Generates a 3D cylinder.
 *
 * @param radius The radius of the cylinder.
 * @param height The height of the cylinder.
 * @param translation The translation component of the cylinder's transformation.
 * @param quaternion The rotation component of the cylinder's transformation.
 * @param vertices Output vector of vertices defining the cylinder.
 * @param normals Output vector of normals for each vertex.
 */
void generate_cylinder3d(
  float radius, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals);

/**
 * @brief Generates a 3D polygon from a 2D polygon and a height.
 *
 * @param polygon_2d The 2D polygon to extrude.
 * @param height The height of the extrusion.
 * @param translation The translation component of the polygon's transformation.
 * @param quaternion The rotation component of the polygon's transformation.
 * @param vertices Output vector of vertices defining the 3D polygon.
 * @param normals Output vector of normals for each vertex.
 */
void generate_polygon3d(
  const geometry_msgs::msg::Polygon & polygon_2d, float height,
  const Eigen::Translation3f & translation, const Eigen::Quaternionf & quaternion,
  std::vector<Vector3> & vertices, std::vector<Vector3> & normals);

/**
 * @brief Overload of generate_polygon3d for a vector of Vector2.
 *
 * @param polygon_2d The 2D polygon defined by a vector of Vector2 to extrude.
 * @param height The height of the extrusion.
 * @param translation The translation component of the polygon's transformation.
 * @param quaternion The rotation component of the polygon's transformation.
 * @param vertices Output vector of vertices defining the 3D polygon.
 * @param normals Output vector of normals for each vertex.
 */
void generate_polygon3d(
  const std::vector<Vector2> & polygon_2d, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals);
