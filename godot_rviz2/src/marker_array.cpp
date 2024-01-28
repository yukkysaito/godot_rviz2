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

#include "marker_array.hpp"

#include <string>

using Type = visualization_msgs::msg::Marker;

namespace
{
bool has_origin_pose(const Type & marker)
{
  return marker.pose.position.x != 0 || marker.pose.position.y != 0 ||
         marker.pose.position.z != 0 || marker.pose.orientation.x != 0 ||
         marker.pose.orientation.y != 0 || marker.pose.orientation.z != 0 ||
         marker.pose.orientation.w != 0;
}
}  // namespace

void MarkerArray::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_triangle_list"), &MarkerArray::get_triangle_list);
  ClassDB::bind_method(D_METHOD("get_color_spheres"), &MarkerArray::get_color_spheres);
  TOPIC_SUBSCRIBER_BIND_METHODS(MarkerArray);
}

Array MarkerArray::get_triangle_list(const String & ns)
{
  Array triangle_list;

  const auto last_msg = get_last_msg();
  if (!last_msg) return triangle_list;

  for (const auto & marker : last_msg.value()->markers) {
    if (to_std(ns) == marker.ns && marker.type == Type::TRIANGLE_LIST) {
      const auto & points = marker.points;
      for (size_t i = 2; i < points.size(); i += 3) {
        const std::array<Eigen::Vector4f, 3> local_vertices = {
          Eigen::Vector4f{points[i - 2].x, points[i - 2].y, points[i - 2].z, 1},
          Eigen::Vector4f{points[i - 1].x, points[i - 1].y, points[i - 1].z, 1},
          Eigen::Vector4f{points[i].x, points[i].y, points[i].z, 1}};

        std::array<Eigen::Vector4f, 3> vertices;
        if (has_origin_pose(marker)) {
          const auto & pos = marker.pose.position;
          const auto & quat = marker.pose.orientation;
          Eigen::Translation3f translation(pos.x, pos.y, pos.z);
          Eigen::Quaternionf quaternion(quat.w, quat.x, quat.y, quat.z);
          Eigen::Matrix4f transform = (translation * quaternion).matrix();
          std::array<Eigen::Vector4f, 3> global_vertices{
            transform * local_vertices[0], transform * local_vertices[1],
            transform * local_vertices[2]};
          vertices = global_vertices;
        } else {
          vertices = local_vertices;
        }

        const auto normal = cross_product(
          vertices[2].head<3>() - vertices[0].head<3>(),
          vertices[1].head<3>() - vertices[0].head<3>());

        {
          Dictionary triangle_point;
          triangle_point["position"] =
            ros2_to_godot(vertices[0][0], vertices[0][1], vertices[0][2]);
          triangle_point["normal"] = ros2_to_godot(normal[0], normal[1], normal[2]);
          triangle_list.append(triangle_point);
        }
        {
          Dictionary triangle_point;
          triangle_point["position"] =
            ros2_to_godot(vertices[1][0], vertices[1][1], vertices[1][2]);
          triangle_point["normal"] = ros2_to_godot(normal[0], normal[1], normal[2]);
          triangle_list.append(triangle_point);
        }
        {
          Dictionary triangle_point;
          triangle_point["position"] =
            ros2_to_godot(vertices[2][0], vertices[2][1], vertices[2][2]);
          triangle_point["normal"] = ros2_to_godot(normal[0], normal[1], normal[2]);
          triangle_list.append(triangle_point);
        }
      }
    }
  }

  return triangle_list;
}

Array MarkerArray::get_color_spheres(const String & ns)
{
  Array color_spheres;
  const auto last_msg = get_last_msg();
  if (!last_msg) return color_spheres;

  for (const auto & marker : last_msg.value()->markers) {
    if (to_std(ns) == marker.ns && marker.type == Type::SPHERE) {
      Array color_sphere;
      Color color(marker.color.r, marker.color.g, marker.color.b, marker.color.a);
      Vector3 position(ros2_to_godot(marker.pose.position));
      Vector3 rotation;                                              // TODO add rotation
      Vector3 size(marker.scale.x, marker.scale.z, marker.scale.y);  // TODO add rotation
      color_sphere.append(color);
      color_sphere.append(position);
      color_sphere.append(rotation);
      color_sphere.append(size);
      color_spheres.append(color_sphere);
    }
    // TODO implement about SPHERE_LIST
    // else if (s == marker.ns && marker.type == Type::SPHERE_LIST)
    // {
    //   for (const auto &point : marker.points)
    //   {
    //     Array color_sphere;
    //     Color color(marker.color.r, marker.color.g, marker.color.b, marker.color.a);
    //     Vector3 position(marker.pose.position.x, marker.pose.position.z, -1.0 *
    //     marker.pose.position.y); Vector3 rotation; color_sphere.append(color);
    //     color_sphere.append(position);
    //     color_sphere.append(rotation);
    //     color_spheres.append(color_sphere);
    //   }
    // }
  }

  return color_spheres;
}
