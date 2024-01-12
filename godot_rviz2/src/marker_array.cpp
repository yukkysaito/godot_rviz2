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

void MarkerArray::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_triangle_marker"), &MarkerArray::get_triangle_marker);
  ClassDB::bind_method(D_METHOD("get_color_spheres"), &MarkerArray::get_color_spheres);
  TOPIC_SUBSCRIBER_BIND_METHODS(MarkerArray);
}

PackedVector3Array MarkerArray::get_triangle_marker(const String & ns)
{
  PackedVector3Array triangle_points;
  const auto last_msg = get_last_msg();
  if (!last_msg) return triangle_points;

  for (const auto & marker : last_msg.value()->markers) {
    if (to_std(ns) == marker.ns) {
      for (const auto & point : marker.points) {
        triangle_points.append(Vector3(point.x, point.z, -1.0 * point.y));
      }
    }
  }

  return triangle_points;
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
