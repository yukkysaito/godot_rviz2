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

#include "dynamic_objects.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

void DynamicObjects::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_triangle_list"), &DynamicObjects::get_triangle_list);
  TOPIC_SUBSCRIBER_BIND_METHODS(DynamicObjects);
}

Array DynamicObjects::get_triangle_list(bool only_known_objects)
{
  Array triangle_list;

  const auto last_msg = get_last_msg();
  if (!last_msg) return triangle_list;

  for (const auto & object : last_msg.value()->objects) {
    if (only_known_objects && object.classification.front().label == Label::UNKNOWN) continue;
    const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto & quat = object.kinematics.initial_pose_with_covariance.pose.orientation;
    const auto & shape = object.shape;

    Eigen::Translation3f translation(pos.x, pos.y, pos.z);
    Eigen::Quaternionf quaternion(quat.w, quat.x, quat.y, quat.z);
    std::vector<Vector3> vertices, normals;

    if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
      generate_boundingbox3d(
        shape.dimensions.x, shape.dimensions.y, shape.dimensions.z, translation, quaternion,
        vertices, normals);
    } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
      generate_cylinder3d(
        shape.dimensions.x / 2, shape.dimensions.z, translation, quaternion, vertices, normals);
    } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
      generate_polygon3d(
        shape.footprint, shape.dimensions.z, translation, quaternion, vertices, normals);
    }
    for (size_t i = 0; i < vertices.size(); ++i) {
      Dictionary point_dict;
      point_dict["position"] = ros2_to_godot(vertices[i].x, vertices[i].y, vertices[i].z);
      point_dict["normal"] = ros2_to_godot(normals[i].x, normals[i].y, normals[i].z);
      triangle_list.append(point_dict);
    }
  }

  return triangle_list;
}
