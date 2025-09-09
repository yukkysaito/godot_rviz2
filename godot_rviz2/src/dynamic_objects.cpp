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

using Label = autoware_perception_msgs::msg::ObjectClassification;

void DynamicObjects::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_triangle_list"), &DynamicObjects::get_triangle_list);
  ClassDB::bind_method(
    D_METHOD("get_dynamic_object_list"), &DynamicObjects::get_dynamic_object_list);
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

    if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
      generate_boundingbox3d(
        shape.dimensions.x, shape.dimensions.y, shape.dimensions.z, translation, quaternion,
        vertices, normals);
    } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
      generate_cylinder3d(
        shape.dimensions.x / 2, shape.dimensions.z, translation, quaternion, vertices, normals);
    } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
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

Array DynamicObjects::get_dynamic_object_list(bool only_known_objects)
{
  Array dynamic_object_list;

  const auto last_msg = get_last_msg();
  if (!last_msg) return dynamic_object_list;

  for (const auto & object : last_msg.value()->objects) {
    if (only_known_objects && object.classification.front().label == Label::UNKNOWN) continue;
    const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto & quat = object.kinematics.initial_pose_with_covariance.pose.orientation;
    const auto & shape = object.shape;

    double roll, pitch, yaw;
    // Convert the quaternion to roll, pitch, yaw
    tf2::Quaternion quaternion(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    Dictionary dynamic_object;
    dynamic_object["position"] = ros2_to_godot(pos.x, pos.y, pos.z);
    dynamic_object["class"] = object.classification.front().label;
    dynamic_object["rotation"] = ros2_to_godot(roll, pitch, yaw);
    dynamic_object["size"] =
      ros2_to_godot(shape.dimensions.x, shape.dimensions.y, shape.dimensions.z);
    dynamic_object["velocity"] = ros2_to_godot(velocity.x, velocity.y, velocity.z);
    if (object.classification.front().label == Label::PEDESTRIAN) {
      dynamic_object["class"] = "pedestrian";
    } else if (object.classification.front().label == Label::BICYCLE) {
      dynamic_object["class"] = "bicycle";
    } else if (object.classification.front().label == Label::CAR) {
      dynamic_object["class"] = "car";
    } else if (object.classification.front().label == Label::TRUCK) {
      dynamic_object["class"] = "truck";
    } else if (object.classification.front().label == Label::MOTORCYCLE) {
      dynamic_object["class"] = "motorcycle";
    } else if (object.classification.front().label == Label::BUS) {
      dynamic_object["class"] = "bus";
    } else if (object.classification.front().label == Label::TRAILER) {
      dynamic_object["class"] = "trailer";
    } else {
      dynamic_object["class"] = "unknown";
    }
    dynamic_object_list.append(dynamic_object);
  }

  return dynamic_object_list;
}
