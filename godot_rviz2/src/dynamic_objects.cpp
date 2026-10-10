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

#include <cmath>
#include <string>
#include <vector>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using Label = autoware_perception_msgs::msg::ObjectClassification;

void DynamicObjects::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_triangle_list"), &DynamicObjects::get_triangle_list);
  ClassDB::bind_method(
    D_METHOD("get_dynamic_object_list"), &DynamicObjects::get_dynamic_object_list);
  ClassDB::bind_method(
    D_METHOD("get_predicted_paths", "width", "min_confidence", "ignore_unknown_object"),
    &DynamicObjects::get_predicted_paths);
  ClassDB::bind_method(
    D_METHOD("get_unknown_object_triangle_list"),
    &DynamicObjects::get_unknown_object_triangle_list);
  TOPIC_SUBSCRIBER_BIND_METHODS(DynamicObjects);
}

namespace
{
// Label with the highest probability (the classification list is not sorted by probability).
// UNKNOWN when the list is empty.
uint8_t main_label(const autoware_perception_msgs::msg::PredictedObject & object)
{
  uint8_t label = Label::UNKNOWN;
  float best = -1.0f;
  for (const auto & classification : object.classification) {
    if (classification.probability > best) {
      best = classification.probability;
      label = classification.label;
    }
  }
  return label;
}

String to_uuid_string(const autoware_perception_msgs::msg::PredictedObject & object)
{
  static const char * hex = "0123456789abcdef";
  std::string id;
  id.reserve(object.object_id.uuid.size() * 2);
  for (const auto byte : object.object_id.uuid) {
    id.push_back(hex[byte >> 4]);
    id.push_back(hex[byte & 0x0F]);
  }
  return String(id.c_str());
}
}  // namespace

Array DynamicObjects::get_triangle_list(bool ignore_unknown_object)
{
  Array triangle_list;

  const auto last_msg = get_last_msg();
  if (!last_msg) return triangle_list;

  for (const auto & object : last_msg.value()->objects) {
    if (ignore_unknown_object && main_label(object) == Label::UNKNOWN) continue;
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

Array DynamicObjects::get_unknown_object_triangle_list()
{
  Array triangle_list;

  const auto last_msg = get_last_msg();
  if (!last_msg) return triangle_list;

  for (const auto & object : last_msg.value()->objects) {
    if (main_label(object) != Label::UNKNOWN) continue;
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

Dictionary DynamicObjects::get_predicted_paths(
  double width, double min_confidence, bool ignore_unknown_object)
{
  PackedVector3Array vertices;
  PackedVector2Array uvs;
  PackedColorArray colors;
  Dictionary result;
  result["vertices"] = vertices;
  result["uvs"] = uvs;
  result["colors"] = colors;

  const auto last_msg = get_last_msg();
  if (!last_msg) return result;

  const double half = width / 2.0;
  for (const auto & object : last_msg.value()->objects) {
    if (ignore_unknown_object && main_label(object) == Label::UNKNOWN) continue;
    for (const auto & predicted : object.kinematics.predicted_paths) {
      const auto & path = predicted.path;
      if (path.size() < 2 || predicted.confidence < min_confidence) continue;
      const Color color(1.0f, 1.0f, 1.0f, predicted.confidence);
      // Left / right edge points (ROS coordinates) from each pose's heading
      std::vector<Vector3> left, right;
      for (const auto & pose : path) {
        const auto & q = pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const Vector3 offset(-std::sin(yaw) * half, std::cos(yaw) * half, 0.0);
        const Vector3 center(pose.position.x, pose.position.y, pose.position.z);
        left.push_back(center + offset);
        right.push_back(center - offset);
      }
      const float last = float(path.size() - 1);
      for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Vector3 quad[4] = {left[i], right[i], left[i + 1], right[i + 1]};
        const Vector2 quad_uv[4] = {
          Vector2(0, i / last), Vector2(1, i / last), Vector2(0, (i + 1) / last),
          Vector2(1, (i + 1) / last)};
        for (const int k : {0, 1, 2, 2, 1, 3}) {
          vertices.push_back(ros2_to_godot(quad[k].x, quad[k].y, quad[k].z));
          uvs.push_back(quad_uv[k]);
          colors.push_back(color);
        }
      }
    }
  }
  result["vertices"] = vertices;
  result["uvs"] = uvs;
  result["colors"] = colors;
  return result;
}

Array DynamicObjects::get_dynamic_object_list(bool ignore_unknown_object)
{
  Array dynamic_object_list;

  const auto last_msg = get_last_msg();
  if (!last_msg) return dynamic_object_list;

  for (const auto & object : last_msg.value()->objects) {
    if (ignore_unknown_object && main_label(object) == Label::UNKNOWN) continue;
    const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto & quat = object.kinematics.initial_pose_with_covariance.pose.orientation;
    const auto & shape = object.shape;

    double roll, pitch, yaw;
    // Convert the quaternion to roll, pitch, yaw
    tf2::Quaternion quaternion(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    const uint8_t label = main_label(object);
    Dictionary dynamic_object;
    dynamic_object["id"] = to_uuid_string(object);
    dynamic_object["position"] = ros2_to_godot(pos.x, pos.y, pos.z);
    dynamic_object["rotation"] = ros2_to_godot(roll, pitch, yaw);
    dynamic_object["size"] =
      ros2_to_godot(shape.dimensions.x, shape.dimensions.y, shape.dimensions.z);
    dynamic_object["velocity"] = ros2_to_godot(velocity.x, velocity.y, velocity.z);
    if (label == Label::PEDESTRIAN) {
      dynamic_object["class"] = "pedestrian";
    } else if (label == Label::BICYCLE) {
      dynamic_object["class"] = "bicycle";
    } else if (label == Label::CAR) {
      dynamic_object["class"] = "car";
    } else if (label == Label::TRUCK) {
      dynamic_object["class"] = "truck";
    } else if (label == Label::MOTORCYCLE) {
      dynamic_object["class"] = "motorcycle";
    } else if (label == Label::BUS) {
      dynamic_object["class"] = "bus";
    } else if (label == Label::TRAILER) {
      dynamic_object["class"] = "trailer";
    } else {
      dynamic_object["class"] = "unknown";
    }
    dynamic_object_list.append(dynamic_object);
  }

  return dynamic_object_list;
}
