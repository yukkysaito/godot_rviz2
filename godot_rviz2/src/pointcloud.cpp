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

#include "pointcloud.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "util.hpp"

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

void PointCloud::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_pointcloud"), &PointCloud::get_pointcloud);
  TOPIC_SUBSCRIBER_BIND_METHODS(PointCloud);
}

bool transform_pointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  geometry_msgs::msg::TransformStamped tf_stamped{};
  try {
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("godot_rviz2"), clock, 5000, "%s", ex.what());
    return false;
  }
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

PackedVector3Array PointCloud::get_pointcloud(const String & frame_id)
{
  PackedVector3Array pointcloud;
  const auto last_msg = get_last_msg();
  if (!last_msg) return pointcloud;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_ptr;
  msg_ptr = last_msg.value();

  // Transform
  std::shared_ptr<sensor_msgs::msg::PointCloud2> transformed_msg_ptr;
  transformed_msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
  const auto tf_buffer = GodotRviz2::get_instance().get_tf_buffer();
  if (to_std(frame_id) != last_msg.value()->header.frame_id) {
    if (!transform_pointcloud(
          *(last_msg.value()), *tf_buffer, to_std(frame_id), *transformed_msg_ptr))
      return pointcloud;
    msg_ptr = transformed_msg_ptr;
  }

  // Convert to godot array
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg_ptr, "x"), iter_y(*msg_ptr, "y"),
    iter_z(*msg_ptr, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    pointcloud.append(ros2_to_godot(*iter_x, *iter_y, *iter_z));
  }

  return pointcloud;
}
