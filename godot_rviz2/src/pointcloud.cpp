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
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include <string>

void PointCloud::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_pointcloud"), &PointCloud::get_pointcloud);
  ClassDB::bind_method(D_METHOD("subscribe"), &PointCloud::subscribe);
  ClassDB::bind_method(D_METHOD("is_new"), &PointCloud::is_new);
  ClassDB::bind_method(D_METHOD("set_old"), &PointCloud::set_old);
}

bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  geometry_msgs::msg::TransformStamped tf_stamped{};
  try {
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("godot_rviz2"), clock, 5000, "%s", ex.what());
    return false;
  }
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

PoolVector3Array PointCloud::get_pointcloud(const String &frame_id)
{
  PoolVector3Array pointcloud;
  if (msg_ptr_ == nullptr)
    return pointcloud;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_ptr;
  msg_ptr = msg_ptr_;

  // Transform
  std::shared_ptr<sensor_msgs::msg::PointCloud2> transformed_msg_ptr;
  transformed_msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
  const auto tf_buffer = GodotRviz2::get_instance().get_tf_buffer();
  if (godot_to_std(frame_id) != msg_ptr_->header.frame_id)
  {
    if (!transformPointcloud(*msg_ptr_, *tf_buffer, godot_to_std(frame_id), *transformed_msg_ptr))
      return pointcloud;
    msg_ptr = transformed_msg_ptr;
  }

  // Convert to godot array
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg_ptr, "x"),
      iter_y(*msg_ptr, "y"), iter_z(*msg_ptr, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    pointcloud.append(Vector3(*iter_x, *iter_z, -1.0 * (*iter_y)));
  }

  return pointcloud;
}

bool PointCloud::is_new()
{
  return is_new_;
}

void PointCloud::set_old()
{
  is_new_ = false;
}

void PointCloud::subscribe(const String &topic, const bool transient_local)
{
  std::wstring ws = topic.c_str();
  std::string s(ws.begin(), ws.end());
  if (transient_local)
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<sensor_msgs::msg::PointCloud2>(
        s, rclcpp::QoS{1}.transient_local(),
        std::bind(&PointCloud::on_pointcloud2, this, std::placeholders::_1));
  else
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<sensor_msgs::msg::PointCloud2>(
        s, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PointCloud::on_pointcloud2, this, std::placeholders::_1));
}

void PointCloud::on_pointcloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  msg_ptr_ = msg;
  is_new_ = true;
}

PointCloud::PointCloud()
{
  is_new_ = false;
}

PointCloud::~PointCloud()
{
}
