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
#include <string>

void PointCloud::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_pointcloud"), &PointCloud::get_pointcloud);
  ClassDB::bind_method(D_METHOD("subscribe"), &PointCloud::subscribe);
  ClassDB::bind_method(D_METHOD("is_new"), &PointCloud::is_new);
}

PoolVector3Array PointCloud::get_pointcloud()
{
  PoolVector3Array pointcloud;
  if (msg_ptr_ == nullptr)
    return pointcloud;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg_ptr_, "x"),
      iter_y(*msg_ptr_, "y"), iter_z(*msg_ptr_, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    pointcloud.append(Vector3(*iter_x, *iter_z, -1.0 * (*iter_y)));
  }
  is_new_ = false;

  return pointcloud;
}

bool PointCloud::is_new()
{
  return is_new_;
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
