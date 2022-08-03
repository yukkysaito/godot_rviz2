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
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <string>

using Type = visualization_msgs::msg::Marker;

void MarkerArray::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_triangle_marker"), &MarkerArray::get_triangle_marker);
  ClassDB::bind_method(D_METHOD("get_color_spheres"), &MarkerArray::get_color_spheres);
  ClassDB::bind_method(D_METHOD("subscribe"), &MarkerArray::subscribe);
  ClassDB::bind_method(D_METHOD("is_new"), &MarkerArray::is_new);
  ClassDB::bind_method(D_METHOD("set_old"), &MarkerArray::set_old);
}

PoolVector3Array MarkerArray::get_triangle_marker(const String &ns)
{
  PoolVector3Array triangle_points;
  if (msg_ptr_ == nullptr)
    return triangle_points;

  std::wstring ws = ns.c_str();
  std::string s(ws.begin(), ws.end());

  for (const auto &marker : msg_ptr_->markers)
  {
    if (s == marker.ns)
    {
      for (const auto &point : marker.points)
      {
        triangle_points.append(Vector3(point.x, point.z, -1.0 * point.y));
      }
    }
  }

  return triangle_points;
}

Array MarkerArray::get_color_spheres(const String &ns){
  Array color_spheres;

  if (msg_ptr_ == nullptr)
    return color_spheres;

  std::wstring ws = ns.c_str();
  std::string s(ws.begin(), ws.end());

  for (const auto &marker : msg_ptr_->markers)
  {
    if (s == marker.ns && marker.type == Type::SPHERE )
    {
        Array color_sphere;
        Color color(marker.color.r, marker.color.g, marker.color.b, marker.color.a);
        Vector3 position(marker.pose.position.x, marker.pose.position.z, -1.0 * marker.pose.position.y);
        Vector3 rotation; // TODO add rotation
        Vector3 size(marker.scale.x, marker.scale.z, marker.scale.y); // TODO add rotation
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
    //     Vector3 position(marker.pose.position.x, marker.pose.position.z, -1.0 * marker.pose.position.y);
    //     Vector3 rotation;
    //     color_sphere.append(color);
    //     color_sphere.append(position);
    //     color_sphere.append(rotation);
    //     color_spheres.append(color_sphere);
    //   }
    // }
  }

  return color_spheres;
}


bool MarkerArray::is_new()
{
  return is_new_;
}

void MarkerArray::set_old()
{
  is_new_ = false;
}

void MarkerArray::subscribe(const String &topic, const bool transient_local)
{
  std::wstring ws = topic.c_str();
  std::string s(ws.begin(), ws.end());
  if (transient_local)
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<visualization_msgs::msg::MarkerArray>(
        s, rclcpp::QoS{1}.transient_local(),
        std::bind(&MarkerArray::on_marker_array, this, std::placeholders::_1));
  else
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<visualization_msgs::msg::MarkerArray>(
        s, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&MarkerArray::on_marker_array, this, std::placeholders::_1));
}

void MarkerArray::on_marker_array(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
{
  msg_ptr_ = msg;
  is_new_ = true;
}

MarkerArray::MarkerArray()
{
  is_new_ = false;
}

MarkerArray::~MarkerArray()
{
}
