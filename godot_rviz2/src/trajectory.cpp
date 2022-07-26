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

#include "trajectory.hpp"
#include <string>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

void Trajectory::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_triangle_strip"), &Trajectory::get_triangle_strip);
  ClassDB::bind_method(D_METHOD("subscribe"), &Trajectory::subscribe);
  ClassDB::bind_method(D_METHOD("is_new"), &Trajectory::is_new);
}

PoolVector3Array Trajectory::get_triangle_strip(const float width)
{
  PoolVector3Array triangle_points;
  if (msg_ptr_ == nullptr)
    return triangle_points;

  for (size_t point_idx = 0; point_idx < msg_ptr_->points.size(); point_idx++)
  {
    const auto &path_pose = msg_ptr_->points.at(point_idx).pose;
    Eigen::Quaternionf quat(
        path_pose.orientation.w, path_pose.orientation.x,
        path_pose.orientation.y, path_pose.orientation.z);
    {
      Eigen::Vector3f vec_in, vec_out;
      vec_in << 0, -(width / 2.0), 0;
      vec_out = quat * vec_in;
      triangle_points.append(Vector3(path_pose.position.x + vec_out.x(),
                                     path_pose.position.z + vec_out.z(),
                                     -1.0 * (path_pose.position.y + vec_out.y())));
    }
    {
      Eigen::Vector3f vec_in, vec_out;
      vec_in << 0, (width / 2.0), 0;
      vec_out = quat * vec_in;
      triangle_points.append(Vector3(path_pose.position.x + vec_out.x(),
                                     path_pose.position.z + vec_out.z(),
                                     -1.0 * (path_pose.position.y + vec_out.y())));
    }
  }


  is_new_ = false;
  return triangle_points;
}

bool Trajectory::is_new()
{
  return is_new_;
}

void Trajectory::subscribe(const String &topic, const bool transient_local)
{
  std::wstring ws = topic.c_str();
  std::string s(ws.begin(), ws.end());
  if (transient_local)
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        s, rclcpp::QoS{1}.transient_local(),
        std::bind(&Trajectory::on_trajectory, this, std::placeholders::_1));
  else
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        s, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&Trajectory::on_trajectory, this, std::placeholders::_1));
}

void Trajectory::on_trajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
	msg_ptr_ = msg;
	is_new_ = true;
}

Trajectory::Trajectory()
{
  is_new_ = false;
}

Trajectory::~Trajectory()
{
}
