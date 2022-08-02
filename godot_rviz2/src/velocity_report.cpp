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

#include "velocity_report.hpp"
#include <string>


void VelocityReport::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_velocity"), &VelocityReport::get_velocity);
  ClassDB::bind_method(D_METHOD("subscribe"), &VelocityReport::subscribe);
  ClassDB::bind_method(D_METHOD("is_new"), &VelocityReport::is_new);
}

double VelocityReport::get_velocity()
{
  if (msg_ptr_ == nullptr)
    return 0.0;


  is_new_ = false;
  return msg_ptr_->longitudinal_velocity;
}

bool VelocityReport::is_new()
{
  return is_new_;
}

void VelocityReport::subscribe(const String &topic, const bool transient_local)
{
  std::wstring ws = topic.c_str();
  std::string s(ws.begin(), ws.end());
  if (transient_local)
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        s, rclcpp::QoS{1}.transient_local(),
        std::bind(&VelocityReport::on_velocity_report, this, std::placeholders::_1));
  else
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        s, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&VelocityReport::on_velocity_report, this, std::placeholders::_1));
}

void VelocityReport::on_velocity_report(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
{
  msg_ptr_ = msg;
  is_new_ = true;
}

VelocityReport::VelocityReport()
{
  is_new_ = false;
}

VelocityReport::~VelocityReport()
{
}
