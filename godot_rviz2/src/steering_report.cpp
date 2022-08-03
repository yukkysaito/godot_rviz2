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

#include "steering_report.hpp"
#include <string>


void SteeringReport::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_angle"), &SteeringReport::get_angle);
  ClassDB::bind_method(D_METHOD("subscribe"), &SteeringReport::subscribe);
  ClassDB::bind_method(D_METHOD("is_new"), &SteeringReport::is_new);
  ClassDB::bind_method(D_METHOD("set_old"), &SteeringReport::set_old);
}

double SteeringReport::get_angle()
{
  if (msg_ptr_ == nullptr)
    return 0.0;

  return msg_ptr_->steering_tire_angle;
}

bool SteeringReport::is_new()
{
  return is_new_;
}

void SteeringReport::set_old()
{
  is_new_ = false;
}

void SteeringReport::subscribe(const String &topic, const bool transient_local)
{
  std::wstring ws = topic.c_str();
  std::string s(ws.begin(), ws.end());
  if (transient_local)
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        s, rclcpp::QoS{1}.transient_local(),
        std::bind(&SteeringReport::on_steering_report, this, std::placeholders::_1));
  else
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        s, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&SteeringReport::on_steering_report, this, std::placeholders::_1));
}

void SteeringReport::on_steering_report(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg)
{
  msg_ptr_ = msg;
  is_new_ = true;
}

SteeringReport::SteeringReport()
{
  is_new_ = false;
}

SteeringReport::~SteeringReport()
{
}
