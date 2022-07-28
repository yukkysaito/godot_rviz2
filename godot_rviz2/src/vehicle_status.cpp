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

#include "vehicle_status.hpp"
#include <string>

void VehicleStatus::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("is_turn_on_right"), &VehicleStatus::is_turn_on_right);
  ClassDB::bind_method(D_METHOD("is_turn_on_left"), &VehicleStatus::is_turn_on_left);
  ClassDB::bind_method(D_METHOD("subscribe"), &VehicleStatus::subscribe);
  ClassDB::bind_method(D_METHOD("is_new"), &VehicleStatus::is_new);
}

bool VehicleStatus::is_turn_on_right()
{
  if (msg_ptr_ == nullptr)
    return false;

  is_new_ = false;
  return msg_ptr_->report == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
}

bool VehicleStatus::is_turn_on_left()
{
  if (msg_ptr_ == nullptr)
    return false;

  is_new_ = false;
  return msg_ptr_->report == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
}

bool VehicleStatus::is_new()
{
  return is_new_;
}

void VehicleStatus::subscribe(const String &topic, const bool transient_local)
{
  std::wstring ws = topic.c_str();
  std::string s(ws.begin(), ws.end());
  if (transient_local)
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
        s, rclcpp::QoS{1}.transient_local(),
        std::bind(&VehicleStatus::on_turn_indicators, this, std::placeholders::_1));
  else
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
        s, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&VehicleStatus::on_turn_indicators, this, std::placeholders::_1));
}

void VehicleStatus::on_turn_indicators(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr msg)
{
  msg_ptr_ = msg;
  is_new_ = true;
}

VehicleStatus::VehicleStatus()
{
  is_new_ = false;
}

VehicleStatus::~VehicleStatus()
{
}
