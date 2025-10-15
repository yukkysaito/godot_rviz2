//
//  Copyright 2025 Yukihiro Saito. All rights reserved.
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

#include "change_operation_mode_client.hpp"

#include <chrono>

void OperationModeChanger::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("create_client"), &OperationModeChanger::create_client);
  ClassDB::bind_method(D_METHOD("is_server_ready"), &OperationModeChanger::is_server_ready);
  ClassDB::bind_method(
    D_METHOD("change_to_autonomous_mode"), &OperationModeChanger::change_to_autonomous_mode);
}

bool OperationModeChanger::create_client(const String & service_name)
{
  if (service_name.is_empty()) {
    return false;
  }

  auto node = GodotRviz2::get_instance().get_node();
  if (!node) {
    return false;
  }

  client_ =
    node->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(to_std(service_name));

  return static_cast<bool>(client_);
}

bool OperationModeChanger::is_server_ready()
{
  if (!client_) {
    return false;
  }

  return client_->service_is_ready();
}

bool OperationModeChanger::change_to_autonomous_mode()
{
  if (!client_) {
    return false;
  }

  auto node = GodotRviz2::get_instance().get_node();
  if (!node) {
    return false;
  }

  if (!client_->service_is_ready()) {
    return false;
  }

  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();

  bool success = false;

  auto future = client_->async_send_request(request);

  constexpr std::chrono::seconds timeout(3);
  const auto result = rclcpp::spin_until_future_complete(node, future, timeout);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  auto response = future.get();
  if (!response) {
    return false;
  }

  success = response->status.success;
  return success;
}
