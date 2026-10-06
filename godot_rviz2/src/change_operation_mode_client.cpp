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


void OperationModeChanger::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("create_client"), &OperationModeChanger::create_client);
  ClassDB::bind_method(D_METHOD("is_server_ready"), &OperationModeChanger::is_server_ready);
  ClassDB::bind_method(
    D_METHOD("change_to_autonomous_mode"), &OperationModeChanger::change_to_autonomous_mode);
  ClassDB::bind_method(
    D_METHOD("get_request_state"), &OperationModeChanger::get_request_state);
}

bool OperationModeChanger::create_client(const String & service_name)
{
  if (service_name.is_empty()) {
    return false;
  }

  client_ = GodotRviz2::get_instance().create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    to_std(service_name));
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
  if (!client_ || !client_->service_is_ready()) {
    return false;
  }
  if (request_state_->load() == static_cast<int>(RequestState::Pending)) {
    return false;
  }

  using Service = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  request_state_->store(static_cast<int>(RequestState::Pending));
  auto state = request_state_;
  client_->async_send_request(
    std::make_shared<Service::Request>(), [state](rclcpp::Client<Service>::SharedFuture future) {
      const auto response = future.get();
      const bool success = response && response->status.success;
      state->store(static_cast<int>(success ? RequestState::Succeeded : RequestState::Failed));
    });
  return true;
}

String OperationModeChanger::get_request_state()
{
  switch (static_cast<RequestState>(request_state_->load())) {
    case RequestState::Pending:
      return "pending";
    case RequestState::Succeeded:
      return "succeeded";
    case RequestState::Failed:
      return "failed";
    default:
      return "idle";
  }
}
