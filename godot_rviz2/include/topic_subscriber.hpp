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

#pragma once

#include "godot_rviz2.hpp"
#include "rclcpp/qos.hpp"
#include "util.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

/**
 * @brief Latest message of a subscription, shared between the ROS executor thread (writer) and
 * the Godot main thread (reader).
 *
 * The subscription callback captures this state by shared_ptr, so a callback that is running
 * while the Godot object is freed never touches freed memory.
 */
template <class MsgT>
class LatestMessage
{
public:
  using ConstSharedPtr = typename MsgT::ConstSharedPtr;

  void set(const ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg_ = msg;
    ++seq_;
  }

  // Returns the latest message and remembers it as "read"
  std::optional<ConstSharedPtr> get()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!msg_) return std::nullopt;
    read_seq_ = seq_;
    return msg_;
  }

  bool has_new()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return seq_ != acked_seq_;
  }

  // Marks the last read message as handled. A message received after it was read stays "new";
  // without a read since the last call, everything received so far is marked handled.
  void set_old()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    acked_seq_ = read_seq_ > acked_seq_ ? read_seq_ : seq_;
  }

private:
  std::mutex mutex_;
  ConstSharedPtr msg_;
  uint64_t seq_ = 0;
  uint64_t read_seq_ = 0;
  uint64_t acked_seq_ = 0;
};

// A macro instead of a template base class because template classes cannot be registered with
// ClassDB (https://godotengine.org/qa/136574/how-to-implement-object-using-template-class).
// Messages are received on the ROS executor thread (see GodotRviz2) and read on the main thread.
#define TOPIC_SUBSCRIBER(CLASS, TYPE)                                                             \
private:                                                                                          \
  using ConstSharedPtr = typename TYPE::ConstSharedPtr;                                           \
  std::shared_ptr<LatestMessage<TYPE>> latest_ = std::make_shared<LatestMessage<TYPE>>();         \
  typename rclcpp::Subscription<TYPE>::SharedPtr subscription_;                                   \
                                                                                                  \
  std::optional<ConstSharedPtr> get_last_msg() { return latest_->get(); }                         \
                                                                                                  \
public:                                                                                           \
  bool has_new() { return latest_->has_new(); }                                                   \
  void set_old() { latest_->set_old(); }                                                          \
                                                                                                  \
  void subscribe(const String & topic, const bool transient_local = false)                        \
  {                                                                                               \
    rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);                                       \
    if (transient_local) qos = rclcpp::QoS{1}.transient_local();                                  \
    auto latest = latest_;                                                                        \
    subscription_ = GodotRviz2::get_instance().create_subscription<TYPE>(                         \
      to_std(topic), qos, [latest](const ConstSharedPtr msg) { latest->set(msg); });              \
  }

#define TOPIC_SUBSCRIBER_BIND_METHODS(TYPE)                      \
  ClassDB::bind_method(D_METHOD("subscribe"), &TYPE::subscribe); \
  ClassDB::bind_method(D_METHOD("has_new"), &TYPE::has_new);     \
  ClassDB::bind_method(D_METHOD("set_old"), &TYPE::set_old)
