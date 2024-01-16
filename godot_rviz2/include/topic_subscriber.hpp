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

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <optional>

// Because template class cannot work to bind_methods and register_class.
// https://godotengine.org/qa/136574/how-to-implement-object-using-template-class
#if 1
#define TOPIC_SUBSCRIBER(CLASS, TYPE)                                                   \
private:                                                                                \
  using ConstSharedPtr = typename TYPE::ConstSharedPtr;                                 \
  ConstSharedPtr msg_ptr_;                                                              \
  bool has_new_ = false;                                                                \
  typename rclcpp::Subscription<TYPE>::SharedPtr subscription_;                         \
                                                                                        \
  void on_callback(const ConstSharedPtr msg)                                            \
  {                                                                                     \
    msg_ptr_ = msg;                                                                     \
    has_new_ = true;                                                                    \
  }                                                                                     \
  std::optional<ConstSharedPtr> get_last_msg()                                          \
  {                                                                                     \
    if (!msg_ptr_) return std::nullopt;                                                 \
    return msg_ptr_;                                                                    \
  }                                                                                     \
                                                                                        \
public:                                                                                 \
  bool has_new() { return has_new_; }                                                   \
  void set_old() { has_new_ = false; }                                                  \
                                                                                        \
  void subscribe(const String & topic, const bool transient_local = false)              \
  {                                                                                     \
    rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);                             \
    if (transient_local) qos = rclcpp::QoS{1}.transient_local();                        \
    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<TYPE>(   \
      to_std(topic), qos, std::bind(&CLASS::on_callback, this, std::placeholders::_1)); \
  }

#define TOPIC_SUBSCRIBER_BIND_METHODS(TYPE)                      \
  ClassDB::bind_method(D_METHOD("subscribe"), &TYPE::subscribe); \
  ClassDB::bind_method(D_METHOD("has_new"), &TYPE::has_new);     \
  ClassDB::bind_method(D_METHOD("set_old"), &TYPE::set_old)

#else
#include "core/object/ref_counted.h"
#include "core/string/ustring.h"
#include "core/variant/variant.h"

template <class T>
class TopicSubscriber : public RefCounted
{
  GDCLASS(TopicSubscriber<T>, RefCounted);

private:
  using ConstSharedPtr = typename T::ConstSharedPtr;

  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  void on_callback(const ConstSharedPtr msg)
  {
    msg_ptr_ = msg;
    is_new_ = true;
  }

  ConstSharedPtr msg_ptr_;
  bool is_new_;

public:
  bool is_new() { return is_new_; }
  void set_old() { is_new_ = false; }
  std::optional<ConstSharedPtr> get_last_msg()
  {
    if (!msg_ptr_) {
      return std::nullopt;
    }
    return msg_ptr_;
  }

  void subscribe(const String & topic, const bool transient_local = false)
  {
    rclcpp::QoS & qos = rclcpp::SensorDataQoS().keep_last(1);
    if (transient_local) {
      qos = rclcpp::QoS{1}.transient_local();
    }

    subscription_ = GodotRviz2::get_instance().get_node()->create_subscription<T>(
      to_std(topic), qos, std::bind(&TopicSubscriber::on_callback, this, std::placeholders::_1));
  }

  TopicSubscriber() { is_new_ = false; }
  virtual ~TopicSubscriber() = default;

protected:
  static void _bind_methods()
  {
    ClassDB::bind_method(D_METHOD("subscribe"), &TopicSubscriber<T>::subscribe);
    ClassDB::bind_method(D_METHOD("is_new"), &TopicSubscriber<T>::is_new);
    ClassDB::bind_method(D_METHOD("set_old"), &TopicSubscriber<T>::set_old);
  }
};
#endif
