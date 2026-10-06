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

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <thread>

/**
 * @class GodotRviz2
 * @brief Singleton that owns the ROS 2 node, the TF2 buffer and the executor thread.
 *
 * Callbacks run on a background executor thread (independent of the frame rate, and large
 * messages such as the point cloud map are deserialized there instead of on the render loop).
 * Subscribers only store the latest message (see topic_subscriber.hpp); Godot objects are never
 * touched from the executor thread.
 */
class GodotRviz2
{
public:  // for singleton
  GodotRviz2(const GodotRviz2 &) = delete;
  GodotRviz2 & operator=(const GodotRviz2 &) = delete;
  GodotRviz2(GodotRviz2 &&) = delete;
  GodotRviz2 & operator=(GodotRviz2 &&) = delete;

  static GodotRviz2 & get_instance()
  {
    static GodotRviz2 instance;
    return instance;
  }

  std::shared_ptr<rclcpp::Node> get_node() { return node_; }

  std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() { return tf_buffer_; }

  /**
   * @brief Creates a subscription served by the executor thread (callbacks may run concurrently
   * with each other and with the Godot main thread).
   */
  template <class MsgT, class CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr create_subscription(
    const std::string & topic, const rclcpp::QoS & qos, CallbackT && callback)
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    return node_->create_subscription<MsgT>(
      topic, qos, std::forward<CallbackT>(callback), options);
  }

  /**
   * @brief Creates a service client served by the executor thread.
   */
  template <class ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr create_client(const std::string & service_name)
  {
    return node_->create_client<ServiceT>(
      service_name, rmw_qos_profile_services_default, callback_group_);
  }

private:
  static constexpr size_t kExecutorThreads = 2;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  GodotRviz2()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("godot_rviz2_node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // The listener spins its own node on its own thread
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), kExecutorThreads);
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  ~GodotRviz2()
  {
    executor_->cancel();
    if (executor_thread_.joinable()) executor_thread_.join();
    rclcpp::shutdown();
  }
};
