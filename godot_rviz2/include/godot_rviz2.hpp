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

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * @class GodotRviz2
 * @brief The GodotRviz2 class is a singleton that initializes and manages ROS 2 node and TF2 buffer
 * for Godot integration.
 *
 * This class is responsible for setting up the ROS 2 environment, creating a node, and managing the
 * TF2 buffer and listener for the Godot engine to interact with ROS 2 applications. It follows the
 * singleton pattern to ensure only one instance of the class exists.
 */
class GodotRviz2
{
public:  // for singleton
  GodotRviz2(const GodotRviz2 &) = delete;
  GodotRviz2 & operator=(const GodotRviz2 &) = default;
  GodotRviz2(GodotRviz2 &&) = delete;
  GodotRviz2 & operator=(GodotRviz2 &&) = delete;

  /**
   * @brief Gets the singleton instance of the GodotRviz2 class.
   *
   * @return GodotRviz2& Reference to the singleton instance.
   */
  static GodotRviz2 & get_instance()
  {
    static GodotRviz2 instance;
    return instance;
  }

  /**
   * @brief Gets the ROS 2 node.
   *
   * @return std::shared_ptr<rclcpp::Node> Shared pointer to the ROS 2 node.
   */
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }

  /**
   * @brief Gets the TF2 buffer.
   *
   * @return std::shared_ptr<tf2_ros::Buffer> Shared pointer to the TF2 buffer.
   */
  std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() { return tf_buffer_; }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

private:
  /**
   * @brief Constructor for GodotRviz2.
   *
   * Initializes the ROS 2 environment and sets up the node, TF2 buffer, and listener.
   */
  GodotRviz2()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("godot_rviz2_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /**
   * @brief Destructor for GodotRviz2.
   *
   * Shuts down the ROS 2 environment upon object destruction.
   */
  ~GodotRviz2() { rclcpp::shutdown(); }
};
