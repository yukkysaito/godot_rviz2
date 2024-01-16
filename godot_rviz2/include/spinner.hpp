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
#include "core/object/ref_counted.h"
#include "godot_rviz2.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class GodotRviz2Spinner
 * @brief The GodotRviz2Spinner class is responsible for handling ROS 2 node spinning within the
 * Godot environment.
 *
 * This class provides a method to spin the ROS 2 node, allowing callbacks to be processed. It is
 * designed to integrate ROS 2 node activity within a Godot application.
 */
class GodotRviz2Spinner : public RefCounted
{
  GDCLASS(GodotRviz2Spinner, RefCounted);

public:
  GodotRviz2Spinner(){};
  ~GodotRviz2Spinner(){};

  /**
   * @brief Spins the ROS 2 node to process callbacks.
   *
   * This method invokes rclcpp::spin_some, allowing the ROS 2 node to handle incoming messages and
   * service requests.
   */
  inline void spin_some() { rclcpp::spin_some(GodotRviz2::get_instance().get_node()); }

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();
};
