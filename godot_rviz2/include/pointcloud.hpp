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

#include "Godot.hpp"
#include "Reference.hpp"
#include "String.hpp"
#include "Variant.hpp"
#include "topic_subscriber.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloud : public godot::Reference
{
  GODOT_CLASS(PointCloud, godot::Reference);
  TOPIC_SUBSCRIBER(PointCloud, sensor_msgs::msg::PointCloud2);

public:
  godot::PoolVector3Array get_pointcloud(const godot::String & frame_id = "map");
  static void _register_methods();

  PointCloud() = default;
  ~PointCloud() = default;
};
