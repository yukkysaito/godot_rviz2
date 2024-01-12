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
#include "core/string/ustring.h"
#include "core/variant/variant.h"
#include "topic_subscriber.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloud : public RefCounted
{
  GDCLASS(PointCloud, RefCounted);
  TOPIC_SUBSCRIBER(PointCloud, sensor_msgs::msg::PointCloud2);

public:
  PackedVector3Array get_pointcloud(const String & frame_id = "map");

  PointCloud() = default;
  ~PointCloud() = default;

protected:
  static void _bind_methods();
};
