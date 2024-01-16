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

#include "visualization_msgs/msg/marker_array.hpp"

class MarkerArray : public RefCounted
{
  GDCLASS(MarkerArray, RefCounted);
  TOPIC_SUBSCRIBER(MarkerArray, visualization_msgs::msg::MarkerArray);

public:
  PackedVector3Array get_triangle_marker(const String & ns);
  Array get_color_spheres(const String & ns);

  MarkerArray() = default;
  ~MarkerArray() = default;

protected:
  static void _bind_methods();
};
