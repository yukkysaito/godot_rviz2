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

#include "core/ustring.h"
#include "core/variant.h"
#include "core/reference.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "godot_rviz2.hpp"

class MarkerArray : public Reference
{
	GDCLASS(MarkerArray, Reference);

protected:
	static void _bind_methods();

private:
	rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
	void on_marker_array(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
	visualization_msgs::msg::MarkerArray::ConstSharedPtr msg_ptr_;
	bool is_new_;

public:
	bool is_new();
	PoolVector3Array get_triangle_marker(const String &ns);
	Array get_color_spheres(const String &ns);
	void subscribe(const String &topic, const bool transient_local = false);

	MarkerArray();
	~MarkerArray();
};
