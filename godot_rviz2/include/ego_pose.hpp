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
#include "util.hpp"
#include "core/reference.h"
#include "godot_rviz2.hpp"


#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class EgoPose : public Reference
{
    GDCLASS(EgoPose, Reference);

public:
    EgoPose(){};
    ~EgoPose(){};
    Vector3 get_ego_position()
    {
        const auto tf_buffer = GodotRviz2::get_instance().get_tf_buffer();
        const auto transform = get_transform(*tf_buffer, "base_link", "map", rclcpp::Time(0));

        return Vector3(transform.value().translation.x, transform.value().translation.z, -1.0 * transform.value().translation.y);        
    }

    Vector3 get_ego_rotation()
    {
        const auto tf_buffer = GodotRviz2::get_instance().get_tf_buffer();
        const auto transform = get_transform(*tf_buffer, "base_link", "map", rclcpp::Time(0));

        const auto &rotation = transform.value().rotation;
        double roll, pitch, yaw;

        tf2::Quaternion quatanion(rotation.x, rotation.y, rotation.z, rotation.w);
        tf2::Matrix3x3(quatanion).getRPY(roll, pitch, yaw);
        return Vector3(roll, yaw, -1.0 * pitch);
    }

protected:
    static void _bind_methods();
};
