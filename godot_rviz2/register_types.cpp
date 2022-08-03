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

#include "register_types.h"
#include "core/class_db.h"
#include "spinner.hpp"
#include "ego_pose.hpp"
#include "pointcloud.hpp"
#include "marker_array.hpp"
#include "trajectory.hpp"
#include "vehicle_status.hpp"
#include "dynamic_objects.hpp"
#include "steering_report.hpp"
#include "velocity_report.hpp"
#include "parameter.hpp"

void register_godot_rviz2_types()
{
	ClassDB::register_class<GodotRviz2Spinner>();
	ClassDB::register_class<MarkerArray>();
	ClassDB::register_class<PointCloud>();
	ClassDB::register_class<Trajectory>();
	ClassDB::register_class<DynamicObjects>();
	ClassDB::register_class<EgoPose>();
	ClassDB::register_class<VehicleStatus>();
	ClassDB::register_class<SteeringReport>();
	ClassDB::register_class<VelocityReport>();
	ClassDB::register_class<Parameter>();
}

void unregister_godot_rviz2_types()
{
}
