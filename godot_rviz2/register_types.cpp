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

#include "behavior_path.hpp"
#include "change_operation_mode_client.hpp"
#include "dynamic_objects.hpp"
#include "ego_pose.hpp"
#include "map_projection.hpp"
#include "marker_array.hpp"
#include "navigation_state.hpp"
#include "operation_mode_state.hpp"
#include "parameter.hpp"
#include "pointcloud.hpp"
#include "route.hpp"
#include "spinner.hpp"
#include "steering_report.hpp"
#include "traffic_lights.hpp"
#include "trajectory.hpp"
#include "vector_map.hpp"
#include "vehicle_status.hpp"
#include "velocity_report.hpp"

#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>

namespace
{
void initialize(ModuleInitializationLevel p_level)
{
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }
  GDREGISTER_CLASS(GodotRviz2Spinner);
  GDREGISTER_CLASS(MarkerArray);
  GDREGISTER_CLASS(PointCloud);
  GDREGISTER_CLASS(BehaviorPath);
  GDREGISTER_CLASS(NavigationState);
  GDREGISTER_CLASS(Route);
  GDREGISTER_CLASS(MapProjection);
  GDREGISTER_CLASS(OperationModeState);
  GDREGISTER_CLASS(OperationModeChanger);
  GDREGISTER_CLASS(Trajectory);
  GDREGISTER_CLASS(TrafficLights);
  GDREGISTER_CLASS(DynamicObjects);
  GDREGISTER_CLASS(EgoPose);
  GDREGISTER_CLASS(VehicleStatus);
  GDREGISTER_CLASS(SteeringReport);
  GDREGISTER_CLASS(VelocityReport);
  GDREGISTER_CLASS(VectorMap);
  GDREGISTER_CLASS(Parameter);
}

void uninitialize(ModuleInitializationLevel p_level)
{
  (void)p_level;
}
}  // namespace

// The entry point godot-project/addons/godot_rviz2/godot_rviz2.gdextension names
extern "C" GDExtensionBool GDE_EXPORT godot_rviz2_library_init(
  GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library,
  GDExtensionInitialization * r_initialization)
{
  GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
  init_obj.register_initializer(initialize);
  init_obj.register_terminator(uninitialize);
  init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
  return init_obj.init();
}
