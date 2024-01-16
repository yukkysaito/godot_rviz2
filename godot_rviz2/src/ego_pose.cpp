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

#include "ego_pose.hpp"
/**
 * @brief Binds methods of the EgoPose class to the Godot system.
 */
void EgoPose::_bind_methods()
{
  // Bind the get_ego_position method to Godot
  ClassDB::bind_method(D_METHOD("get_ego_position"), &EgoPose::get_ego_position);
  // Bind the get_ego_rotation method to Godot
  ClassDB::bind_method(D_METHOD("get_ego_rotation"), &EgoPose::get_ego_rotation);
}
