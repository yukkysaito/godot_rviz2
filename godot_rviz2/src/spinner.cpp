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

#include "spinner.hpp"

/**
 * @brief Binds methods of the GodotRviz2Spinner class to the Godot system.
 */
void GodotRviz2Spinner::_bind_methods()
{
  // Bind the spin_some method to Godot
  ClassDB::bind_method(D_METHOD("spin_some"), &GodotRviz2Spinner::spin_some);
}
