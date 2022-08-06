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

#include "parameter.hpp"

#include <string>

void Parameter::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_double_value"), &Parameter::get_double_value);
  ClassDB::bind_method(D_METHOD("has_parameter"), &Parameter::has_parameter);
}

bool Parameter::has_parameter(const String & name)
{
  return GodotRviz2::get_instance().get_node()->has_parameter(to_std(name));
}

double Parameter::get_double_value(const String & name)
{
  return GodotRviz2::get_instance().get_node()->get_parameter(to_std(name)).get_value<double>();
}
