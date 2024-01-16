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

/**
 * @class SteeringReport
 * @brief The SteeringReport class provides an interface to obtain the steering angle from
 * Autoware's SteeringReport messages.
 *
 * This class subscribes to Autoware's SteeringReport messages and allows for retrieval of the
 * current steering angle.
 */
class SteeringReport : public RefCounted
{
  GDCLASS(SteeringReport, RefCounted);
  TOPIC_SUBSCRIBER(SteeringReport, autoware_auto_vehicle_msgs::msg::SteeringReport);

public:
  /**
   * @brief Retrieves the current steering angle.
   *
   * @return double The current steering angle in degrees.
   */
  double get_angle();

  SteeringReport() = default;
  ~SteeringReport() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();
};
