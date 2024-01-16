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
 * @class EgoPose
 * @brief The EgoPose class provides methods to obtain the position and orientation (pose) of the
 * ego vehicle.
 *
 * This class interfaces with the ROS 2 transformation system to retrieve the current pose of the
 * ego vehicle relative to the map frame.
 */
class EgoPose : public RefCounted
{
  GDCLASS(EgoPose, RefCounted);

public:
  EgoPose() = default;
  ~EgoPose() = default;

  /**
   * @brief Retrieves the ego vehicle's position in the map frame.
   *
   * @return Vector3 Position of the ego vehicle in Godot's coordinate system.
   */
  Vector3 get_ego_position()
  {
    // Access the shared TF2 buffer from the singleton instance of GodotRviz2
    const auto tf_buffer = GodotRviz2::get_instance().get_tf_buffer();
    // Retrieve the transformation from base_link to map frame
    const auto transform = get_transform(*tf_buffer, "base_link", "map", rclcpp::Time(0));
    // Return a default Vector3 if no transformation is found
    if (!transform) return Vector3(0, 0, 0);

    // Convert ROS 2 coordinates to Godot's coordinate system
    return ros2_to_godot(transform.value().translation);
  }

  /**
   * @brief Retrieves the ego vehicle's rotation (roll, pitch, yaw) in the map frame.
   *
   * @return Vector3 Rotation of the ego vehicle in Godot's coordinate system.
   */
  Vector3 get_ego_rotation()
  {
    // Access the shared TF2 buffer from the singleton instance of GodotRviz2
    const auto tf_buffer = GodotRviz2::get_instance().get_tf_buffer();
    // Retrieve the transformation from base_link to map frame
    const auto transform = get_transform(*tf_buffer, "base_link", "map", rclcpp::Time(0));
    // Return a default Vector3 if no transformation is found
    if (!transform) return Vector3(0, 0, 0);

    // Extract the quaternion rotation from the transformation
    const auto & rotation = transform.value().rotation;
    double roll, pitch, yaw;

    // Convert the quaternion to roll, pitch, yaw
    tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // Convert ROS 2 coordinates to Godot's coordinate system
    return ros2_to_godot(roll, pitch, yaw);
  }

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();
};
