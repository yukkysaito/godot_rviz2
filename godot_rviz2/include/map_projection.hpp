//
//  Copyright 2026 Yukihiro Saito. All rights reserved.
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
#include "core/variant/variant.h"
#include "topic_subscriber.hpp"

#include "autoware_map_msgs/msg/map_projector_info.hpp"

#include <optional>
#include <string>

/**
 * @class MapProjection
 * @brief Converts between the map frame and latitude / longitude, with the map's projection
 * (MGRS, LocalCartesianUTM or TransverseMercator).
 */
class MapProjection : public RefCounted
{
  GDCLASS(MapProjection, RefCounted);
  TOPIC_SUBSCRIBER(MapProjection, autoware_map_msgs::msg::MapProjectorInfo);

public:
  /**
   * @brief Whether the map's projection is known and supported.
   */
  bool is_available();

  /**
   * @brief Number of projections received so far (and changes of the origin of the view, see
   * map_origin): positions converted before it changed (e.g. another map was loaded) are out of
   * date.
   */
  int64_t get_revision();

  /**
   * @brief Latitude and longitude [deg] (x, y) of a position in Godot's coordinate system.
   */
  Vector2 to_lat_lon(const Vector3 & position);

  /**
   * @brief Position in Godot's coordinate system (at height 0) of a latitude and longitude [deg].
   */
  Vector3 from_lat_lon(double latitude, double longitude);

  MapProjection() = default;
  ~MapProjection() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();

private:
  // Origin of the map frame in its UTM zone, or in the transverse Mercator projection
  struct Origin
  {
    std::string type;
    int zone = 0;
    bool northp = true;
    double easting = 0.0;
    double northing = 0.0;
    double central_meridian = 0.0;
    double scale_factor = 1.0;
  };
  std::optional<Origin> origin_;
  int64_t revision_ = 0;
  bool view_origin_set_ = false;  // map_origin::is_set() at the last revision

  const std::optional<Origin> & get_origin();
};
