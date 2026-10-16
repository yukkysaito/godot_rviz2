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

#include "map_projection.hpp"

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/UTMUPS.hpp>

using ProjectorInfo = autoware_map_msgs::msg::MapProjectorInfo;

void MapProjection::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("is_available"), &MapProjection::is_available);
  ClassDB::bind_method(D_METHOD("get_revision"), &MapProjection::get_revision);
  ClassDB::bind_method(D_METHOD("to_lat_lon", "position"), &MapProjection::to_lat_lon);
  ClassDB::bind_method(
    D_METHOD("from_lat_lon", "latitude", "longitude"), &MapProjection::from_lat_lon);
  TOPIC_SUBSCRIBER_BIND_METHODS(MapProjection);
}

const std::optional<MapProjection::Origin> & MapProjection::get_origin()
{
  if (!has_new()) return origin_;
  const auto last_msg = get_last_msg();
  set_old();
  origin_.reset();
  ++revision_;
  if (!last_msg) return origin_;

  const auto & info = *last_msg.value();
  Origin origin;
  origin.type = info.projector_type;
  try {
    if (info.projector_type == ProjectorInfo::MGRS) {
      // The map frame starts at the corner of the MGRS grid square
      int precision = 0;
      GeographicLib::MGRS::Reverse(
        info.mgrs_grid, origin.zone, origin.northp, origin.easting, origin.northing, precision,
        false);
    } else if (info.projector_type == ProjectorInfo::LOCAL_CARTESIAN_UTM) {
      double gamma = 0.0;
      double k = 0.0;
      GeographicLib::UTMUPS::Forward(
        info.map_origin.latitude, info.map_origin.longitude, origin.zone, origin.northp,
        origin.easting, origin.northing, gamma, k);
    } else if (info.projector_type == ProjectorInfo::TRANSVERSE_MERCATOR) {
      origin.central_meridian = info.map_origin.longitude;
      origin.scale_factor = info.scale_factor > 0.0 ? info.scale_factor : 0.9996;
      const GeographicLib::TransverseMercator projection(
        GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f(),
        origin.scale_factor);
      projection.Forward(
        origin.central_meridian, info.map_origin.latitude, info.map_origin.longitude,
        origin.easting, origin.northing);
    } else {
      return origin_;  // local maps have no geographic position
    }
  } catch (const GeographicLib::GeographicErr &) {
    return origin_;
  }
  origin_ = origin;
  return origin_;
}

bool MapProjection::is_available() { return get_origin().has_value(); }

int64_t MapProjection::get_revision()
{
  get_origin();
  // Positions in Godot's coordinates also change when the origin of the view is set
  if (map_origin::is_set() != view_origin_set_) {
    view_origin_set_ = map_origin::is_set();
    ++revision_;
  }
  return revision_;
}

Vector2 MapProjection::to_lat_lon(const Vector3 & position)
{
  const auto & origin = get_origin();
  if (!origin) return Vector2();

  const Eigen::Vector3d map_position = ViewFrame(kMapFrame).from_godot(position);
  const double x = origin->easting + map_position.x();
  const double y = origin->northing + map_position.y();
  double latitude = 0.0;
  double longitude = 0.0;
  if (origin->type == ProjectorInfo::TRANSVERSE_MERCATOR) {
    const GeographicLib::TransverseMercator projection(
      GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f(),
      origin->scale_factor);
    projection.Reverse(origin->central_meridian, x, y, latitude, longitude);
  } else {
    GeographicLib::UTMUPS::Reverse(origin->zone, origin->northp, x, y, latitude, longitude);
  }
  return Vector2(latitude, longitude);
}

Vector3 MapProjection::from_lat_lon(double latitude, double longitude)
{
  const auto & origin = get_origin();
  if (!origin) return Vector3();

  double x = 0.0;
  double y = 0.0;
  if (origin->type == ProjectorInfo::TRANSVERSE_MERCATOR) {
    const GeographicLib::TransverseMercator projection(
      GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f(),
      origin->scale_factor);
    projection.Forward(origin->central_meridian, latitude, longitude, x, y);
  } else {
    // In the map's UTM zone, also for positions beyond it
    int zone = 0;
    bool northp = true;
    double gamma = 0.0;
    double k = 0.0;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x, y, gamma, k, origin->zone);
  }
  Vector3 position =
    ViewFrame(kMapFrame).to_godot(Eigen::Vector3d(x - origin->easting, y - origin->northing, 0.0));
  position.y = 0.0f;  // at height 0 in Godot's coordinates
  return position;
}
