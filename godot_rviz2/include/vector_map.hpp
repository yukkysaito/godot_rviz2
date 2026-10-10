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

#include "autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "core/object/ref_counted.h"
#include "core/string/ustring.h"
#include "core/variant/variant.h"
#include "lanelet2_core/LaneletMap.h"
#include "async_task.hpp"
#include "topic_subscriber.hpp"

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"

#include <vector>

struct LightBulb
{
  std::string color;
  Vector3 position;
  Vector3 normal;
  std::string arrow = "none";
  float radius = 0.15;
};
struct Board
{
  Vector3 right_top_position;
  Vector3 left_top_position;
  Vector3 right_bottom_position;
  Vector3 left_bottom_position;
  Vector3 normal;
};

struct TrafficLight
{
  std::vector<LightBulb> light_bulbs;
  Board board;
};

struct TrafficLightGroup
{
  std::vector<TrafficLight> traffic_lights;
  int group_id;
};

class VectorMap : public RefCounted
{
  GDCLASS(VectorMap, RefCounted);
  TOPIC_SUBSCRIBER(VectorMap, autoware_map_msgs::msg::LaneletMapBin);

public:
  bool generate_graph_structure();
  Array get_lanelet_triangle_list(const String & name);
  Array get_polygon_triangle_list(const String & name);
  Array get_linestring_triangle_list(const String & name, const float width);
  Array get_traffic_light_list();

  /**
   * @brief Decodes the last map and builds the requested geometry on a worker thread.
   *
   * @param layers Array of Dictionary {"name": String, "parts": Array of parts}. A part is
   *   [kind, layer(, width)] with kind "lanelet", "polygon" or "linestring" (see the
   *   get_*_triangle_list methods), or a line string query by type / subtype (comma separated):
   *   ["shared_lines", width, dash, gap] for the lane lines shared by two lanelets (dashed as dashes
   *   of dash [m] separated by gap [m]), or ["road_borders", style, width, height] for a shape
   *   along the road borders (see build_road_borders()).
   * @param tile_size Each layer is split into square tiles of this size [m] on the ground plane
   *   (by triangle centroid), so that only the tiles in view need meshes. 0: a single tile.
   * @return false if there is no map message or a build is already running.
   *
   * While a build runs, the other getters must not be used (they share the decoded map).
   * The message is released once the build started, to free its memory.
   */
  bool start_build(const Array & layers, double tile_size);

  /// True when a build started with start_build() has finished.
  bool is_build_done();

  /**
   * @brief Takes the result of the finished build.
   * @return Dictionary {"layers": {name: Array of tiles {"center": Vector3, "vertices":
   *   PackedVector3Array of triangle vertices relative to center, "normals": PackedVector3Array,
   *   "uvs": PackedVector2Array}},
   *   "traffic_lights": Array (see get_traffic_light_list())}
   */
  Dictionary take_build_result();

  VectorMap();
  ~VectorMap() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();

private:
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::ConstLanelets all_lanelets_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  lanelet::ConstLanelets crosswalk_lanelets_;
  lanelet::ConstLanelets walkway_lanelets_;

  lanelet::ConstLineStrings3d pedestrian_markings_;
  lanelet::ConstLineStrings3d curbstones_;
  lanelet::ConstLineStrings3d parking_spaces_;
  lanelet::ConstLineStrings3d stop_lines_;

  lanelet::ConstPolygons3d no_obstacle_segmentation_area_;
  lanelet::ConstPolygons3d no_obstacle_segmentation_area_for_run_out_;
  lanelet::ConstPolygons3d hatched_road_markings_area_;
  lanelet::ConstPolygons3d intersection_areas_;
  lanelet::ConstPolygons3d parking_lots_;
  lanelet::ConstPolygons3d obstacle_polygons_;

  std::vector<lanelet::AutowareTrafficLightConstPtr> traffic_lights_;

  // Polygons that could not be triangulated in the current build (reported once at its end)
  mutable size_t triangulation_failures_ = 0;

  // Declared last: waits for a running build (which uses the members above) on destruction
  AsyncTask<Dictionary> build_task_;

  bool decode(const autoware_map_msgs::msg::LaneletMapBin & msg);
  struct LayerGeometry
  {
    std::vector<Vector3> vertices, normals;  // triangles
    std::vector<Vector2> uvs;
  };
  void build_layer(const Array & parts, LayerGeometry & geometry);
  void build_road_borders(const Array & part, LayerGeometry & geometry) const;
  void build_lines(const Array & part, LayerGeometry & geometry) const;
  lanelet::ConstLineStrings3d get_shared_white_lines() const;
  static Array split_into_tiles(const LayerGeometry & geometry, double tile_size);

  Array get_as_triangle_list(const lanelet::ConstPolygons3d & polygons) const;
  Array get_as_triangle_list(const lanelet::ConstLanelets & lanelets) const;
  Array get_as_triangle_list(const lanelet::ConstLineStrings3d & linestring_polygon) const;
  Array get_as_triangle_list(
    const lanelet::ConstLineStrings3d & linestring, const float width) const;
};
