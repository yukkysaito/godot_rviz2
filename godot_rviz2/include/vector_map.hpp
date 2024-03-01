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

#include "core/object/ref_counted.h"
#include "core/string/ustring.h"
#include "core/variant/variant.h"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "topic_subscriber.hpp"

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"

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
  TOPIC_SUBSCRIBER(VectorMap, autoware_auto_mapping_msgs::msg::HADMapBin);

public:
  bool generate_graph_structure();
  Array get_lanelet_triangle_list(const String & name);
  Array get_polygon_triangle_list(const String & name);
  Array get_linestring_triangle_list(const String & name, const float width);
  Array get_traffic_light_list();

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

  Array get_as_triangle_list(const lanelet::ConstPolygons3d & polygons) const;
  Array get_as_triangle_list(const lanelet::ConstLanelets & lanelets) const;
  Array get_as_triangle_list(const lanelet::ConstLineStrings3d & linestring_polygon) const;
  Array get_as_triangle_list(
    const lanelet::ConstLineStrings3d & linestring, const float width) const;
};
