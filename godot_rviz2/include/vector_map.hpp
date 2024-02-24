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
#include "topic_subscriber.hpp"

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"

#include <vector>

class VectorMap : public RefCounted
{
  GDCLASS(VectorMap, RefCounted);
  TOPIC_SUBSCRIBER(VectorMap, autoware_auto_mapping_msgs::msg::HADMapBin);

public:
  bool generate_graph_structure();
  Array get_lanelet_triangle_list(const String & name);
  Array get_polygon_triangle_list(const String & name);
  Array get_linestring_triangle_list(const String & name, const float width);

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

  lanelet::ConstLineStrings3d pedestrian_line_markings_;
  lanelet::ConstLineStrings3d curbstones_;
  lanelet::ConstLineStrings3d parking_spaces_;
  lanelet::ConstLineStrings3d stop_lines_;

  lanelet::ConstPolygons3d no_obstacle_segmentation_area_;
  lanelet::ConstPolygons3d no_obstacle_segmentation_area_for_run_out_;
  lanelet::ConstPolygons3d hatched_road_markings_area_;
  lanelet::ConstPolygons3d intersection_areas_;
  lanelet::ConstPolygons3d parking_lots_;
  lanelet::ConstPolygons3d obstacle_polygons_;

  Array get_as_triangle_list(const lanelet::ConstPolygons3d & polygons) const;
  Array get_as_triangle_list(const lanelet::ConstLanelets & lanelets) const;
  Array get_as_triangle_list(const lanelet::ConstLineStrings3d & linestring_polygon) const;
  Array get_as_triangle_list(
    const lanelet::ConstLineStrings3d & linestring, const float width) const;
};
