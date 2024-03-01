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

#include "vector_map.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "util.hpp"

namespace
{
Array convert_strip_to_list(const Array & strip)
{
  Array list;
  for (int i = 2; i < strip.size(); ++i) {
    if (i % 2 == 0) {  // clockwise
      list.append(strip[i - 2]);
      list.append(strip[i - 1]);
      list.append(strip[i]);
    } else {  // counterclockwise
      list.append(strip[i - 1]);
      list.append(strip[i - 2]);
      list.append(strip[i]);
    }
  }
  return list;
}

bool has_label(const lanelet::ConstLineString3d & linestring, const std::set<std::string> & labels)
{
  if (!linestring.hasAttribute(lanelet::AttributeName::Type)) return false;
  lanelet::Attribute attr = linestring.attribute(lanelet::AttributeName::Type);
  if (labels.count(attr.value()) == 0) return false;
  return true;
}

bool is_attribute_value(
  const lanelet::ConstPoint3d p, const std::string attr_str, const std::string value_str)
{
  lanelet::Attribute attr = p.attribute(attr_str);
  if (attr.value().compare(value_str) == 0) {
    return true;
  }
  return false;
}
}  // namespace

namespace lanelet_utils
{
lanelet::ConstLineStrings3d get_linestrings(
  const lanelet::LaneletMapConstPtr & lanelet_map, const std::string & type)
{
  lanelet::ConstLineStrings3d linestring_polygons;
  for (const auto & ls : lanelet_map->lineStringLayer) {
    const std::string & attr = ls.attributeOr(lanelet::AttributeName::Type, "none");
    if (attr == type) {
      linestring_polygons.push_back(ls);
    }
  }
  return linestring_polygons;
}

lanelet::ConstPolygons3d get_polygons(
  const lanelet::LaneletMapConstPtr & lanelet_map, const std::string & type)
{
  lanelet::ConstPolygons3d polygons;
  for (const auto & poly : lanelet_map->polygonLayer) {
    const std::string & attr = poly.attributeOr(lanelet::AttributeName::Type, "none");
    if (attr == type) {
      polygons.push_back(poly);
    }
  }
  return polygons;
}

lanelet::ConstLanelets get_lanelets(
  const lanelet::LaneletMapConstPtr & lanelet_map, const std::string & type)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & lanelet : lanelet_map->laneletLayer) {
    if (!lanelet.hasAttribute(lanelet::AttributeName::Subtype)) continue;
    const lanelet::Attribute & attr = lanelet.attribute(lanelet::AttributeName::Subtype);
    if (attr.value() == type) {
      lanelets.push_back(lanelet);
    }
  }

  return lanelets;
}
}  // namespace lanelet_utils

namespace triangulation
{
struct Vertex
{
  size_t index;
  Vector2 point;
};

bool is_convex_angle(const Vector2 & prev, const Vector2 & self, const Vector2 & next)
{
  return (prev.x - self.x) * (next.y - self.y) - (prev.y - self.y) * (next.x - self.x) >= 0.0;
}

bool is_point_inside_triangle(
  const Vector2 & p0, const Vector2 & p1, const Vector2 & p2, const Vector2 & p)
{
  const auto c1 = (p1.x - p0.x) * (p.y - p1.y) - (p1.y - p0.y) * (p.x - p1.x);
  const auto c2 = (p2.x - p1.x) * (p.y - p2.y) - (p2.y - p1.y) * (p.x - p2.x);
  const auto c3 = (p0.x - p2.x) * (p.y - p0.y) - (p0.y - p2.y) * (p.x - p0.x);

  return c1 > 0.0 && c2 > 0.0 && c3 > 0.0 || c1 < 0.0 && c2 < 0.0 && c3 < 0.0;
}

bool is_ear(
  const std::vector<Vertex> & vertices, const size_t self_index, const size_t prev_index,
  const size_t next_index)
{
  const auto is_convex = is_convex_angle(
    vertices.at(prev_index).point, vertices.at(self_index).point, vertices.at(next_index).point);

  if (!is_convex) {
    return false;
  }

  for (size_t i = 0; i < vertices.size(); ++i) {
    if (i == prev_index || i == self_index || i == next_index) {
      continue;
    }
    if (is_point_inside_triangle(
          vertices.at(prev_index).point, vertices.at(self_index).point,
          vertices.at(next_index).point, vertices.at(i).point)) {
      return false;
    }
  }
  return true;
}

void triangulate(const std::vector<Vector3> & polygon, std::vector<Vector3> & triangles)
{
  if (polygon.size() < 3) {
    return;
  }
  if (polygon.size() == 3) {
    triangles = polygon;
    return;
  }

  std::vector<Vector2> polygon_2d;
  for (const auto & point : polygon) {
    polygon_2d.push_back(Vector2(point.x, point.y));
  }

  std::vector<Vertex> vertices;
  if (is_clockwise(polygon_2d)) {
    for (size_t i = 0; i < polygon_2d.size(); ++i) {
      vertices.push_back(Vertex{i, Vector2(polygon_2d.at(i).x, polygon_2d.at(i).y)});
    }
  } else {
    for (int i = polygon_2d.size() - 1; 0 <= i; --i) {
      vertices.push_back(Vertex{i, Vector2(polygon_2d.at(i).x, polygon_2d.at(i).y)});
    }
  }

  int i = 0;
  while (2 < vertices.size()) {
    if (vertices.size() <= i) {
      std::cerr << "triangulation failed" << std::endl;
      for (const auto & vertex : vertices) {
        std::cerr << "index = " << vertex.index << ", point = (" << vertex.point.x << ", "
                  << vertex.point.y << ")" << std::endl;
      }
      break;
    }
    const auto previous = (i == 0) ? vertices.size() - 1 : i - 1;
    const auto next = (i == vertices.size() - 1) ? 0 : i + 1;
    if (is_ear(vertices, i, previous, next)) {
      // create triangle
      triangles.push_back(polygon.at(vertices.at(previous).index));
      triangles.push_back(polygon.at(vertices.at(i).index));
      triangles.push_back(polygon.at(vertices.at(next).index));
      // remove vertex of center of angle
      vertices.erase(vertices.begin() + i);
      // reset index
      i = 0;
      continue;
    }
    i++;
  }
}
}  // namespace triangulation

VectorMap::VectorMap() : lanelet_map_(new lanelet::LaneletMap) {}

void VectorMap::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("generate_graph_structure"), &VectorMap::generate_graph_structure);
  ClassDB::bind_method(
    D_METHOD("get_lanelet_triangle_list"), &VectorMap::get_lanelet_triangle_list);
  ClassDB::bind_method(
    D_METHOD("get_polygon_triangle_list"), &VectorMap::get_polygon_triangle_list);
  ClassDB::bind_method(
    D_METHOD("get_linestring_triangle_list"), &VectorMap::get_linestring_triangle_list);
  ClassDB::bind_method(D_METHOD("get_traffic_light_list"), &VectorMap::get_traffic_light_list);

  TOPIC_SUBSCRIBER_BIND_METHODS(VectorMap);
}

bool VectorMap::generate_graph_structure()
{
  const auto last_msg = get_last_msg();
  if (!last_msg) return false;

  lanelet::utils::conversion::fromBinMsg(*(last_msg.value()), lanelet_map_);
  // lanelet
  all_lanelets_ = lanelet::utils::query::laneletLayer(lanelet_map_);
  road_lanelets_ = lanelet_utils::get_lanelets(lanelet_map_, lanelet::AttributeValueString::Road);
  shoulder_lanelets_ = lanelet_utils::get_lanelets(lanelet_map_, "road_shoulder");
  crosswalk_lanelets_ =
    lanelet_utils::get_lanelets(lanelet_map_, lanelet::AttributeValueString::Crosswalk);
  walkway_lanelets_ =
    lanelet_utils::get_lanelets(lanelet_map_, lanelet::AttributeValueString::Walkway);

  // line string
  pedestrian_markings_ = lanelet_utils::get_linestrings(lanelet_map_, "pedestrian_marking");
  stop_lines_ = lanelet::utils::query::stopLinesLanelets(road_lanelets_);
  curbstones_ = lanelet_utils::get_linestrings(lanelet_map_, "curbstone");
  parking_spaces_ = lanelet_utils::get_linestrings(lanelet_map_, "parking_space");

  // polygon
  no_obstacle_segmentation_area_ =
    lanelet_utils::get_polygons(lanelet_map_, "no_obstacle_segmentation_area");
  no_obstacle_segmentation_area_for_run_out_ =
    lanelet_utils::get_polygons(lanelet_map_, "no_obstacle_segmentation_area_for_run_out");
  hatched_road_markings_area_ = lanelet_utils::get_polygons(lanelet_map_, "hatched_road_markings");
  intersection_areas_ = lanelet_utils::get_polygons(lanelet_map_, "intersection_area");
  parking_lots_ = lanelet_utils::get_polygons(lanelet_map_, "parking_lot");
  obstacle_polygons_ = lanelet_utils::get_polygons(lanelet_map_, "obstacle");

  // traffic light
  traffic_lights_ = lanelet::utils::query::autowareTrafficLights(all_lanelets_);

  return true;
}

Array VectorMap::get_lanelet_triangle_list(const String & name)
{
  Array triangle_list;

  if (name == "road") {
    triangle_list = get_as_triangle_list(road_lanelets_);
  } else if (name == "shoulder") {
    triangle_list = get_as_triangle_list(shoulder_lanelets_);
  } else if (name == "crosswalk") {
    triangle_list = get_as_triangle_list(crosswalk_lanelets_);
  } else if (name == "walkway") {
    triangle_list = get_as_triangle_list(walkway_lanelets_);
  } else {
    std::cerr << "invalid lanelet name" << std::endl;
  }

  return triangle_list;
}

Array VectorMap::get_polygon_triangle_list(const String & name)
{
  Array triangle_list;
  if (name == "pedestrian_marking") {
    triangle_list = get_as_triangle_list(pedestrian_markings_);
  } else if (name == "intersection_area") {
    triangle_list = get_as_triangle_list(intersection_areas_);
  } else if (name == "hatched_road_markings_area") {
    triangle_list = get_as_triangle_list(hatched_road_markings_area_);
  } else if (name == "parking_lots") {
    triangle_list = get_as_triangle_list(parking_lots_);
  } else {
    std::cerr << "invalid polygon name" << std::endl;
  }

  return triangle_list;
}

Array VectorMap::get_linestring_triangle_list(const String & name, const float width)
{
  Array triangle_list;
  if (name == "shared_white_line") {
    std::unordered_set<lanelet::ConstLineString3d> added_shared_white_lines;
    const std::set<std::string> ground_labels = {"line_thin", "line_thick"};

    for (const auto & lanelet : road_lanelets_) {
      lanelet::ConstLineString3d left_ls = lanelet.leftBound();
      lanelet::ConstLineString3d right_ls = lanelet.rightBound();

      lanelet::Lanelets right_lane_candidates =
        lanelet_map_->laneletLayer.findUsages(lanelet.rightBound());
      for (auto & candidate : right_lane_candidates) {
        // exclude self lanelet
        if (candidate == lanelet) continue;
        // exclude not shared lanelet
        if (
          candidate.leftBound() != lanelet.rightBound() &&
          candidate.rightBound() != lanelet.rightBound())
          continue;

        if (has_label(lanelet.rightBound(), ground_labels))
          added_shared_white_lines.insert(lanelet.rightBound());
      }

      lanelet::Lanelets left_lane_candidates =
        lanelet_map_->laneletLayer.findUsages(lanelet.rightBound());
      for (auto & candidate : left_lane_candidates) {
        // exclude self lanelet
        if (candidate == lanelet) continue;
        // exclude not shared lanelet
        if (
          candidate.rightBound() != lanelet.leftBound() &&
          candidate.leftBound() != lanelet.leftBound())
          continue;

        if (has_label(lanelet.rightBound(), ground_labels))
          added_shared_white_lines.insert(lanelet.rightBound());
      }
    }

    lanelet::ConstLineStrings3d shared_white_lines;
    for (const auto & line : added_shared_white_lines) {
      shared_white_lines.push_back(line);
    }
    triangle_list = get_as_triangle_list(shared_white_lines, width);
  } else if (name == "stop_line") {
    triangle_list = get_as_triangle_list(stop_lines_, width);
  } else if (name == "curbstone") {
    triangle_list = get_as_triangle_list(curbstones_, width);
  } else if (name == "white_line") {
    std::unordered_set<lanelet::ConstLineString3d> added_white_lines;
    const std::set<std::string> ground_labels = {"line_thin", "line_thick"};
    for (const auto & lanelet : road_lanelets_) {
      if (has_label(lanelet.leftBound(), ground_labels))
        added_white_lines.insert(lanelet.leftBound());
      if (has_label(lanelet.rightBound(), ground_labels))
        added_white_lines.insert(lanelet.rightBound());
    }

    lanelet::ConstLineStrings3d white_lines;
    for (const auto & line : added_white_lines) {
      white_lines.push_back(line);
    }
    triangle_list = get_as_triangle_list(white_lines, width);

  } else {
    std::cerr << "invalid polygon name" << std::endl;
  }

  return triangle_list;
}

Array VectorMap::get_as_triangle_list(const lanelet::ConstPolygons3d & polygons) const
{
  Array triangle_list;

  for (const auto & ll2_polygon : polygons) {
    std::vector<Vector3> triangles;

    std::vector<Vector3> polygon;

    for (const auto & point : ll2_polygon) {
      polygon.push_back(
        Vector3(point.basicPoint().x(), point.basicPoint().y(), point.basicPoint().z()));
    }

    triangulation::triangulate(polygon, triangles);

    for (const auto & triangle : triangles) {
      Dictionary dict;
      dict["position"] = ros2_to_godot(triangle.x, triangle.y, triangle.z);
      dict["normal"] = ros2_to_godot(0.f, 0.f, 1.f);
      triangle_list.append(dict);
    }
  }
  return triangle_list;
}

Array VectorMap::get_as_triangle_list(const lanelet::ConstLineStrings3d & linestring_polygons) const
{
  Array triangle_list;

  for (const auto & linestring : linestring_polygons) {
    if (linestring.size() < 3) {
      continue;
    }
    std::vector<Vector3> triangles;

    std::vector<Vector3> polygon;

    for (const auto & point : linestring) {
      polygon.push_back(
        Vector3(point.basicPoint().x(), point.basicPoint().y(), point.basicPoint().z()));
    }
    if (linestring.front().id() == linestring.back().id()) {
      polygon.pop_back();
    }

    triangulation::triangulate(polygon, triangles);

    for (const auto & triangle : triangles) {
      Dictionary dict;
      dict["position"] = ros2_to_godot(triangle.x, triangle.y, triangle.z);
      dict["normal"] = ros2_to_godot(0.f, 0.f, 1.f);
      triangle_list.append(dict);
    }
  }
  return triangle_list;
}

Array VectorMap::get_as_triangle_list(
  const lanelet::ConstLineStrings3d & linestrings, const float width) const
{
  Array triangle_list;
  for (const auto & linestring : linestrings) {
    std::vector<geometry_msgs::msg::Point> line;
    for (const auto & point : linestring) {
      geometry_msgs::msg::Point p;
      p.x = point.basicPoint().x();
      p.y = point.basicPoint().y();
      p.z = point.basicPoint().z();
      line.push_back(p);
    }
    triangle_list.append_array(
      convert_strip_to_list(calculate_line_as_triangle_strip(line, width)));
  }
  return triangle_list;
}

Array VectorMap::get_as_triangle_list(const lanelet::ConstLanelets & lanelets) const
{
  Array triangle_list;

  for (const auto & lanelet : lanelets) {
    std::vector<Vector3> triangles;

    std::vector<Vector3> polygon;
    lanelet::CompoundPolygon3d lanelet_polygon = lanelet.polygon3d();
    for (const auto & lanelet_point : lanelet_polygon) {
      Vector3 point(
        lanelet_point.basicPoint().x(), lanelet_point.basicPoint().y(),
        lanelet_point.basicPoint().z());
      polygon.push_back(point);
    }

    triangulation::triangulate(polygon, triangles);

    for (const auto & triangle : triangles) {
      Dictionary dict;
      dict["position"] = ros2_to_godot(triangle.x, triangle.y, triangle.z);
      dict["normal"] = ros2_to_godot(0.f, 0.f, 1.f);
      triangle_list.append(dict);
    }
  }
  return triangle_list;
}

void convert_to_godot_array(
  const std::vector<TrafficLightGroup> & traffic_light_groups, Array & traffic_light_list)
{
  for (const auto & traffic_light_group : traffic_light_groups) {
    Dictionary traffic_light_group_dict;
    traffic_light_group_dict["group_id"] = traffic_light_group.group_id;

    Array traffic_lights;
    for (const auto & traffic_light : traffic_light_group.traffic_lights) {
      Dictionary traffic_light_dict;
      Dictionary board_dict;
      board_dict["right_top_position"] = traffic_light.board.right_top_position;
      board_dict["left_top_position"] = traffic_light.board.left_top_position;
      board_dict["right_bottom_position"] = traffic_light.board.right_bottom_position;
      board_dict["left_bottom_position"] = traffic_light.board.left_bottom_position;
      board_dict["normal"] = traffic_light.board.normal;
      traffic_light_dict["board"] = board_dict;
      Array light_bulbs;
      for (const auto & light_bulb : traffic_light.light_bulbs) {
        Dictionary light_bulb_dict;
        light_bulb_dict["color"] = String(light_bulb.color.c_str());
        light_bulb_dict["position"] = light_bulb.position;
        light_bulb_dict["normal"] = light_bulb.normal;
        light_bulb_dict["arrow"] = String(light_bulb.arrow.c_str());
        light_bulb_dict["radius"] = light_bulb.radius;
        light_bulbs.append(light_bulb_dict);
      }
      traffic_light_dict["light_bulbs"] = light_bulbs;
      traffic_lights.append(traffic_light_dict);
    }
    traffic_light_group_dict["traffic_lights"] = traffic_lights;
    traffic_light_list.append(traffic_light_group_dict);
  }

  return;
}

void get_traffic_light_groups_from_lanelet_map(
  const std::vector<lanelet::AutowareTrafficLightConstPtr> & regulatory_elements,
  std::vector<TrafficLightGroup> & traffic_light_groups)
{
  for (const auto & regulatory_element : regulatory_elements) {
    const auto reg_traffic_lights = regulatory_element->trafficLights();
    std::unordered_map<int /* board id */, Board> boards_map;
    std::unordered_map<int /* board id */, std::vector<LightBulb>> light_bulbs_map;

    // board
    for (const auto & reg_traffic_light : reg_traffic_lights) {
      if (!reg_traffic_light.isLineString()) continue;

      lanelet::ConstLineString3d linestring =
        static_cast<lanelet::ConstLineString3d>(reg_traffic_light);

      float height = 0.7;
      if (linestring.hasAttribute("height")) {
        height = std::stof(linestring.attribute("height").value());
      }
      Board board;
      Eigen::Vector3f right_top(
        linestring.back().x(), linestring.back().y(), linestring.back().z() + height);
      Eigen::Vector3f left_top(
        linestring.front().x(), linestring.front().y(), linestring.front().z() + height);
      Eigen::Vector3f right_bottom(
        linestring.back().x(), linestring.back().y(), linestring.back().z());
      Eigen::Vector3f left_bottom(
        linestring.front().x(), linestring.front().y(), linestring.front().z());

      board.right_top_position = ros2_to_godot(right_top);
      board.left_top_position = ros2_to_godot(left_top);
      board.right_bottom_position = ros2_to_godot(right_bottom);
      board.left_bottom_position = ros2_to_godot(left_bottom);

      board.normal = ros2_to_godot(cross_product(right_top - left_top, left_bottom - left_top));

      boards_map[linestring.id()] = board;
    }

    // light bulbs
    for (auto linestring_light_bulbs : regulatory_element->lightBulbs()) {
      if (!linestring_light_bulbs.hasAttribute("traffic_light_id")) {
        continue;
      }
      int board_id = std::stoi(linestring_light_bulbs.attribute("traffic_light_id").value());
      std::vector<LightBulb> light_bulbs;

      for (auto point_light_bulb : linestring_light_bulbs) {
        if (!point_light_bulb.hasAttribute("color")) {
          std::cerr << "light bulb has no color attribute. traffic light group id is"
                    << regulatory_element->id() << std::endl;
          continue;
        }

        LightBulb light_bulb;
        light_bulb.color = point_light_bulb.attribute("color").value();
        light_bulb.position =
          ros2_to_godot(point_light_bulb.x(), point_light_bulb.y(), point_light_bulb.z());
        if (point_light_bulb.hasAttribute("arrow")) {
          light_bulb.arrow = point_light_bulb.attribute("arrow").value();
        }
        if (point_light_bulb.hasAttribute("radius")) {
          light_bulb.radius = std::stof(point_light_bulb.attribute("radius").value());
        }
        light_bulb.normal = boards_map[board_id].normal;
        light_bulbs.push_back(light_bulb);
      }

      light_bulbs_map[board_id] = light_bulbs;
    }

    // traffic light group
    TrafficLightGroup traffic_light_group;
    for (const auto & board : boards_map) {
      TrafficLight traffic_light;
      traffic_light.board = board.second;
      if (light_bulbs_map.find(board.first) != light_bulbs_map.end())
        traffic_light.light_bulbs = light_bulbs_map[board.first];
      else
        std::cerr << "no light bulbs for board id " << board.first << std::endl;
      traffic_light_group.traffic_lights.push_back(traffic_light);
    }
    traffic_light_group.group_id = regulatory_element->id();

    traffic_light_groups.push_back(traffic_light_group);
  }
}

Array VectorMap::get_traffic_light_list()
{
  std::vector<TrafficLightGroup> traffic_light_groups;
  get_traffic_light_groups_from_lanelet_map(traffic_lights_, traffic_light_groups);

  Array traffic_light_list;
  convert_to_godot_array(traffic_light_groups, traffic_light_list);
  return traffic_light_list;
}
