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

#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "util.hpp"

#include <mapbox/earcut.hpp>

#include <array>
#include <set>
#include <cmath>
#include <unordered_map>
#include <vector>

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
// z of the cross product (b - a) x (c - a) on the ground plane (ROS x/y)
double cross(const Vector3 & a, const Vector3 & b, const Vector3 & c)
{
  return (double(b.x) - a.x) * (double(c.y) - a.y) - (double(b.y) - a.y) * (double(c.x) - a.x);
}

// Appends the triangle clockwise seen from above (so that it faces up after ros2_to_godot()),
// skipping degenerate ones
void push_triangle(
  std::vector<Vector3> & triangles, const Vector3 & a, const Vector3 & b, const Vector3 & c)
{
  const double area2 = cross(a, b, c);
  if (std::abs(area2) < 1e-9) return;
  triangles.push_back(a);
  if (area2 > 0.0) {
    triangles.push_back(c);
    triangles.push_back(b);
  } else {
    triangles.push_back(b);
    triangles.push_back(c);
  }
}

// True if a and b are the same point on the ground plane (absolute tolerance: map coordinates
// are large, so a relative one would merge distinct points)
bool same_point(const Vector3 & a, const Vector3 & b)
{
  const double dx = double(a.x) - b.x;
  const double dy = double(a.y) - b.y;
  return dx * dx + dy * dy < 1e-6;
}

// Removes consecutive duplicate points (and a closing point equal to the first one)
std::vector<Vector3> remove_duplicates(const std::vector<Vector3> & points)
{
  std::vector<Vector3> result;
  for (const auto & p : points) {
    if (result.empty() || !same_point(p, result.back())) result.push_back(p);
  }
  while (result.size() > 1 && same_point(result.front(), result.back())) result.pop_back();
  return result;
}

/**
 * @brief Triangulates a simple polygon (ROS coordinates) on the ground plane with earcut, which
 * also copes with slightly malformed polygons (touching or self-intersecting edges).
 * @return false if no triangle could be made
 */
bool triangulate(const std::vector<Vector3> & polygon, std::vector<Vector3> & triangles)
{
  const auto points = remove_duplicates(polygon);
  if (points.size() < 3) return false;

  // Relative to the first point: map coordinates are large, earcut works in double anyway
  using EarcutPoint = std::array<double, 2>;
  std::vector<std::vector<EarcutPoint>> rings(1);
  for (const auto & p : points) {
    rings[0].push_back({double(p.x) - points[0].x, double(p.y) - points[0].y});
  }
  const std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(rings);
  const size_t before = triangles.size();
  for (size_t k = 0; k + 2 < indices.size(); k += 3) {
    push_triangle(triangles, points[indices[k]], points[indices[k + 1]], points[indices[k + 2]]);
  }
  return triangles.size() > before;
}

/**
 * @brief Triangulates the strip between the left and right bound of a lanelet. Unlike polygon
 * triangulation this never fails, follows the height of both bounds, and neighbouring lanelets
 * share their bound points, so the road surface has no gaps.
 */
void triangulate_strip(
  const std::vector<Vector3> & left_in, const std::vector<Vector3> & right_in,
  std::vector<Vector3> & triangles)
{
  const auto left = remove_duplicates(left_in);
  const auto right = remove_duplicates(right_in);
  if (left.empty() || right.empty() || left.size() + right.size() < 3) return;

  auto distance2 = [](const Vector3 & a, const Vector3 & b) {
    const double dx = double(a.x) - b.x;
    const double dy = double(a.y) - b.y;
    return dx * dx + dy * dy;
  };
  // Advance along the bound whose next point makes the shorter diagonal
  size_t i = 0, j = 0;
  while (i + 1 < left.size() || j + 1 < right.size()) {
    const bool advance_left =
      j + 1 >= right.size() ||
      (i + 1 < left.size() && distance2(left[i + 1], right[j]) <= distance2(left[i], right[j + 1]));
    if (advance_left) {
      push_triangle(triangles, left[i], right[j], left[i + 1]);
      ++i;
    } else {
      push_triangle(triangles, left[i], right[j], right[j + 1]);
      ++j;
    }
  }
}
}  // namespace triangulation

namespace line_geometry
{
using Points = std::vector<Vector3>;  // ROS coordinates

Points to_points(const lanelet::ConstLineString3d & linestring)
{
  Points points;
  for (const auto & point : linestring) {
    points.emplace_back(point.basicPoint().x(), point.basicPoint().y(), point.basicPoint().z());
  }
  return triangulation::remove_duplicates(points);
}

// Splits a polyline into dashes of dash_length [m] separated by gaps of gap_length [m]
std::vector<Points> split_into_dashes(const Points & line, double dash_length, double gap_length)
{
  std::vector<Points> dashes;
  if (line.size() < 2 || dash_length <= 0.0) return {line};
  Points dash{line[0]};
  bool in_dash = true;
  double remaining = dash_length;  // of the current dash or gap
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    Vector3 from = line[i];
    const Vector3 to = line[i + 1];
    double length = from.distance_to(to);
    while (length > remaining) {
      const Vector3 split = from + (to - from) * float(remaining / length);
      if (in_dash) {
        dash.push_back(split);
        dashes.push_back(dash);
        dash.clear();
      } else {
        dash = {split};
      }
      in_dash = !in_dash;
      length -= remaining;
      from = split;
      remaining = in_dash ? dash_length : gap_length;
    }
    remaining -= length;
    if (in_dash) dash.push_back(to);
  }
  if (in_dash && dash.size() >= 2) dashes.push_back(dash);
  return dashes;
}

// Flat strip of width [m] centered on line, as up-facing triangles
void append_ribbon(const Points & line, double width, std::vector<Vector3> & triangles)
{
  if (line.size() < 2) return;
  const double half = width / 2.0;
  Points left, right;
  for (size_t i = 0; i < line.size(); ++i) {
    // Direction at the point: average of the adjacent segments (miter), limited at sharp corners
    const Vector3 & prev = line[i == 0 ? 0 : i - 1];
    const Vector3 & next = line[i + 1 < line.size() ? i + 1 : i];
    const Vector2 back = Vector2(line[i].x - prev.x, line[i].y - prev.y).normalized();
    const Vector2 front = Vector2(next.x - line[i].x, next.y - line[i].y).normalized();
    Vector2 direction = (back + front).normalized();
    if (direction == Vector2()) direction = front == Vector2() ? back : front;
    const Vector2 normal(-direction.y, direction.x);
    // Keep the strip width at corners (limited, so sharp corners do not spike)
    const Vector2 segment = front == Vector2() ? back : front;
    const double miter = 1.0 / std::max(0.5, double(normal.dot(Vector2(-segment.y, segment.x))));
    const Vector3 offset(normal.x * half * miter, normal.y * half * miter, 0.0);
    left.push_back(line[i] + offset);
    right.push_back(line[i] - offset);
  }
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    triangulation::push_triangle(triangles, left[i], right[i], left[i + 1]);
    triangulation::push_triangle(triangles, left[i + 1], right[i], right[i + 1]);
  }
}

}  // namespace line_geometry

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
  ClassDB::bind_method(D_METHOD("start_build", "layers", "tile_size"), &VectorMap::start_build);
  ClassDB::bind_method(D_METHOD("is_build_done"), &VectorMap::is_build_done);
  ClassDB::bind_method(D_METHOD("take_build_result"), &VectorMap::take_build_result);

  TOPIC_SUBSCRIBER_BIND_METHODS(VectorMap);
}

bool VectorMap::generate_graph_structure()
{
  const auto last_msg = get_last_msg();
  if (!last_msg) return false;
  return decode(*last_msg.value());
}

bool VectorMap::start_build(const Array & layers, double tile_size)
{
  if (build_task_.is_running()) return false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return false;

  const auto msg = last_msg.value();
  release_last_msg();  // the worker holds the only reference: freed once the build finished
  return build_task_.start([this, msg, layers, tile_size]() {
    Dictionary result;
    Dictionary layer_vertices;
    triangulation_failures_ = 0;
    if (decode(*msg)) {
      for (int i = 0; i < layers.size(); ++i) {
        const Dictionary layer = layers[i];
        layer_vertices[layer["name"]] = split_into_tiles(build_layer(layer["parts"]), tile_size);
      }
      result["traffic_lights"] = get_traffic_light_list();
    }
    if (triangulation_failures_ > 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("godot_rviz2"), "%zu map polygons could not be triangulated",
        triangulation_failures_);
    }
    result["layers"] = layer_vertices;
    return result;
  });
}

bool VectorMap::is_build_done() { return build_task_.is_done(); }

Dictionary VectorMap::take_build_result() { return build_task_.take(); }

PackedVector3Array VectorMap::build_layer(const Array & parts)
{
  PackedVector3Array vertices;
  for (int i = 0; i < parts.size(); ++i) {
    const Array part = parts[i];
    if (part.size() < 2) continue;
    const String kind = part[0];
    const String name = part[1];
    if (kind == "shared_lines") {
      // Built directly as vertices (ROS coordinates -> Godot)
      std::vector<Vector3> ros_triangles = build_lines(part);
      const int64_t offset = vertices.size();
      vertices.resize(offset + int64_t(ros_triangles.size()));
      Vector3 * dst = vertices.ptrw();
      for (size_t j = 0; j < ros_triangles.size(); ++j) {
        const Vector3 & p = ros_triangles[j];
        dst[offset + int64_t(j)] = ros2_to_godot(p.x, p.y, p.z);
      }
      continue;
    }
    Array triangles;
    if (kind == "lanelet") {
      triangles = get_lanelet_triangle_list(name);
    } else if (kind == "polygon") {
      triangles = get_polygon_triangle_list(name);
    } else if (kind == "linestring") {
      triangles = get_linestring_triangle_list(name, part.size() > 2 ? float(part[2]) : 0.1f);
    }
    const int64_t offset = vertices.size();
    vertices.resize(offset + triangles.size());
    Vector3 * dst = vertices.ptrw();
    for (int j = 0; j < triangles.size(); ++j) {
      const Dictionary vertex = triangles[j];
      dst[offset + j] = vertex["position"];
    }
  }
  return vertices;
}

lanelet::ConstLineStrings3d VectorMap::get_shared_white_lines() const
{
  // Lane lines (line_thin / line_thick) shared by a road lanelet and another lanelet
  std::unordered_set<lanelet::ConstLineString3d> shared;
  const std::set<std::string> ground_labels = {"line_thin", "line_thick"};
  auto add_if_shared = [&](const lanelet::ConstLanelet & lanelet, const lanelet::ConstLineString3d & bound) {
    if (!has_label(bound, ground_labels)) return;
    for (const auto & candidate : lanelet_map_->laneletLayer.findUsages(bound)) {
      if (candidate == lanelet) continue;
      if (candidate.leftBound() == bound || candidate.rightBound() == bound) {
        shared.insert(bound);
        return;
      }
    }
  };
  for (const auto & lanelet : road_lanelets_) {
    add_if_shared(lanelet, lanelet.leftBound());
    add_if_shared(lanelet, lanelet.rightBound());
  }
  return lanelet::ConstLineStrings3d(shared.begin(), shared.end());
}

std::vector<Vector3> VectorMap::build_lines(const Array & part) const
{
  // ["shared_lines", width, dash, gap]: the shared lane lines, of width [m]; lines with the dashed
  // subtype are drawn as dashes of dash [m] separated by gap [m]
  auto param = [&part](int index, double fallback) {
    return part.size() > index ? double(part[index]) : fallback;
  };
  const double width = param(1, 0.05);
  std::vector<Vector3> triangles;
  for (const auto & linestring : get_shared_white_lines()) {
    const auto line = line_geometry::to_points(linestring);
    const bool dashed =
      linestring.attributeOr(lanelet::AttributeName::Subtype, "") == std::string("dashed");
    if (!dashed) {
      line_geometry::append_ribbon(line, width, triangles);
      continue;
    }
    for (const auto & dash : line_geometry::split_into_dashes(line, param(2, 1.0), param(3, 1.0))) {
      line_geometry::append_ribbon(dash, width, triangles);
    }
  }
  return triangles;
}

Array VectorMap::split_into_tiles(const PackedVector3Array & vertices, double tile_size)
{
  Array tiles;
  if (tile_size <= 0.0) {
    Dictionary tile;
    tile["center"] = Vector3();
    tile["vertices"] = vertices;
    tiles.append(tile);
    return tiles;
  }

  // Each triangle goes to the tile (on the ground plane) that contains its centroid
  std::unordered_map<uint64_t, std::vector<Vector3>> buckets;
  const Vector3 * src = vertices.ptr();
  for (int64_t i = 0; i + 2 < vertices.size(); i += 3) {
    const Vector3 centroid = (src[i] + src[i + 1] + src[i + 2]) / 3.0f;
    const auto tx = static_cast<int64_t>(std::floor(centroid.x / tile_size));
    const auto tz = static_cast<int64_t>(std::floor(centroid.z / tile_size));
    auto & bucket = buckets[(static_cast<uint64_t>(tx) << 32) ^ static_cast<uint32_t>(tz)];
    bucket.insert(bucket.end(), {src[i], src[i + 1], src[i + 2]});
  }

  for (const auto & [key, triangles] : buckets) {
    const auto tx = static_cast<int32_t>(key >> 32);
    const auto tz = static_cast<int32_t>(key & 0xFFFFFFFF);
    // Vertices relative to the tile center (better precision far from the map origin)
    const Vector3 center((tx + 0.5) * tile_size, 0.0f, (tz + 0.5) * tile_size);
    PackedVector3Array relative;
    relative.resize(static_cast<int64_t>(triangles.size()));
    Vector3 * dst = relative.ptrw();
    for (size_t i = 0; i < triangles.size(); ++i) dst[i] = triangles[i] - center;

    Dictionary tile;
    tile["center"] = center;
    tile["vertices"] = relative;
    tiles.append(tile);
  }
  return tiles;
}

bool VectorMap::decode(const autoware_map_msgs::msg::LaneletMapBin & msg)
{
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
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
    triangle_list = get_as_triangle_list(get_shared_white_lines(), width);
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

    if (!triangulation::triangulate(polygon, triangles)) ++triangulation_failures_;

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

    if (!triangulation::triangulate(polygon, triangles)) ++triangulation_failures_;

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

    auto to_points = [](const auto & bound) {
      std::vector<Vector3> points;
      for (const auto & point : bound) {
        points.emplace_back(point.basicPoint().x(), point.basicPoint().y(), point.basicPoint().z());
      }
      return points;
    };
    triangulation::triangulate_strip(
      to_points(lanelet.leftBound3d()), to_points(lanelet.rightBound3d()), triangles);

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
