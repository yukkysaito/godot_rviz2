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

#include "pointcloud.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "util.hpp"

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

void PointCloud::_bind_methods()
{
  // Bind the get_pointcloud method to Godot
  ClassDB::bind_method(D_METHOD("get_pointcloud"), &PointCloud::get_pointcloud);
  ClassDB::bind_method(
    D_METHOD("get_pointcloud_tiles", "frame_id", "voxel_size", "tile_size"),
    &PointCloud::get_pointcloud_tiles);
  ClassDB::bind_method(
    D_METHOD("start_tiles", "frame_id", "voxel_size", "tile_size", "origin"),
    &PointCloud::start_tiles);
  ClassDB::bind_method(D_METHOD("is_tiling"), &PointCloud::is_tiling);
  ClassDB::bind_method(D_METHOD("take_tiles"), &PointCloud::take_tiles);
  TOPIC_SUBSCRIBER_BIND_METHODS(PointCloud);
}

/**
 * @brief Transforms a point cloud to a specified target frame.
 *
 * @param input The input point cloud.
 * @param tf2 The TF2 buffer for looking up transformations.
 * @param target_frame The target frame to which the point cloud will be transformed.
 * @param output The output transformed point cloud.
 * @return bool True if the transformation is successful, false otherwise.
 */
bool transform_pointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  geometry_msgs::msg::TransformStamped tf_stamped{};
  try {
    // Do not wait: this runs on the render loop. If the transform for this stamp is not
    // available yet, the cloud is skipped and the next message is used.
    tf_stamped = tf2.lookupTransform(target_frame, input.header.frame_id, input.header.stamp);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("godot_rviz2"), clock, 5000, "%s", ex.what());
    return false;
  }
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

/**
 * @brief Returns the last message in frame_id (transformed if needed), or nullptr.
 */
static sensor_msgs::msg::PointCloud2::ConstSharedPtr get_msg_in_frame(
  const std::optional<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & last_msg,
  const std::string & frame_id)
{
  if (!last_msg) return nullptr;
  if (frame_id == last_msg.value()->header.frame_id) return last_msg.value();

  auto transformed_msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
  const auto tf_buffer = GodotRviz2::get_instance().get_tf_buffer();
  if (!transform_pointcloud(*(last_msg.value()), *tf_buffer, frame_id, *transformed_msg_ptr)) {
    return nullptr;
  }
  return transformed_msg_ptr;
}

PackedVector3Array PointCloud::get_pointcloud(const String & frame_id)
{
  PackedVector3Array pointcloud;
  const auto msg_ptr = get_msg_in_frame(get_last_msg(), to_std(frame_id));
  if (!msg_ptr) return pointcloud;

  // Convert the point cloud to a Godot array
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg_ptr, "x"), iter_y(*msg_ptr, "y"),
    iter_z(*msg_ptr, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Append each point to the Godot array after converting from ROS 2 to Godot's coordinate system
    pointcloud.append(ros2_to_godot(*iter_x, *iter_y, *iter_z));
  }

  return pointcloud;
}

namespace
{
// Open-addressing hash set of non-zero 64-bit keys, reused across tiles (no per-key allocation)
class VoxelSet
{
public:
  void reset(size_t expected)
  {
    size_t capacity = 16;
    while (capacity < expected * 2) capacity <<= 1;
    if (slots_.size() < capacity) slots_.resize(capacity);
    mask_ = capacity - 1;
    std::fill(slots_.begin(), slots_.begin() + capacity, 0);
  }

  // Returns true if the key was not in the set yet
  bool insert(uint64_t key)
  {
    key |= 1ULL << 63;  // keep keys non-zero (0 marks an empty slot)
    size_t i = (key * 0x9E3779B97F4A7C15ULL) >> 20 & mask_;
    while (slots_[i] != 0) {
      if (slots_[i] == key) return false;
      i = (i + 1) & mask_;
    }
    slots_[i] = key;
    return true;
  }

private:
  std::vector<uint64_t> slots_;
  size_t mask_ = 0;
};

// Byte offset of a FLOAT32 field, or -1
int float_field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}
}  // namespace

/**
 * @brief Downsamples msg and splits it into tiles (see PointCloud::get_pointcloud_tiles()),
 * passing each tile to emit, nearest to (origin_x, origin_y) [ROS coordinates] first.
 */
static void make_tiles(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg_ptr, double voxel_size,
  double tile_size, double origin_x, double origin_y,
  const std::function<void(const Dictionary &)> & emit)
{
  if (!msg_ptr || tile_size <= 0.0) return;

  const auto & msg = *msg_ptr;
  const int off_x = float_field_offset(msg, "x");
  const int off_y = float_field_offset(msg, "y");
  const int off_z = float_field_offset(msg, "z");
  if (off_x < 0 || off_y < 0 || off_z < 0) return;

  const size_t num_points = static_cast<size_t>(msg.width) * msg.height;
  const uint8_t * data = msg.data.data();
  auto point_at = [&](size_t i, float & x, float & y, float & z) {
    const size_t row = i / msg.width;
    const uint8_t * p = data + row * msg.row_step + (i - row * msg.width) * msg.point_step;
    std::memcpy(&x, p + off_x, sizeof(float));
    std::memcpy(&y, p + off_y, sizeof(float));
    std::memcpy(&z, p + off_z, sizeof(float));
  };

  // 1. Bucket the points by tile (counting sort of point indices)
  std::unordered_map<uint64_t, uint32_t> tile_ids;
  std::vector<std::pair<int64_t, int64_t>> tile_indices;
  std::vector<uint32_t> point_tile(num_points, UINT32_MAX);
  std::vector<uint32_t> tile_counts;
  for (size_t i = 0; i < num_points; ++i) {
    float x, y, z;
    point_at(i, x, y, z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const auto tx = static_cast<int64_t>(std::floor(x / tile_size));
    const auto ty = static_cast<int64_t>(std::floor(y / tile_size));
    const uint64_t key = (static_cast<uint64_t>(tx) << 32) ^ static_cast<uint32_t>(ty);
    auto [it, inserted] = tile_ids.try_emplace(key, static_cast<uint32_t>(tile_counts.size()));
    if (inserted) {
      tile_counts.push_back(0);
      tile_indices.emplace_back(tx, ty);
    }
    point_tile[i] = it->second;
    ++tile_counts[it->second];
  }
  std::vector<size_t> tile_begin(tile_counts.size() + 1, 0);
  for (size_t t = 0; t < tile_counts.size(); ++t) tile_begin[t + 1] = tile_begin[t] + tile_counts[t];
  std::vector<uint32_t> order(tile_begin.back());
  {
    std::vector<size_t> cursor(tile_begin.begin(), tile_begin.end() - 1);
    for (size_t i = 0; i < num_points; ++i) {
      if (point_tile[i] != UINT32_MAX) order[cursor[point_tile[i]]++] = static_cast<uint32_t>(i);
    }
  }
  std::vector<uint32_t>().swap(point_tile);

  // Process the tiles nearest to the origin first (they are shown first while tiling runs)
  std::vector<uint32_t> tile_order(tile_counts.size());
  std::vector<double> tile_distance(tile_counts.size());
  for (size_t t = 0; t < tile_counts.size(); ++t) {
    tile_order[t] = static_cast<uint32_t>(t);
    const double dx = (tile_indices[t].first + 0.5) * tile_size - origin_x;
    const double dy = (tile_indices[t].second + 0.5) * tile_size - origin_y;
    tile_distance[t] = dx * dx + dy * dy;
  }
  std::sort(tile_order.begin(), tile_order.end(), [&](uint32_t a, uint32_t b) {
    return tile_distance[a] < tile_distance[b];
  });

  // 2. Per tile: keep one point per voxel, convert relative to the tile center
  const bool downsample = voxel_size > 0.0;
  VoxelSet voxels;
  std::vector<Vector3> kept;
  for (const uint32_t t : tile_order) {
    kept.clear();
    if (downsample) voxels.reset(tile_counts[t]);
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (size_t k = tile_begin[t]; k < tile_begin[t + 1]; ++k) {
      float x, y, z;
      point_at(order[k], x, y, z);
      if (downsample) {
        const auto ix = static_cast<uint64_t>(static_cast<int64_t>(std::floor(x / voxel_size)));
        const auto iy = static_cast<uint64_t>(static_cast<int64_t>(std::floor(y / voxel_size)));
        const auto iz = static_cast<uint64_t>(static_cast<int64_t>(std::floor(z / voxel_size)));
        if (!voxels.insert(((ix & 0x1FFFFF) << 42) | ((iy & 0x1FFFFF) << 21) | (iz & 0x1FFFFF))) {
          continue;
        }
      }
      kept.push_back(ros2_to_godot(x, y, z));
      sum_x += x;
      sum_y += y;
      sum_z += z;
    }
    if (kept.empty()) continue;

    const double n = static_cast<double>(kept.size());
    const Vector3 center = ros2_to_godot(sum_x / n, sum_y / n, sum_z / n);
    PackedVector3Array points;
    points.resize(static_cast<int64_t>(kept.size()));
    Vector3 * dst = points.ptrw();
    for (size_t i = 0; i < kept.size(); ++i) {
      dst[i] = kept[i] - center;  // relative to the tile center (better precision)
    }

    Dictionary tile_dict;
    tile_dict["center"] = center;
    tile_dict["points"] = points;
    emit(tile_dict);
  }
}

Array PointCloud::get_pointcloud_tiles(const String & frame_id, double voxel_size, double tile_size)
{
  Array tiles;
  make_tiles(
    get_msg_in_frame(get_last_msg(), to_std(frame_id)), voxel_size, tile_size, 0.0, 0.0,
    [&tiles](const Dictionary & tile) { tiles.append(tile); });
  return tiles;
}

bool PointCloud::start_tiles(
  const String & frame_id, double voxel_size, double tile_size, const Vector3 & origin)
{
  if (tiles_task_.is_running()) return false;
  const auto last_msg = get_last_msg();
  if (!last_msg) return false;
  release_last_msg();  // the worker holds the only reference: freed once tiling finished

  {
    std::lock_guard<std::mutex> lock(tile_queue_->mutex);
    tile_queue_->tiles.clear();
  }
  const auto queue = tile_queue_;
  // Godot -> ROS coordinates (see ros2_to_godot())
  const double origin_x = origin.x;
  const double origin_y = -origin.z;
  return tiles_task_.start([last_msg, frame = to_std(frame_id), voxel_size, tile_size, origin_x,
                            origin_y, queue]() {
    make_tiles(
      get_msg_in_frame(last_msg, frame), voxel_size, tile_size, origin_x, origin_y,
      [&queue](const Dictionary & tile) {
        std::lock_guard<std::mutex> lock(queue->mutex);
        queue->tiles.append(tile);
      });
    return true;
  });
}

bool PointCloud::is_tiling()
{
  if (tiles_task_.is_running()) return true;
  std::lock_guard<std::mutex> lock(tile_queue_->mutex);
  return !tile_queue_->tiles.is_empty();
}

Array PointCloud::take_tiles()
{
  std::lock_guard<std::mutex> lock(tile_queue_->mutex);
  Array tiles = tile_queue_->tiles;
  tile_queue_->tiles = Array();
  return tiles;
}
