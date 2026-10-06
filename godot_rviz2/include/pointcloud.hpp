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
#include "async_task.hpp"
#include "topic_subscriber.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <memory>
#include <mutex>
#include <vector>

/**
 * @brief A tile of a point cloud map with its points at several levels of detail, quantized to 16
 * bits per axis within the bounds of the tile (min + q * scale, Godot coordinates relative to
 * center).
 */
struct PointCloudTile
{
  int64_t grid_x = 0;
  int64_t grid_y = 0;
  Vector3 center;
  Vector3 min;
  Vector3 scale;
  std::vector<std::vector<uint16_t>> levels;  // x, y, z per point

  PackedVector3Array get_points(size_t level) const;
};

/**
 * @class PointCloud
 * @brief The PointCloud class provides an interface to process and retrieve data from PointCloud2
 * sensor messages.
 *
 * This class subscribes to PointCloud2 messages and converts them into a Godot-friendly format.
 */
class PointCloud : public RefCounted
{
  GDCLASS(PointCloud, RefCounted);
  TOPIC_SUBSCRIBER(PointCloud, sensor_msgs::msg::PointCloud2);

public:
  /**
   * @brief Retrieves the point cloud data, optionally transforming it to a specified frame.
   *
   * @param frame_id The target frame ID to which the point cloud should be transformed. Defaults to
   * "map".
   * @return PackedVector3Array A Godot array containing the point cloud data.
   */
  PackedVector3Array get_pointcloud(const String & frame_id = "map");

  /**
   * @brief Splits the point cloud into square tiles on the ground plane and downsamples each tile
   * at several levels of detail, on a worker thread (for large maps).
   *
   * Level k keeps one point per voxel of voxel_sizes[k] [m] (0: no downsampling); sizes should
   * increase. Tiles nearest to origin (Godot coordinates, e.g. the ego position) are produced
   * first and can be taken while tiling runs. The points are kept here compactly (16 bits per
   * axis) and decoded on request with get_tile_points().
   *
   * The message is released once tiling started, to free its memory: start_tiles() works again
   * only after a new message arrives. The tiles of the previous call are discarded.
   *
   * @return false if there is no message or tiling is already running.
   */
  bool start_tiles(
    const String & frame_id, const PackedFloat64Array & voxel_sizes, double tile_size,
    const Vector3 & origin);

  /// True while tiling runs or produced tiles have not been taken yet.
  bool is_tiling();

  /**
   * @brief Takes the tiles produced since the last call.
   * @return Array of Dictionary {"id": int, "center": Vector3, "grid": Vector2i (tile index in
   *   ROS x/y), "counts": PackedInt32Array (points per level)}
   */
  Array take_tiles();

  /// Points of a tile at a level, relative to the tile center (Godot coordinates).
  PackedVector3Array get_tile_points(int64_t id, int64_t level);

  PointCloud() = default;
  ~PointCloud() = default;

protected:
  /**
   * @brief Binds methods to the Godot system.
   */
  static void _bind_methods();

private:
  struct TileStore
  {
    std::mutex mutex;
    std::vector<std::shared_ptr<const PointCloudTile>> tiles;
  };
  // Shared with the worker thread, which appends tiles
  std::shared_ptr<TileStore> tiles_ = std::make_shared<TileStore>();
  size_t tiles_taken_ = 0;
  AsyncTask<bool> tiles_task_;
};
