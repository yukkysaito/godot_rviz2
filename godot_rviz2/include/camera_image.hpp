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

#include "core/io/image.h"
#include "core/variant/variant.h"
#include "cv_bridge/cv_bridge.h"
#include "godot_rviz2.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/qos.hpp"
#include "scene/gui/texture_rect.h"
#include "util.hpp"

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <optional>

class CameraImage : public TextureRect
{
  GDCLASS(CameraImage, TextureRect);
  PackedByteArray bytes_;
  Ref<Image> image_;

  using ConstSharedPtr = typename sensor_msgs::msg::Image::ConstSharedPtr;
  ConstSharedPtr msg_ptr_;
  bool has_new_ = false;
  image_transport::Subscriber subscription_;
  void on_callback(const ConstSharedPtr msg)
  {
    if (msg_ptr_ == nullptr) {
      bytes_.resize(msg->height * msg->width * 3);
      image_ = Image::create_from_data(msg->width, msg->height, false, Image::FORMAT_RGB8, bytes_);
    }
    msg_ptr_ = msg;
    has_new_ = true;
  }
  std::optional<ConstSharedPtr> get_last_msg()
  {
    if (!msg_ptr_) return std::nullopt;
    return msg_ptr_;
  }

public:
  CameraImage();
  ~CameraImage();
  Ref<Image> get_image();
  bool has_new() { return has_new_; }
  void set_old() { has_new_ = false; }
  void subscribe(const String & topic, const String & image_type)
  {
    subscription_ = image_transport::create_subscription(
      GodotRviz2::get_instance().get_node().get(), to_std(topic),
      std::bind(&CameraImage::on_callback, this, std::placeholders::_1), to_std(image_type),
      rmw_qos_profile_sensor_data);
  }

protected:
  static void _bind_methods();
};
