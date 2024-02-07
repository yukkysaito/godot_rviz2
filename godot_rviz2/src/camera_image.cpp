#include "camera_image.hpp"

#include "iostream"

CameraImage::CameraImage() {}
CameraImage::~CameraImage() {}

void CameraImage::_bind_methods()
{
  ClassDB::bind_method(D_METHOD("get_image"), &CameraImage::get_image);
  ClassDB::bind_method(D_METHOD("subscribe"), &CameraImage::subscribe);
  ClassDB::bind_method(D_METHOD("has_new"), &CameraImage::has_new);
  ClassDB::bind_method(D_METHOD("set_old"), &CameraImage::set_old);
}

Ref<Image> CameraImage::get_image()
{
  auto last_msg = get_last_msg();
  if (!last_msg) return image_;

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(last_msg.value(), last_msg.value()->encoding);
  } catch (cv_bridge::Exception & e) {
    std::cerr << ("cv_bridge exception: %s", e.what()) << std::endl;
    return image_;
  }

  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;

  if (last_msg.value()->encoding == sensor_msgs::image_encodings::BGR8) {
    cv::Mat rgb_image;
    cv::cvtColor(in_image_ptr->image, rgb_image, cv::COLOR_BGR2RGB);
    memcpy(bytes_.ptrw(), (char *)rgb_image.data, width * height * 3);
  } else if (last_msg.value()->encoding == sensor_msgs::image_encodings::RGB8) {
    memcpy(bytes_.ptrw(), (char *)in_image_ptr->image.data, width * height * 3);
  } else {
    std::cerr << "Unsupported image encoding: " << last_msg.value()->encoding << std::endl;
    return image_;
  }

  image_->set_data(width, height, false, Image::FORMAT_RGB8, bytes_);

  return image_;
}
