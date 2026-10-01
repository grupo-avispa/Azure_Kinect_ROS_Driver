// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_driver_node.h"

// Project headers
//
#include "azure_kinect_ros_driver/subscriber_count.h"

namespace azure_kinect_ros_driver
{
namespace
{
/** @brief Bytes of one BGRA pixel. */
constexpr size_t kBgraBytesPerPixel = 4;
}  // namespace

bool K4ADriverNode::hasSubscribers(const ImagePublisher& image_publisher,
                                   const InfoPublisher& info_publisher) const
{
  return subscriberCount(image_publisher) > 0 || subscriberCount(info_publisher) > 0;
}

bool K4ADriverNode::hasImages(bool needs_depth, bool needs_color, bool needs_ir,
                              const k4a::capture& capture) const
{
  // Recordings may not have synchronized captures: skip the streams whose images are missing
  if (source_->capturesAreComplete())
  {
    return true;
  }
  return (!needs_depth || capture.get_depth_image()) &&
         (!needs_color || capture.get_color_image()) && (!needs_ir || capture.get_ir_image());
}

void K4ADriverNode::publishImageWithInfo(const ImagePublisher& image_publisher,
                                         sensor_msgs::msg::Image::UniquePtr image,
                                         const InfoPublisher& info_publisher,
                                         sensor_msgs::msg::CameraInfo& camera_info,
                                         const rclcpp::Time& stamp, const std::string& frame_id)
{
  // The camera info message is cached, so its stamp has to follow the image
  image->header.stamp = stamp;
  image->header.frame_id = frame_id;
  camera_info.header.stamp = stamp;

  image_publisher->publish(std::move(image));
  info_publisher->publish(camera_info);
}

bool K4ADriverNode::publishCapture(const k4a::capture& capture)
{
  bool succeeded = true;

  if (params_.depth_enabled)
  {
    succeeded &= publishIr(capture);

    // Depth images are not available in PASSIVE_IR mode
    if (calibration_data_->k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR)
    {
      succeeded &= publishDepth(capture);
      succeeded &= publishDepthToRgb(capture);

#if defined(K4A_BODY_TRACKING)
      // Hand the capture to the body tracker when something listens to its results
      if (params_.body_tracking_enabled && k4abt_tracker_queue_size_ < 3 &&
          (subscriberCount(body_marker_publisher_) > 0 ||
           subscriberCount(body_index_map_publisher_) > 0))
      {
        if (!k4abt_tracker_.enqueue_capture(capture))
        {
          succeeded = false;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "Failed to add the capture to the tracker queue, skipping it");
        }
        else
        {
          ++k4abt_tracker_queue_size_;
        }
      }
#endif
    }
  }

  if (params_.color_enabled)
  {
    succeeded &= publishColor(capture);
    succeeded &= publishRgbToDepth(capture);
  }

  succeeded &= publishPointCloud(capture);
  return succeeded;
}

bool K4ADriverNode::publishIr(const k4a::capture& capture)
{
  if (!hasSubscribers(ir_publisher_, ir_info_publisher_) || !hasImages(false, false, true, capture))
  {
    return true;
  }

  // IR images are available in all depth modes
  const k4a::image ir = capture.get_ir_image();
  auto msg =
    conversions::irToImage(ir, params_.rescale_ir_to_mono8, params_.ir_mono8_scaling_factor);
  if (!msg)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to get raw IR frame, skipping it");
    return false;
  }

  publishImageWithInfo(ir_publisher_, std::move(msg), ir_info_publisher_, ir_info_,
                       toRosTime(ir.get_device_timestamp()), depth_frame_);
  return true;
}

bool K4ADriverNode::publishDepth(const k4a::capture& capture)
{
  if (!hasSubscribers(depth_publisher_, depth_info_publisher_) ||
      !hasImages(true, false, false, capture))
  {
    return true;
  }

  const k4a::image depth = capture.get_depth_image();
  auto msg = conversions::depthToImage(depth, depth_unit_);
  if (!msg)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to get raw depth frame, skipping it");
    return false;
  }

  publishImageWithInfo(depth_publisher_, std::move(msg), depth_info_publisher_, depth_info_,
                       toRosTime(depth.get_device_timestamp()), depth_frame_);
  return true;
}

bool K4ADriverNode::publishDepthToRgb(const k4a::capture& capture)
{
  // The depth can only be moved into the color co-ordinates if the color camera is enabled
  if (!params_.color_enabled ||
      !hasSubscribers(depth_to_rgb_publisher_, depth_to_rgb_info_publisher_) ||
      !hasImages(true, false, false, capture))
  {
    return true;
  }

  const k4a::image depth = capture.get_depth_image();
  calibration_data_->k4a_transformation_.depth_image_to_color_camera(
    depth, &calibration_data_->transformed_depth_image_);

  auto msg = conversions::depthToImage(calibration_data_->transformed_depth_image_, depth_unit_);
  if (!msg)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to get rectified depth frame, skipping it");
    return false;
  }

  publishImageWithInfo(depth_to_rgb_publisher_, std::move(msg), depth_to_rgb_info_publisher_,
                       depth_to_rgb_info_, toRosTime(depth.get_device_timestamp()), rgb_frame_);
  return true;
}

bool K4ADriverNode::publishColor(const k4a::capture& capture)
{
  if (!hasImages(false, true, false, capture))
  {
    return true;
  }

  if (params_.color_format == "jpeg")
  {
    if (subscriberCount(rgb_jpeg_publisher_) == 0 && subscriberCount(rgb_info_publisher_) == 0)
    {
      return true;
    }

    const k4a::image jpeg = capture.get_color_image();
    auto msg = conversions::jpegToCompressed(jpeg);
    if (!msg)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Failed to get Jpeg frame, skipping it");
      return false;
    }

    const rclcpp::Time stamp = toRosTime(jpeg.get_device_timestamp());
    msg->header.stamp = stamp;
    msg->header.frame_id = rgb_frame_;
    rgb_jpeg_publisher_->publish(std::move(msg));

    rgb_info_.header.stamp = stamp;
    rgb_info_publisher_->publish(rgb_info_);
    return true;
  }

  if (!hasSubscribers(rgb_publisher_, rgb_info_publisher_))
  {
    return true;
  }

  const k4a::image bgra = capture.get_color_image();
  auto msg = conversions::bgraToImage(bgra);
  if (!msg)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Invalid k4a_bgra_frame returned from K4A");
    return false;
  }

  publishImageWithInfo(rgb_publisher_, std::move(msg), rgb_info_publisher_, rgb_info_,
                       toRosTime(bgra.get_device_timestamp()), rgb_frame_);
  return true;
}

bool K4ADriverNode::publishRgbToDepth(const k4a::capture& capture)
{
  // The color can only be moved into the depth co-ordinates if the depth camera is enabled and
  // gives depth data
  if (params_.color_format != "bgra" || !params_.depth_enabled ||
      calibration_data_->k4a_calibration_.depth_mode == K4A_DEPTH_MODE_PASSIVE_IR ||
      !hasSubscribers(rgb_to_depth_publisher_, rgb_to_depth_info_publisher_) ||
      !hasImages(true, true, false, capture))
  {
    return true;
  }

  const k4a::image bgra = capture.get_color_image();
  const bool tightly_packed = bgra.get_size() == static_cast<size_t>(bgra.get_width_pixels()) *
                                                   bgra.get_height_pixels() * kBgraBytesPerPixel;
  if (!tightly_packed)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Invalid k4a_bgra_frame returned from K4A");
    return false;
  }

  calibration_data_->k4a_transformation_.color_image_to_depth_camera(
    capture.get_depth_image(), bgra, &calibration_data_->transformed_rgb_image_);

  auto msg = conversions::bgraToImage(calibration_data_->transformed_rgb_image_);
  if (!msg)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to get rectified RGB frame, skipping it");
    return false;
  }

  publishImageWithInfo(rgb_to_depth_publisher_, std::move(msg), rgb_to_depth_info_publisher_,
                       rgb_to_depth_info_, toRosTime(bgra.get_device_timestamp()), depth_frame_);
  return true;
}

bool K4ADriverNode::publishPointCloud(const k4a::capture& capture)
{
  if (subscriberCount(point_cloud_publisher_) == 0 ||
      !(params_.point_cloud || params_.rgb_point_cloud) || !hasImages(true, true, false, capture))
  {
    return true;
  }

  auto cloud = buildPointCloud(capture);
  if (!cloud)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to get Point Cloud, skipping it");
    return false;
  }

  point_cloud_publisher_->publish(std::move(cloud));
  return true;
}

sensor_msgs::msg::PointCloud2::UniquePtr K4ADriverNode::buildPointCloud(const k4a::capture& capture)
{
  const k4a::image depth = capture.get_depth_image();
  if (!depth)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Cannot render point cloud: no depth frame");
    return nullptr;
  }

  const k4a::transformation& transformation = calibration_data_->k4a_transformation_;
  sensor_msgs::msg::PointCloud2::UniquePtr cloud;
  std::string frame_id = depth_frame_;

  if (params_.rgb_point_cloud)
  {
    const k4a::image bgra = capture.get_color_image();
    if (!bgra)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Cannot render RGB point cloud: no BGRA frame");
      return nullptr;
    }

    if (params_.point_cloud_in_depth_frame)
    {
      // Transform the color image into the depth camera frame, and the depth into a cloud
      transformation.color_image_to_depth_camera(depth, bgra,
                                                 &calibration_data_->transformed_rgb_image_);
      transformation.depth_image_to_point_cloud(depth, K4A_CALIBRATION_TYPE_DEPTH,
                                                &calibration_data_->point_cloud_image_);
      cloud = conversions::pointCloudToMsg(calibration_data_->point_cloud_image_,
                                           &calibration_data_->transformed_rgb_image_);
    }
    else
    {
      // Transform the depth into the color camera geometry, and build the cloud from there
      transformation.depth_image_to_color_camera(depth,
                                                 &calibration_data_->transformed_depth_image_);
      transformation.depth_image_to_point_cloud(calibration_data_->transformed_depth_image_,
                                                K4A_CALIBRATION_TYPE_COLOR,
                                                &calibration_data_->point_cloud_image_);
      cloud = conversions::pointCloudToMsg(calibration_data_->point_cloud_image_, &bgra);
      frame_id = rgb_frame_;
    }
  }
  else
  {
    transformation.depth_image_to_point_cloud(depth, K4A_CALIBRATION_TYPE_DEPTH,
                                              &calibration_data_->point_cloud_image_);
    cloud = conversions::pointCloudToMsg(calibration_data_->point_cloud_image_, nullptr);
  }

  if (cloud)
  {
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = toRosTime(depth.get_device_timestamp());
  }
  return cloud;
}
}  // namespace azure_kinect_ros_driver
