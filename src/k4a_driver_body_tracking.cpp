// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// This file is only built with the Azure Kinect Body Tracking SDK
#if defined(K4A_BODY_TRACKING)

// Associated header
//
#include "azure_kinect_ros_driver/k4a_driver_node.h"

// Library headers
//
#include <sensor_msgs/image_encodings.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/subscriber_count.h"

namespace azure_kinect_ros_driver
{
namespace
{
/**
 * @brief Builds the marker of one joint of a body.
 *
 * @param body The body.
 * @param joint_type The index of the joint.
 * @param frame_id The frame of the depth camera, where the joints are expressed.
 * @param stamp The capture time.
 * @return The marker.
 */
visualization_msgs::msg::Marker makeJointMarker(const k4abt_body_t& body, int joint_type,
                                                const std::string& frame_id,
                                                const rclcpp::Time& stamp)
{
  using visualization_msgs::msg::Marker;

  const k4a_float3_t position = body.skeleton.joints[joint_type].position;
  const k4a_quaternion_t orientation = body.skeleton.joints[joint_type].orientation;

  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;

  // Set the lifetime to 0.25 to prevent flickering for even 5fps configurations.
  // New markers with the same ID will replace old markers as soon as they arrive.
  marker.lifetime = rclcpp::Duration::from_seconds(0.25);
  marker.id = body.id * 100 + joint_type;
  marker.type = Marker::SPHERE;

  const Color color = BODY_COLOR_PALETTE[body.id % BODY_COLOR_PALETTE.size()];
  marker.color.a = color.a;
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.pose.position.x = position.v[0] / 1000.0f;
  marker.pose.position.y = position.v[1] / 1000.0f;
  marker.pose.position.z = position.v[2] / 1000.0f;
  marker.pose.orientation.w = orientation.wxyz.w;
  marker.pose.orientation.x = orientation.wxyz.x;
  marker.pose.orientation.y = orientation.wxyz.y;
  marker.pose.orientation.z = orientation.wxyz.z;
  return marker;
}

/**
 * @brief Builds the body index map image of a tracked frame.
 *
 * If the pixel does not belong to a detected body its value is 255
 * (K4ABT_BODY_INDEX_MAP_BACKGROUND). If it belongs to a detected body the value is the body id
 * modulo 255, so up to body id 254 it equals the body id and afterwards it only segments the image.
 *
 * @param body_frame The tracked frame.
 * @return The image, or nullptr if the frame has no body index map.
 */
sensor_msgs::msg::Image::UniquePtr makeBodyIndexMap(const k4abt::frame& body_frame)
{
  k4a::image index_map = body_frame.get_body_index_map();
  if (!index_map)
  {
    return nullptr;
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->height = index_map.get_height_pixels();
  msg->width = index_map.get_width_pixels();
  msg->encoding = sensor_msgs::image_encodings::MONO8;
  msg->is_bigendian = false;
  msg->step = index_map.get_width_pixels() * sizeof(BodyIndexMapPixel);
  msg->data.resize(msg->height * msg->step);

  const BodyIndexMapPixel* pixels = index_map.get_buffer();
  const size_t pixel_count = index_map.get_size() / sizeof(BodyIndexMapPixel);
  for (size_t i = 0; i < pixel_count; ++i)
  {
    if (pixels[i] == K4ABT_BODY_INDEX_MAP_BACKGROUND)
    {
      msg->data[i] = K4ABT_BODY_INDEX_MAP_BACKGROUND;
    }
    else
    {
      const auto body_id = k4abt_frame_get_body_id(body_frame.handle(), pixels[i]);
      msg->data[i] = body_id % K4ABT_BODY_INDEX_MAP_BACKGROUND;
    }
  }
  return msg;
}
}  // namespace

void K4ADriverNode::bodyThread()
{
  while (running_ && rclcpp::ok())
  {
    if (k4abt_tracker_queue_size_ <= 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds{ 20 });
      continue;
    }

    k4abt::frame body_frame = k4abt_tracker_.pop_result();
    --k4abt_tracker_queue_size_;

    if (body_frame == nullptr)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Failed to pop a body frame result, skipping it");
      continue;
    }

    const rclcpp::Time capture_time = toRosTime(body_frame.get_device_timestamp());

    if (subscriberCount(body_marker_publisher_) > 0)
    {
      // Joint marker array
      auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
      const size_t body_count = body_frame.get_num_bodies();
      markers->markers.reserve(body_count * K4ABT_JOINT_COUNT);
      for (size_t i = 0; i < body_count; ++i)
      {
        const k4abt_body_t body = body_frame.get_body(i);
        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); ++joint)
        {
          markers->markers.push_back(makeJointMarker(body, joint, depth_frame_, capture_time));
        }
      }
      body_marker_publisher_->publish(std::move(markers));
    }

    if (subscriberCount(body_index_map_publisher_) > 0)
    {
      auto index_map = makeBodyIndexMap(body_frame);
      if (!index_map)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Failed to get body index map, skipping it");
        continue;
      }

      index_map->header.stamp = capture_time;
      index_map->header.frame_id = depth_frame_;
      body_index_map_publisher_->publish(std::move(index_map));
    }
  }
}
}  // namespace azure_kinect_ros_driver

#endif  // K4A_BODY_TRACKING
