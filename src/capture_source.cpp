// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/capture_source.h"

// System headers
//
#include <stdexcept>

// Project headers
//
#include "azure_kinect_ros_driver/device_source.h"
#include "azure_kinect_ros_driver/playback_source.h"

namespace azure_kinect_ros_driver
{
std::chrono::microseconds captureTimestamp(const k4a::capture& capture)
{
  const k4a::image ir_image = capture.get_ir_image();
  if (ir_image)
  {
    return ir_image.get_device_timestamp();
  }

  const k4a::image color_image = capture.get_color_image();
  if (color_image)
  {
    return color_image.get_device_timestamp();
  }

  return std::chrono::microseconds::zero();
}

std::unique_ptr<CaptureSource> makeCaptureSource(K4AROSDeviceParams& params, rclcpp::Logger logger)
{
  if (!params.recording_file.empty())
  {
    return std::make_unique<PlaybackSource>(params, logger);
  }

  k4a_device_configuration_t configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  if (params.GetDeviceConfig(&configuration) != K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to generate a device configuration");
  }

  return std::make_unique<DeviceSource>(params.sensor_sn, configuration, logger);
}
}  // namespace azure_kinect_ros_driver
