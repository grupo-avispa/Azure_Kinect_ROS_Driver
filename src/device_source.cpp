// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/device_source.h"

// System headers
//
#include <sstream>
#include <stdexcept>

// Library headers
//
#include <rclcpp/logging.hpp>

namespace azure_kinect_ros_driver
{
namespace
{
/**
 * @brief Formats a firmware version as `major.minor.iteration`.
 *
 * @param version The version.
 * @return The text.
 */
std::string versionToString(const k4a_version_t& version)
{
  std::ostringstream text;
  text << version.major << "." << version.minor << "." << version.iteration;
  return text.str();
}

/**
 * @brief Opens the device that matches a serial number, or the first one if there is none given.
 *
 * @param serial_number The serial number, or empty.
 * @param logger The logger for the messages.
 * @return The open device.
 * @throws std::runtime_error If no matching device can be opened.
 */
k4a::device openDevice(const std::string& serial_number, rclcpp::Logger logger)
{
  const uint32_t device_count = k4a::device::get_installed_count();
  RCLCPP_INFO_STREAM(logger, "Found " << device_count << " sensors");

  if (!serial_number.empty())
  {
    RCLCPP_INFO_STREAM(logger, "Searching for sensor with serial number: " << serial_number);
  }
  else
  {
    RCLCPP_INFO(logger, "No serial number provided: picking first sensor");
    RCLCPP_WARN_EXPRESSION(logger, device_count > 1,
                           "Multiple sensors connected! Picking first sensor.");
  }

  for (uint32_t i = 0; i < device_count; i++)
  {
    k4a::device device;
    try
    {
      device = k4a::device::open(i);
    }
    catch (const std::exception&)
    {
      RCLCPP_ERROR_STREAM(logger, "Failed to open K4A device at index " << i);
      continue;
    }

    RCLCPP_INFO_STREAM(logger, "K4A[" << i << "] : " << device.get_serialnum());

    // Pick the first device, or the one with the serial number that was asked for
    if (serial_number.empty() ? i == 0 : device.get_serialnum() == serial_number)
    {
      return device;
    }
  }

  throw std::runtime_error("Failed to open a K4A device");
}
}  // namespace

DeviceSource::DeviceSource(const std::string& serial_number,
                           const k4a_device_configuration_t& configuration, rclcpp::Logger logger)
  : device_(openDevice(serial_number, logger)), configuration_(configuration)
{
  const DeviceInfo info = deviceInfo();
  RCLCPP_INFO_STREAM(logger, "K4A Serial Number: " << info.serial_number);
  RCLCPP_INFO_STREAM(logger, "RGB Version: " << info.rgb_version);
  RCLCPP_INFO_STREAM(logger, "Depth Version: " << info.depth_version);
  RCLCPP_INFO_STREAM(logger, "Audio Version: " << info.audio_version);
  RCLCPP_INFO_STREAM(logger, "Depth Sensor Version: " << info.depth_sensor_version);

  calibration_ =
    device_.get_calibration(configuration_.depth_mode, configuration_.color_resolution);
}

DeviceSource::~DeviceSource()
{
  try
  {
    stop();
  }
  catch (...)
  {
    // Nothing can be done about a device that cannot be stopped while destroying it
  }
}

std::string DeviceSource::description() const
{
  return "device " + device_.get_serialnum();
}

CaptureSource::DeviceInfo DeviceSource::deviceInfo() const
{
  const k4a_hardware_version_t version = device_.get_version();
  return DeviceInfo{ device_.get_serialnum(), versionToString(version.rgb),
                     versionToString(version.depth), versionToString(version.audio),
                     versionToString(version.depth_sensor) };
}

bool DeviceSource::capturesAreComplete() const
{
  return true;
}

bool DeviceSource::providesSystemTimestamps() const
{
  return true;
}

const k4a::calibration& DeviceSource::calibration() const
{
  return calibration_;
}

void DeviceSource::start()
{
  if (started_)
  {
    return;
  }

  device_.start_cameras(&configuration_);
  started_ = true;
  device_.start_imu();
}

void DeviceSource::stop()
{
  if (!started_)
  {
    return;
  }

  started_ = false;
  device_.stop_imu();
  device_.stop_cameras();
}

CaptureSource::Status DeviceSource::nextCapture(k4a::capture& capture,
                                                std::chrono::milliseconds timeout)
{
  return device_.get_capture(&capture, timeout) ? Status::kOk : Status::kTimeout;
}

CaptureSource::Status DeviceSource::nextImuSample(k4a_imu_sample_t& sample,
                                                  std::chrono::milliseconds timeout)
{
  return device_.get_imu_sample(&sample, timeout) ? Status::kOk : Status::kTimeout;
}
}  // namespace azure_kinect_ros_driver
