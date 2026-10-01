// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef DEVICE_SOURCE_H
#define DEVICE_SOURCE_H

// Project headers
//
#include "azure_kinect_ros_driver/capture_source.h"

namespace azure_kinect_ros_driver
{
/**
 * @brief Captures and IMU samples of a physical Azure Kinect.
 */
class DeviceSource : public CaptureSource
{
public:
  /**
   * @brief Opens a device.
   *
   * @param serial_number The serial number of the device to open, or empty for the first one.
   * @param configuration The configuration the cameras will be started with.
   * @param logger The logger for the messages while opening.
   * @throws std::runtime_error If no matching device can be opened.
   * @throws k4a::error If the calibration for the configuration cannot be read.
   */
  DeviceSource(const std::string& serial_number, const k4a_device_configuration_t& configuration,
               rclcpp::Logger logger);

  ~DeviceSource() override;

  std::string description() const override;
  DeviceInfo deviceInfo() const override;
  bool capturesAreComplete() const override;
  bool providesSystemTimestamps() const override;
  const k4a::calibration& calibration() const override;
  void start() override;
  void stop() override;
  Status nextCapture(k4a::capture& capture, std::chrono::milliseconds timeout) override;
  Status nextImuSample(k4a_imu_sample_t& sample, std::chrono::milliseconds timeout) override;

private:
  /** @brief The open device. */
  k4a::device device_;

  /** @brief The configuration the cameras are started with. */
  k4a_device_configuration_t configuration_;

  /** @brief The calibration for that configuration. */
  k4a::calibration calibration_;

  /** @brief Whether the cameras and the IMU are running. */
  bool started_ = false;
};
}  // namespace azure_kinect_ros_driver

#endif  // DEVICE_SOURCE_H
