// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef CAPTURE_SOURCE_H
#define CAPTURE_SOURCE_H

// System headers
//
#include <chrono>
#include <memory>
#include <string>

// Library headers
//
#include <k4a/k4a.hpp>
#include <rclcpp/logger.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"

namespace azure_kinect_ros_driver
{
/**
 * @brief Where the captures and IMU samples come from: a device or a recording.
 *
 * The node only talks to this interface, so that it does not need to know whether the data is
 * live, and tests can feed it synthetic data.
 *
 * `nextCapture()` and `nextImuSample()` are meant to be called from two different threads; the
 * implementations make that safe.
 */
class CaptureSource
{
public:
  /**
   * @brief Outcome of waiting for the next capture or IMU sample.
   */
  enum class Status
  {
    /** @brief A new element was returned. */
    kOk,
    /** @brief A new capture was returned after the recording was rewound to its beginning. */
    kRestarted,
    /** @brief Nothing arrived in time. */
    kTimeout,
    /** @brief There is nothing more to read. */
    kEndOfStream
  };

  /**
   * @brief Identification of a physical device. All the fields are empty for a recording.
   */
  struct DeviceInfo
  {
    /** @brief Serial number. */
    std::string serial_number;
    /** @brief Firmware version of the RGB camera. */
    std::string rgb_version;
    /** @brief Firmware version of the depth camera. */
    std::string depth_version;
    /** @brief Firmware version of the audio device. */
    std::string audio_version;
    /** @brief Firmware version of the depth sensor. */
    std::string depth_sensor_version;
  };

  virtual ~CaptureSource() = default;

  /**
   * @brief Describes the source for the logs.
   *
   * @return For example the serial number or the path of the recording.
   */
  virtual std::string description() const = 0;

  /**
   * @brief Identifies the physical device.
   *
   * @return The identification, empty for a recording.
   */
  virtual DeviceInfo deviceInfo() const = 0;

  /**
   * @brief Whether every capture holds all the images that were enabled.
   *
   * A recording may hold captures that miss some of them, so the node has to check.
   *
   * @return True for a device.
   */
  virtual bool capturesAreComplete() const = 0;

  /**
   * @brief Whether the images carry the time at which they arrived at the host.
   *
   * @return True for a device.
   */
  virtual bool providesSystemTimestamps() const = 0;

  /**
   * @brief The calibration of the cameras.
   *
   * @return The calibration of the device or of the recording.
   */
  virtual const k4a::calibration& calibration() const = 0;

  /**
   * @brief Starts the cameras and the IMU. Does nothing if they are already started.
   *
   * @throws k4a::error If the device cannot start, for example for an unsupported configuration.
   */
  virtual void start() = 0;

  /**
   * @brief Stops the cameras and the IMU. Does nothing if they are not started.
   */
  virtual void stop() = 0;

  /**
   * @brief Waits for the next capture.
   *
   * A recording is paced at the frame rate of the cameras, and ignores the timeout.
   *
   * @param capture The capture that was read.
   * @param timeout How long to wait for it, for a device.
   * @return `kOk` or `kRestarted` if a capture was read, `kTimeout` or `kEndOfStream` if not.
   * @throws k4a::error On a failure of the SDK.
   */
  virtual Status nextCapture(k4a::capture& capture, std::chrono::milliseconds timeout) = 0;

  /**
   * @brief Waits for the next IMU sample.
   *
   * A recording only gives the samples that are not ahead of the last capture it returned, so
   * that both streams stay in step.
   *
   * @param sample The sample that was read.
   * @param timeout How long to wait for it.
   * @return `kOk` if a sample was read, otherwise `kTimeout` or `kEndOfStream`.
   * @throws k4a::error On a failure of the SDK.
   */
  virtual Status nextImuSample(k4a_imu_sample_t& sample, std::chrono::milliseconds timeout) = 0;
};

/**
 * @brief Gets the timestamp of a capture, which is the one of its images.
 *
 * Captures do not have a timestamp, but their images do. The IR image is checked first because in
 * passive IR mode a capture has an IR image but no depth image, and there is no mode with a depth
 * image but no IR image.
 *
 * @param capture The capture.
 * @return The device timestamp of the first image that is found, or zero if there is none.
 */
std::chrono::microseconds captureTimestamp(const k4a::capture& capture);

/**
 * @brief Opens the source that the parameters ask for: a recording or a device.
 *
 * For a recording the parameters are adapted to what it contains, for example the color camera is
 * disabled if it has no color track.
 *
 * @param params The parameters, which may be changed.
 * @param logger The logger for the messages while opening.
 * @return The source, not started.
 * @throws std::runtime_error If the source cannot be opened or does not fit the parameters.
 */
std::unique_ptr<CaptureSource> makeCaptureSource(K4AROSDeviceParams& params, rclcpp::Logger logger);
}  // namespace azure_kinect_ros_driver

#endif  // CAPTURE_SOURCE_H
