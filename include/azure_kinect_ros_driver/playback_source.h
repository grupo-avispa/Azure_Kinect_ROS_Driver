// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef PLAYBACK_SOURCE_H
#define PLAYBACK_SOURCE_H

// System headers
//
#include <atomic>
#include <mutex>

// Library headers
//
#include <k4arecord/playback.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/capture_source.h"

namespace azure_kinect_ros_driver
{
/**
 * @brief Adapts the parameters to the content of a recording.
 *
 * The frame rate is taken from the recording, and the color camera, the depth camera and the point
 * clouds are disabled if the recording has no track for them.
 *
 * @param record_config The configuration of the recording.
 * @param params The parameters to adapt.
 * @param logger The logger for the warnings.
 */
void adaptParamsToRecording(const k4a_record_configuration_t& record_config,
                            K4AROSDeviceParams& params, rclcpp::Logger logger);

/**
 * @brief Captures and IMU samples read from a recording.
 */
class PlaybackSource : public CaptureSource
{
public:
  /**
   * @brief Opens a recording.
   *
   * @param params The parameters; they are adapted to the content of the recording.
   * @param logger The logger for the messages while opening.
   * @throws std::runtime_error If the recording cannot be opened or its color format cannot be
   * converted to the one that was asked for.
   */
  PlaybackSource(K4AROSDeviceParams& params, rclcpp::Logger logger);

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
  /** @brief The path of the recording. */
  std::string path_;

  /** @brief Whether to start again at the end of the recording. */
  bool loop_;

  /** @brief Time between two captures. */
  std::chrono::nanoseconds frame_period_;

  /** @brief The open recording. */
  k4a::playback playback_;

  /** @brief The calibration stored in the recording. */
  k4a::calibration calibration_;

  /** @brief Serializes the reads of the capture and the IMU threads. */
  std::mutex playback_mutex_;

  /** @brief When the next capture is due. */
  std::chrono::steady_clock::time_point next_capture_time_;

  /** @brief Timestamp of the last capture returned, to keep the IMU samples in step. */
  std::atomic_uint64_t last_capture_time_usec_{ 0 };

  /** @brief Timestamp of the last IMU sample returned. */
  std::atomic_uint64_t last_imu_time_usec_{ 0 };

  /** @brief Whether the IMU track has been read to its end. */
  std::atomic_bool imu_stream_end_of_file_{ false };
};
}  // namespace azure_kinect_ros_driver

#endif  // PLAYBACK_SOURCE_H
