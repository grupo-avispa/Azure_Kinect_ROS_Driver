// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/playback_source.h"

// System headers
//
#include <algorithm>
#include <stdexcept>
#include <thread>

// Library headers
//
#include <rclcpp/logging.hpp>

namespace azure_kinect_ros_driver
{
void adaptParamsToRecording(const k4a_record_configuration_t& record_config,
                            K4AROSDeviceParams& params, rclcpp::Logger logger)
{
  // Use the frame rate of the recording for a correct pacing of the frame thread
  switch (record_config.camera_fps)
  {
    case K4A_FRAMES_PER_SECOND_5:
      params.fps = 5;
      break;
    case K4A_FRAMES_PER_SECOND_15:
      params.fps = 15;
      break;
    case K4A_FRAMES_PER_SECOND_30:
      params.fps = 30;
      break;
    default:
      break;
  }

  // Disable color if the recording has no color track
  if (params.color_enabled && !record_config.color_track_enabled)
  {
    RCLCPP_WARN(logger, "Disabling color and rgb_point_cloud because recording has no color track");
    params.color_enabled = false;
    params.rgb_point_cloud = false;
  }

  // Disable depth if the recording has neither ir track nor depth track
  if (!record_config.ir_track_enabled && !record_config.depth_track_enabled && params.depth_enabled)
  {
    RCLCPP_WARN(logger, "Disabling depth because recording has neither ir track nor depth track");
    params.depth_enabled = false;
  }

  // Disable the point clouds if the recording has no depth track
  if (!record_config.depth_track_enabled)
  {
    if (params.point_cloud)
    {
      RCLCPP_WARN(logger, "Disabling point cloud because recording has no depth track");
      params.point_cloud = false;
    }
    if (params.rgb_point_cloud)
    {
      RCLCPP_WARN(logger, "Disabling rgb point cloud because recording has no depth track");
      params.rgb_point_cloud = false;
    }
  }
}

PlaybackSource::PlaybackSource(K4AROSDeviceParams& params, rclcpp::Logger logger)
  : path_(params.recording_file), loop_(params.recording_loop_enabled), playback_(nullptr)
{
  RCLCPP_INFO(logger, "Node is started in playback mode");
  RCLCPP_INFO_STREAM(logger, "Try to open recording file " << path_);

  try
  {
    playback_ = k4a::playback::open(path_.c_str());
  }
  catch (const k4a::error& error)
  {
    throw std::runtime_error("Failed to open recording file " + path_ + ": " + error.what());
  }

  RCLCPP_INFO_STREAM(logger, "Successfully openend recording file. Recording is "
                               << playback_.get_recording_length().count() / 1000000
                               << " seconds long");

  // Get the recordings configuration to overwrite node parameters
  const k4a_record_configuration_t record_config = playback_.get_record_configuration();
  adaptParamsToRecording(record_config, params, logger);
  frame_period_ = std::chrono::nanoseconds(1000000000LL / params.fps);

  // The converted images only support the formats that the node publishes
  if (params.color_enabled && record_config.color_track_enabled)
  {
    if (params.color_format == "jpeg" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_MJPG)
    {
      throw std::runtime_error(
        "Converting color images to K4A_IMAGE_FORMAT_COLOR_MJPG is not supported.");
    }
    if (params.color_format == "bgra" &&
        record_config.color_format != K4A_IMAGE_FORMAT_COLOR_BGRA32)
    {
      playback_.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
    }
  }

  calibration_ = playback_.get_calibration();
}

std::string PlaybackSource::description() const
{
  return "recording " + path_;
}

CaptureSource::DeviceInfo PlaybackSource::deviceInfo() const
{
  return DeviceInfo{};
}

bool PlaybackSource::capturesAreComplete() const
{
  return false;
}

bool PlaybackSource::providesSystemTimestamps() const
{
  return false;
}

const k4a::calibration& PlaybackSource::calibration() const
{
  return calibration_;
}

void PlaybackSource::start()
{
  next_capture_time_ = std::chrono::steady_clock::now();
}

void PlaybackSource::stop()
{
}

CaptureSource::Status PlaybackSource::nextCapture(k4a::capture& capture,
                                                  std::chrono::milliseconds /*timeout*/)
{
  // A recording has no hardware pacing, so give the captures at the frame rate of the cameras
  std::this_thread::sleep_until(next_capture_time_);
  next_capture_time_ =
    std::max(next_capture_time_ + frame_period_, std::chrono::steady_clock::now());

  Status status = Status::kOk;
  {
    std::lock_guard<std::mutex> guard(playback_mutex_);
    if (!playback_.get_next_capture(&capture))
    {
      if (!loop_)
      {
        return Status::kEndOfStream;
      }

      // Rewind the recording
      playback_.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
      playback_.get_next_capture(&capture);
      imu_stream_end_of_file_ = false;
      last_imu_time_usec_ = 0;
      status = Status::kRestarted;
    }
  }

  last_capture_time_usec_ = captureTimestamp(capture).count();
  return status;
}

CaptureSource::Status PlaybackSource::nextImuSample(k4a_imu_sample_t& sample,
                                                    std::chrono::milliseconds timeout)
{
  // Only give the samples that are not ahead of the cameras. Comparing signed with unsigned
  // should not be a problem because timestamps are always positive.
  if (imu_stream_end_of_file_ || last_imu_time_usec_ > last_capture_time_usec_)
  {
    // The caller is not paced by anything else: wait a little for the cameras to catch up
    std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(3)));
    return imu_stream_end_of_file_ ? Status::kEndOfStream : Status::kTimeout;
  }

  std::lock_guard<std::mutex> guard(playback_mutex_);
  if (!playback_.get_next_imu_sample(&sample))
  {
    imu_stream_end_of_file_ = true;
    return Status::kEndOfStream;
  }

  last_imu_time_usec_ = sample.acc_timestamp_usec;
  return Status::kOk;
}
}  // namespace azure_kinect_ros_driver
