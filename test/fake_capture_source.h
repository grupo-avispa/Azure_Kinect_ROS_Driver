// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef FAKE_CAPTURE_SOURCE_H
#define FAKE_CAPTURE_SOURCE_H

// System headers
//
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

// Project headers
//
#include "azure_kinect_ros_driver/capture_source.h"

namespace azure_kinect_ros_driver
{
namespace testing
{
/**
 * @brief State shared between a fake source and the test that created it.
 *
 * The node owns the source, so the test keeps this to control and observe it.
 */
struct FakeSourceState
{
  /** @brief How many times `start()` was called. */
  std::atomic_int starts{ 0 };

  /** @brief How many times `stop()` was called. */
  std::atomic_int stops{ 0 };

  /** @brief How many captures were given. */
  std::atomic_int captures{ 0 };

  /** @brief How many IMU samples were given. */
  std::atomic_int imu_samples{ 0 };

  /** @brief Captures after which the source reports the end of the stream; negative for never. */
  std::atomic_int end_after{ -1 };

  /** @brief Captures after which the source stops delivering them (device lost); negative for
   * never. */
  std::atomic_int stall_after{ -1 };

  /** @brief Number of times the next capture reads throw an exception. */
  std::atomic_int throws{ 0 };

  /** @brief Time between two captures. */
  std::chrono::milliseconds capture_period{ 20 };

  /** @brief Time between two IMU samples. */
  std::chrono::microseconds imu_period{ 600 };
};

/**
 * @brief A source that gives synthetic captures and IMU samples, without any device.
 *
 * The depth and IR images are 640x576 and carry the number of the capture, and the calibration
 * is the one stored in `test/data/k4a_calibration.json` with the color camera off.
 */
class FakeCaptureSource : public CaptureSource
{
public:
  /**
   * @brief Constructs the source.
   *
   * @param state The state to share with the test.
   * @param data_dir The directory with the calibration of the test data.
   */
  FakeCaptureSource(std::shared_ptr<FakeSourceState> state, const std::string& data_dir)
    : state_(std::move(state))
  {
    std::ifstream file(data_dir + "/k4a_calibration.json", std::ios::binary);
    std::stringstream content;
    content << file.rdbuf();
    std::string raw = content.str();
    if (raw.empty())
    {
      throw std::runtime_error("cannot read the test calibration");
    }
    calibration_ = k4a::calibration::get_from_raw(
      &raw[0], raw.size() + 1, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_OFF);
  }

  std::string description() const override
  {
    return "fake source";
  }

  DeviceInfo deviceInfo() const override
  {
    return DeviceInfo{ "FAKE", "1.0.0", "1.0.0", "1.0.0", "1.0.0" };
  }

  bool capturesAreComplete() const override
  {
    return true;
  }

  bool providesSystemTimestamps() const override
  {
    return false;
  }

  const k4a::calibration& calibration() const override
  {
    return calibration_;
  }

  void start() override
  {
    ++state_->starts;
    next_capture_ = std::chrono::steady_clock::now();
    next_imu_ = std::chrono::steady_clock::now();
  }

  void stop() override
  {
    ++state_->stops;
  }

  Status nextCapture(k4a::capture& capture, std::chrono::milliseconds timeout) override
  {
    if (state_->throws > 0)
    {
      --state_->throws;
      throw std::runtime_error("fake capture failure");
    }

    const int given = state_->captures;
    if (state_->end_after >= 0 && given >= state_->end_after)
    {
      return Status::kEndOfStream;
    }
    if (state_->stall_after >= 0 && given >= state_->stall_after)
    {
      std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(50)));
      return Status::kTimeout;
    }

    std::this_thread::sleep_until(next_capture_);
    next_capture_ += state_->capture_period;

    capture = k4a::capture::create();
    const std::chrono::microseconds timestamp(1000000 +
                                              given * state_->capture_period.count() * 1000);

    k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 640, 576, 640 * 2);
    uint16_t* depth_pixels = reinterpret_cast<uint16_t*>(depth.get_buffer());
    for (int i = 0; i < 640 * 576; ++i)
    {
      depth_pixels[i] = static_cast<uint16_t>(given + 1);
    }
    depth.set_timestamp(timestamp);
    capture.set_depth_image(depth);

    k4a::image ir = k4a::image::create(K4A_IMAGE_FORMAT_IR16, 640, 576, 640 * 2);
    uint16_t* ir_pixels = reinterpret_cast<uint16_t*>(ir.get_buffer());
    for (int i = 0; i < 640 * 576; ++i)
    {
      ir_pixels[i] = static_cast<uint16_t>(2 * (given + 1));
    }
    ir.set_timestamp(timestamp);
    capture.set_ir_image(ir);

    ++state_->captures;
    return Status::kOk;
  }

  Status nextImuSample(k4a_imu_sample_t& sample, std::chrono::milliseconds timeout) override
  {
    if (std::chrono::steady_clock::now() < next_imu_)
    {
      std::this_thread::sleep_for(std::min<std::chrono::steady_clock::duration>(
        timeout, next_imu_ - std::chrono::steady_clock::now()));
      if (std::chrono::steady_clock::now() < next_imu_)
      {
        return Status::kTimeout;
      }
    }
    next_imu_ += state_->imu_period;

    const int index = state_->imu_samples++;
    sample = k4a_imu_sample_t{};
    sample.temperature = 30.0f + 0.01f * static_cast<float>(index % 100);
    sample.acc_timestamp_usec = 1000000 + static_cast<uint64_t>(index) * state_->imu_period.count();
    sample.gyro_timestamp_usec = sample.acc_timestamp_usec;
    sample.acc_sample.xyz.z = 9.8f;
    sample.gyro_sample.xyz.x = 0.01f * static_cast<float>(1 + index % 7);
    return Status::kOk;
  }

private:
  /** @brief State shared with the test. */
  std::shared_ptr<FakeSourceState> state_;

  /** @brief The calibration of the test data. */
  k4a::calibration calibration_;

  /** @brief When the next capture is due. */
  std::chrono::steady_clock::time_point next_capture_;

  /** @brief When the next IMU sample is due. */
  std::chrono::steady_clock::time_point next_imu_;
};
}  // namespace testing
}  // namespace azure_kinect_ros_driver

#endif  // FAKE_CAPTURE_SOURCE_H
