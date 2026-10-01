// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/driver_diagnostics.h"

// Library headers
//
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace azure_kinect_ros_driver
{
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
/** @brief Shortest time over which a rate is measured, in seconds. */
constexpr double kMinMeasurementSeconds = 0.5;
}  // namespace

DriverDiagnostics::DriverDiagnostics(Clock clock) : clock_(std::move(clock))
{
}

void DriverDiagnostics::configured(const CaptureSource::DeviceInfo& info,
                                   const std::string& source_description,
                                   double expected_capture_rate, double expected_imu_rate)
{
  std::lock_guard<std::mutex> guard(mutex_);
  info_ = info;
  source_description_ = source_description;
  expected_capture_rate_ = expected_capture_rate;
  expected_imu_rate_ = expected_imu_rate;
  error_.clear();
  captures_ = 0;
  failed_captures_ = 0;
  clock_resyncs_ = 0;
  imu_samples_ = 0;
  imu_messages_ = 0;
  last_capture_ticks_ = 0;
  capture_meter_ = RateMeter{};
  imu_meter_ = RateMeter{};
  reported_failed_captures_ = 0;
  phase_ = Phase::kConfigured;
}

void DriverDiagnostics::unconfigured()
{
  std::lock_guard<std::mutex> guard(mutex_);
  info_ = CaptureSource::DeviceInfo{};
  source_description_.clear();
  phase_ = Phase::kUnconfigured;
}

void DriverDiagnostics::setStreaming(bool streaming)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (phase_ == Phase::kUnconfigured)
  {
    return;
  }
  if (streaming)
  {
    // The rates are measured from now on, and the device has just started to deliver
    capture_meter_ = RateMeter{};
    imu_meter_ = RateMeter{};
    last_capture_ticks_ = clock_().time_since_epoch().count();
  }
  phase_ = streaming ? Phase::kStreaming : Phase::kConfigured;
}

void DriverDiagnostics::setError(const std::string& reason)
{
  std::lock_guard<std::mutex> guard(mutex_);
  error_ = reason;
}

void DriverDiagnostics::captureReceived()
{
  ++captures_;
  last_capture_ticks_ = clock_().time_since_epoch().count();
}

void DriverDiagnostics::captureFailed()
{
  ++failed_captures_;
}

void DriverDiagnostics::clockResynchronized()
{
  ++clock_resyncs_;
}

void DriverDiagnostics::imuSampleReceived(float temperature_c)
{
  ++imu_samples_;
  temperature_c_ = temperature_c;
}

void DriverDiagnostics::imuMessagePublished()
{
  ++imu_messages_;
}

std::optional<double> DriverDiagnostics::measureRate(uint64_t count,
                                                     std::chrono::steady_clock::time_point now,
                                                     RateMeter& meter)
{
  if (!meter.started)
  {
    meter.started = true;
    meter.last_count = count;
    meter.last_time = now;
    meter.rate.reset();
    return meter.rate;
  }

  const double seconds = std::chrono::duration<double>(now - meter.last_time).count();
  if (seconds >= kMinMeasurementSeconds)
  {
    meter.rate = static_cast<double>(count - meter.last_count) / seconds;
    meter.last_count = count;
    meter.last_time = now;
  }
  return meter.rate;
}

void DriverDiagnostics::reportRate(diagnostic_updater::DiagnosticStatusWrapper& status,
                                   const std::string& name, std::optional<double> rate,
                                   double expected, Phase phase) const
{
  status.add("Expected rate (Hz)", expected);
  if (rate)
  {
    status.add("Measured rate (Hz)", *rate);
  }

  if (phase != Phase::kStreaming)
  {
    status.summary(DiagnosticStatus::WARN, "Not streaming");
  }
  else if (!rate)
  {
    status.summary(DiagnosticStatus::OK, "Measuring the rate of " + name);
  }
  else if (*rate <= 0.0)
  {
    status.summary(DiagnosticStatus::ERROR, "No " + name);
  }
  else if (*rate < kLowRateRatio * expected)
  {
    status.summary(DiagnosticStatus::WARN, "The rate of " + name + " is below the expected one");
  }
  else if (*rate > kHighRateRatio * expected)
  {
    status.summary(DiagnosticStatus::WARN, "The rate of " + name + " is above the expected one");
  }
  else
  {
    status.summary(DiagnosticStatus::OK, "The rate of " + name + " is as expected");
  }
}

void DriverDiagnostics::deviceStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const Phase phase = phase_;
  const auto now = clock_();

  status.add("Source", source_description_);
  status.add("Serial number", info_.serial_number);
  status.add("RGB firmware", info_.rgb_version);
  status.add("Depth firmware", info_.depth_version);
  status.add("Audio firmware", info_.audio_version);
  status.add("Depth sensor firmware", info_.depth_sensor_version);
  status.add("Captures received", static_cast<int64_t>(captures_));
  status.add("Captures with failed streams", static_cast<int64_t>(failed_captures_));
  status.add("IMU samples received", static_cast<int64_t>(imu_samples_));
  status.add("IMU temperature (C)", static_cast<double>(temperature_c_));
  status.add("Clock offset resynchronizations", static_cast<int64_t>(clock_resyncs_));

  const uint64_t new_failures = failed_captures_ - reported_failed_captures_;
  reported_failed_captures_ = failed_captures_;

  if (!error_.empty())
  {
    status.summary(DiagnosticStatus::ERROR, error_);
    return;
  }
  if (phase == Phase::kUnconfigured)
  {
    status.summary(DiagnosticStatus::WARN, "Not configured");
    return;
  }
  if (phase == Phase::kConfigured)
  {
    status.summary(DiagnosticStatus::WARN, "Not streaming");
    return;
  }

  const auto last_capture = std::chrono::steady_clock::time_point(
    std::chrono::steady_clock::duration(last_capture_ticks_.load()));
  const double seconds_since_capture = std::chrono::duration<double>(now - last_capture).count();
  status.add("Seconds since the last capture", seconds_since_capture);

  if (seconds_since_capture > std::chrono::duration<double>(kStallTime).count())
  {
    status.summary(DiagnosticStatus::ERROR, "The device does not deliver captures");
  }
  else if (new_failures > 0)
  {
    status.summary(DiagnosticStatus::WARN, "Some captures could not be turned into messages");
  }
  else
  {
    status.summary(DiagnosticStatus::OK, "Streaming");
  }
}

void DriverDiagnostics::captureRateStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const std::optional<double> rate = measureRate(captures_, clock_(), capture_meter_);
  reportRate(status, "captures", rate, expected_capture_rate_, phase_);
}

void DriverDiagnostics::imuRateStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const std::optional<double> rate = measureRate(imu_messages_, clock_(), imu_meter_);
  reportRate(status, "IMU messages", rate, expected_imu_rate_, phase_);
  status.add("Temperature (C)", static_cast<double>(temperature_c_));
}
}  // namespace azure_kinect_ros_driver
