// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef DRIVER_DIAGNOSTICS_H
#define DRIVER_DIAGNOSTICS_H

// System headers
//
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

// Library headers
//
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/capture_source.h"

namespace azure_kinect_ros_driver
{
/**
 * @brief Keeps the figures of the health of the driver and turns them into diagnostic statuses.
 *
 * The streaming threads report what happens with cheap atomic counters, and the diagnostic
 * updater calls the `*Status()` methods once a second. The class does not depend on a node, and
 * its clock is replaceable, so it can be tested on its own.
 */
class DriverDiagnostics
{
public:
  /** @brief Source of the monotonic time. */
  using Clock = std::function<std::chrono::steady_clock::time_point()>;

  /** @brief Lowest measured rate, relative to the expected one, that is still considered normal. */
  static constexpr double kLowRateRatio = 0.9;

  /** @brief Highest measured rate, relative to the expected one, that is still considered normal.
   */
  static constexpr double kHighRateRatio = 1.1;

  /** @brief Time without captures after which the device is reported as not delivering. */
  static constexpr std::chrono::seconds kStallTime{ 2 };

  /**
   * @brief Constructs the diagnostics of an unconfigured driver.
   *
   * @param clock The monotonic clock; defaults to `std::chrono::steady_clock`.
   */
  explicit DriverDiagnostics(Clock clock = &std::chrono::steady_clock::now);

  /**
   * @brief Records that the driver is configured, with the source it reads from.
   *
   * @param info The identification of the device, empty for a recording.
   * @param source_description Describes the source for the report.
   * @param expected_capture_rate The frame rate of the cameras, in Hz.
   * @param expected_imu_rate The rate of the IMU messages, in Hz.
   */
  void configured(const CaptureSource::DeviceInfo& info, const std::string& source_description,
                  double expected_capture_rate, double expected_imu_rate);

  /** @brief Records that the driver was cleaned up; forgets the source and the counters. */
  void unconfigured();

  /**
   * @brief Records whether the driver is streaming.
   *
   * @param streaming True once the cameras run, false when they stop.
   */
  void setStreaming(bool streaming);

  /**
   * @brief Records that the driver failed.
   *
   * @param reason What failed; it is reported until the driver is configured again.
   */
  void setError(const std::string& reason);

  /** @brief Counts a capture that was read from the source. */
  void captureReceived();

  /** @brief Counts a capture whose streams could not all be produced. */
  void captureFailed();

  /** @brief Counts that the clock offset had to be replaced instead of filtered. */
  void clockResynchronized();

  /**
   * @brief Counts an IMU sample and remembers its temperature.
   *
   * @param temperature_c The temperature of the sample, in degrees Celsius.
   */
  void imuSampleReceived(float temperature_c);

  /** @brief Counts an IMU message that was published. */
  void imuMessagePublished();

  /**
   * @brief Fills the general status of the device and of the driver.
   *
   * @param status The status to fill.
   */
  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper& status);

  /**
   * @brief Fills the status of the rate at which captures arrive.
   *
   * @param status The status to fill.
   */
  void captureRateStatus(diagnostic_updater::DiagnosticStatusWrapper& status);

  /**
   * @brief Fills the status of the rate of the IMU messages.
   *
   * @param status The status to fill.
   */
  void imuRateStatus(diagnostic_updater::DiagnosticStatusWrapper& status);

private:
  /** @brief What the driver is doing. */
  enum class Phase
  {
    /** @brief Not configured. */
    kUnconfigured,
    /** @brief Configured, with the cameras stopped. */
    kConfigured,
    /** @brief Configured and reading from the source. */
    kStreaming
  };

  /**
   * @brief A counter whose rate is measured between two reports.
   */
  struct RateMeter
  {
    /** @brief Value of the counter at the previous report. */
    uint64_t last_count = 0;

    /** @brief Time of the previous report. */
    std::chrono::steady_clock::time_point last_time;

    /** @brief Rate measured at the previous report, in Hz, once an interval has been measured. */
    std::optional<double> rate;

    /** @brief Whether there has been a previous report. */
    bool started = false;
  };

  /**
   * @brief Measures the rate of a counter since the previous report.
   *
   * @param count The current value of the counter.
   * @param now The current time.
   * @param meter The meter of that counter.
   * @return The rate in Hz; the one of the previous report if too little time has passed, and
   * nothing before the first interval has been measured.
   */
  static std::optional<double> measureRate(uint64_t count,
                                           std::chrono::steady_clock::time_point now,
                                           RateMeter& meter);

  /**
   * @brief Fills a status that reports a rate against the one that was expected.
   *
   * @param status The status to fill.
   * @param name What is measured, for the message.
   * @param rate The measured rate, in Hz, or nothing if it is not known yet.
   * @param expected The expected rate, in Hz.
   * @param phase The current phase of the driver.
   */
  void reportRate(diagnostic_updater::DiagnosticStatusWrapper& status, const std::string& name,
                  std::optional<double> rate, double expected, Phase phase) const;

  /** @brief The monotonic clock. */
  Clock clock_;

  /** @brief Protects the strings and the meters. */
  mutable std::mutex mutex_;

  /** @brief What the driver is doing. */
  std::atomic<Phase> phase_{ Phase::kUnconfigured };

  /** @brief Identification of the device. */
  CaptureSource::DeviceInfo info_;

  /** @brief Describes the source. */
  std::string source_description_;

  /** @brief The reason of the last failure, or empty. */
  std::string error_;

  /** @brief Frame rate of the cameras, in Hz. */
  double expected_capture_rate_ = 0.0;

  /** @brief Rate of the IMU messages, in Hz. */
  double expected_imu_rate_ = 0.0;

  /** @brief Captures read from the source. */
  std::atomic<uint64_t> captures_{ 0 };

  /** @brief Captures with a stream that could not be produced. */
  std::atomic<uint64_t> failed_captures_{ 0 };

  /** @brief Replacements of the clock offset. */
  std::atomic<uint64_t> clock_resyncs_{ 0 };

  /** @brief IMU samples read from the source. */
  std::atomic<uint64_t> imu_samples_{ 0 };

  /** @brief IMU messages published. */
  std::atomic<uint64_t> imu_messages_{ 0 };

  /** @brief Temperature of the last IMU sample, in degrees Celsius. */
  std::atomic<float> temperature_c_{ 0.0f };

  /** @brief When the last capture arrived, as ticks of the clock. */
  std::atomic<int64_t> last_capture_ticks_{ 0 };

  /** @brief Meter of the capture rate. */
  RateMeter capture_meter_;

  /** @brief Meter of the IMU message rate. */
  RateMeter imu_meter_;

  /** @brief Failed captures at the previous report of the device status. */
  uint64_t reported_failed_captures_ = 0;
};
}  // namespace azure_kinect_ros_driver

#endif  // DRIVER_DIAGNOSTICS_H
