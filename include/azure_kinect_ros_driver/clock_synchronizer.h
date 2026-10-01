// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef CLOCK_SYNCHRONIZER_H
#define CLOCK_SYNCHRONIZER_H

// System headers
//
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>

namespace azure_kinect_ros_driver
{
/**
 * @brief Maps the timestamps of the device clock to the realtime clock.
 *
 * The device stamps its images and IMU samples with its own hardware clock. The SDK also gives,
 * for every image, the monotonic system time at which it arrived at the USB bus. Comparing both
 * gives the offset between the device and the realtime clock, which is tracked with a low-pass
 * filter so that the stamps follow the device clock smoothly.
 *
 * All the methods can be called from different threads.
 */
class ClockSynchronizer
{
public:
  /**
   * @brief Sources of the realtime and monotonic system time, replaceable in tests.
   */
  struct Clocks
  {
    /** @brief Time since the epoch of the realtime clock. */
    std::function<std::chrono::nanoseconds()> realtime;
    /** @brief Time since the epoch of the monotonic clock. */
    std::function<std::chrono::nanoseconds()> monotonic;
  };

  /**
   * @brief What `update()` did with a measurement.
   */
  enum class UpdateResult
  {
    /** @brief The measurement was close to the estimate and was low-pass filtered into it. */
    kFiltered,
    /** @brief The estimate was missing or too far off, so it was replaced by the measurement. */
    kSnapped
  };

  /** @brief Measurements further than this from the estimate replace it instead of filtering. */
  static constexpr std::chrono::nanoseconds kSnapThreshold = std::chrono::milliseconds(10);

  /** @brief Weight of a new measurement in the low-pass filter. */
  static constexpr double kFilterAlpha = 0.10;

  /**
   * @brief Constructs a synchronizer.
   *
   * @param clocks The system clocks to use; defaults to `std::chrono::system_clock` and
   * `std::chrono::steady_clock`.
   */
  explicit ClockSynchronizer(Clocks clocks = systemClocks());

  /**
   * @brief Whether an offset has been estimated yet.
   *
   * @return True once `update()` or `initializeFromWallClock()` has been called.
   */
  bool synchronized() const;

  /**
   * @brief Sets a first guess of the offset from the current wall clock.
   *
   * It is the best that can be done before the first image arrives; `update()` replaces it.
   *
   * @param device_timestamp A device timestamp, assumed to be taken right now.
   * @return The offset that was set.
   */
  std::chrono::nanoseconds initializeFromWallClock(std::chrono::microseconds device_timestamp);

  /**
   * @brief Folds a measurement into the estimate of the offset.
   *
   * @param device_timestamp The device timestamp of an image.
   * @param system_timestamp The monotonic system time at which the image arrived.
   * @return Whether the estimate was filtered or replaced.
   */
  UpdateResult update(std::chrono::microseconds device_timestamp,
                      std::chrono::nanoseconds system_timestamp);

  /**
   * @brief Converts a device timestamp to the realtime clock.
   *
   * @param device_timestamp The device timestamp.
   * @return Time since the epoch. Only meaningful if `synchronized()`.
   */
  std::chrono::nanoseconds toRealtime(std::chrono::microseconds device_timestamp) const;

  /**
   * @brief Keeps the converted time moving forward when the device clock restarts.
   *
   * Playing a recording in a loop makes the device timestamps jump back to the start of the
   * recording. This shifts the offset so that the first timestamp of the new loop is converted to
   * one frame period after the last timestamp of the previous loop.
   *
   * @param last_device_timestamp The last device timestamp before the restart.
   * @param first_device_timestamp The first device timestamp after the restart.
   * @param frame_period The time between two frames.
   */
  void continueAfterRestart(std::chrono::microseconds last_device_timestamp,
                            std::chrono::microseconds first_device_timestamp,
                            std::chrono::nanoseconds frame_period);

private:
  /**
   * @brief The clocks of the operating system.
   *
   * @return `std::chrono::system_clock` and `std::chrono::steady_clock`.
   */
  static Clocks systemClocks();

  /** @brief The system clocks in use. */
  Clocks clocks_;

  /** @brief Device-to-realtime offset in nanoseconds; zero means "not estimated yet". */
  std::atomic<int64_t> offset_ns_{ 0 };
};
}  // namespace azure_kinect_ros_driver

#endif  // CLOCK_SYNCHRONIZER_H
