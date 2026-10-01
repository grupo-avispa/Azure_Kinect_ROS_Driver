// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/clock_synchronizer.h"

// System headers
//
#include <cmath>
#include <cstdlib>

namespace azure_kinect_ros_driver
{
ClockSynchronizer::ClockSynchronizer(Clocks clocks) : clocks_(std::move(clocks))
{
}

ClockSynchronizer::Clocks ClockSynchronizer::systemClocks()
{
  return Clocks{
    []() { return std::chrono::system_clock::now().time_since_epoch(); },
    []() { return std::chrono::steady_clock::now().time_since_epoch(); },
  };
}

bool ClockSynchronizer::synchronized() const
{
  return offset_ns_.load() != 0;
}

std::chrono::nanoseconds ClockSynchronizer::initializeFromWallClock(
  std::chrono::microseconds device_timestamp)
{
  const std::chrono::nanoseconds offset = clocks_.realtime() - device_timestamp;
  offset_ns_.store(offset.count());
  return offset;
}

ClockSynchronizer::UpdateResult ClockSynchronizer::update(
  std::chrono::microseconds device_timestamp, std::chrono::nanoseconds system_timestamp)
{
  // The system timestamp is on the monotonic system clock and the device timestamp on the device
  // hardware clock. The offset to the realtime clock has two parts: device to monotonic, and
  // monotonic to realtime. The second one changes over time, so it is measured on every update.
  const std::chrono::nanoseconds monotonic_to_realtime = clocks_.realtime() - clocks_.monotonic();
  const std::chrono::nanoseconds measured =
    system_timestamp - device_timestamp + monotonic_to_realtime;

  const std::chrono::nanoseconds current(offset_ns_.load());
  if (current.count() == 0 || std::abs((current - measured).count()) > kSnapThreshold.count())
  {
    offset_ns_.store(measured.count());
    return UpdateResult::kSnapped;
  }

  const auto correction = static_cast<int64_t>(
    std::floor(kFilterAlpha * static_cast<double>((measured - current).count())));
  offset_ns_.store(current.count() + correction);
  return UpdateResult::kFiltered;
}

std::chrono::nanoseconds ClockSynchronizer::toRealtime(
  std::chrono::microseconds device_timestamp) const
{
  return device_timestamp + std::chrono::nanoseconds(offset_ns_.load());
}

void ClockSynchronizer::continueAfterRestart(std::chrono::microseconds last_device_timestamp,
                                             std::chrono::microseconds first_device_timestamp,
                                             std::chrono::nanoseconds frame_period)
{
  const std::chrono::nanoseconds last_stamp = toRealtime(last_device_timestamp);
  const std::chrono::nanoseconds offset = last_stamp + frame_period - first_device_timestamp;
  offset_ns_.store(offset.count());
}
}  // namespace azure_kinect_ros_driver
