// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <chrono>
#include <thread>
#include <vector>

// Library headers
//
#include <gtest/gtest.h>

// Project headers
//
#include "azure_kinect_ros_driver/clock_synchronizer.h"

using azure_kinect_ros_driver::ClockSynchronizer;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using std::chrono::seconds;

/**
 * @brief Test fixture with controllable system clocks.
 *
 * The realtime clock is 900 s ahead of the monotonic clock, so the monotonic-to-realtime part of
 * the offset is a known, non-zero value.
 */
class ClockSynchronizerTest : public ::testing::Test
{
protected:
  /** @brief Builds the synchronizer on the fake clocks. */
  ClockSynchronizerTest()
    : synchronizer_(ClockSynchronizer::Clocks{ [this]() { return realtime_now_; },
                                               [this]() { return monotonic_now_; } })
  {
  }

  /** @brief The current fake realtime. */
  nanoseconds realtime_now_{ seconds(1000) };

  /** @brief The current fake monotonic time. */
  nanoseconds monotonic_now_{ seconds(100) };

  /** @brief The synchronizer under test. */
  ClockSynchronizer synchronizer_;
};

/**
 * @brief A new synchronizer has no offset.
 */
TEST_F(ClockSynchronizerTest, StartsUnsynchronized)
{
  EXPECT_FALSE(synchronizer_.synchronized());
}

/**
 * @brief The first guess maps a device timestamp taken "now" to the current wall clock.
 */
TEST_F(ClockSynchronizerTest, InitializeFromWallClockMapsTheTimestampToNow)
{
  const microseconds device_timestamp = seconds(5);

  synchronizer_.initializeFromWallClock(device_timestamp);

  EXPECT_TRUE(synchronizer_.synchronized());
  EXPECT_EQ(synchronizer_.toRealtime(device_timestamp), realtime_now_);
  EXPECT_EQ(synchronizer_.toRealtime(device_timestamp + milliseconds(100)),
            realtime_now_ + milliseconds(100));
}

/**
 * @brief The first measurement replaces the missing estimate.
 */
TEST_F(ClockSynchronizerTest, FirstUpdateSnapsToTheMeasurement)
{
  // The image was exposed at device time 5 s and arrived at monotonic time 100.5 s, that is,
  // realtime 1000.5 s
  const auto result = synchronizer_.update(seconds(5), milliseconds(100500));

  EXPECT_EQ(result, ClockSynchronizer::UpdateResult::kSnapped);
  EXPECT_EQ(synchronizer_.toRealtime(seconds(5)), milliseconds(1000500));
}

/**
 * @brief A measurement within the threshold only moves the estimate by the filter weight.
 */
TEST_F(ClockSynchronizerTest, CloseMeasurementIsLowPassFiltered)
{
  synchronizer_.update(seconds(5), milliseconds(100500));

  // The next image shows the clocks 5 ms further apart than estimated
  const auto result = synchronizer_.update(seconds(6), milliseconds(101505));

  EXPECT_EQ(result, ClockSynchronizer::UpdateResult::kFiltered);
  // The estimate moves by alpha * 5 ms = 0.5 ms
  EXPECT_EQ(synchronizer_.toRealtime(seconds(6)), milliseconds(1001500) + microseconds(500));
}

/**
 * @brief A measurement beyond the threshold is taken as a clock step and replaces the estimate.
 */
TEST_F(ClockSynchronizerTest, FarMeasurementSnaps)
{
  synchronizer_.update(seconds(5), milliseconds(100500));

  const auto result = synchronizer_.update(seconds(6), milliseconds(101600));

  EXPECT_EQ(result, ClockSynchronizer::UpdateResult::kSnapped);
  EXPECT_EQ(synchronizer_.toRealtime(seconds(6)), milliseconds(1001600));
}

/**
 * @brief Repeating a measurement that is slightly off converges to it.
 */
TEST_F(ClockSynchronizerTest, FilteredEstimateConvergesToTheMeasurements)
{
  synchronizer_.update(seconds(5), milliseconds(100500));

  // Every image now says that the offset is 8 ms larger (within the snap threshold)
  for (int i = 1; i <= 100; ++i)
  {
    synchronizer_.update(seconds(5) + milliseconds(66 * i), milliseconds(100500 + 66 * i + 8));
  }

  const nanoseconds estimated = synchronizer_.toRealtime(seconds(5));
  EXPECT_NEAR(static_cast<double>(estimated.count()),
              static_cast<double>(nanoseconds(milliseconds(1000508)).count()), 1000.0);
}

/**
 * @brief The estimate follows the drift of the monotonic-to-realtime offset.
 */
TEST_F(ClockSynchronizerTest, UsesTheCurrentMonotonicToRealtimeOffset)
{
  synchronizer_.update(seconds(5), milliseconds(100500));

  // The realtime clock is adjusted 4 ms forward; the same measurement now maps 4 ms later
  realtime_now_ += milliseconds(4);
  synchronizer_.update(seconds(5), milliseconds(100500));

  EXPECT_EQ(synchronizer_.toRealtime(seconds(5)), milliseconds(1000500) + microseconds(400));
}

/**
 * @brief After a restart of the device clock the converted time keeps moving forward.
 */
TEST_F(ClockSynchronizerTest, ContinueAfterRestartKeepsTimeMonotonic)
{
  synchronizer_.update(seconds(5), milliseconds(100500));
  const microseconds last_device_timestamp = seconds(9);
  const nanoseconds last_stamp = synchronizer_.toRealtime(last_device_timestamp);

  // The recording loops: device timestamps start again at 0.5 s
  const microseconds first_device_timestamp = milliseconds(500);
  const nanoseconds frame_period = milliseconds(66);
  synchronizer_.continueAfterRestart(last_device_timestamp, first_device_timestamp, frame_period);

  EXPECT_EQ(synchronizer_.toRealtime(first_device_timestamp), last_stamp + frame_period);
  EXPECT_GT(synchronizer_.toRealtime(first_device_timestamp + milliseconds(66)),
            synchronizer_.toRealtime(first_device_timestamp));
}

/**
 * @brief The default constructor uses the operating system clocks.
 */
TEST(ClockSynchronizerSystemClocks, ConvertsToTheCurrentTime)
{
  ClockSynchronizer synchronizer;
  const nanoseconds before = std::chrono::system_clock::now().time_since_epoch();

  synchronizer.initializeFromWallClock(seconds(7));
  const nanoseconds converted = synchronizer.toRealtime(seconds(7));

  const nanoseconds after = std::chrono::system_clock::now().time_since_epoch();
  EXPECT_GE(converted, before);
  EXPECT_LE(converted, after);
}

/**
 * @brief Concurrent updates and conversions do not corrupt the offset.
 */
TEST_F(ClockSynchronizerTest, SupportsConcurrentUse)
{
  synchronizer_.update(seconds(5), milliseconds(100500));
  std::vector<std::thread> threads;
  threads.emplace_back(
    [this]()
    {
      for (int i = 0; i < 2000; ++i)
      {
        synchronizer_.update(seconds(5), milliseconds(100500));
      }
    });
  threads.emplace_back(
    [this]()
    {
      for (int i = 0; i < 2000; ++i)
      {
        EXPECT_EQ(synchronizer_.toRealtime(seconds(5)), milliseconds(1000500));
      }
    });
  for (auto& thread : threads)
  {
    thread.join();
  }
}

/**
 * @brief Runs the unit tests.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
