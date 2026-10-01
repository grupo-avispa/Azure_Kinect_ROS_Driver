// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <chrono>
#include <string>

// Library headers
//
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>

// Project headers
//
#include "azure_kinect_ros_driver/driver_diagnostics.h"

using azure_kinect_ros_driver::CaptureSource;
using azure_kinect_ros_driver::DriverDiagnostics;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_updater::DiagnosticStatusWrapper;
using namespace std::chrono_literals;

/**
 * @brief Test fixture with a controllable clock.
 */
class DriverDiagnosticsTest : public ::testing::Test
{
protected:
  /** @brief Builds the diagnostics on the fake clock. */
  DriverDiagnosticsTest()
    : diagnostics_([this]() { return std::chrono::steady_clock::time_point(now_); })
  {
  }

  /**
   * @brief Looks for a value of a status.
   *
   * @param status The status.
   * @param key The key of the value.
   * @return The value, or an empty string if the status has none with that key.
   */
  static std::string value(const DiagnosticStatusWrapper& status, const std::string& key)
  {
    for (const auto& item : status.values)
    {
      if (item.key == key)
      {
        return item.value;
      }
    }
    return "";
  }

  /**
   * @brief Configures the driver as if it was reading from a device.
   *
   * @param capture_rate The expected frame rate.
   * @param imu_rate The expected IMU rate.
   */
  void configure(double capture_rate = 15.0, double imu_rate = 100.0)
  {
    diagnostics_.configured(
      CaptureSource::DeviceInfo{ "SERIAL1", "1.6.1", "1.6.2", "1.6.3", "6109.7.0" },
      "device SERIAL1", capture_rate, imu_rate);
  }

  /**
   * @brief Lets time pass while the cameras deliver captures at a rate.
   *
   * @param seconds How long to run.
   * @param rate The captures per second.
   */
  void deliverCaptures(double seconds, double rate)
  {
    const int count = static_cast<int>(seconds * rate);
    for (int i = 0; i < count; ++i)
    {
      now_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
      diagnostics_.captureReceived();
    }
  }

  /** @brief The fake time. */
  std::chrono::steady_clock::duration now_{ 100s };

  /** @brief The diagnostics under test. */
  DriverDiagnostics diagnostics_;
};

/**
 * @brief A driver that is not configured reports a warning.
 */
TEST_F(DriverDiagnosticsTest, UnconfiguredIsAWarning)
{
  DiagnosticStatusWrapper status;
  diagnostics_.deviceStatus(status);

  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "Not configured");
}

/**
 * @brief A configured driver that does not stream reports a warning and the device identification.
 */
TEST_F(DriverDiagnosticsTest, ConfiguredButNotStreamingReportsTheDevice)
{
  configure();

  DiagnosticStatusWrapper status;
  diagnostics_.deviceStatus(status);

  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "Not streaming");
  EXPECT_EQ(value(status, "Serial number"), "SERIAL1");
  EXPECT_EQ(value(status, "Depth sensor firmware"), "6109.7.0");
  EXPECT_EQ(value(status, "Source"), "device SERIAL1");
}

/**
 * @brief A streaming device that delivers captures is healthy.
 */
TEST_F(DriverDiagnosticsTest, StreamingWithCapturesIsOk)
{
  configure();
  diagnostics_.setStreaming(true);
  deliverCaptures(1.0, 15.0);

  DiagnosticStatusWrapper status;
  diagnostics_.deviceStatus(status);

  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Streaming");
  EXPECT_EQ(value(status, "Captures received"), "15");
}

/**
 * @brief A device that stops delivering captures is an error.
 */
TEST_F(DriverDiagnosticsTest, NoCapturesForTooLongIsAnError)
{
  configure();
  diagnostics_.setStreaming(true);
  deliverCaptures(1.0, 15.0);
  now_ += 3s;

  DiagnosticStatusWrapper status;
  diagnostics_.deviceStatus(status);

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "The device does not deliver captures");
}

/**
 * @brief Captures whose streams failed are reported once, as a warning.
 */
TEST_F(DriverDiagnosticsTest, FailedCapturesAreAWarningOnce)
{
  configure();
  diagnostics_.setStreaming(true);
  deliverCaptures(1.0, 15.0);
  diagnostics_.captureFailed();

  DiagnosticStatusWrapper first;
  diagnostics_.deviceStatus(first);
  EXPECT_EQ(first.level, DiagnosticStatus::WARN);
  EXPECT_EQ(value(first, "Captures with failed streams"), "1");

  deliverCaptures(1.0, 15.0);
  DiagnosticStatusWrapper second;
  diagnostics_.deviceStatus(second);
  EXPECT_EQ(second.level, DiagnosticStatus::OK);
}

/**
 * @brief A failure is reported as an error until the driver is configured again.
 */
TEST_F(DriverDiagnosticsTest, ErrorIsReportedUntilConfiguredAgain)
{
  configure();
  diagnostics_.setError("Failed to start the cameras");

  DiagnosticStatusWrapper status;
  diagnostics_.deviceStatus(status);
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Failed to start the cameras");

  configure();
  DiagnosticStatusWrapper after;
  diagnostics_.deviceStatus(after);
  EXPECT_EQ(after.level, DiagnosticStatus::WARN);
}

/**
 * @brief The clock resynchronizations and the IMU temperature are reported.
 */
TEST_F(DriverDiagnosticsTest, ReportsClockResynchronizationsAndTemperature)
{
  configure();
  diagnostics_.clockResynchronized();
  diagnostics_.clockResynchronized();
  diagnostics_.imuSampleReceived(31.5f);

  DiagnosticStatusWrapper status;
  diagnostics_.deviceStatus(status);

  EXPECT_EQ(value(status, "Clock offset resynchronizations"), "2");
  EXPECT_EQ(value(status, "IMU samples received"), "1");
  EXPECT_NEAR(std::stod(value(status, "IMU temperature (C)")), 31.5, 1e-6);
}

/**
 * @brief The first report of a rate only says that it is being measured.
 */
TEST_F(DriverDiagnosticsTest, FirstRateReportIsNeutral)
{
  configure();
  diagnostics_.setStreaming(true);

  DiagnosticStatusWrapper status;
  diagnostics_.captureRateStatus(status);

  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_NE(status.message.find("Measuring"), std::string::npos);
}

/**
 * @brief A capture rate close to the expected one is healthy.
 */
TEST_F(DriverDiagnosticsTest, ExpectedCaptureRateIsOk)
{
  configure(15.0);
  diagnostics_.setStreaming(true);
  DiagnosticStatusWrapper first;
  diagnostics_.captureRateStatus(first);

  deliverCaptures(2.0, 15.0);
  DiagnosticStatusWrapper status;
  diagnostics_.captureRateStatus(status);

  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_NEAR(std::stod(value(status, "Measured rate (Hz)")), 15.0, 0.6);
}

/**
 * @brief A capture rate below or above the expected one is a warning, and no captures an error.
 */
TEST_F(DriverDiagnosticsTest, WrongCaptureRateIsReported)
{
  configure(15.0);
  diagnostics_.setStreaming(true);
  DiagnosticStatusWrapper first;
  diagnostics_.captureRateStatus(first);

  deliverCaptures(2.0, 10.0);
  DiagnosticStatusWrapper low;
  diagnostics_.captureRateStatus(low);
  EXPECT_EQ(low.level, DiagnosticStatus::WARN);
  EXPECT_NE(low.message.find("below"), std::string::npos);

  deliverCaptures(2.0, 20.0);
  DiagnosticStatusWrapper high;
  diagnostics_.captureRateStatus(high);
  EXPECT_EQ(high.level, DiagnosticStatus::WARN);
  EXPECT_NE(high.message.find("above"), std::string::npos);

  now_ += 2s;
  DiagnosticStatusWrapper none;
  diagnostics_.captureRateStatus(none);
  EXPECT_EQ(none.level, DiagnosticStatus::ERROR);
}

/**
 * @brief The rate is not reported as healthy when the driver does not stream.
 */
TEST_F(DriverDiagnosticsTest, RateIsAWarningWhenNotStreaming)
{
  configure();

  DiagnosticStatusWrapper status;
  diagnostics_.captureRateStatus(status);
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "Not streaming");
}

/**
 * @brief The IMU rate is measured on the messages that were published.
 */
TEST_F(DriverDiagnosticsTest, ImuRateIsMeasuredOnThePublishedMessages)
{
  configure(15.0, 100.0);
  diagnostics_.setStreaming(true);
  DiagnosticStatusWrapper first;
  diagnostics_.imuRateStatus(first);

  for (int i = 0; i < 210; ++i)
  {
    diagnostics_.imuMessagePublished();
  }
  now_ += 2s;
  diagnostics_.imuSampleReceived(29.0f);
  DiagnosticStatusWrapper status;
  diagnostics_.imuRateStatus(status);

  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_NEAR(std::stod(value(status, "Measured rate (Hz)")), 105.0, 0.1);
  EXPECT_NEAR(std::stod(value(status, "Temperature (C)")), 29.0, 1e-6);
}

/**
 * @brief Cleaning up forgets the source.
 */
TEST_F(DriverDiagnosticsTest, UnconfiguredForgetsTheSource)
{
  configure();
  diagnostics_.setStreaming(true);
  diagnostics_.unconfigured();

  DiagnosticStatusWrapper status;
  diagnostics_.deviceStatus(status);
  EXPECT_EQ(status.message, "Not configured");
  EXPECT_EQ(value(status, "Serial number"), "");
}

/**
 * @brief Runs the unit tests.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
