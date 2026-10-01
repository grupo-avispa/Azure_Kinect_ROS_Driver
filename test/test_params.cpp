// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Library headers
//
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"
#include "azure_kinect_ros_driver/playback_source.h"

using azure_kinect_ros_driver::adaptParamsToRecording;

/**
 * @brief Test fixture holding a set of parameters with their default values.
 */
class ParamsTest : public ::testing::Test
{
protected:
  /** @brief Builds the parameters, logging through a test logger. */
  ParamsTest() : params_(rclcpp::get_logger("test_params"))
  {
  }

  /** @brief The parameters under test; every member starts at its default. */
  K4AROSDeviceParams params_;

  /** @brief The device configuration that `GetDeviceConfig()` fills. */
  k4a_device_configuration_t configuration_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
};

/**
 * @brief The default parameters give a valid, depth only configuration.
 */
TEST_F(ParamsTest, DefaultsGiveAValidDepthOnlyConfiguration)
{
  ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED);

  EXPECT_EQ(configuration_.depth_mode, K4A_DEPTH_MODE_NFOV_UNBINNED);
  EXPECT_EQ(configuration_.color_resolution, K4A_COLOR_RESOLUTION_OFF);
  EXPECT_EQ(configuration_.camera_fps, K4A_FRAMES_PER_SECOND_5);
  EXPECT_EQ(configuration_.wired_sync_mode, K4A_WIRED_SYNC_MODE_STANDALONE);
  EXPECT_FALSE(configuration_.synchronized_images_only);
}

/**
 * @brief Every depth mode name maps to its SDK mode, and depth can be switched off.
 */
TEST_F(ParamsTest, DepthModesAreMapped)
{
  const std::pair<const char*, k4a_depth_mode_t> modes[] = {
    { "NFOV_2X2BINNED", K4A_DEPTH_MODE_NFOV_2X2BINNED },
    { "NFOV_UNBINNED", K4A_DEPTH_MODE_NFOV_UNBINNED },
    { "WFOV_2X2BINNED", K4A_DEPTH_MODE_WFOV_2X2BINNED },
    { "WFOV_UNBINNED", K4A_DEPTH_MODE_WFOV_UNBINNED },
  };
  for (const auto& [name, mode] : modes)
  {
    params_.depth_mode = name;
    ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED) << name;
    EXPECT_EQ(configuration_.depth_mode, mode) << name;
  }

  params_.depth_enabled = false;
  ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED);
  EXPECT_EQ(configuration_.depth_mode, K4A_DEPTH_MODE_OFF);
}

/**
 * @brief Every color resolution and format maps to its SDK value.
 */
TEST_F(ParamsTest, ColorResolutionsAndFormatsAreMapped)
{
  const std::pair<const char*, k4a_color_resolution_t> resolutions[] = {
    { "720P", K4A_COLOR_RESOLUTION_720P },   { "1080P", K4A_COLOR_RESOLUTION_1080P },
    { "1440P", K4A_COLOR_RESOLUTION_1440P }, { "1536P", K4A_COLOR_RESOLUTION_1536P },
    { "2160P", K4A_COLOR_RESOLUTION_2160P }, { "3072P", K4A_COLOR_RESOLUTION_3072P },
  };
  params_.color_enabled = true;
  for (const auto& [name, resolution] : resolutions)
  {
    params_.color_resolution = name;
    ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED) << name;
    EXPECT_EQ(configuration_.color_resolution, resolution) << name;
  }

  params_.color_format = "jpeg";
  ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED);
  EXPECT_EQ(configuration_.color_format, K4A_IMAGE_FORMAT_COLOR_MJPG);
  params_.color_format = "bgra";
  ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED);
  EXPECT_EQ(configuration_.color_format, K4A_IMAGE_FORMAT_COLOR_BGRA32);
}

/**
 * @brief The frame rates, the wired sync modes and the synchronized images are mapped.
 */
TEST_F(ParamsTest, FrameRatesSyncModesAndSynchronizedImagesAreMapped)
{
  const std::pair<int, k4a_fps_t> rates[] = { { 5, K4A_FRAMES_PER_SECOND_5 },
                                              { 15, K4A_FRAMES_PER_SECOND_15 },
                                              { 30, K4A_FRAMES_PER_SECOND_30 } };
  for (const auto& [fps, expected] : rates)
  {
    params_.fps = fps;
    ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED) << fps;
    EXPECT_EQ(configuration_.camera_fps, expected) << fps;
  }

  const std::pair<int, k4a_wired_sync_mode_t> modes[] = {
    { 0, K4A_WIRED_SYNC_MODE_STANDALONE },
    { 1, K4A_WIRED_SYNC_MODE_MASTER },
    { 2, K4A_WIRED_SYNC_MODE_SUBORDINATE },
  };
  for (const auto& [mode, expected] : modes)
  {
    params_.wired_sync_mode = mode;
    ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED) << mode;
    EXPECT_EQ(configuration_.wired_sync_mode, expected) << mode;
  }

  // Both cameras enabled ask for synchronized images
  params_.color_enabled = true;
  ASSERT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED);
  EXPECT_TRUE(configuration_.synchronized_images_only);
}

/**
 * @brief Values that are not valid options are rejected.
 */
TEST_F(ParamsTest, InvalidOptionsAreRejected)
{
  params_.fps = 20;
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
  params_.fps = 15;

  params_.wired_sync_mode = 3;
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
  params_.wired_sync_mode = 0;

  params_.depth_mode = "bogus";
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
  params_.depth_mode = "NFOV_UNBINNED";

  params_.color_enabled = true;
  params_.color_resolution = "bogus";
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
  params_.color_resolution = "720P";

  params_.color_format = "png";
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
}

/**
 * @brief The options of the point clouds that cannot work together are rejected.
 */
TEST_F(ParamsTest, IncompatiblePointCloudOptionsAreRejected)
{
  // A point cloud needs depth, which passive IR does not give
  params_.depth_mode = "PASSIVE_IR";
  params_.point_cloud = true;
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
  params_.point_cloud = false;
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED);
  params_.depth_mode = "NFOV_UNBINNED";

  // The RGB point cloud needs the point cloud, the color camera and raw color images
  params_.rgb_point_cloud = true;
  params_.point_cloud = false;
  params_.color_enabled = true;
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
  params_.point_cloud = true;
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_SUCCEEDED);
  params_.color_enabled = false;
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
  params_.color_enabled = true;
  params_.color_format = "jpeg";
  EXPECT_EQ(params_.GetDeviceConfig(&configuration_), K4A_RESULT_FAILED);
}

/**
 * @brief A target IMU rate of zero means the maximum rate.
 */
TEST_F(ParamsTest, ZeroImuRateMeansTheMaximum)
{
  params_.imu_rate_target = 0;

  EXPECT_EQ(params_.ValidateImuRate(), K4A_RESULT_SUCCEEDED);
  EXPECT_EQ(params_.imu_rate_target, IMU_MAX_RATE);
}

/**
 * @brief A feasible IMU rate is kept and one outside of the range is rejected.
 */
TEST_F(ParamsTest, ImuRateOutsideTheRangeIsRejected)
{
  params_.imu_rate_target = 100;
  EXPECT_EQ(params_.ValidateImuRate(), K4A_RESULT_SUCCEEDED);
  EXPECT_EQ(params_.imu_rate_target, 100);

  params_.imu_rate_target = IMU_MAX_RATE + 1;
  EXPECT_EQ(params_.ValidateImuRate(), K4A_RESULT_FAILED);
  params_.imu_rate_target = -5;
  EXPECT_EQ(params_.ValidateImuRate(), K4A_RESULT_FAILED);
}

/**
 * @brief The frame rate of a recording replaces the one of the parameters.
 */
TEST_F(ParamsTest, RecordingFrameRateReplacesTheParameter)
{
  k4a_record_configuration_t recording = {};
  recording.color_track_enabled = true;
  recording.depth_track_enabled = true;
  recording.ir_track_enabled = true;

  recording.camera_fps = K4A_FRAMES_PER_SECOND_30;
  adaptParamsToRecording(recording, params_, rclcpp::get_logger("test_params"));
  EXPECT_EQ(params_.fps, 30);

  recording.camera_fps = K4A_FRAMES_PER_SECOND_15;
  adaptParamsToRecording(recording, params_, rclcpp::get_logger("test_params"));
  EXPECT_EQ(params_.fps, 15);
}

/**
 * @brief Streams without a track in the recording are disabled.
 */
TEST_F(ParamsTest, StreamsWithoutATrackAreDisabled)
{
  params_.color_enabled = true;
  params_.rgb_point_cloud = true;
  params_.point_cloud = true;
  k4a_record_configuration_t recording = {};
  recording.camera_fps = K4A_FRAMES_PER_SECOND_15;

  // Only a depth track: no color, so no RGB point cloud
  recording.depth_track_enabled = true;
  adaptParamsToRecording(recording, params_, rclcpp::get_logger("test_params"));
  EXPECT_FALSE(params_.color_enabled);
  EXPECT_FALSE(params_.rgb_point_cloud);
  EXPECT_TRUE(params_.point_cloud);
  EXPECT_TRUE(params_.depth_enabled);

  // Only an IR track: no depth, so no point cloud, but the IR stream remains
  recording.depth_track_enabled = false;
  recording.ir_track_enabled = true;
  adaptParamsToRecording(recording, params_, rclcpp::get_logger("test_params"));
  EXPECT_FALSE(params_.point_cloud);
  EXPECT_TRUE(params_.depth_enabled);

  // Neither IR nor depth
  recording.ir_track_enabled = false;
  adaptParamsToRecording(recording, params_, rclcpp::get_logger("test_params"));
  EXPECT_FALSE(params_.depth_enabled);
}

/**
 * @brief Runs the unit tests.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
