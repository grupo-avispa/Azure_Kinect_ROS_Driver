// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_CONVERSIONS_H
#define K4A_CONVERSIONS_H

// System headers
//
#include <string>

// Library headers
//
#include <k4a/k4a.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace azure_kinect_ros_driver
{
/**
 * @brief Pure conversions from Azure Kinect SDK images and calibration to ROS messages.
 *
 * None of them needs a device, a node or a clock, so they can be tested with synthetic images.
 * Image conversions return the message by `unique_ptr` so that it can be published without
 * copies, and `nullptr` if the input is invalid. The header (stamp and frame id) is left empty
 * for the caller to fill.
 */
namespace conversions
{
/**
 * @brief Unit of the depth images published by the driver.
 */
enum class DepthUnit
{
  /** @brief 16 bit integers in millimetres, the native format of the sensor (`16UC1`). */
  kMillimeters,
  /** @brief 32 bit floats in metres (`32FC1`). */
  kMeters
};

/**
 * @brief Parses the `depth_unit` parameter, which holds the ROS encoding name.
 *
 * @param name The encoding name, `16UC1` or `32FC1`.
 * @param unit The parsed unit; unchanged if the name is not recognized.
 * @return True if the name was recognized.
 */
bool parseDepthUnit(const std::string& name, DepthUnit& unit);

/**
 * @brief Converts a `K4A_IMAGE_FORMAT_DEPTH16` image.
 *
 * @param depth The SDK depth image.
 * @param unit The unit of the output image.
 * @return The image message, or nullptr if `depth` is not valid.
 */
sensor_msgs::msg::Image::UniquePtr depthToImage(const k4a::image& depth, DepthUnit unit);

/**
 * @brief Converts a `K4A_IMAGE_FORMAT_IR16` image.
 *
 * @param ir The SDK infrared image.
 * @param rescale_to_mono8 If true the values are multiplied by `mono8_scale` and saturated to
 * 8 bits (`mono8`), otherwise the image is published as `mono16`.
 * @param mono8_scale The scale applied when rescaling to 8 bits.
 * @return The image message, or nullptr if `ir` is not valid.
 */
sensor_msgs::msg::Image::UniquePtr irToImage(const k4a::image& ir, bool rescale_to_mono8,
                                             float mono8_scale);

/**
 * @brief Converts a `K4A_IMAGE_FORMAT_COLOR_BGRA32` image.
 *
 * @param bgra The SDK color image.
 * @return The image message, or nullptr if `bgra` is not valid or its size is inconsistent.
 */
sensor_msgs::msg::Image::UniquePtr bgraToImage(const k4a::image& bgra);

/**
 * @brief Wraps a `K4A_IMAGE_FORMAT_COLOR_MJPG` image, without decoding it.
 *
 * @param jpeg The SDK color image.
 * @return The compressed image message, or nullptr if `jpeg` is not valid.
 */
sensor_msgs::msg::CompressedImage::UniquePtr jpegToCompressed(const k4a::image& jpeg);

/**
 * @brief Converts an SDK point cloud image to a `PointCloud2` in metres.
 *
 * Points without depth become NaN. If a color image is given the cloud also carries the color
 * (`rgb`) and the points whose pixel has zero alpha become NaN as well.
 *
 * @param point_cloud The SDK point cloud image (x, y, z as int16 millimetres).
 * @param bgra The color image with as many pixels as the cloud, or nullptr for no color.
 * @return The cloud, or nullptr if an image is not valid or their sizes do not match.
 */
sensor_msgs::msg::PointCloud2::UniquePtr pointCloudToMsg(const k4a::image& point_cloud,
                                                         const k4a::image* bgra);

/**
 * @brief Builds the `CameraInfo` of one camera from its SDK calibration.
 *
 * The raw images are not rectified, so the projection matrix repeats the intrinsic matrix and the
 * rectification matrix is the identity. The distortion model is `rational_polynomial`.
 *
 * @param calibration The SDK calibration of the camera.
 * @param frame_id The frame id of the camera.
 * @return The camera info, with an empty stamp.
 */
sensor_msgs::msg::CameraInfo cameraInfoFromCalibration(const k4a_calibration_camera_t& calibration,
                                                       const std::string& frame_id);
}  // namespace conversions
}  // namespace azure_kinect_ros_driver

#endif  // K4A_CONVERSIONS_H
