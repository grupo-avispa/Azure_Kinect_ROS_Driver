// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_conversions.h"

// System headers
//
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

// Library headers
//
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace azure_kinect_ros_driver
{
namespace conversions
{
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

namespace
{
/** @brief Bytes of one BGRA pixel. */
constexpr size_t kBgraBytesPerPixel = 4;

/** @brief Factor from the millimetres of the SDK to the metres of ROS. */
constexpr float kMillimeterToMeter = 1.0f / 1000.0f;

/**
 * @brief Creates an image message with the layout of `source` and room for the converted pixels.
 */
Image::UniquePtr makeImage(const k4a::image& source, const std::string& encoding,
                           size_t bytes_per_pixel)
{
  auto msg = std::make_unique<Image>();
  msg->height = source.get_height_pixels();
  msg->width = source.get_width_pixels();
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = msg->width * bytes_per_pixel;
  msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
  return msg;
}

/**
 * @brief Copies the rows of an SDK image into a message whose rows are tightly packed.
 */
void copyRows(const k4a::image& source, size_t row_bytes, Image& msg)
{
  const uint8_t* source_row = source.get_buffer();
  const size_t stride = source.get_stride_bytes();
  uint8_t* target_row = msg.data.data();
  for (uint32_t y = 0; y < msg.height; ++y)
  {
    std::memcpy(target_row, source_row, row_bytes);
    source_row += stride;
    target_row += msg.step;
  }
}

/**
 * @brief Fills the points of an already sized cloud, invalid points becoming NaN.
 *
 * @tparam kWithColor Whether the color is filled too, from a BGRA image.
 * @param cloud The cloud, already sized and with the fields of the chosen layout.
 * @param point_cloud_buffer The x, y, z values of the SDK point cloud image, in millimetres.
 * @param color_buffer The BGRA pixels, only read if `kWithColor`.
 * @param point_count The number of points.
 */
template <bool kWithColor>
void fillPoints(PointCloud2& cloud, const int16_t* point_cloud_buffer, const uint8_t* color_buffer,
                size_t point_count)
{
  /** @brief Stands in for the color iterators of a cloud that has no color. */
  struct NoColorIterator
  {
    NoColorIterator(PointCloud2&, const std::string&)
    {
    }
    NoColorIterator& operator++()
    {
      return *this;
    }
  };
  using ColorIterator =
    std::conditional_t<kWithColor, sensor_msgs::PointCloud2Iterator<uint8_t>, NoColorIterator>;

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  ColorIterator iter_r(cloud, "r");
  ColorIterator iter_g(cloud, "g");
  ColorIterator iter_b(cloud, "b");

  for (size_t i = 0; i < point_count;
       i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    // Z in image frame:
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

    bool valid = z > 0.0f;
    if constexpr (kWithColor)
    {
      // Alpha value:
      valid = valid && color_buffer[4 * i + 3] != 0;
    }

    if (!valid)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      if constexpr (kWithColor)
      {
        *iter_r = *iter_g = *iter_b = 0;
      }
    }
    else
    {
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;
      if constexpr (kWithColor)
      {
        *iter_r = color_buffer[4 * i + 2];
        *iter_g = color_buffer[4 * i + 1];
        *iter_b = color_buffer[4 * i + 0];
      }
    }
  }
}
}  // namespace

bool parseDepthUnit(const std::string& name, DepthUnit& unit)
{
  if (name == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    unit = DepthUnit::kMillimeters;
    return true;
  }
  if (name == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    unit = DepthUnit::kMeters;
    return true;
  }
  return false;
}

Image::UniquePtr depthToImage(const k4a::image& depth, DepthUnit unit)
{
  if (!depth)
  {
    return nullptr;
  }

  if (unit == DepthUnit::kMillimeters)
  {
    // The source data is already in the 'K4A_IMAGE_FORMAT_DEPTH16' format
    auto msg = makeImage(depth, sensor_msgs::image_encodings::TYPE_16UC1, sizeof(uint16_t));
    copyRows(depth, msg->step, *msg);
    return msg;
  }

  // Convert from 16 bit integer millimetres to 32 bit float metres
  auto msg = makeImage(depth, sensor_msgs::image_encodings::TYPE_32FC1, sizeof(float));
  for (uint32_t y = 0; y < msg->height; ++y)
  {
    const uint16_t* source_row =
      reinterpret_cast<const uint16_t*>(depth.get_buffer() + y * depth.get_stride_bytes());
    float* target_row = reinterpret_cast<float*>(msg->data.data() + y * msg->step);
    for (uint32_t x = 0; x < msg->width; ++x)
    {
      target_row[x] = static_cast<float>(source_row[x]) * kMillimeterToMeter;
    }
  }
  return msg;
}

Image::UniquePtr irToImage(const k4a::image& ir, bool rescale_to_mono8, float mono8_scale)
{
  if (!ir)
  {
    return nullptr;
  }

  if (!rescale_to_mono8)
  {
    auto msg = makeImage(ir, sensor_msgs::image_encodings::MONO16, sizeof(uint16_t));
    copyRows(ir, msg->step, *msg);
    return msg;
  }

  // Rescale the image to mono8 for visualization and usage for visual(-inertial) odometry.
  auto msg = makeImage(ir, sensor_msgs::image_encodings::MONO8, sizeof(uint8_t));
  for (uint32_t y = 0; y < msg->height; ++y)
  {
    const uint16_t* source_row =
      reinterpret_cast<const uint16_t*>(ir.get_buffer() + y * ir.get_stride_bytes());
    uint8_t* target_row = msg->data.data() + y * msg->step;
    for (uint32_t x = 0; x < msg->width; ++x)
    {
      const long scaled = std::lrintf(static_cast<float>(source_row[x]) * mono8_scale);
      target_row[x] = static_cast<uint8_t>(std::clamp<long>(scaled, 0, 255));
    }
  }
  return msg;
}

Image::UniquePtr bgraToImage(const k4a::image& bgra)
{
  if (!bgra)
  {
    return nullptr;
  }

  const size_t expected_size =
    static_cast<size_t>(bgra.get_width_pixels()) * bgra.get_height_pixels() * kBgraBytesPerPixel;
  if (bgra.get_size() != expected_size)
  {
    return nullptr;
  }

  auto msg = makeImage(bgra, sensor_msgs::image_encodings::BGRA8, kBgraBytesPerPixel);
  copyRows(bgra, msg->step, *msg);
  return msg;
}

CompressedImage::UniquePtr jpegToCompressed(const k4a::image& jpeg)
{
  if (!jpeg)
  {
    return nullptr;
  }

  auto msg = std::make_unique<CompressedImage>();
  msg->format = "bgra8; jpeg compressed bgr8";
  const uint8_t* buffer = jpeg.get_buffer();
  msg->data.assign(buffer, buffer + jpeg.get_size());
  return msg;
}

PointCloud2::UniquePtr pointCloudToMsg(const k4a::image& point_cloud, const k4a::image* bgra)
{
  if (!point_cloud)
  {
    return nullptr;
  }

  const size_t point_count =
    static_cast<size_t>(point_cloud.get_height_pixels()) * point_cloud.get_width_pixels();
  if (bgra != nullptr && (!*bgra || bgra->get_size() / kBgraBytesPerPixel != point_count))
  {
    return nullptr;
  }

  auto cloud = std::make_unique<PointCloud2>();
  cloud->height = point_cloud.get_height_pixels();
  cloud->width = point_cloud.get_width_pixels();
  cloud->is_dense = false;
  cloud->is_bigendian = false;

  // The cloud has to be sized before the iterators are created
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  if (bgra != nullptr)
  {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  }
  else
  {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(point_cloud.get_buffer());
  if (bgra != nullptr)
  {
    fillPoints<true>(*cloud, point_cloud_buffer, bgra->get_buffer(), point_count);
  }
  else
  {
    fillPoints<false>(*cloud, point_cloud_buffer, nullptr, point_count);
  }
  return cloud;
}

CameraInfo cameraInfoFromCalibration(const k4a_calibration_camera_t& calibration,
                                     const std::string& frame_id)
{
  CameraInfo camera_info;
  camera_info.header.frame_id = frame_id;
  camera_info.width = calibration.resolution_width;
  camera_info.height = calibration.resolution_height;
  camera_info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  const k4a_calibration_intrinsic_parameters_t* parameters = &calibration.intrinsics.parameters;

  // The distortion parameters, size depending on the distortion model.
  // For "rational_polynomial", the 8 parameters are: (k1, k2, p1, p2, k3, k4, k5, k6).
  camera_info.d = { parameters->param.k1, parameters->param.k2, parameters->param.p1,
                    parameters->param.p2, parameters->param.k3, parameters->param.k4,
                    parameters->param.k5, parameters->param.k6 };

  // clang-format off
  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel
  // coordinates using the focal lengths (fx, fy) and principal point
  // (cx, cy).
  camera_info.k = {parameters->param.fx,  0.0f,                   parameters->param.cx,
                   0.0f,                  parameters->param.fy,   parameters->param.cy,
                   0.0f,                  0.0,                    1.0f};

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  // The raw images are not rectified, so P repeats K and Tx = Ty = 0 (monocular camera).
  camera_info.p = {parameters->param.fx,  0.0f,                   parameters->param.cx,   0.0f,
                   0.0f,                  parameters->param.fy,   parameters->param.cy,   0.0f,
                   0.0f,                  0.0,                    1.0f,                   0.0f};

  // Rectification matrix (stereo cameras only): the identity for a monocular camera
  camera_info.r = {1.0f, 0.0f, 0.0f,
                   0.0f, 1.0f, 0.0f,
                   0.0f, 0.0f, 1.0f};
  // clang-format on

  return camera_info;
}
}  // namespace conversions
}  // namespace azure_kinect_ros_driver
