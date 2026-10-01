// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

// Library headers
//
#include <gtest/gtest.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_conversions.h"

using namespace azure_kinect_ros_driver::conversions;

namespace
{

template <typename ValueFn>
k4a::image makeImage16(k4a_image_format_t format, int width, int height, ValueFn value)
{
  k4a::image image =
    k4a::image::create(format, width, height, width * static_cast<int>(sizeof(uint16_t)));
  uint16_t* pixels = reinterpret_cast<uint16_t*>(image.get_buffer());
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      pixels[y * width + x] = value(x, y);
    }
  }
  return image;
}

/** @brief Creates a point cloud image where `point(i)` gives the {x, y, z} of point `i` in mm. */
template <typename PointFn>
k4a::image makePointCloud(int width, int height, PointFn point)
{
  k4a::image image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, width, height,
                                        width * 3 * static_cast<int>(sizeof(int16_t)));
  int16_t* values = reinterpret_cast<int16_t*>(image.get_buffer());
  for (int i = 0; i < width * height; ++i)
  {
    const auto p = point(i);
    values[3 * i + 0] = p[0];
    values[3 * i + 1] = p[1];
    values[3 * i + 2] = p[2];
  }
  return image;
}

/** @brief Creates an SDK BGRA image filled with one color. */
k4a::image makeBgra(int width, int height, uint8_t b, uint8_t g, uint8_t r, uint8_t a)
{
  k4a::image image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4);
  uint8_t* pixels = image.get_buffer();
  for (int i = 0; i < width * height; ++i)
  {
    pixels[4 * i + 0] = b;
    pixels[4 * i + 1] = g;
    pixels[4 * i + 2] = r;
    pixels[4 * i + 3] = a;
  }
  return image;
}
}  // namespace

/**
 * @brief The depth unit is parsed from the ROS encoding names and unknown names are rejected.
 */
TEST(Conversions, ParseDepthUnit)
{
  DepthUnit unit = DepthUnit::kMeters;
  EXPECT_TRUE(parseDepthUnit("16UC1", unit));
  EXPECT_EQ(unit, DepthUnit::kMillimeters);
  EXPECT_TRUE(parseDepthUnit("32FC1", unit));
  EXPECT_EQ(unit, DepthUnit::kMeters);
  EXPECT_FALSE(parseDepthUnit("bogus", unit));
  EXPECT_FALSE(parseDepthUnit("", unit));
}

/**
 * @brief Depth in millimetres is a byte-for-byte copy of the SDK image.
 */
TEST(Conversions, DepthInMillimetersKeepsThePixels)
{
  k4a::image depth =
    makeImage16(K4A_IMAGE_FORMAT_DEPTH16, 4, 3, [](int x, int y) { return 100 * y + x; });

  auto msg = depthToImage(depth, DepthUnit::kMillimeters);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->encoding, sensor_msgs::image_encodings::TYPE_16UC1);
  EXPECT_EQ(msg->width, 4u);
  EXPECT_EQ(msg->height, 3u);
  EXPECT_EQ(msg->step, 8u);
  ASSERT_EQ(msg->data.size(), 24u);
  EXPECT_EQ(std::memcmp(msg->data.data(), depth.get_buffer(), 24), 0);
}

/**
 * @brief Depth in metres divides the millimetre values by 1000 and uses `32FC1`.
 */
TEST(Conversions, DepthInMetersScalesThePixels)
{
  k4a::image depth =
    makeImage16(K4A_IMAGE_FORMAT_DEPTH16, 2, 2, [](int x, int y) { return 1500 * (x + y); });

  auto msg = depthToImage(depth, DepthUnit::kMeters);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->encoding, sensor_msgs::image_encodings::TYPE_32FC1);
  EXPECT_EQ(msg->step, 8u);
  const float* values = reinterpret_cast<const float*>(msg->data.data());
  EXPECT_FLOAT_EQ(values[0], 0.0f);
  EXPECT_FLOAT_EQ(values[1], 1.5f);
  EXPECT_FLOAT_EQ(values[2], 1.5f);
  EXPECT_FLOAT_EQ(values[3], 3.0f);
}

/**
 * @brief Every image conversion returns nullptr for an empty SDK image.
 */
TEST(Conversions, InvalidImagesGiveNull)
{
  k4a::image empty;
  EXPECT_EQ(depthToImage(empty, DepthUnit::kMillimeters), nullptr);
  EXPECT_EQ(irToImage(empty, false, 1.0f), nullptr);
  EXPECT_EQ(bgraToImage(empty), nullptr);
  EXPECT_EQ(jpegToCompressed(empty), nullptr);
  EXPECT_EQ(pointCloudToMsg(empty, nullptr), nullptr);
}

/**
 * @brief Without rescaling the IR image is a byte-for-byte `mono16` copy.
 */
TEST(Conversions, IrWithoutRescalingIsMono16)
{
  k4a::image ir =
    makeImage16(K4A_IMAGE_FORMAT_IR16, 3, 1, [](int x, int) { return 1000 * (x + 1); });

  auto msg = irToImage(ir, false, 10.0f);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->encoding, sensor_msgs::image_encodings::MONO16);
  EXPECT_EQ(msg->step, 6u);
  EXPECT_EQ(std::memcmp(msg->data.data(), ir.get_buffer(), 6), 0);
}

/**
 * @brief Rescaled IR is scaled, rounded half to even and saturated to 8 bits.
 */
TEST(Conversions, IrRescaledToMono8ScalesRoundsAndSaturates)
{
  // 1000 * 0.1 = 100, 5 * 0.5 = 2.5 -> 2 (round half to even), 7 * 0.5 = 3.5 -> 4, 65535 saturates
  const uint16_t input[] = { 1000, 5, 7, 65535 };
  const float scales[] = { 0.1f, 0.5f, 0.5f, 1.0f };
  const uint8_t expected[] = { 100, 2, 4, 255 };
  for (int i = 0; i < 4; ++i)
  {
    k4a::image ir = makeImage16(K4A_IMAGE_FORMAT_IR16, 1, 1, [&](int, int) { return input[i]; });
    auto msg = irToImage(ir, true, scales[i]);
    ASSERT_NE(msg, nullptr);
    EXPECT_EQ(msg->encoding, sensor_msgs::image_encodings::MONO8);
    EXPECT_EQ(msg->step, 1u);
    EXPECT_EQ(msg->data[0], expected[i]) << "input " << input[i] << " scale " << scales[i];
  }
}

/**
 * @brief A BGRA image is a byte-for-byte `bgra8` copy.
 */
TEST(Conversions, BgraKeepsThePixels)
{
  k4a::image bgra = makeBgra(3, 2, 10, 20, 30, 255);

  auto msg = bgraToImage(bgra);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->encoding, sensor_msgs::image_encodings::BGRA8);
  EXPECT_EQ(msg->width, 3u);
  EXPECT_EQ(msg->height, 2u);
  EXPECT_EQ(msg->step, 12u);
  ASSERT_EQ(msg->data.size(), 24u);
  EXPECT_EQ(std::memcmp(msg->data.data(), bgra.get_buffer(), 24), 0);
}

/**
 * @brief A BGRA image whose size does not match its dimensions is rejected.
 */
TEST(Conversions, BgraWithPaddedRowsIsRejected)
{
  k4a::image padded = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, 3, 2, 16);

  EXPECT_EQ(bgraToImage(padded), nullptr);
}

/**
 * @brief An MJPG image is wrapped as a compressed image with the same bytes.
 */
TEST(Conversions, JpegIsWrappedWithoutDecoding)
{
  uint8_t data[] = { 0xFF, 0xD8, 0x01, 0x02, 0xFF, 0xD9 };
  k4a::image buffer = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_COLOR_MJPG, 640, 480, 0, data,
                                                     sizeof(data), nullptr, nullptr);

  auto msg = jpegToCompressed(buffer);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->format, "bgra8; jpeg compressed bgr8");
  ASSERT_EQ(msg->data.size(), sizeof(data));
  EXPECT_EQ(std::memcmp(msg->data.data(), data, sizeof(data)), 0);
}

/**
 * @brief Points are converted to metres and those without depth become NaN.
 */
TEST(Conversions, PointCloudConvertsToMetersAndMarksInvalidPointsAsNan)
{
  // Two points: one valid, one without depth
  k4a::image cloud = makePointCloud(2, 1,
                                    [](int i)
                                    {
                                      return i == 0 ? std::array<int16_t, 3>{ 1000, -2000, 3000 }
                                                    : std::array<int16_t, 3>{ 5, 6, 0 };
                                    });

  auto msg = pointCloudToMsg(cloud, nullptr);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->width, 2u);
  EXPECT_EQ(msg->height, 1u);
  EXPECT_FALSE(msg->is_dense);
  EXPECT_EQ(msg->fields.size(), 3u);

  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
  EXPECT_FLOAT_EQ(*x, 1.0f);
  EXPECT_FLOAT_EQ(*y, -2.0f);
  EXPECT_FLOAT_EQ(*z, 3.0f);
  ++x;
  ++y;
  ++z;
  EXPECT_TRUE(std::isnan(*x));
  EXPECT_TRUE(std::isnan(*y));
  EXPECT_TRUE(std::isnan(*z));
}

/**
 * @brief The colored cloud swaps BGR to RGB and drops transparent pixels to NaN.
 */
TEST(Conversions, ColoredPointCloudSwapsBgraToRgbAndDropsTransparentPixels)
{
  k4a::image cloud =
    makePointCloud(2, 1, [](int) { return std::array<int16_t, 3>{ 1000, 2000, 3000 }; });
  // First pixel opaque, second transparent
  k4a::image bgra = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, 2, 1, 8);
  const uint8_t pixels[] = { 10, 20, 30, 255, 40, 50, 60, 0 };
  std::memcpy(bgra.get_buffer(), pixels, sizeof(pixels));

  auto msg = pointCloudToMsg(cloud, &bgra);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->fields.size(), 4u);
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> r(*msg, "r"), g(*msg, "g"), b(*msg, "b");
  EXPECT_FLOAT_EQ(*x, 1.0f);
  EXPECT_EQ(*r, 30);
  EXPECT_EQ(*g, 20);
  EXPECT_EQ(*b, 10);
  ++x;
  ++r;
  ++g;
  ++b;
  EXPECT_TRUE(std::isnan(*x));
  EXPECT_EQ(*r, 0);
  EXPECT_EQ(*g, 0);
  EXPECT_EQ(*b, 0);
}

/**
 * @brief A colored cloud with a different number of pixels than points is rejected.
 */
TEST(Conversions, ColoredPointCloudWithMismatchedImageIsRejected)
{
  k4a::image cloud = makePointCloud(2, 2, [](int) { return std::array<int16_t, 3>{ 1, 2, 3 }; });
  k4a::image bgra = makeBgra(3, 1, 0, 0, 0, 255);

  EXPECT_EQ(pointCloudToMsg(cloud, &bgra), nullptr);
}

/**
 * @brief The camera info orders the distortion coefficients as ROS expects and keeps P and R
 * trivial.
 */
TEST(Conversions, CameraInfoFollowsTheRationalPolynomialModel)
{
  k4a_calibration_camera_t calibration = {};
  calibration.resolution_width = 640;
  calibration.resolution_height = 576;
  auto& p = calibration.intrinsics.parameters.param;
  p.fx = 500.0f;
  p.fy = 510.0f;
  p.cx = 320.5f;
  p.cy = 288.5f;
  p.k1 = 1.0f;
  p.k2 = 2.0f;
  p.k3 = 3.0f;
  p.k4 = 4.0f;
  p.k5 = 5.0f;
  p.k6 = 6.0f;
  p.p1 = 7.0f;
  p.p2 = 8.0f;

  auto info = cameraInfoFromCalibration(calibration, "cam_link");

  EXPECT_EQ(info.header.frame_id, "cam_link");
  EXPECT_EQ(info.width, 640u);
  EXPECT_EQ(info.height, 576u);
  EXPECT_EQ(info.distortion_model, "rational_polynomial");
  // (k1, k2, p1, p2, k3, k4, k5, k6)
  const std::vector<double> expected_d = { 1, 2, 7, 8, 3, 4, 5, 6 };
  EXPECT_EQ(info.d, expected_d);
  EXPECT_DOUBLE_EQ(info.k[0], 500.0);
  EXPECT_DOUBLE_EQ(info.k[2], 320.5);
  EXPECT_DOUBLE_EQ(info.k[4], 510.0);
  EXPECT_DOUBLE_EQ(info.k[5], 288.5);
  EXPECT_DOUBLE_EQ(info.k[8], 1.0);
  // The raw images are not rectified: P repeats K with no translation and R is the identity
  EXPECT_DOUBLE_EQ(info.p[0], 500.0);
  EXPECT_DOUBLE_EQ(info.p[2], 320.5);
  EXPECT_DOUBLE_EQ(info.p[3], 0.0);
  EXPECT_DOUBLE_EQ(info.p[10], 1.0);
  EXPECT_DOUBLE_EQ(info.r[0], 1.0);
  EXPECT_DOUBLE_EQ(info.r[4], 1.0);
  EXPECT_DOUBLE_EQ(info.r[8], 1.0);
  EXPECT_DOUBLE_EQ(info.r[1], 0.0);
}

/** @brief Runs the unit tests. */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
