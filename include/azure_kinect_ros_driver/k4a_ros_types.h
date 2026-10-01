// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_TYPES_H
#define K4A_ROS_TYPES_H

#include <array>
#include <vector>

#include <k4a/k4atypes.h>

using DepthPixel = uint16_t;
using IrPixel = uint16_t;

struct BgraPixel
{
  uint8_t Blue;
  uint8_t Green;
  uint8_t Red;
  uint8_t Alpha;
};

#if defined(K4A_BODY_TRACKING)
using BodyIndexMapPixel = uint8_t;

struct Color
{
  float r, g, b, a;
};

using ColorPalette = std::array<Color, 8>;
// a palette of 8 colors to colorize the different body markers and the body index map
static const ColorPalette BODY_COLOR_PALETTE{ { { 1.0f, 0.0f, 0.0f, 1.0f },
                                                { 0.0f, 1.0f, 0.0f, 1.0f },
                                                { 0.0f, 0.0f, 1.0f, 1.0f },
                                                { 1.0f, 1.0f, 0.0f, 1.0f },
                                                { 1.0f, 0.0f, 1.0f, 1.0f },
                                                { 0.0f, 1.0f, 1.0f, 1.0f },
                                                { 0.0f, 0.0f, 0.0f, 1.0f },
                                                { 1.0f, 1.0f, 1.0f, 1.0f } } };
#endif

/** Three dimensional double precision floating point vector.
 *
 * \xmlonly
 * <requirements>
 *   <requirement name="Header">k4atypes.h (include k4a/k4a.h)</requirement>
 * </requirements>
 * \endxmlonly
 */
typedef struct _k4a_double3_t
{
  double x = 0.0; /**< X component of a vector. */
  double y = 0.0; /**< Y component of a vector. */
  double z = 0.0; /**< Z component of a vector. */
} k4a_double3_t;

/** IMU sample in double precision.
 *
 * \xmlonly
 * <requirements>
 *   <requirement name="Header">k4atypes.h (include k4a/k4a.h)</requirement>
 * </requirements>
 * \endxmlonly
 */
typedef struct _k4a_imu_accumulator_t
{
  double temperature = 0.0;  /**< Temperature reading of this sample (Celsius). */
  k4a_double3_t acc_sample;  /**< Accelerometer sample in meters per second squared. */
  k4a_double3_t gyro_sample; /**< Gyro sample in radians per second. */

  _k4a_imu_accumulator_t& operator+=(const k4a_imu_sample_t& a)
  {
    temperature += a.temperature;
    acc_sample.x += a.acc_sample.xyz.x;
    acc_sample.y += a.acc_sample.xyz.y;
    acc_sample.z += a.acc_sample.xyz.z;
    gyro_sample.x += a.gyro_sample.xyz.x;
    gyro_sample.y += a.gyro_sample.xyz.y;
    gyro_sample.z += a.gyro_sample.xyz.z;
    return *this;
  }

  _k4a_imu_accumulator_t& operator/=(float div)
  {
    temperature /= div;
    acc_sample.x /= div;
    acc_sample.y /= div;
    acc_sample.z /= div;
    gyro_sample.x /= div;
    gyro_sample.y /= div;
    gyro_sample.z /= div;
    return *this;
  }

  void to_float(k4a_imu_sample_t& sample)
  {
    sample.temperature = static_cast<float>(temperature);
    sample.acc_sample.xyz.x = static_cast<float>(acc_sample.x);
    sample.acc_sample.xyz.y = static_cast<float>(acc_sample.y);
    sample.acc_sample.xyz.z = static_cast<float>(acc_sample.z);
    sample.gyro_sample.xyz.x = static_cast<float>(gyro_sample.x);
    sample.gyro_sample.xyz.y = static_cast<float>(gyro_sample.y);
    sample.gyro_sample.xyz.z = static_cast<float>(gyro_sample.z);
  }
} k4a_imu_accumulator_t;

/** Reduces the IMU output rate by averaging groups of consecutive samples.
 *
 * With `samples_per_output <= 1` every sample is passed through unchanged.
 */
class ImuThrottler
{
public:
  explicit ImuThrottler(unsigned int samples_per_output) : samples_per_output_(samples_per_output)
  {
    samples_.reserve(samples_per_output_);
  }

  /** Adds a sample. Returns true, and fills `output`, once a full group has been collected. */
  bool add(const k4a_imu_sample_t& sample, k4a_imu_sample_t& output)
  {
    if (samples_per_output_ <= 1)
    {
      output = sample;
      return true;
    }

    samples_.push_back(sample);
    if (samples_.size() < samples_per_output_)
    {
      return false;
    }

    // Use the double-precision accumulator to avoid overflow while summing
    k4a_imu_accumulator_t mean;
    for (const auto& imu_sample : samples_)
    {
      mean += imu_sample;
    }
    mean /= static_cast<float>(samples_.size());
    mean.to_float(output);

    // Use the timestamps of the most recent sample
    output.acc_timestamp_usec = samples_.back().acc_timestamp_usec;
    output.gyro_timestamp_usec = samples_.back().gyro_timestamp_usec;

    samples_.clear();
    return true;
  }

private:
  unsigned int samples_per_output_;
  std::vector<k4a_imu_sample_t> samples_;
};

#endif  // K4A_ROS_TYPES_H