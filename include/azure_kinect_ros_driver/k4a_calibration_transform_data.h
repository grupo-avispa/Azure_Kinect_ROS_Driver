// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_CALIBRATION_TRANSFORM_DATA_H
#define K4A_CALIBRATION_TRANSFORM_DATA_H

// System headers
//
#include <vector>

// Library headers
//
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/msg/camera_info.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"

class K4ACalibrationTransformData
{
public:
  // The node provides the logger, the clock and the transform broadcaster. It must outlive this object.
  explicit K4ACalibrationTransformData(rclcpp_lifecycle::LifecycleNode* node);
  void initialize(const k4a::calibration& calibration, const K4AROSDeviceParams& params);
  int getDepthWidth();
  int getDepthHeight();
  int getColorWidth();
  int getColorHeight();
  void getDepthCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);
  void getRgbCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);
  void print();

  // Whether the SDK transformation was created, which the streams that change the geometry of an
  // image or build a point cloud need
  bool hasTransformation() const;

  k4a::calibration k4a_calibration_;
  k4a::transformation k4a_transformation_;

  k4a::image point_cloud_image_;
  k4a::image transformed_rgb_image_;
  k4a::image transformed_depth_image_;

  // The transformation is only created if some stream needs it, because it needs a graphical
  // context for the depth engine
  bool transformation_created_ = false;

  std::string tf_prefix_ = "";
  std::string camera_base_frame_ = "camera_base";
  std::string rgb_camera_frame_ = "rgb_camera_link";
  std::string depth_camera_frame_ = "depth_camera_link";
  std::string imu_frame_ = "imu_link";

private:
  void printCameraCalibration(k4a_calibration_camera_t& calibration);
  void printExtrinsics(k4a_calibration_extrinsics_t& extrinsics);

  void publishRgbToDepthTf();
  void publishImuToDepthTf();
  void publishDepthToBaseTf();

  // Publishes the static transform from the depth camera frame to the frame of another sensor
  void publishDepthToSensorTf(k4a_calibration_type_t sensor, const std::string& sensor_frame);

  tf2::Quaternion getDepthToBaseRotationCorrection();
  tf2::Vector3 getDepthToBaseTranslationCorrection();

  rclcpp_lifecycle::LifecycleNode* node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

#endif
