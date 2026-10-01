// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <sstream>

// Library headers
//
#include "rclcpp/rclcpp.hpp"
#include <k4a/k4a.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Setup the K4A device. It is the only node of the process: the parameter services and the
  // rest of the node callbacks are served by the spin below, while the publisher threads only
  // publish.
  auto device = std::make_shared<K4AROSDevice>();
  rclcpp::Logger logger = device->get_logger();

  k4a_result_t result = device->startCameras();

  if (result != K4A_RESULT_SUCCEEDED)
  {
    RCLCPP_ERROR_STREAM(logger, "Failed to start cameras");
    rclcpp::shutdown();
    return -1;
  }

  result = device->startImu();
  if (result != K4A_RESULT_SUCCEEDED)
  {
    RCLCPP_ERROR_STREAM(logger, "Failed to start IMU");
    rclcpp::shutdown();
    return -2;
  }

  RCLCPP_INFO(logger, "K4A Started");

  rclcpp::spin(device);

  RCLCPP_INFO(logger, "ROS Exit Started");

  device.reset();

  RCLCPP_INFO(logger, "ROS Exit");

  rclcpp::shutdown();

  return 0;
}
