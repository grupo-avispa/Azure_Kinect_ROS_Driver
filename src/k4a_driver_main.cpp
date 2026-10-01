// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <memory>

// Library headers
//
#include <rclcpp/rclcpp.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_driver_node.h"

/**
 * @brief Runs the driver as a standalone process.
 *
 * The node configures and activates itself, and the process ends when it stops by itself (the
 * recording ended or an error happened).
 *
 * @param argc The number of arguments.
 * @param argv The arguments.
 * @return Zero, or -1 if the driver failed.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.append_parameter_override("shutdown_on_stop", true);
  auto node = std::make_shared<azure_kinect_ros_driver::K4ADriverNode>(options);

  rclcpp::spin(node->get_node_base_interface());

  const bool failed = node->failed();
  RCLCPP_INFO(node->get_logger(), "ROS Exit Started");
  node.reset();
  rclcpp::shutdown();
  return failed ? -1 : 0;
}
