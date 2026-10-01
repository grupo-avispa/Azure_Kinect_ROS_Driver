// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_DRIVER_NODE_H
#define K4A_DRIVER_NODE_H

// System headers
//
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

// Library headers
//
#include <k4a/k4a.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#if defined(K4A_BODY_TRACKING)
#include <k4abt.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

// Project headers
//
#include "azure_kinect_ros_driver/capture_source.h"
#include "azure_kinect_ros_driver/clock_synchronizer.h"
#include "azure_kinect_ros_driver/k4a_calibration_transform_data.h"
#include "azure_kinect_ros_driver/k4a_conversions.h"
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"
#include "azure_kinect_ros_driver/k4a_ros_types.h"

namespace azure_kinect_ros_driver
{
/**
 * @brief Publishes the data of an Azure Kinect, from a device or from a recording.
 *
 * It is a managed (lifecycle) node and a composable component:
 *  - `configure` reads the parameters, opens the source, publishes the static transforms and
 *    creates the publishers of the streams that the parameters enable;
 *  - `activate` starts the cameras and the threads that publish;
 *  - `deactivate` stops them, and `cleanup` closes the source.
 *
 * With the `autostart` parameter (the default) the node configures and activates itself as soon
 * as it is spinning.
 *
 * The messages are published by `unique_ptr`, so that when the node is composed in a container
 * with intra-process communication enabled the subscribers of that container get them without
 * any copy.
 */
class K4ADriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /** @brief Result of the lifecycle callbacks. */
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /** @brief Creates the source of the data; replaceable in tests. */
  using SourceFactory =
    std::function<std::unique_ptr<CaptureSource>(K4AROSDeviceParams&, rclcpp::Logger)>;

  /**
   * @brief Constructs the node, which reads its data from a device or a recording.
   *
   * @param options The options of the node, for example to enable intra-process communication.
   */
  explicit K4ADriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Constructs the node with a custom source of data.
   *
   * @param options The options of the node.
   * @param source_factory Called when the node is configured, to create the source.
   */
  K4ADriverNode(const rclcpp::NodeOptions& options, SourceFactory source_factory);

  ~K4ADriverNode() override;

  /**
   * @brief Whether the driver stopped because of an error.
   *
   * @return True if configuring, activating or streaming failed.
   */
  bool failed() const;

  /** @brief Reads the parameters, opens the source and creates the publishers. */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /** @brief Starts the cameras and the publishing threads. */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /** @brief Stops the publishing threads and the cameras. */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /** @brief Closes the source and destroys the publishers. */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /** @brief Stops everything before the node is destroyed. */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  /** @brief Releases the resources after a failure. */
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

private:
  /** @brief Why the streaming thread asks the node to stop. */
  enum class StopRequest
  {
    /** @brief Nothing was requested. */
    kNone,
    /** @brief The recording reached its end. */
    kEndOfStream,
    /** @brief The device stopped delivering captures, or rendering keeps failing. */
    kFatalError
  };

  /** @brief Publisher of images. */
  using ImagePublisher = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr;

  /** @brief Publisher of camera infos. */
  using InfoPublisher =
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr;

  /** @brief Declares the parameters from `ROS_PARAM_LIST`. */
  void declareParameters();

  /** @brief Reads the current value of the parameters. */
  void readParameters();

  /** @brief Creates the publishers of the streams that the parameters enable. */
  void createPublishers();

  /** @brief Destroys the publishers. */
  void destroyPublishers();

  /**
   * @brief Creates a publisher with the default QoS of the driver, which users can override.
   *
   * @tparam MessageT The message type.
   * @param topic The topic.
   * @param depth The history depth.
   * @return The publisher.
   */
  template <typename MessageT>
  typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr createPublisher(
    const std::string& topic, size_t depth);

  /** @brief Builds the camera info messages that are published with the images. */
  void createCameraInfos();

  /** @brief Starts the threads that read the source and publish. */
  void startThreads();

  /** @brief Stops the threads, if they are running. */
  void stopThreads();

  /** @brief Stops the streaming, if started, and closes the source. */
  void releaseSource();

  /** @brief Called periodically to react to a request of the streaming thread to stop. */
  void checkStopRequest();

  /** @brief Called once when the node starts spinning, if `autostart` is set. */
  void autostart();

  /**
   * @brief Records that the driver failed.
   *
   * @param reason What failed.
   */
  void fail(const std::string& reason);

  /**
   * @brief Runs the body of a thread, restarting it if it throws.
   *
   * An exception escaping a `std::thread` terminates the process, so exceptions (e.g. a
   * `k4a::error` of the SDK) are logged here and the body is started again after a pause.
   *
   * @param name The name of the thread, for the logs.
   * @param thread_body The member function to run.
   */
  void runGuarded(const char* name, void (K4ADriverNode::*thread_body)());

  /** @brief Reads the captures from the source and publishes their streams. */
  void captureThread();

  /** @brief Reads the IMU samples from the source and publishes them. */
  void imuThread();

  /**
   * @brief Publishes all the streams of a capture that have subscribers.
   *
   * @param capture The capture.
   * @return False if any of the streams could not be produced.
   */
  bool publishCapture(const k4a::capture& capture);

  /** @brief Publishes the infrared image. @param capture The capture. @return False on failure. */
  bool publishIr(const k4a::capture& capture);

  /** @brief Publishes the depth image. @param capture The capture. @return False on failure. */
  bool publishDepth(const k4a::capture& capture);

  /** @brief Publishes depth in the color camera. @param capture The capture. @return False on
   * failure. */
  bool publishDepthToRgb(const k4a::capture& capture);

  /** @brief Publishes the color image. @param capture The capture. @return False on failure. */
  bool publishColor(const k4a::capture& capture);

  /** @brief Publishes color in the depth camera. @param capture The capture. @return False on
   * failure. */
  bool publishRgbToDepth(const k4a::capture& capture);

  /** @brief Publishes the point cloud. @param capture The capture. @return False on failure. */
  bool publishPointCloud(const k4a::capture& capture);

  /**
   * @brief Builds the point cloud of a capture, with the color if the parameters ask for it.
   *
   * @param capture The capture.
   * @return The cloud, or nullptr if it could not be built.
   */
  sensor_msgs::msg::PointCloud2::UniquePtr buildPointCloud(const k4a::capture& capture);

  /**
   * @brief Publishes an image together with its camera info.
   *
   * @param image_publisher The publisher of the image.
   * @param image The image; its header is filled here.
   * @param info_publisher The publisher of the camera info.
   * @param camera_info The cached camera info; its stamp is updated here.
   * @param stamp The capture time.
   * @param frame_id The frame of the image.
   */
  void publishImageWithInfo(const ImagePublisher& image_publisher,
                            sensor_msgs::msg::Image::UniquePtr image,
                            const InfoPublisher& info_publisher,
                            sensor_msgs::msg::CameraInfo& camera_info, const rclcpp::Time& stamp,
                            const std::string& frame_id);

  /**
   * @brief Whether anyone listens to a stream, inside or outside of this process.
   *
   * @param image_publisher The publisher of the image, or nullptr.
   * @param info_publisher The publisher of the camera info, or nullptr.
   * @return True if either has subscribers.
   */
  bool hasSubscribers(const ImagePublisher& image_publisher,
                      const InfoPublisher& info_publisher) const;

  /**
   * @brief Whether a capture has the images a stream needs.
   *
   * A device always delivers them, a recording may not.
   *
   * @param needs_depth The stream needs the depth image.
   * @param needs_color The stream needs the color image.
   * @param needs_ir The stream needs the infrared image.
   * @param capture The capture.
   * @return True if the stream can be produced.
   */
  bool hasImages(bool needs_depth, bool needs_color, bool needs_ir,
                 const k4a::capture& capture) const;

  /**
   * @brief Converts a device timestamp to ROS time, starting the clock if needed.
   *
   * @param device_timestamp The timestamp of the device.
   * @return The time in the realtime clock.
   */
  rclcpp::Time toRosTime(std::chrono::microseconds device_timestamp);

  /**
   * @brief Folds the arrival time of an image into the estimate of the clock offset.
   *
   * @param capture A capture of a device.
   */
  void updateClock(const k4a::capture& capture);

  /**
   * @brief Converts an IMU sample to a message and publishes it.
   *
   * @param sample The sample.
   */
  void publishImuSample(const k4a_imu_sample_t& sample);

  /** @brief The parameters. */
  K4AROSDeviceParams params_;

  /** @brief Creates the source on configure. */
  SourceFactory source_factory_;

  /** @brief Where the captures and the IMU samples come from; only while configured. */
  std::unique_ptr<CaptureSource> source_;

  /** @brief Calibration, static transforms and the transformation of the images. */
  std::unique_ptr<K4ACalibrationTransformData> calibration_data_;

  /** @brief Maps the device timestamps to the realtime clock. */
  ClockSynchronizer clock_;

  /** @brief Unit of the depth images. */
  conversions::DepthUnit depth_unit_ = conversions::DepthUnit::kMillimeters;

  /** @brief Frame of the depth camera, with the prefix. */
  std::string depth_frame_;

  /** @brief Frame of the color camera, with the prefix. */
  std::string rgb_frame_;

  /** @brief Frame of the IMU, with the prefix. */
  std::string imu_frame_;

  /** @brief Camera info of the raw color images. */
  sensor_msgs::msg::CameraInfo rgb_info_;

  /** @brief Camera info of the raw depth images. */
  sensor_msgs::msg::CameraInfo depth_info_;

  /** @brief Camera info of the infrared images. */
  sensor_msgs::msg::CameraInfo ir_info_;

  /** @brief Camera info of the depth images in the color camera. */
  sensor_msgs::msg::CameraInfo depth_to_rgb_info_;

  /** @brief Camera info of the color images in the depth camera. */
  sensor_msgs::msg::CameraInfo rgb_to_depth_info_;

  /** @brief Publisher of the raw color images. */
  ImagePublisher rgb_publisher_;

  /** @brief Publisher of the JPEG color images. */
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr
    rgb_jpeg_publisher_;

  /** @brief Publisher of the camera info of the color images. */
  InfoPublisher rgb_info_publisher_;

  /** @brief Publisher of the raw depth images. */
  ImagePublisher depth_publisher_;

  /** @brief Publisher of the camera info of the depth images. */
  InfoPublisher depth_info_publisher_;

  /** @brief Publisher of the infrared images. */
  ImagePublisher ir_publisher_;

  /** @brief Publisher of the camera info of the infrared images. */
  InfoPublisher ir_info_publisher_;

  /** @brief Publisher of the depth images in the color camera. */
  ImagePublisher depth_to_rgb_publisher_;

  /** @brief Publisher of the camera info of the depth images in the color camera. */
  InfoPublisher depth_to_rgb_info_publisher_;

  /** @brief Publisher of the color images in the depth camera. */
  ImagePublisher rgb_to_depth_publisher_;

  /** @brief Publisher of the camera info of the color images in the depth camera. */
  InfoPublisher rgb_to_depth_info_publisher_;

  /** @brief Publisher of the IMU samples. */
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  /** @brief Publisher of the point cloud. */
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;

#if defined(K4A_BODY_TRACKING)
  /** @brief Publisher of the joints of the bodies. */
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    body_marker_publisher_;

  /** @brief Publisher of the body index map. */
  ImagePublisher body_index_map_publisher_;

  /** @brief The body tracker. */
  k4abt::tracker k4abt_tracker_{ nullptr };

  /** @brief Captures waiting in the tracker. */
  std::atomic_int16_t k4abt_tracker_queue_size_{ 0 };

  /** @brief Publishes the results of the tracker. */
  std::thread body_thread_;

  /** @brief Publishes the results of the body tracker. */
  void bodyThread();
#endif

  /** @brief Reads the captures and publishes the streams. */
  std::thread capture_thread_;

  /** @brief Reads the IMU samples and publishes them. */
  std::thread imu_thread_;

  /** @brief Whether the threads have to keep running. */
  std::atomic_bool running_{ false };

  /** @brief What the streaming thread asked for, as a `StopRequest`. */
  std::atomic<StopRequest> stop_request_{ StopRequest::kNone };

  /** @brief Whether the driver stopped because of an error. */
  std::atomic_bool failed_{ false };

  /** @brief Calls `checkStopRequest()` periodically. */
  rclcpp::TimerBase::SharedPtr stop_request_timer_;

  /** @brief Configures and activates the node once, if `autostart` is set. */
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};
}  // namespace azure_kinect_ros_driver

#endif  // K4A_DRIVER_NODE_H
