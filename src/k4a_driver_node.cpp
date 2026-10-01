// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_driver_node.h"

// System headers
//
#include <cfloat>
#include <chrono>
#include <cmath>

// Library headers
//
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace azure_kinect_ros_driver
{
using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;

namespace
{
/** @brief Consecutive captures that may fail to render before the node gives up. */
constexpr int kMaxConsecutiveFrameFailures = 30;

/** @brief Pause before a thread that threw is started again. */
constexpr std::chrono::milliseconds kThreadRestartDelay = 500ms;

/** @brief How long the IMU thread waits for a sample before it checks whether to stop. */
constexpr std::chrono::milliseconds kImuWaitTime = 10ms;
}  // namespace

K4ADriverNode::K4ADriverNode(const rclcpp::NodeOptions& options)
  : K4ADriverNode(options, SourceFactory(&makeCaptureSource))
{
}

K4ADriverNode::K4ADriverNode(const rclcpp::NodeOptions& options, SourceFactory source_factory)
  : rclcpp_lifecycle::LifecycleNode("k4a_ros_device_node", options)
  , params_(this->get_logger())
  , source_factory_(std::move(source_factory))
  , calibration_data_(std::make_unique<K4ACalibrationTransformData>(this))
{
  declareParameters();
  readParameters();

  stop_request_timer_ = this->create_wall_timer(100ms, [this]() { checkStopRequest(); });

  if (params_.autostart)
  {
    autostart_timer_ = this->create_wall_timer(0ms,
                                               [this]()
                                               {
                                                 autostart_timer_->cancel();
                                                 autostart();
                                               });
  }
}

K4ADriverNode::~K4ADriverNode()
{
  stopThreads();
  releaseSource();
}

bool K4ADriverNode::failed() const
{
  return failed_;
}

void K4ADriverNode::declareParameters()
{
  // Declare the parameters from the single list that also defines their defaults and help
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)    \
  {                                                                                     \
    rcl_interfaces::msg::ParameterDescriptor descriptor;                                \
    descriptor.description = param_help_string;                                         \
    this->declare_parameter(#param_variable, rclcpp::ParameterValue(param_default_val), \
                            descriptor);                                                \
  }
  ROS_PARAM_LIST
#undef LIST_ENTRY
}

void K4ADriverNode::readParameters(){
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  this->get_parameter_or(#param_variable, params_.param_variable, param_default_val);
  ROS_PARAM_LIST
#undef LIST_ENTRY
}

K4ADriverNode::CallbackReturn K4ADriverNode::on_configure(const rclcpp_lifecycle::State&)
{
  readParameters();
  failed_ = false;
  stop_request_ = StopRequest::kNone;

  RCLCPP_INFO(this->get_logger(), "K4A Parameters:");
  params_.Print();

  if (params_.ValidateImuRate() != K4A_RESULT_SUCCEEDED)
  {
    fail("Invalid IMU rate. Not starting the cameras!");
    return CallbackReturn::FAILURE;
  }

  if (!conversions::parseDepthUnit(params_.depth_unit, depth_unit_))
  {
    fail("Invalid depth unit: " + params_.depth_unit);
    return CallbackReturn::FAILURE;
  }

  try
  {
    source_ = source_factory_(params_, this->get_logger());
    calibration_data_->initialize(source_->calibration(), params_);

#if defined(K4A_BODY_TRACKING)
    // When calibration is initialized the body tracker can be created with the device calibration
    if (params_.body_tracking_enabled)
    {
      k4abt_tracker_ = k4abt::tracker::create(calibration_data_->k4a_calibration_);
      k4abt_tracker_.set_temporal_smoothing(params_.body_tracking_smoothing_factor);
    }
#endif
  }
  catch (const std::exception& error)
  {
    releaseSource();
    fail(std::string("Cannot continue: ") + error.what());
    return CallbackReturn::FAILURE;
  }

  depth_frame_ = calibration_data_->tf_prefix_ + calibration_data_->depth_camera_frame_;
  rgb_frame_ = calibration_data_->tf_prefix_ + calibration_data_->rgb_camera_frame_;
  imu_frame_ = calibration_data_->tf_prefix_ + calibration_data_->imu_frame_;
  createCameraInfos();
  createPublishers();
  clock_.reset();

  return CallbackReturn::SUCCESS;
}

K4ADriverNode::CallbackReturn K4ADriverNode::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "STARTING " << source_->description());
  try
  {
    source_->start();
  }
  catch (const k4a::error& error)
  {
    fail(std::string("Failed to start the cameras: ") + error.what());
    return CallbackReturn::FAILURE;
  }

  startThreads();

  // The base class activates the publishers, which only publish while the node is active
  return rclcpp_lifecycle::LifecycleNode::on_activate(state);
}

K4ADriverNode::CallbackReturn K4ADriverNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  stopThreads();
  if (source_)
  {
    RCLCPP_INFO(this->get_logger(), "Stopping the source");
    source_->stop();
  }
  // The base class deactivates the publishers
  return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

K4ADriverNode::CallbackReturn K4ADriverNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  releaseSource();
  destroyPublishers();
  return CallbackReturn::SUCCESS;
}

K4ADriverNode::CallbackReturn K4ADriverNode::on_shutdown(const rclcpp_lifecycle::State&)
{
  stopThreads();
  releaseSource();
  destroyPublishers();
  return CallbackReturn::SUCCESS;
}

K4ADriverNode::CallbackReturn K4ADriverNode::on_error(const rclcpp_lifecycle::State&)
{
  stopThreads();
  releaseSource();
  destroyPublishers();
  return CallbackReturn::SUCCESS;
}

void K4ADriverNode::fail(const std::string& reason)
{
  RCLCPP_ERROR_STREAM(this->get_logger(), reason);
  failed_ = true;
  if (params_.shutdown_on_stop)
  {
    rclcpp::shutdown();
  }
}

void K4ADriverNode::autostart()
{
  if (this->configure().id() != State::PRIMARY_STATE_INACTIVE ||
      this->activate().id() != State::PRIMARY_STATE_ACTIVE)
  {
    // The callbacks have already logged the reason and shut the process down if it was asked to
    failed_ = true;
  }
}

void K4ADriverNode::checkStopRequest()
{
  const StopRequest request = stop_request_.exchange(StopRequest::kNone);
  if (request == StopRequest::kNone)
  {
    return;
  }

  // The thread has already logged why. Stop streaming and give the camera back
  failed_ = failed_ || request == StopRequest::kFatalError;
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
  {
    this->deactivate();
  }
  if (this->get_current_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    this->cleanup();
  }

  if (params_.shutdown_on_stop)
  {
    rclcpp::shutdown();
  }
}

void K4ADriverNode::startThreads()
{
  running_ = true;
  capture_thread_ =
    std::thread(&K4ADriverNode::runGuarded, this, "camera", &K4ADriverNode::captureThread);
  imu_thread_ = std::thread(&K4ADriverNode::runGuarded, this, "IMU", &K4ADriverNode::imuThread);
#if defined(K4A_BODY_TRACKING)
  body_thread_ = std::thread(&K4ADriverNode::runGuarded, this, "body", &K4ADriverNode::bodyThread);
#endif
}

void K4ADriverNode::stopThreads()
{
  running_ = false;

  // The threads only exist if the node was activated
#if defined(K4A_BODY_TRACKING)
  if (body_thread_.joinable())
  {
    body_thread_.join();
  }
#endif
  if (capture_thread_.joinable())
  {
    RCLCPP_INFO(this->get_logger(), "Joining camera publisher thread");
    capture_thread_.join();
  }
  if (imu_thread_.joinable())
  {
    RCLCPP_INFO(this->get_logger(), "Joining IMU publisher thread");
    imu_thread_.join();
  }
}

void K4ADriverNode::releaseSource()
{
  if (source_)
  {
    source_->stop();
    source_.reset();
  }

#if defined(K4A_BODY_TRACKING)
  if (k4abt_tracker_)
  {
    k4abt_tracker_.shutdown();
  }
#endif
}

void K4ADriverNode::runGuarded(const char* name, void (K4ADriverNode::*thread_body)())
{
  while (running_ && rclcpp::ok())
  {
    try
    {
      (this->*thread_body)();
      return;
    }
    catch (const std::exception& error)
    {
      // Exceptions are expected while shutting down
      if (!running_ || !rclcpp::ok())
      {
        return;
      }
      RCLCPP_ERROR(this->get_logger(), "Exception in the %s thread, restarting it: %s", name,
                   error.what());
      std::this_thread::sleep_for(kThreadRestartDelay);
    }
  }
}

template <typename MessageT>
typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr K4ADriverNode::createPublisher(
  const std::string& topic, size_t depth)
{
  // Let users change the QoS of every topic with the `qos_overrides` parameters
  rclcpp::PublisherOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  return this->create_publisher<MessageT>(topic, rclcpp::QoS(depth), options);
}

void K4ADriverNode::createPublishers()
{
  using sensor_msgs::msg::CameraInfo;
  using sensor_msgs::msg::CompressedImage;
  using sensor_msgs::msg::Image;

  // Only the streams the configuration can produce are advertised, so that e.g.
  // `rgb_to_depth/*` does not show up in the graph when the color camera is disabled.
  if (params_.color_enabled)
  {
    if (params_.color_format == "jpeg")
    {
      // JPEG images are directly published as `rgb/image_raw/compressed` so that others can
      // subscribe to `rgb/image_raw` with compressed_image_transport.
      rgb_jpeg_publisher_ = createPublisher<CompressedImage>("rgb/image_raw/compressed", 1);
    }
    else if (params_.color_format == "bgra")
    {
      rgb_publisher_ = createPublisher<Image>("rgb/image_raw", 1);
    }
    rgb_info_publisher_ = createPublisher<CameraInfo>("rgb/camera_info", 1);
  }

  if (params_.depth_enabled)
  {
    depth_publisher_ = createPublisher<Image>("depth/image_raw", 1);
    depth_info_publisher_ = createPublisher<CameraInfo>("depth/camera_info", 1);

    ir_publisher_ = createPublisher<Image>("ir/image_raw", 1);
    ir_info_publisher_ = createPublisher<CameraInfo>("ir/camera_info", 1);

    if (params_.color_enabled)
    {
      depth_to_rgb_publisher_ = createPublisher<Image>("depth_to_rgb/image_raw", 1);
      depth_to_rgb_info_publisher_ = createPublisher<CameraInfo>("depth_to_rgb/camera_info", 1);

      if (params_.color_format == "bgra")
      {
        rgb_to_depth_publisher_ = createPublisher<Image>("rgb_to_depth/image_raw", 1);
        rgb_to_depth_info_publisher_ = createPublisher<CameraInfo>("rgb_to_depth/camera_info", 1);
      }
    }
  }

  imu_publisher_ = createPublisher<sensor_msgs::msg::Imu>("imu", 200);

  if (params_.point_cloud || params_.rgb_point_cloud)
  {
    point_cloud_publisher_ = createPublisher<sensor_msgs::msg::PointCloud2>("points2", 1);
  }

#if defined(K4A_BODY_TRACKING)
  if (params_.body_tracking_enabled)
  {
    body_marker_publisher_ =
      createPublisher<visualization_msgs::msg::MarkerArray>("body_tracking_data", 1);
    body_index_map_publisher_ = createPublisher<Image>("body_index_map/image_raw", 1);
  }
#endif
}

void K4ADriverNode::destroyPublishers()
{
  rgb_publisher_.reset();
  rgb_jpeg_publisher_.reset();
  rgb_info_publisher_.reset();
  depth_publisher_.reset();
  depth_info_publisher_.reset();
  ir_publisher_.reset();
  ir_info_publisher_.reset();
  depth_to_rgb_publisher_.reset();
  depth_to_rgb_info_publisher_.reset();
  rgb_to_depth_publisher_.reset();
  rgb_to_depth_info_publisher_.reset();
  imu_publisher_.reset();
  point_cloud_publisher_.reset();
#if defined(K4A_BODY_TRACKING)
  body_marker_publisher_.reset();
  body_index_map_publisher_.reset();
#endif
}

void K4ADriverNode::createCameraInfos()
{
  calibration_data_->getRgbCameraInfo(rgb_info_);
  calibration_data_->getDepthCameraInfo(depth_info_);
  calibration_data_->getDepthCameraInfo(ir_info_);
  // The depth image in the color camera is distorted like the color image, and the other way round
  calibration_data_->getRgbCameraInfo(depth_to_rgb_info_);
  calibration_data_->getDepthCameraInfo(rgb_to_depth_info_);
}

rclcpp::Time K4ADriverNode::toRosTime(std::chrono::microseconds device_timestamp)
{
  // This gives INCORRECT timestamps until the first image
  if (!clock_.synchronized())
  {
    const std::chrono::nanoseconds offset = clock_.initializeFromWallClock(device_timestamp);
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Initializing the device to realtime offset based on wall clock: "
                         << offset.count() << " ns");
  }

  return rclcpp::Time(clock_.toRealtime(device_timestamp).count(), RCL_ROS_TIME);
}

void K4ADriverNode::updateClock(const k4a::capture& capture)
{
  // Use the system timestamp (the arrival at the USB bus) and the device timestamp (the hardware
  // clock at exposure start) of the first image that the cameras give
  k4a::image image = params_.depth_enabled ? capture.get_ir_image() : capture.get_color_image();
  if (!image)
  {
    return;
  }

  if (clock_.update(image.get_device_timestamp(), image.get_system_timestamp()) ==
      ClockSynchronizer::UpdateResult::kSnapped)
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Initializing or re-initializing the device to realtime offset: "
                         << clock_.toRealtime(std::chrono::microseconds(0)).count() << " ns");
  }
}

void K4ADriverNode::captureThread()
{
  const std::chrono::milliseconds first_frame_wait_time = 4000ms;
  const std::chrono::milliseconds regular_frame_wait_time(1000 * 5 / params_.fps);
  std::chrono::milliseconds wait_time = first_frame_wait_time;

  int consecutive_failures = 0;
  std::chrono::microseconds last_timestamp{ 0 };
  k4a::capture capture;

  while (running_ && rclcpp::ok())
  {
    CaptureSource::Status status;
    try
    {
      status = source_->nextCapture(capture, wait_time);
    }
    catch (const k4a::error& error)
    {
      // A hardware-level capture failure (e.g. a corrupted USB frame breaking the sensor's
      // internal MJPEG decode) surfaces as an exception rather than as a timeout; treat it as
      // recoverable and retry instead of letting the node crash.
      RCLCPP_ERROR(this->get_logger(), "Failed to get capture, retrying: %s", error.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / params_.fps));
      continue;
    }

    if (status == CaptureSource::Status::kTimeout)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to poll cameras: node cannot continue.");
      stop_request_ = StopRequest::kFatalError;
      return;
    }
    if (status == CaptureSource::Status::kEndOfStream)
    {
      RCLCPP_INFO(this->get_logger(), "Recording reached end of file. node cannot continue.");
      stop_request_ = StopRequest::kEndOfStream;
      return;
    }

    const std::chrono::microseconds timestamp = captureTimestamp(capture);
    if (status == CaptureSource::Status::kRestarted)
    {
      // The device timestamps restart from the beginning of the recording; keep the ROS time
      // moving forward
      clock_.continueAfterRestart(last_timestamp, timestamp,
                                  std::chrono::nanoseconds(1000000000LL / params_.fps));
    }
    last_timestamp = timestamp;

    if (source_->providesSystemTimestamps())
    {
      updateClock(capture);
      wait_time = regular_frame_wait_time;
    }

    // A frame that cannot be rendered (e.g. corrupted by USB) is only skipped; the node gives up
    // only if the failures do not stop.
    if (publishCapture(capture))
    {
      consecutive_failures = 0;
    }
    else if (++consecutive_failures >= kMaxConsecutiveFrameFailures)
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to render %d captures in a row: node cannot continue.",
                   consecutive_failures);
      stop_request_ = StopRequest::kFatalError;
      return;
    }
  }
}

void K4ADriverNode::imuThread()
{
  k4a_imu_sample_t sample;
  k4a_imu_sample_t output;

  // For IMU throttling
  ImuThrottler throttler(IMU_MAX_RATE / params_.imu_rate_target);

  while (running_ && rclcpp::ok())
  {
    // IMU messages are delivered in batches at 300 Hz. Block until the first one of a batch
    // arrives (with a timeout so that the thread can notice it has to stop), then drain the
    // rest of the queue without waiting.
    CaptureSource::Status status = source_->nextImuSample(sample, kImuWaitTime);
    while (status == CaptureSource::Status::kOk)
    {
      if (throttler.add(sample, output))
      {
        publishImuSample(output);
      }
      status = source_->nextImuSample(sample, 0ms);
    }
  }
}

void K4ADriverNode::publishImuSample(const k4a_imu_sample_t& sample)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.frame_id = imu_frame_;
  msg->header.stamp = toRosTime(std::chrono::microseconds(sample.acc_timestamp_usec));

  // The correct convention in ROS is to publish the raw sensor data, in the sensor coordinate
  // frame. Do that here.
  msg->angular_velocity.x = sample.gyro_sample.xyz.x;
  msg->angular_velocity.y = sample.gyro_sample.xyz.y;
  msg->angular_velocity.z = sample.gyro_sample.xyz.z;

  msg->linear_acceleration.x = sample.acc_sample.xyz.x;
  msg->linear_acceleration.y = sample.acc_sample.xyz.y;
  msg->linear_acceleration.z = sample.acc_sample.xyz.z;

  // Disable the orientation component of the IMU message since it's invalid
  msg->orientation_covariance[0] = -1.0;

  // Samples whose angular velocity is exactly zero on every axis are not published. A real
  // reading always carries sensor noise (no such sample appeared in 13500 samples measured at
  // rest), so this only guards against empty samples.
  if (std::abs(msg->angular_velocity.x) > DBL_EPSILON ||
      std::abs(msg->angular_velocity.y) > DBL_EPSILON ||
      std::abs(msg->angular_velocity.z) > DBL_EPSILON)
  {
    if (imu_publisher_ && imu_publisher_->is_activated())
    {
      imu_publisher_->publish(std::move(msg));
    }
  }
}
}  // namespace azure_kinect_ros_driver

RCLCPP_COMPONENTS_REGISTER_NODE(azure_kinect_ros_driver::K4ADriverNode)
