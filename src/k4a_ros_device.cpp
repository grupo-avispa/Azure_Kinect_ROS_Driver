// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

// System headers
//
#include <cfloat>
#include <thread>
#include <iomanip>
#include <limits>
#include <type_traits>
#include <unordered_map>

// Library headers
//
#include <angles/angles.h>
#include <k4a/k4a.h>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <k4a/k4a.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_conversions.h"
#include "azure_kinect_ros_driver/k4a_ros_types.h"

using namespace rclcpp;
using namespace sensor_msgs::msg;
using namespace image_transport;
using namespace std;

#if defined(K4A_BODY_TRACKING)
using namespace visualization_msgs::msg;
#endif

namespace
{
// Consecutive captures that may fail to render before the node gives up
constexpr int kMaxConsecutiveFrameFailures = 30;

// Number of subscribers of a publisher that may not have been created (disabled stream)
template<typename PublisherT>
size_t subscriberCount(const std::shared_ptr<PublisherT> & publisher)
{
  return publisher ? publisher->get_subscription_count() : 0;
}

// An image_transport publisher sums the subscribers of every transport (raw, compressed...)
size_t subscriberCount(const image_transport::Publisher & publisher)
{
  return publisher.getNumSubscribers();
}
}  // namespace

K4AROSDevice::K4AROSDevice()
: Node("k4a_ros_device_node"),
  params_(this->get_logger()),
// clang-format off
#if defined(K4A_BODY_TRACKING)
  calibration_data_(this),
  k4abt_tracker_(nullptr),
  k4abt_tracker_queue_size_(0)
#else
  calibration_data_(this)
#endif
    // clang-format on
{
  // Image topics are advertised through the free-function API, which does not take ownership of
  // this node (building a shared_ptr from `this` here would create a second control block).
  const rmw_qos_profile_t image_qos = rclcpp::QoS(1).get_rmw_qos_profile();

  // Declare depth topics
  static const std::string depth_raw_topic = "depth/image_raw";
  static const std::string depth_rect_topic = "depth_to_rgb/image_raw";
  static const std::string compressed_format = "/compressed/format";
  static const std::string compressed_png_level = "/compressed/png_level";

  // Declare node parameters from the single list that also defines their defaults and help
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)     \
  {                                                                                      \
    rcl_interfaces::msg::ParameterDescriptor descriptor;                                 \
    descriptor.description = param_help_string;                                          \
    this->declare_parameter(#param_variable, rclcpp::ParameterValue(param_default_val),  \
      descriptor);                                                                       \
  }
  ROS_PARAM_LIST
#undef LIST_ENTRY

  // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  this->get_parameter_or(#param_variable, params_.param_variable, param_default_val);
  ROS_PARAM_LIST
#undef LIST_ENTRY

  // Print all parameters
  RCLCPP_INFO(this->get_logger(), "K4A Parameters:");
  params_.Print();

  try {
    source_ = azure_kinect_ros_driver::makeCaptureSource(params_, this->get_logger());
  } catch (const std::exception & error) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot continue: " << error.what());
    return;
  }

  // Register our topics. Only the streams the configuration can produce are advertised, so that
  // e.g. `rgb_to_depth/*` does not show up in the graph when the color camera is disabled.
  if (params_.color_enabled) {
    if (params_.color_format == "jpeg") {
      // JPEG images are directly published on 'rgb/image_raw/compressed' so that
      // others can subscribe to 'rgb/image_raw' with compressed_image_transport.
      // This technique is described in:
      // http://wiki.ros.org/compressed_image_transport#Publishing_compressed_images_directly
      rgb_jpeg_publisher_ = this->create_publisher<CompressedImage>("rgb/image_raw/compressed", 1);
    } else if (params_.color_format == "bgra") {
      rgb_raw_publisher_ = image_transport::create_publisher(this, "rgb/image_raw", image_qos);
    }
    rgb_raw_camerainfo_publisher_ = this->create_publisher<CameraInfo>("rgb/camera_info", 1);
  }

  if (params_.depth_enabled) {
    depth_raw_publisher_ = image_transport::create_publisher(this, depth_raw_topic, image_qos);
    depth_raw_camerainfo_publisher_ = this->create_publisher<CameraInfo>("depth/camera_info", 1);

    ir_raw_publisher_ = image_transport::create_publisher(this, "ir/image_raw", image_qos);
    ir_raw_camerainfo_publisher_ = this->create_publisher<CameraInfo>("ir/camera_info", 1);

    if (params_.color_enabled) {
      depth_rect_publisher_ = image_transport::create_publisher(this, depth_rect_topic, image_qos);
      depth_rect_camerainfo_publisher_ =
        this->create_publisher<CameraInfo>("depth_to_rgb/camera_info", 1);

      if (params_.color_format == "bgra") {
        rgb_rect_publisher_ =
          image_transport::create_publisher(this, "rgb_to_depth/image_raw", image_qos);
        rgb_rect_camerainfo_publisher_ =
          this->create_publisher<CameraInfo>("rgb_to_depth/camera_info", 1);
      }
    }
  }

  imu_orientation_publisher_ = this->create_publisher<Imu>("imu", 200);

  if (params_.point_cloud || params_.rgb_point_cloud) {
    pointcloud_publisher_ = this->create_publisher<PointCloud2>("points2", 1);
  }

#if defined(K4A_BODY_TRACKING)
  if (params_.body_tracking_enabled) {
    body_marker_publisher_ = this->create_publisher<MarkerArray>("body_tracking_data", 1);

    body_index_map_publisher_ = image_transport::create_publisher(this, "body_index_map/image_raw");
  }
#endif
}

K4AROSDevice::~K4AROSDevice()
{
  // Start tearing down the publisher threads
  running_ = false;

  // The threads only exist if the cameras and the IMU were started successfully
#if defined(K4A_BODY_TRACKING)
  if (body_publisher_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Joining body publisher thread");
    body_publisher_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Body publisher thread joined");
  }
#endif

  if (frame_publisher_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Joining camera publisher thread");
    frame_publisher_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Camera publisher thread joined");
  }

  if (imu_publisher_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Joining IMU publisher thread");
    imu_publisher_thread_.join();
    RCLCPP_INFO(this->get_logger(), "IMU publisher thread joined");
  }

  stopCameras();
  stopImu();

#if defined(K4A_BODY_TRACKING)
  if (k4abt_tracker_) {
    k4abt_tracker_.shutdown();
  }
#endif
}

k4a_result_t K4AROSDevice::startCameras()
{
  if (params_.ValidateImuRate() != K4A_RESULT_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Invalid IMU rate. Not starting camera!");
    return K4A_RESULT_FAILED;
  }

  if (!source_) {
    RCLCPP_ERROR(this->get_logger(),
      "Neither a K4A device nor a recording is open. Not starting camera!");
    return K4A_RESULT_FAILED;
  }

  // Now that the source is open, initialize the class which will take care of the calibration
  calibration_data_.initialize(source_->calibration(), params_);

#if defined(K4A_BODY_TRACKING)
  // When calibration is initialized the body tracker can be created with the device calibration
  if (params_.body_tracking_enabled) {
    k4abt_tracker_ = k4abt::tracker::create(calibration_data_.k4a_calibration_);
    k4abt_tracker_.set_temporal_smoothing(params_.body_tracking_smoothing_factor);
  }
#endif

  RCLCPP_INFO_STREAM(this->get_logger(), "STARTING " << source_->description());
  try {
    source_->start();
  } catch (const k4a::error & error) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to start the cameras: " << error.what());
    return K4A_RESULT_FAILED;
  }

  // Prevent the worker thread from exiting immediately
  running_ = true;

  // Start the thread that will poll the cameras and publish frames
  frame_publisher_thread_ =
    thread(&K4AROSDevice::runGuarded, this, "camera", &K4AROSDevice::framePublisherThread);
#if defined(K4A_BODY_TRACKING)
  body_publisher_thread_ =
    thread(&K4AROSDevice::runGuarded, this, "body", &K4AROSDevice::bodyPublisherThread);
#endif

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::startImu()
{
  // The source has already started the IMU together with the cameras; start the publisher thread
  imu_publisher_thread_ =
    thread(&K4AROSDevice::runGuarded, this, "IMU", &K4AROSDevice::imuPublisherThread);

  return K4A_RESULT_SUCCEEDED;
}

void K4AROSDevice::stopCameras()
{
  if (source_) {
    RCLCPP_INFO(this->get_logger(), "Stopping the source");
    source_->stop();
    RCLCPP_INFO(this->get_logger(), "Source stopped");
  }
}

void K4AROSDevice::stopImu()
{
  // The IMU is stopped together with the cameras
}

k4a_result_t K4AROSDevice::getDepthFrame(
  const k4a::capture & capture, std::shared_ptr<sensor_msgs::msg::Image> & depth_image,
  bool rectified)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cannot render depth frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if (rectified) {
    calibration_data_.k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
                                                                      &calibration_data_.
      transformed_depth_image_);

    return renderDepthToROS(depth_image, calibration_data_.transformed_depth_image_);
  }

  return renderDepthToROS(depth_image, k4a_depth_frame);
}

k4a_result_t K4AROSDevice::renderDepthToROS(
  std::shared_ptr<sensor_msgs::msg::Image> & depth_image,
  k4a::image & k4a_depth_frame)
{
  azure_kinect_ros_driver::conversions::DepthUnit unit;
  if (!azure_kinect_ros_driver::conversions::parseDepthUnit(params_.depth_unit, unit)) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Invalid depth unit: " << params_.depth_unit);
    return K4A_RESULT_FAILED;
  }

  depth_image = azure_kinect_ros_driver::conversions::depthToImage(k4a_depth_frame, unit);
  return depth_image ? K4A_RESULT_SUCCEEDED : K4A_RESULT_FAILED;
}

k4a_result_t K4AROSDevice::getIrFrame(
  const k4a::capture & capture,
  std::shared_ptr<sensor_msgs::msg::Image> & ir_image)
{
  k4a::image k4a_ir_frame = capture.get_ir_image();

  if (!k4a_ir_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cannot render IR frame: no frame");
    return K4A_RESULT_FAILED;
  }

  return renderIrToROS(ir_image, k4a_ir_frame);
}

k4a_result_t K4AROSDevice::renderIrToROS(
  std::shared_ptr<sensor_msgs::msg::Image> & ir_image,
  k4a::image & k4a_ir_frame)
{
  // If using the illuminators, a scaling factor of 1 is appropriate. If using PASSIVE_IR, then a
  // value of 10 is more appropriate; k4aviewer does a similar conversion.
  ir_image = azure_kinect_ros_driver::conversions::irToImage(k4a_ir_frame, params_.rescale_ir_to_mono8,
    params_.ir_mono8_scaling_factor);
  return ir_image ? K4A_RESULT_SUCCEEDED : K4A_RESULT_FAILED;
}

k4a_result_t K4AROSDevice::getJpegRgbFrame(
  const k4a::capture & capture,
  std::shared_ptr<sensor_msgs::msg::CompressedImage> & jpeg_image)
{
  k4a::image k4a_jpeg_frame = capture.get_color_image();

  if (!k4a_jpeg_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cannot render Jpeg frame: no frame");
    return K4A_RESULT_FAILED;
  }

  jpeg_image = azure_kinect_ros_driver::conversions::jpegToCompressed(k4a_jpeg_frame);
  return jpeg_image ? K4A_RESULT_SUCCEEDED : K4A_RESULT_FAILED;
}

k4a_result_t K4AROSDevice::getRgbFrame(
  const k4a::capture & capture, std::shared_ptr<sensor_msgs::msg::Image> & rgb_image,
  bool rectified)
{
  k4a::image k4a_bgra_frame = capture.get_color_image();

  if (!k4a_bgra_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cannot render BGRA frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if (rectified) {
    k4a::image k4a_depth_frame = capture.get_depth_image();

    calibration_data_.k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame,
      k4a_bgra_frame,
                                                                      &calibration_data_.
      transformed_rgb_image_);

    return renderBGRA32ToROS(rgb_image, calibration_data_.transformed_rgb_image_);
  }

  return renderBGRA32ToROS(rgb_image, k4a_bgra_frame);
}

k4a_result_t K4AROSDevice::renderBGRA32ToROS(
  std::shared_ptr<sensor_msgs::msg::Image> & rgb_image,
  k4a::image & k4a_bgra_frame)
{
  rgb_image = azure_kinect_ros_driver::conversions::bgraToImage(k4a_bgra_frame);
  if (!rgb_image) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Invalid k4a_bgra_frame returned from K4A");
    return K4A_RESULT_FAILED;
  }
  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getRgbPointCloudInDepthFrame(
  const k4a::capture & capture,
  std::shared_ptr<sensor_msgs::msg::PointCloud2> & point_cloud)
{
  const k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  const k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // Transform color image into the depth camera frame:
  calibration_data_.k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame, k4a_bgra_frame,
    &calibration_data_.transformed_rgb_image_);

  // Tranform depth image to point cloud
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame,
    K4A_CALIBRATION_TYPE_DEPTH, &calibration_data_.point_cloud_image_);

  return buildPointCloud(calibration_data_.point_cloud_image_,
    &calibration_data_.transformed_rgb_image_,
    calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_,
    k4a_depth_frame, point_cloud);
}

k4a_result_t K4AROSDevice::getRgbPointCloudInRgbFrame(
  const k4a::capture & capture,
  std::shared_ptr<sensor_msgs::msg::PointCloud2> & point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // transform depth image into color camera geometry
  calibration_data_.k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
    &calibration_data_.transformed_depth_image_);

  // Tranform depth image to point cloud (note that this is now from the perspective of the color camera)
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(
    calibration_data_.transformed_depth_image_, K4A_CALIBRATION_TYPE_COLOR,
    &calibration_data_.point_cloud_image_);

  return buildPointCloud(calibration_data_.point_cloud_image_, &k4a_bgra_frame,
    calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_, k4a_depth_frame,
    point_cloud);
}

k4a_result_t K4AROSDevice::getPointCloud(
  const k4a::capture & capture,
  std::shared_ptr<sensor_msgs::msg::PointCloud2> & point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Cannot render point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  // Tranform depth image to point cloud
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame,
    K4A_CALIBRATION_TYPE_DEPTH, &calibration_data_.point_cloud_image_);

  return buildPointCloud(calibration_data_.point_cloud_image_, nullptr,
    calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_, k4a_depth_frame,
    point_cloud);
}

k4a_result_t K4AROSDevice::buildPointCloud(
  const k4a::image & pointcloud_image, const k4a::image * color_image, const std::string & frame_id,
  const k4a::image & depth_image, std::shared_ptr<sensor_msgs::msg::PointCloud2> & point_cloud)
{
  point_cloud = azure_kinect_ros_driver::conversions::pointCloudToMsg(pointcloud_image, color_image);
  if (!point_cloud) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Color and depth image sizes do not match!");
    return K4A_RESULT_FAILED;
  }

  point_cloud->header.frame_id = frame_id;
  point_cloud->header.stamp = timestampToROS(depth_image.get_device_timestamp());
  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getImuFrame(
  const k4a_imu_sample_t & sample,
  std::shared_ptr<sensor_msgs::msg::Imu> & imu_msg)
{
  imu_msg->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.imu_frame_;
  imu_msg->header.stamp = timestampToROS(sample.acc_timestamp_usec);

  // The correct convention in ROS is to publish the raw sensor data, in the
  // sensor coordinate frame. Do that here.
  imu_msg->angular_velocity.x = sample.gyro_sample.xyz.x;
  imu_msg->angular_velocity.y = sample.gyro_sample.xyz.y;
  imu_msg->angular_velocity.z = sample.gyro_sample.xyz.z;

  imu_msg->linear_acceleration.x = sample.acc_sample.xyz.x;
  imu_msg->linear_acceleration.y = sample.acc_sample.xyz.y;
  imu_msg->linear_acceleration.z = sample.acc_sample.xyz.z;

  // Disable the orientation component of the IMU message since it's invalid
  imu_msg->orientation_covariance[0] = -1.0;

  return K4A_RESULT_SUCCEEDED;
}

#if defined(K4A_BODY_TRACKING)
k4a_result_t K4AROSDevice::getBodyMarker(
  const k4abt_body_t & body, std::shared_ptr<visualization_msgs::msg::Marker> marker_msg,
  int jointType,
  rclcpp::Time capture_time)
{
  k4a_float3_t position = body.skeleton.joints[jointType].position;
  k4a_quaternion_t orientation = body.skeleton.joints[jointType].orientation;

  marker_msg->header.frame_id = calibration_data_.tf_prefix_ +
    calibration_data_.depth_camera_frame_;
  marker_msg->header.stamp = capture_time;

  // Set the lifetime to 0.25 to prevent flickering for even 5fps configurations.
  // New markers with the same ID will replace old markers as soon as they arrive.
  marker_msg->lifetime = rclcpp::Duration::from_seconds(0.25);
  marker_msg->id = body.id * 100 + jointType;
  marker_msg->type = Marker::SPHERE;

  Color color = BODY_COLOR_PALETTE[body.id % BODY_COLOR_PALETTE.size()];

  marker_msg->color.a = color.a;
  marker_msg->color.r = color.r;
  marker_msg->color.g = color.g;
  marker_msg->color.b = color.b;

  marker_msg->scale.x = 0.05;
  marker_msg->scale.y = 0.05;
  marker_msg->scale.z = 0.05;

  marker_msg->pose.position.x = position.v[0] / 1000.0f;
  marker_msg->pose.position.y = position.v[1] / 1000.0f;
  marker_msg->pose.position.z = position.v[2] / 1000.0f;
  marker_msg->pose.orientation.w = orientation.wxyz.w;
  marker_msg->pose.orientation.x = orientation.wxyz.x;
  marker_msg->pose.orientation.y = orientation.wxyz.y;
  marker_msg->pose.orientation.z = orientation.wxyz.z;

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getBodyIndexMap(
  const k4abt::frame & body_frame,
  std::shared_ptr<sensor_msgs::msg::Image> body_index_map_image)
{
  k4a::image k4a_body_index_map = body_frame.get_body_index_map();

  if (!k4a_body_index_map) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cannot render body index map: no body index map");
    return K4A_RESULT_FAILED;
  }

  return renderBodyIndexMapToROS(body_index_map_image, k4a_body_index_map, body_frame);
}

k4a_result_t K4AROSDevice::renderBodyIndexMapToROS(
  std::shared_ptr<sensor_msgs::msg::Image> body_index_map_image,
  k4a::image & k4a_body_index_map, const k4abt::frame & body_frame)
{
  // Access the body index map as an array of uint8 pixels
  BodyIndexMapPixel * body_index_map_frame_buffer = k4a_body_index_map.get_buffer();
  auto body_index_map_pixel_count = k4a_body_index_map.get_size() / sizeof(BodyIndexMapPixel);

  // Build the ROS message
  body_index_map_image->height = k4a_body_index_map.get_height_pixels();
  body_index_map_image->width = k4a_body_index_map.get_width_pixels();
  body_index_map_image->encoding = sensor_msgs::image_encodings::MONO8;
  body_index_map_image->is_bigendian = false;
  body_index_map_image->step = k4a_body_index_map.get_width_pixels() * sizeof(BodyIndexMapPixel);

  // Enlarge the data buffer in the ROS message to hold the frame
  body_index_map_image->data.resize(body_index_map_image->height * body_index_map_image->step);

  // If the pixel doesn't belong to a detected body the pixels value will be 255 (K4ABT_BODY_INDEX_MAP_BACKGROUND).
  // If the pixel belongs to a detected body the value is calculated by body id mod 255.
  // This means that up to body id 254 the value is equals the body id.
  // Afterwards it will lose the relation to the body id and is only a information for the segmentation of the image.
  for (size_t i = 0; i < body_index_map_pixel_count; ++i) {
    BodyIndexMapPixel val = body_index_map_frame_buffer[i];
    if (val == K4ABT_BODY_INDEX_MAP_BACKGROUND) {
      body_index_map_image->data[i] = K4ABT_BODY_INDEX_MAP_BACKGROUND;
    } else {
      auto body_id = k4abt_frame_get_body_id(body_frame.handle(), val);
      body_index_map_image->data[i] = body_id % K4ABT_BODY_INDEX_MAP_BACKGROUND;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}
#endif

void K4AROSDevice::publishImageWithInfo(
  image_transport::Publisher & image_publisher, const Image::SharedPtr & image,
  rclcpp::Publisher<CameraInfo>::SharedPtr & camera_info_publisher, CameraInfo & camera_info,
  const Time & stamp, const std::string & frame_id)
{
  // The camera info message is cached, so its stamp has to follow the image
  image->header.stamp = stamp;
  image->header.frame_id = frame_id;
  camera_info.header.stamp = stamp;

  image_publisher.publish(image);
  camera_info_publisher->publish(camera_info);
}

void K4AROSDevice::runGuarded(const char * name, void (K4AROSDevice::*thread_body)())
{
  while (running_ && rclcpp::ok()) {
    try {
      (this->*thread_body)();
      return;
    } catch (const std::exception & e) {
      // Exceptions are expected while shutting down (e.g. the context becomes invalid)
      if (!running_ || !rclcpp::ok()) {
        return;
      }
      RCLCPP_ERROR(this->get_logger(), "Exception in the %s thread, restarting it: %s", name,
        e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

void K4AROSDevice::framePublisherThread()
{
  k4a_result_t result;

  CameraInfo rgb_raw_camera_info;
  CameraInfo depth_raw_camera_info;
  CameraInfo rgb_rect_camera_info;
  CameraInfo depth_rect_camera_info;
  CameraInfo ir_raw_camera_info;

  Time capture_time;

  k4a::capture capture;

  calibration_data_.getDepthCameraInfo(depth_raw_camera_info);
  calibration_data_.getRgbCameraInfo(rgb_raw_camera_info);
  calibration_data_.getDepthCameraInfo(rgb_rect_camera_info);
  calibration_data_.getRgbCameraInfo(depth_rect_camera_info);
  calibration_data_.getDepthCameraInfo(ir_raw_camera_info);

  const std::chrono::milliseconds firstFrameWaitTime = std::chrono::milliseconds(4 * 1000);
  const std::chrono::milliseconds regularFrameWaitTime = std::chrono::milliseconds(1000 * 5 /
    params_.fps);
  std::chrono::milliseconds waitTime = firstFrameWaitTime;

  int consecutive_failures = 0;
  std::chrono::microseconds last_capture_timestamp{ 0 };

  while (running_ && rclcpp::ok()) {
    azure_kinect_ros_driver::CaptureSource::Status status;
    try {
      status = source_->nextCapture(capture, waitTime);
    } catch (const k4a::error & e) {
      // A hardware-level capture failure (e.g. a corrupted USB frame breaking the
      // sensor's internal MJPEG decode) surfaces here as an exception rather than
      // through the plain timeout path below; treat it as recoverable and retry on
      // the next iteration instead of letting the node crash.
      RCLCPP_ERROR(this->get_logger(), "Failed to get capture, retrying: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / params_.fps));
      continue;
    }

    if (status == azure_kinect_ros_driver::CaptureSource::Status::kTimeout) {
      RCLCPP_FATAL(this->get_logger(), "Failed to poll cameras: node cannot continue.");
      rclcpp::shutdown();
      return;
    }
    if (status == azure_kinect_ros_driver::CaptureSource::Status::kEndOfStream) {
      RCLCPP_INFO(this->get_logger(), "Recording reached end of file. node cannot continue.");
      rclcpp::shutdown();
      return;
    }

    if (status == azure_kinect_ros_driver::CaptureSource::Status::kRestarted) {
      // The device timestamps restart from the beginning of the recording; keep the ROS time
      // moving forward
      clock_.continueAfterRestart(
        last_capture_timestamp, azure_kinect_ros_driver::captureTimestamp(capture),
        std::chrono::nanoseconds(1000000000LL / params_.fps));
    }
    last_capture_timestamp = azure_kinect_ros_driver::captureTimestamp(capture);

    if (source_->providesSystemTimestamps()) {
      if (params_.depth_enabled) {
        // Update the timestamp offset based on the difference between the system timestamp (i.e.,
        // arrival at USB bus) and device timestamp (i.e., hardware clock at exposure start).
        updateClock(capture.get_ir_image().get_device_timestamp(),
          capture.get_ir_image().get_system_timestamp());
      } else if (params_.color_enabled) {
        updateClock(capture.get_color_image().get_device_timestamp(),
          capture.get_color_image().get_system_timestamp());
      }
      waitTime = regularFrameWaitTime;
    }

    // Set when any stream of this capture could not be produced
    bool iteration_failed = false;

    CompressedImage::SharedPtr rgb_jpeg_frame(new CompressedImage);
    Image::SharedPtr rgb_raw_frame(new Image);
    Image::SharedPtr rgb_rect_frame(new Image);
    Image::SharedPtr depth_raw_frame(new Image);
    Image::SharedPtr depth_rect_frame(new Image);
    Image::SharedPtr ir_raw_frame(new Image);
    PointCloud2::SharedPtr point_cloud(new PointCloud2);

    if (params_.depth_enabled) {
      // Only do compute if we have subscribers
      // Only create ir frame when we are using a device or we have an ir image.
      // Recordings may not have synchronized captures. For unsynchronized captures without ir image skip ir frame.

      if ((subscriberCount(ir_raw_publisher_) > 0 ||
        subscriberCount(ir_raw_camerainfo_publisher_) > 0) &&
        (source_->capturesAreComplete() || capture.get_ir_image() != nullptr))
      {
        // IR images are available in all depth modes
        result = getIrFrame(capture, ir_raw_frame);

        if (result != K4A_RESULT_SUCCEEDED) {
          iteration_failed = true;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Failed to get raw IR frame, skipping it");
        } else {
          capture_time = timestampToROS(capture.get_ir_image().get_device_timestamp());

          publishImageWithInfo(ir_raw_publisher_, ir_raw_frame, ir_raw_camerainfo_publisher_,
            ir_raw_camera_info, capture_time,
            calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_);
        }
      }

      // Depth images are not available in PASSIVE_IR mode
      if (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR) {
        // Only create depth frame when we are using a device or we have an depth image.
        // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip depth
        // frame.

        if ((subscriberCount(depth_raw_publisher_) > 0 ||
          subscriberCount(depth_raw_camerainfo_publisher_) > 0) &&
          (source_->capturesAreComplete() || capture.get_depth_image() != nullptr))
        {
          result = getDepthFrame(capture, depth_raw_frame);

          if (result != K4A_RESULT_SUCCEEDED) {
            iteration_failed = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Failed to get raw depth frame, skipping it");
          } else {
            capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());

            publishImageWithInfo(depth_raw_publisher_, depth_raw_frame, depth_raw_camerainfo_publisher_,
              depth_raw_camera_info, capture_time,
              calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_);
          }
        }

        // We can only rectify the depth into the color co-ordinates if the color camera is enabled!
        // Only create rect depth frame when we are using a device or we have an depth image.
        // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip rect
        // depth frame.

        if (params_.color_enabled &&
          (subscriberCount(depth_rect_publisher_) > 0 ||
          subscriberCount(depth_rect_camerainfo_publisher_) > 0) &&
          (source_->capturesAreComplete() || capture.get_depth_image() != nullptr))
        {
          result = getDepthFrame(capture, depth_rect_frame, true /* rectified */);

          if (result != K4A_RESULT_SUCCEEDED) {
            iteration_failed = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Failed to get rectifed depth frame, skipping it");
          } else {
            capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());

            publishImageWithInfo(depth_rect_publisher_, depth_rect_frame, depth_rect_camerainfo_publisher_,
              depth_rect_camera_info, capture_time,
              calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_);
          }
        }

#if defined(K4A_BODY_TRACKING)
        // Publish body markers when body tracking is enabled and a depth image is available
        if (params_.body_tracking_enabled && k4abt_tracker_queue_size_ < 3 &&
          (subscriberCount(body_marker_publisher_) > 0 ||
          subscriberCount(body_index_map_publisher_) > 0))
        {
          if (!k4abt_tracker_.enqueue_capture(capture)) {
            iteration_failed = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Failed to add the capture to the tracker queue, skipping it");
          } else {
            ++k4abt_tracker_queue_size_;
          }
        }
#endif
      }
    }

    if (params_.color_enabled) {
      // Only create rgb frame when we are using a device or we have a color image.
      // Recordings may not have synchronized captures. For unsynchronized captures without color image skip rgb frame.
      if (params_.color_format == "jpeg") {
        if ((subscriberCount(rgb_jpeg_publisher_) > 0 ||
          subscriberCount(rgb_raw_camerainfo_publisher_) > 0) &&
          (source_->capturesAreComplete() || capture.get_color_image() != nullptr))
        {
          result = getJpegRgbFrame(capture, rgb_jpeg_frame);

          if (result != K4A_RESULT_SUCCEEDED) {
            iteration_failed = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Failed to get Jpeg frame, skipping it");
          } else {
            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());

            rgb_jpeg_frame->header.stamp = capture_time;
            rgb_jpeg_frame->header.frame_id = calibration_data_.tf_prefix_ +
              calibration_data_.rgb_camera_frame_;
            rgb_jpeg_publisher_->publish(*rgb_jpeg_frame);

            // Re-synchronize the header timestamps since we cache the camera calibration message
            rgb_raw_camera_info.header.stamp = capture_time;
            rgb_raw_camerainfo_publisher_->publish(rgb_raw_camera_info);
          }
        }
      } else if (params_.color_format == "bgra") {
        if ((subscriberCount(rgb_raw_publisher_) > 0 ||
          subscriberCount(rgb_raw_camerainfo_publisher_) > 0) &&
          (source_->capturesAreComplete() || capture.get_color_image() != nullptr))
        {
          result = getRgbFrame(capture, rgb_raw_frame);

          if (result != K4A_RESULT_SUCCEEDED) {
            iteration_failed = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Failed to get RGB frame, skipping it");
          } else {
            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());

            publishImageWithInfo(rgb_raw_publisher_, rgb_raw_frame, rgb_raw_camerainfo_publisher_,
              rgb_raw_camera_info, capture_time,
              calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_);
          }
        }

        // We can only rectify the color into the depth co-ordinates if the depth camera is enabled and processing depth
        // data Only create rgb rect frame when we are using a device or we have a synchronized image. Recordings may
        // not have synchronized captures. For unsynchronized captures image skip rgb rect frame.

        if (params_.depth_enabled &&
          (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR) &&
          (subscriberCount(rgb_rect_publisher_) > 0 ||
          subscriberCount(rgb_rect_camerainfo_publisher_) > 0) &&
          (source_->capturesAreComplete() ||
          (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
        {
          result = getRgbFrame(capture, rgb_rect_frame, true /* rectified */);

          if (result != K4A_RESULT_SUCCEEDED) {
            iteration_failed = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Failed to get rectified RGB frame, skipping it");
          } else {
            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());

            publishImageWithInfo(rgb_rect_publisher_, rgb_rect_frame, rgb_rect_camerainfo_publisher_,
              rgb_rect_camera_info, capture_time,
              calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_);
          }
        }
      }
    }

    // Only create pointcloud when we are using a device or we have a synchronized image.
    // Recordings may not have synchronized captures. In unsynchronized captures skip point cloud.

    if (subscriberCount(pointcloud_publisher_) > 0 &&
      (source_->capturesAreComplete() ||
      (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
    {
      if (params_.rgb_point_cloud) {
        if (params_.point_cloud_in_depth_frame) {
          result = getRgbPointCloudInDepthFrame(capture, point_cloud);
        } else {
          result = getRgbPointCloudInRgbFrame(capture, point_cloud);
        }

        if (result != K4A_RESULT_SUCCEEDED) {
          iteration_failed = true;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Failed to get RGB Point Cloud, skipping it");
        }
      } else if (params_.point_cloud) {
        result = getPointCloud(capture, point_cloud);

        if (result != K4A_RESULT_SUCCEEDED) {
          iteration_failed = true;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Failed to get Point Cloud, skipping it");
        }
      }

      if (result == K4A_RESULT_SUCCEEDED && (params_.point_cloud || params_.rgb_point_cloud)) {
        pointcloud_publisher_->publish(*point_cloud);
      }
    }

    // A frame that cannot be rendered (e.g. corrupted by USB) is only skipped; the node gives up
    // only if the failures do not stop.
    if (iteration_failed) {
      if (++consecutive_failures >= kMaxConsecutiveFrameFailures) {
        RCLCPP_FATAL(this->get_logger(), "Failed to render %d captures in a row: node cannot "
          "continue.", consecutive_failures);
        rclcpp::shutdown();
        return;
      }
    } else {
      consecutive_failures = 0;
    }
  }
}

#if defined(K4A_BODY_TRACKING)
void K4AROSDevice::bodyPublisherThread()
{
  while (running_ && rclcpp::ok()) {
    if (k4abt_tracker_queue_size_ > 0) {
      k4abt::frame body_frame = k4abt_tracker_.pop_result();
      --k4abt_tracker_queue_size_;

      if (body_frame == nullptr) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Failed to pop a body frame result, skipping it");
      } else {
        auto capture_time = timestampToROS(body_frame.get_device_timestamp());

        if (subscriberCount(body_marker_publisher_) > 0) {
          // Joint marker array
          MarkerArray::SharedPtr markerArrayPtr(new MarkerArray);
          auto num_bodies = body_frame.get_num_bodies();
          for (size_t i = 0; i < num_bodies; ++i) {
            k4abt_body_t body = body_frame.get_body(i);
            for (int j = 0; j < (int) K4ABT_JOINT_COUNT; ++j) {
              Marker::SharedPtr markerPtr(new Marker);
              getBodyMarker(body, markerPtr, j, capture_time);
              markerArrayPtr->markers.push_back(*markerPtr);
            }
          }
          body_marker_publisher_->publish(*markerArrayPtr);
        }

        if (subscriberCount(body_index_map_publisher_) > 0) {
          // Body index map
          Image::SharedPtr body_index_map_frame(new Image);
          auto result = getBodyIndexMap(body_frame, body_index_map_frame);

          if (result != K4A_RESULT_SUCCEEDED) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "Failed to get body index map, skipping it");
          } else {
            body_index_map_frame->header.stamp = capture_time;
            body_index_map_frame->header.frame_id =
              calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;

            body_index_map_publisher_.publish(body_index_map_frame);
          }
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
  }
}
#endif

void K4AROSDevice::publishImuSample(const k4a_imu_sample_t & sample)
{
  Imu::SharedPtr imu_msg(new Imu);
  k4a_result_t result = getImuFrame(sample, imu_msg);

  RCLCPP_ERROR_EXPRESSION(this->get_logger(), result != K4A_RESULT_SUCCEEDED,
    "Failed to get IMU frame");

  // Samples whose angular velocity is exactly zero on every axis are not published. A real
  // reading always carries sensor noise (no such sample appeared in 13500 samples measured at
  // rest), so this only guards against empty samples.
  if (std::abs(imu_msg->angular_velocity.x) > DBL_EPSILON ||
    std::abs(imu_msg->angular_velocity.y) > DBL_EPSILON ||
    std::abs(imu_msg->angular_velocity.z) > DBL_EPSILON)
  {
    imu_orientation_publisher_->publish(*imu_msg);
  }
}

void K4AROSDevice::imuPublisherThread()
{
  k4a_imu_sample_t sample;
  k4a_imu_sample_t output;

  // For IMU throttling
  ImuThrottler throttler(IMU_MAX_RATE / params_.imu_rate_target);

  while (running_ && rclcpp::ok()) {
    // IMU messages are delivered in batches at 300 Hz. Block until the first one of a batch
    // arrives (with a timeout so that the thread can notice it has to stop), then drain the
    // rest of the queue without waiting.
    azure_kinect_ros_driver::CaptureSource::Status status =
      source_->nextImuSample(sample, std::chrono::milliseconds(10));
    while (status == azure_kinect_ros_driver::CaptureSource::Status::kOk) {
      if (throttler.add(sample, output)) {
        publishImuSample(output);
      }
      status = source_->nextImuSample(sample, std::chrono::milliseconds(0));
    }
  }
}

// Converts a k4a *device* timestamp to a ros::Time object
rclcpp::Time K4AROSDevice::timestampToROS(const std::chrono::microseconds & k4a_timestamp_us)
{
  // This will give INCORRECT timestamps until the first image.
  if (!clock_.synchronized()) {
    const std::chrono::nanoseconds offset = clock_.initializeFromWallClock(k4a_timestamp_us);
    RCLCPP_WARN_STREAM(this->get_logger(),
      "Initializing the device to realtime offset based on wall clock: " << offset.count() <<
        " ns");
  }

  return rclcpp::Time(clock_.toRealtime(k4a_timestamp_us).count(), RCL_ROS_TIME);
}

// Converts a k4a_imu_sample_t timestamp to a ros::Time object
rclcpp::Time K4AROSDevice::timestampToROS(const uint64_t & k4a_timestamp_us)
{
  return timestampToROS(std::chrono::microseconds(k4a_timestamp_us));
}

void K4AROSDevice::updateClock(
  const std::chrono::microseconds & k4a_device_timestamp_us,
  const std::chrono::nanoseconds & k4a_system_timestamp_ns)
{
  if (clock_.update(k4a_device_timestamp_us, k4a_system_timestamp_ns) ==
    azure_kinect_ros_driver::ClockSynchronizer::UpdateResult::kSnapped)
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
      "Initializing or re-initializing the device to realtime offset: " <<
        clock_.toRealtime(std::chrono::microseconds(0)).count() << " ns");
  }
}
