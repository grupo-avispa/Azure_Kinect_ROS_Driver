// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Library headers
//
#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_driver_node.h"
#include "fake_capture_source.h"

using azure_kinect_ros_driver::K4ADriverNode;
using azure_kinect_ros_driver::testing::FakeCaptureSource;
using azure_kinect_ros_driver::testing::FakeSourceState;
using lifecycle_msgs::msg::State;
using namespace std::chrono_literals;

/**
 * @brief Test fixture that runs a driver node on a fake source, with a node to probe its topics.
 *
 * The node does not start by itself: tests drive it with the lifecycle transitions unless they
 * ask otherwise.
 */
class DriverNodeTest : public ::testing::Test
{
public:
  /** @brief Creates the shared state of the fake source. */
  void SetUp() override
  {
    state_ = std::make_shared<FakeSourceState>();
  }

  /** @brief Stops the executor and releases the nodes. */
  void TearDown() override
  {
    stopSpinning();
    probe_.reset();
    node_.reset();
  }

  /**
   * @brief Creates the driver node.
   *
   * @param extra_parameters Parameters to set besides the defaults of the tests.
   * @param intra_process Whether the nodes use intra-process communication.
   */
  void makeNode(const std::vector<rclcpp::Parameter>& extra_parameters = {},
                bool intra_process = false)
  {
    std::vector<rclcpp::Parameter> parameters = { rclcpp::Parameter("autostart", false),
                                                  rclcpp::Parameter("color_enabled", false),
                                                  rclcpp::Parameter("point_cloud", false),
                                                  rclcpp::Parameter("fps", 30),
                                                  rclcpp::Parameter("imu_rate_target", 100) };
    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());

    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    options.use_intra_process_comms(intra_process);
    node_ = std::make_shared<K4ADriverNode>(
      options,
      [this](K4AROSDeviceParams&,
             rclcpp::Logger) -> std::unique_ptr<azure_kinect_ros_driver::CaptureSource>
      {
        if (factory_fails_)
        {
          throw std::runtime_error("the source cannot be opened");
        }
        return std::make_unique<FakeCaptureSource>(state_, TEST_DATA_DIR);
      });

    rclcpp::NodeOptions probe_options;
    probe_options.use_intra_process_comms(intra_process);
    probe_ = std::make_shared<rclcpp::Node>("driver_probe", probe_options);
  }

  /** @brief Starts a thread that spins the driver and the probe node. */
  void startSpinning()
  {
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(probe_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  /** @brief Stops the thread started by `startSpinning()`. */
  void stopSpinning()
  {
    if (spin_thread_.joinable())
    {
      executor_.cancel();
      spin_thread_.join();
    }
  }

  /**
   * @brief Waits until a condition holds.
   *
   * @param condition What to wait for.
   * @param timeout How long to wait.
   * @return Whether the condition held in time.
   */
  static bool waitFor(const std::function<bool()>& condition,
                      std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (condition())
      {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  }

  /**
   * @brief The id of the current lifecycle state of the driver.
   *
   * @return A `lifecycle_msgs::msg::State::PRIMARY_STATE_*` value.
   */
  uint8_t stateId() const
  {
    return node_->get_current_state().id();
  }

  /**
   * @brief Whether a topic is currently published by the driver, as the probe node sees it.
   *
   * @param topic The full name of the topic.
   * @return True if the topic has a publisher.
   */
  bool isPublished(const std::string& topic) const
  {
    return probe_->count_publishers(topic) > 0;
  }

  /** @brief State shared with the fake source. */
  std::shared_ptr<FakeSourceState> state_;

  /** @brief Whether creating the source fails. */
  bool factory_fails_ = false;

  /** @brief The driver node. */
  std::shared_ptr<K4ADriverNode> node_;

  /** @brief A node to subscribe to the topics of the driver. */
  std::shared_ptr<rclcpp::Node> probe_;

  /** @brief Spins the nodes. */
  rclcpp::executors::SingleThreadedExecutor executor_;

  /** @brief The thread that spins the nodes. */
  std::thread spin_thread_;
};

/**
 * @brief Configuring creates the publishers of the enabled streams and nothing else.
 */
TEST_F(DriverNodeTest, ConfigureCreatesOnlyTheEnabledStreams)
{
  makeNode();

  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  EXPECT_TRUE(waitFor([&]() { return isPublished("/depth/image_raw"); }));
  EXPECT_TRUE(isPublished("/depth/camera_info"));
  EXPECT_TRUE(isPublished("/ir/image_raw"));
  EXPECT_TRUE(isPublished("/imu"));
  EXPECT_FALSE(isPublished("/rgb/image_raw"));
  EXPECT_FALSE(isPublished("/depth_to_rgb/image_raw"));
  EXPECT_FALSE(isPublished("/points2"));
  EXPECT_EQ(state_->starts, 0);
}

/**
 * @brief The source starts when the node is activated and stops when it is deactivated.
 */
TEST_F(DriverNodeTest, TransitionsStartAndStopTheSource)
{
  makeNode();

  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(state_->starts, 1);
  EXPECT_TRUE(waitFor([&]() { return state_->captures > 3; }));

  ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_GE(state_->stops, 1);
  const int captures_when_stopped = state_->captures;
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(state_->captures, captures_when_stopped);

  ASSERT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(node_->failed());
}

/**
 * @brief Depth images carry the frame, the encoding and a stamp that moves forward.
 */
TEST_F(DriverNodeTest, PublishesDepthWithFrameEncodingAndMonotonicStamps)
{
  makeNode({ rclcpp::Parameter("tf_prefix", "k4a_") });
  std::mutex mutex;
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> infos;
  auto image_subscription = probe_->create_subscription<sensor_msgs::msg::Image>(
    "depth/image_raw", 10,
    [&](sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(mutex);
      images.push_back(msg);
    });
  auto info_subscription = probe_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "depth/camera_info", 10,
    [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(mutex);
      infos.push_back(msg);
    });

  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  startSpinning();

  ASSERT_TRUE(waitFor(
    [&]()
    {
      std::lock_guard<std::mutex> guard(mutex);
      return images.size() >= 5 && infos.size() >= 5;
    }));

  std::lock_guard<std::mutex> guard(mutex);
  for (const auto& image : images)
  {
    EXPECT_EQ(image->header.frame_id, "k4a_depth_camera_link");
    EXPECT_EQ(image->encoding, sensor_msgs::image_encodings::TYPE_16UC1);
    EXPECT_EQ(image->width, 640u);
    EXPECT_EQ(image->height, 576u);
  }
  for (size_t i = 1; i < images.size(); ++i)
  {
    EXPECT_GT(rclcpp::Time(images[i]->header.stamp), rclcpp::Time(images[i - 1]->header.stamp));
  }
  EXPECT_EQ(infos.front()->header.frame_id, "k4a_depth_camera_link");
  EXPECT_EQ(infos.front()->width, 640u);
  EXPECT_EQ(infos.front()->distortion_model, "rational_polynomial");
}

/**
 * @brief The IMU is averaged down to the target rate.
 */
TEST_F(DriverNodeTest, ImuIsThrottledToTheTargetRate)
{
  makeNode();
  std::atomic_int received{ 0 };
  auto subscription = probe_->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 1000, [&](sensor_msgs::msg::Imu::ConstSharedPtr) { ++received; });

  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  startSpinning();
  ASSERT_TRUE(waitFor([&]() { return state_->imu_samples > 1600; }, 10s));
  stopSpinning();
  ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);

  // 1600 / 100 = 16 samples are averaged into every message
  const int expected = state_->imu_samples / 16;
  EXPECT_NEAR(received, expected, 3);
}

/**
 * @brief A source that cannot be opened makes the configuration fail.
 */
TEST_F(DriverNodeTest, FailedSourceFailsTheConfiguration)
{
  factory_fails_ = true;
  makeNode();

  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(node_->failed());
  EXPECT_FALSE(isPublished("/depth/image_raw"));
}

/**
 * @brief Invalid parameters make the configuration fail before the source is opened.
 */
TEST_F(DriverNodeTest, InvalidParametersFailTheConfiguration)
{
  makeNode({ rclcpp::Parameter("depth_unit", "bogus") });
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(node_->failed());

  node_->set_parameter(rclcpp::Parameter("depth_unit", "32FC1"));
  node_->set_parameter(rclcpp::Parameter("imu_rate_target", 5000));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(node_->failed());

  // Once the parameters are fixed the same node can be configured
  node_->set_parameter(rclcpp::Parameter("imu_rate_target", 100));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(node_->failed());
}

/**
 * @brief The end of a recording stops the node without it being an error.
 */
TEST_F(DriverNodeTest, EndOfStreamStopsTheNodeWithoutError)
{
  state_->end_after = 10;
  makeNode();
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  startSpinning();

  EXPECT_TRUE(waitFor([&]() { return stateId() == State::PRIMARY_STATE_UNCONFIGURED; }));
  EXPECT_FALSE(node_->failed());
  EXPECT_GE(state_->stops, 1);
}

/**
 * @brief A device that stops delivering captures is a fatal error that releases the source.
 */
TEST_F(DriverNodeTest, StalledDeviceIsFatal)
{
  state_->stall_after = 5;
  makeNode();
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  startSpinning();

  EXPECT_TRUE(waitFor([&]() { return stateId() == State::PRIMARY_STATE_UNCONFIGURED; }, 10s));
  EXPECT_TRUE(node_->failed());
  EXPECT_GE(state_->stops, 1);
}

/**
 * @brief An exception in the capture thread is survived: the thread is restarted.
 */
TEST_F(DriverNodeTest, ExceptionInTheCaptureThreadDoesNotStopThePublishing)
{
  state_->throws = 2;
  makeNode();
  std::atomic_int received{ 0 };
  auto subscription = probe_->create_subscription<sensor_msgs::msg::Image>(
    "depth/image_raw", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr) { ++received; });
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  startSpinning();

  EXPECT_TRUE(waitFor([&]() { return received >= 5; }, 10s));
  EXPECT_EQ(stateId(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_FALSE(node_->failed());
}

/**
 * @brief A cleaned up node can be configured again with other parameters.
 */
TEST_F(DriverNodeTest, CanBeReconfiguredWithOtherParameters)
{
  makeNode();
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(waitFor([&]() { return isPublished("/ir/image_raw"); }));
  ASSERT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(waitFor([&]() { return !isPublished("/ir/image_raw"); }));

  // Nothing can be published without depth: only the IMU remains
  node_->set_parameter(rclcpp::Parameter("depth_enabled", false));
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(waitFor([&]() { return isPublished("/imu"); }));
  EXPECT_FALSE(isPublished("/ir/image_raw"));
  EXPECT_FALSE(isPublished("/depth/image_raw"));

  ASSERT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  node_->set_parameter(rclcpp::Parameter("depth_enabled", true));
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(waitFor([&]() { return isPublished("/depth/image_raw"); }));
}

/**
 * @brief With `autostart` the node configures and activates itself when it spins.
 */
TEST_F(DriverNodeTest, AutostartActivatesTheNode)
{
  makeNode({ rclcpp::Parameter("autostart", true) });
  EXPECT_EQ(stateId(), State::PRIMARY_STATE_UNCONFIGURED);

  startSpinning();

  EXPECT_TRUE(waitFor([&]() { return stateId() == State::PRIMARY_STATE_ACTIVE; }));
  EXPECT_TRUE(waitFor([&]() { return state_->captures > 3; }));
}

/**
 * @brief Collects the depth images that two subscribers of the probe node receive.
 *
 * Both subscribers take the message by shared pointer, so with intra-process communication they
 * get the very same object, and over the network each one gets its own copy.
 *
 * @param test The fixture.
 * @param intra_process Whether the nodes use intra-process communication.
 * @param same The number of stamps for which both subscribers got the same object.
 * @param different The number of stamps for which they got different objects.
 */
static void compareSubscribers(DriverNodeTest& test, bool intra_process, int& same, int& different)
{
  test.makeNode({}, intra_process);
  using Image = sensor_msgs::msg::Image;
  std::mutex mutex;
  // The messages are kept alive, otherwise the memory of one could be reused for the next
  std::map<int64_t, std::vector<Image::ConstSharedPtr>> received;
  auto record = [&](Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mutex);
    const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (received.size() < 12 || received.count(stamp) > 0)
    {
      received[stamp].push_back(msg);
    }
  };
  auto first = test.probe_->create_subscription<Image>("depth/image_raw", 10, record);
  auto second = test.probe_->create_subscription<Image>("depth/image_raw", 10, record);

  ASSERT_EQ(test.node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(test.node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  test.startSpinning();

  ASSERT_TRUE(DriverNodeTest::waitFor(
    [&]()
    {
      std::lock_guard<std::mutex> guard(mutex);
      int complete = 0;
      for (const auto& [stamp, pointers] : received)
      {
        complete += pointers.size() == 2 ? 1 : 0;
      }
      return complete >= 5;
    }));

  std::lock_guard<std::mutex> guard(mutex);
  same = 0;
  different = 0;
  for (const auto& [stamp, pointers] : received)
  {
    if (pointers.size() == 2)
    {
      (pointers[0].get() == pointers[1].get() ? same : different)++;
    }
  }
}

/**
 * @brief Finds a diagnostic status of the driver by the name of its task.
 *
 * @param statuses The statuses received so far.
 * @param task The name of the task, e.g. `K4A device`.
 * @return The last status of that task, or nullptr if there is none.
 */
static const diagnostic_msgs::msg::DiagnosticStatus* findStatus(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus>& statuses, const std::string& task)
{
  const diagnostic_msgs::msg::DiagnosticStatus* found = nullptr;
  for (const auto& status : statuses)
  {
    if (status.name.find(task) != std::string::npos)
    {
      found = &status;
    }
  }
  return found;
}

/**
 * @brief A streaming driver reports its health on `/diagnostics` and the IMU temperature.
 */
TEST_F(DriverNodeTest, PublishesDiagnosticsAndTemperature)
{
  makeNode();
  std::mutex mutex;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
  std::vector<sensor_msgs::msg::Temperature::ConstSharedPtr> temperatures;
  auto diagnostics_subscription =
    probe_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
      {
        std::lock_guard<std::mutex> guard(mutex);
        statuses.insert(statuses.end(), msg->status.begin(), msg->status.end());
      });
  auto temperature_subscription = probe_->create_subscription<sensor_msgs::msg::Temperature>(
    "temperature", 10,
    [&](sensor_msgs::msg::Temperature::ConstSharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(mutex);
      temperatures.push_back(msg);
    });

  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  startSpinning();

  // The fake IMU delivers one second of device time in about 0.6 s, so a few temperatures arrive
  ASSERT_TRUE(waitFor(
    [&]()
    {
      std::lock_guard<std::mutex> guard(mutex);
      const auto* device = findStatus(statuses, "K4A device");
      const auto* rate = findStatus(statuses, "K4A capture rate");
      return temperatures.size() >= 2 && device != nullptr && rate != nullptr &&
             device->level == diagnostic_msgs::msg::DiagnosticStatus::OK;
    },
    15s));

  std::lock_guard<std::mutex> guard(mutex);
  const auto* device = findStatus(statuses, "K4A device");
  EXPECT_EQ(device->hardware_id, "FAKE");
  EXPECT_EQ(device->message, "Streaming");
  for (const auto& temperature : temperatures)
  {
    EXPECT_EQ(temperature->header.frame_id, "imu_link");
    EXPECT_GE(temperature->temperature, 30.0);
    EXPECT_LE(temperature->temperature, 31.0);
  }
  EXPECT_GT(rclcpp::Time(temperatures[1]->header.stamp),
            rclcpp::Time(temperatures[0]->header.stamp));
}

/**
 * @brief A failed configuration is reported as an error on `/diagnostics`.
 */
TEST_F(DriverNodeTest, DiagnosticsReportAFailedConfiguration)
{
  factory_fails_ = true;
  makeNode();
  std::mutex mutex;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses;
  auto subscription = probe_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
    {
      std::lock_guard<std::mutex> guard(mutex);
      statuses.insert(statuses.end(), msg->status.begin(), msg->status.end());
    });
  startSpinning();
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);

  ASSERT_TRUE(waitFor(
    [&]()
    {
      std::lock_guard<std::mutex> guard(mutex);
      const auto* device = findStatus(statuses, "K4A device");
      return device != nullptr && device->level == diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    },
    10s));

  std::lock_guard<std::mutex> guard(mutex);
  EXPECT_NE(findStatus(statuses, "K4A device")->message.find("the source cannot be opened"),
            std::string::npos);
}

/**
 * @brief Within a process with intra-process communication the subscribers share the message.
 */
TEST_F(DriverNodeTest, IntraProcessSubscribersShareTheSameMessage)
{
  int same = 0;
  int different = 0;
  compareSubscribers(*this, true, same, different);

  EXPECT_GE(same, 5);
  EXPECT_EQ(different, 0);
}

/**
 * @brief Without intra-process communication each subscriber gets its own copy, which shows that
 * the previous test tells the two cases apart.
 */
TEST_F(DriverNodeTest, WithoutIntraProcessEachSubscriberGetsItsOwnCopy)
{
  int same = 0;
  int different = 0;
  compareSubscribers(*this, false, same, different);

  EXPECT_EQ(same, 0);
  EXPECT_GE(different, 5);
}

/**
 * @brief Runs the unit tests in a ROS domain of their own, away from any robot on the network.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(87);
  rclcpp::init(argc, argv, init_options);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
