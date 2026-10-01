Changelog
=========

All notable changes to ``azure_kinect_ros_driver`` are documented in this file. The format is
based on `Keep a Changelog <https://keepachangelog.com/>`_ and the package follows
`Semantic Versioning <https://semver.org/>`_.

[Unreleased]
------------

No tests were added: the package has none yet. Changes were checked on a Jetson with an Azure
Kinect DK (ROS 2 Jazzy, Azure Kinect Sensor SDK 1.4) using live captures and a recording.

Added
^^^^^

* ``ImuThrottler`` in ``k4a_ros_types.h``: the IMU averaging throttle, shared by the device and
  the recording paths.
* ``K4AROSDeviceParams::ValidateImuRate()``: validates ``imu_rate_target`` and replaces ``0`` by
  ``IMU_MAX_RATE``, independently of the device configuration.
* ``K4AROSDevice::runGuarded()``, ``publishImageWithInfo()`` and ``publishImuSample()``.
* ``CHANGELOG.rst``.

Changed
^^^^^^^^

* The driver is a single ROS node (``k4a_ros_device_node``). ``K4AROSDeviceParams`` and
  ``K4ACalibrationTransformData`` no longer inherit from ``rclcpp::Node``, the empty
  ``k4a_bridge`` node was removed and ``main`` spins the device node, so the camera thread no
  longer calls ``spin_some()``.
* Parameters are declared from ``ROS_PARAM_LIST`` with their help string as description. As a
  result ``depth_unit`` and ``tf_prefix`` can now be set; before they were silently ignored.
* Only the streams that the configuration can produce are advertised (``depth/*``, ``ir/*``,
  ``depth_to_rgb/*``, ``rgb/*``, ``rgb_to_depth/*``). The duplicated advertisement of
  ``depth/image_raw`` was removed.
* A frame that fails to render is skipped with a throttled warning instead of calling
  ``rclcpp::shutdown()``; the node gives up after 30 consecutive failed captures.
* Subscribers are counted through the publishers, so subscribers of
  ``<topic>/compressed`` (or any ``image_transport`` plugin) now trigger frame generation.
* ``getRbgFrame()`` renamed to ``getRgbFrame()``.
* The IMU thread blocks on the first sample of a batch instead of polling at 300 Hz when a
  device is used.
* The calibration dump is logged at debug level.
* Build: C++17, ``package.xml`` format 3 with the launch dependencies as ``exec_depend``.

Fixed
^^^^^

* ``K4AROSDevice`` no longer leaks an ``ImageTransport`` nor creates a second ``shared_ptr``
  control block from ``this``.
* ``running_`` and the device-to-realtime clock offset are atomic.
* Exceptions escaping the camera, IMU and body threads no longer call ``std::terminate``; the
  thread body is logged and restarted. This also removes an abort seen when stopping the node
  while ``count_subscribers()`` ran on an invalid context.
* The node exits with an error code when no device or recording can be opened, or when the
  device configuration is invalid, instead of staying idle or aborting in the destructor.
* A zero or invalid ``imu_rate_target`` no longer causes a division by zero in the IMU thread.
* Playback: the ROS time stays monotonic when ``recording_loop_enabled`` rewinds the recording
  (it used to jump back by the length of the recording), the parameters are printed, and a
  recording that cannot be opened is reported instead of throwing from the constructor.
* ``getBodyMarker()`` used ``rclcpp::Duration(0.25)``, which does not compile; it now uses
  ``Duration::from_seconds(0.25)``. This code is only built with the Body Tracking SDK and was
  not compiled here.
* ``driver.launch.py`` writes the generated URDF to a temporary directory instead of the
  install space.

Removed
^^^^^^^

* ``launch/rectify_test.launch`` and ``launch/slam_rtabmap.launch`` (ROS 1 nodelet launch
  files that cannot run on ROS 2).
* ``K4AROSDeviceParams::Help()``, which was never called and printed its macro arguments
  literally.
