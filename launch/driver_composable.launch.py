# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

"""
Run the Azure Kinect driver as a component of a container.

Intra-process communication is enabled, so the other components loaded in
the same container (with `use_intra_process_comms` enabled too) get the
images and point clouds of the driver without any copy. For example, set
`republish_rgb_compressed` to also get `rgb/image_raw/compressed`, which the
driver no longer publishes by itself.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

# Parameters of the driver that can be set from the command line, as
# (name, default value, description)
PARAMETERS = [
    ('depth_enabled', 'true', 'Enable or disable the depth camera'),
    ('depth_mode', 'WFOV_UNBINNED',
     'Depth camera mode: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, '
     'WFOV_2X2BINNED or PASSIVE_IR'),
    ('depth_unit', '16UC1',
     'Depth units: 16UC1 (millimetres) or 32FC1 (metres)'),
    ('color_enabled', 'true', 'Enable or disable the color camera'),
    ('color_format', 'bgra', 'Format of the color camera: bgra or jpeg'),
    ('color_resolution', '1536P',
     'Color camera resolution: 720P, 1080P, 1440P, 1536P, 2160P or 3072P'),
    ('fps', '5', 'Frames per second of both cameras: 5, 15 or 30'),
    ('point_cloud', 'true', 'Publish a point cloud from the depth data'),
    ('rgb_point_cloud', 'true',
     'Colorize the point cloud with the color camera'),
    ('point_cloud_in_depth_frame', 'false',
     'Build the colored point cloud in the depth frame'),
    ('sensor_sn', '', 'Serial number of the sensor; empty for the first one'),
    ('recording_file', '',
     'Absolute path of a recording to play instead of using a device'),
    ('recording_loop_enabled', 'false',
     'Play the recording again when it ends'),
    ('tf_prefix', '', 'Prefix of the TF frames'),
    ('imu_rate_target', '0', 'Rate of the IMU messages; 0 for the maximum'),
]


def generate_launch_description() -> LaunchDescription:
    """
    Describe the container, the driver and the optional republisher.

    Returns
    -------
    LaunchDescription
        The launch arguments, the container with the driver and the
        republisher, which is only loaded when it is asked for.
    """
    arguments = [
        DeclareLaunchArgument(
            name, default_value=default, description=description)
        for name, default, description in PARAMETERS
    ]
    arguments += [
        DeclareLaunchArgument(
            'container_name', default_value='k4a_container',
            description='Name of the container'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the driver when it is '
            'loaded. Set it to false to drive the node with `ros2 lifecycle '
            'set /k4a_ros_device_node configure|activate`'),
        DeclareLaunchArgument(
            'republish_rgb_compressed', default_value='false',
            description='Also publish rgb/image_raw/compressed, with an '
            'image_transport republisher in the same container'),
    ]

    parameters = {name: LaunchConfiguration(name) for name, _, _ in PARAMETERS}
    parameters['autostart'] = LaunchConfiguration('autostart')

    # The driver and what consumes its data share one process and exchange
    # the messages by pointer
    intra_process = [{'use_intra_process_comms': True}]

    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='azure_kinect_ros_driver',
                plugin='azure_kinect_ros_driver::K4ADriverNode',
                name='k4a_ros_device_node',
                parameters=[parameters],
                extra_arguments=intra_process),
        ])

    # The republisher only subscribes to its input while its output has
    # subscribers, and image_transport names the output `<out>/compressed`
    republisher = LoadComposableNodes(
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='image_transport',
                plugin='image_transport::Republisher',
                name='rgb_compressed_republisher',
                parameters=[{
                    'in_transport': 'raw',
                    'out_transport': 'compressed',
                }],
                remappings=[
                    ('in', 'rgb/image_raw'),
                    ('out/compressed', 'rgb/image_raw/compressed'),
                ],
                extra_arguments=intra_process),
        ],
        condition=IfCondition(LaunchConfiguration('republish_rgb_compressed')))

    return LaunchDescription(arguments + [container, republisher])
