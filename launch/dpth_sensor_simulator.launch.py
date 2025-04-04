# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
# Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
# All rights reserved.

import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Declare the path to the config YAML file
    sensor_coarse_config_file_path = os.path.join(
        get_package_share_directory("dpth_sensor_simulator_package"),  # noqa
        "config",  # noqa
        "keller_series_10lhpx_30bar.yaml",  # noqa
    )

    sensor_fine_config_file_path = os.path.join(
        get_package_share_directory("dpth_sensor_simulator_package"),  # noqa
        "config",  # noqa
        "keller_series_10lhpx_3bar.yaml",  # noqa
    )

    # Open the YAML file and load the parameters
    with open(sensor_coarse_config_file_path, "r") as file:
        config_coarse = yaml.safe_load(file)

    # Open the YAML file and load the parameters
    with open(sensor_fine_config_file_path, "r") as file:
        config_fine = yaml.safe_load(file)

    topic_name_arg = DeclareLaunchArgument(
        "topic_name",
        default_value="/auv/odometry",
        description="Topic name of the ground truth odometry from vehicle",
    )
    # Add the launch argument to the launch description
    ld.add_action(topic_name_arg)

    # Create the node for the coarse sensor
    dpth_sensor_coarse_simulator_node = Node(
        package="dpth_sensor_simulator_package",
        namespace="/auv/gnc/navigation_sensors/external_pressure_sensor_coarse",
        executable="dpth_sensor_simulator_package_node",
        name="dpth_sensor_coarse_simulator_node",
        output="screen",
        parameters=[config_coarse, {"topic_name": LaunchConfiguration("topic_name")}],
    )

    # Create the node for the fine sensor
    dpth_sensor_fine_simulator_node = Node(
        package="dpth_sensor_simulator_package",
        namespace="/auv/gnc/navigation_sensors/external_pressure_sensor_fine",
        executable="dpth_sensor_simulator_package_node",
        name="dpth_sensor_fine_simulator_node",
        output="screen",
        parameters=[config_fine, {"topic_name": LaunchConfiguration("topic_name")}],
    )

    ld.add_action(dpth_sensor_coarse_simulator_node)
    ld.add_action(dpth_sensor_fine_simulator_node)

    return ld
