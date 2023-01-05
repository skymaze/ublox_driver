import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ublox_driver_dir = get_package_share_directory("ublox_driver")
    config_path = os.path.join(
        ublox_driver_dir, "config/driver_config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="ublox_driver",
                executable="ublox_driver",
                name="ublox_driver",
                output="screen",
                parameters=[config_path]
            )  
        ]
    )
