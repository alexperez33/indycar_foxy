import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    localization_node = Node(
        package='indyav_localization',
        executable='dumb_truth_odom_tf',
        name='ins_odom_localization',
        parameters=[{
            "topic_name": "/ins_odom",
        }]
    )

    return LaunchDescription([
        localization_node,
    ])


