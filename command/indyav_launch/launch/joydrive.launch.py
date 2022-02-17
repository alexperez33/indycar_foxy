import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[{
            "dev": "/dev/input/js0",
            "autorepeat_rate": 10.0,
        }]
    )
    
    joydrive_node = Node(
        package='indyav_control',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[{
            "axis_steer": 0,
            "axis_throttle": 5,
            "scale_steer": 0.785398,
            "scale_trigger": 74.9906262,
            "throttle_topic": "/throttle",
            "steering_topic": "/steering",
        }]
    )

    return LaunchDescription([
        joy_node,
        joydrive_node,
    ])


