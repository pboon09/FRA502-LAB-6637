#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    desc_pkg = get_package_share_directory('robot_description')

    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'simple_display.launch.py')
        )
    )

    controller = Node(
        package='robot_controller',
        executable='controller.py',
        name='robot_controller',
        output='screen'
    )

    random_target = Node(
        package='robot_controller',
        executable='random_target.py',
        name='random_target_node',
        output='screen',
        parameters=[{
            'r_min': 0.020,
            'r_max': 0.251,
            'z_min': 0.002,
            'z_max': 0.450
        }]
    )

    end_effector_pub = Node(
        package='robot_controller',
        executable='end_effector_pose.py',
        name='end_effector_publisher',
        output='screen',
        parameters=[{
            'debug': False
        }]
    )

    delay_sec = TimerAction(
        period=5.0,
        actions=[
            controller,
            random_target,
            end_effector_pub
        ]
    )

    ld = LaunchDescription()
    ld.add_action(robot_desc_launch)
    ld.add_action(delay_sec)

    return ld
