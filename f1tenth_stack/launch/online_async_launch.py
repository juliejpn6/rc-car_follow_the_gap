#!/usr/bin/env python3
"""
online_async_launch.py

slam_toolbox の async_slam_toolbox_node を起動する独立したlaunchファイル。
bringup_tt02_launch.py とは別に起動・停止できる。

使用方法:
  ros2 launch f1tenth_stack online_async_launch.py
  ros2 launch f1tenth_stack online_async_launch.py use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── パラメータファイルのパス ──────────────────────
    pkg_share = get_package_share_directory('f1tenth_stack')
    default_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # ── Launch Arguments ──────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params,
        description='Full path to slam_toolbox params yaml'
    )

    # ── async_slam_toolbox_node ───────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_arg,
        slam_node,
    ])
