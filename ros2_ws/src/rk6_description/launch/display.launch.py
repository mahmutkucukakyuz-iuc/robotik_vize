#!/usr/bin/env python3
"""rk6 modelini yalnizca RViz'de gosterir (Gazebo yok).

joint_state_publisher_gui ile eklemler kaydirici (slider) ile oynatilabilir;
TF agacinda base_link -> tool0 donusumu DH 0T6 ile birebir aynidir.

Kullanim:
  ros2 launch rk6_description display.launch.py
  ros2 launch rk6_description display.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    gui = LaunchConfiguration('gui')

    pkg_share = get_package_share_directory('rk6_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rk6.urdf.xacro')
    rviz_file = PathJoinSubstitution([FindPackageShare('rk6_description'), 'rviz', 'rk6.rviz'])

    # xacro Python API: yol bosluk icerse bile calisir.
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint_state_publisher_gui kullanilsin mi'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui)),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui)),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file]),
    ])
