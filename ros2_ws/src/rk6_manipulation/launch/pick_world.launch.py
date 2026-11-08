#!/usr/bin/env python3
"""Manipulasyon senaryosu: rk6'yi kuplu dunyada (worlds/rk6_pick_world.sdf) baslatir.

Vize senaryosundaki gazebo.launch.py'nin aynisini, yalnizca dunya dosyasini degistirerek calistirir.

Kullanim:
  ros2 launch rk6_manipulation pick_world.launch.py
  ros2 launch rk6_manipulation pick_world.launch.py rviz:=true
Ardindan ikinci terminalde:
  ros2 run rk6_manipulation scan_and_stack --ros-args -p use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = os.path.join(get_package_share_directory('rk6_manipulation'), 'worlds',
                         'rk6_pick_world.sdf')
    gazebo_launch = os.path.join(get_package_share_directory('rk6_description'), 'launch',
                                 'gazebo.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='RViz acilsin mi'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world, 'rviz': LaunchConfiguration('rviz')}.items()),
    ])
