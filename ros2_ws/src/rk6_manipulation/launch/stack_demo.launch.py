#!/usr/bin/env python3
"""Senaryo 2 tek komut: kuplu dunya + Gazebo + kontrolculer, ardindan scan_and_stack.

Kullanim:
  ros2 launch rk6_manipulation stack_demo.launch.py
  ros2 launch rk6_manipulation stack_demo.launch.py rviz:=true
  ros2 launch rk6_manipulation stack_demo.launch.py run_task:=false   # sadece simulasyon

scan_and_stack 'start_delay' saniye sonra baslar (kontrolculerin yuklenmesi icin); ayrica
kendisi de joint_states ve action sunucularini bekler. Gorev bitince simulasyon acik kalir;
tekrar denemek icin Ctrl+C ile kapatip yeniden baslatin (kupler yerine doner).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pick_world = os.path.join(get_package_share_directory('rk6_manipulation'), 'launch',
                              'pick_world.launch.py')

    scan_and_stack = Node(
        package='rk6_manipulation',
        executable='scan_and_stack',
        name='scan_and_stack',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'show_camera': ParameterValue(LaunchConfiguration('show_camera'), value_type=bool),
        }],
        condition=IfCondition(LaunchConfiguration('run_task')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='RViz acilsin mi'),
        DeclareLaunchArgument('run_task', default_value='true',
                              description='scan_and_stack baslatilsin mi (false: sadece simulasyon)'),
        DeclareLaunchArgument('show_camera', default_value='true',
                              description='"rk6 bilek kamerasi" penceresi acilsin mi'),
        DeclareLaunchArgument('start_delay', default_value='10.0',
                              description='scan_and_stack baslamadan once bekleme [s]'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pick_world),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()),
        TimerAction(period=LaunchConfiguration('start_delay'), actions=[scan_and_stack]),
    ])
