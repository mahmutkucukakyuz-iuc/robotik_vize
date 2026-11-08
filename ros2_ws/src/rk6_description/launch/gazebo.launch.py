#!/usr/bin/env python3
"""rk6 robot kolunu Gazebo Jetty'de baslatir.

Calistirilanlar:
  1. Gazebo (ros_gz_sim gz_sim.launch.py) + worlds/rk6_world.sdf
  2. robot_state_publisher (xacro -> robot_description, /tf)
  3. ros_gz_sim create  (robot_description topic'inden 'rk6' modelini spawn eder)
  4. controller spawner'lar: joint_state_broadcaster, arm_controller, gripper_controller
  5. ros_gz_bridge parameter_bridge (config/gz_bridge.yaml): /clock, kamera, IMU, link pozlari
  6. (istege bagli) RViz:  rviz:=true
  7. (istege bagli) rk6_control/ee_pose_publisher -> /ee_pose:  ee_pose:=true

Kullanim:
  ros2 launch rk6_description gazebo.launch.py
  ros2 launch rk6_description gazebo.launch.py rviz:=true ee_pose:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('rk6_description')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')

    xacro_file = os.path.join(pkg_share, 'urdf', 'rk6.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'rk6_controllers.yaml')
    bridge_file = PathJoinSubstitution([FindPackageShare('rk6_description'), 'config', 'gz_bridge.yaml'])
    rviz_file = PathJoinSubstitution([FindPackageShare('rk6_description'), 'rviz', 'rk6.rviz'])

    # xacro Python API: yol bosluk icerse bile calisir (Command(['xacro ', ...]) kelimelere boler).
    robot_description = xacro.process_file(
        xacro_file, mappings={'controllers_file': controllers_file}).toxml()

    # Gazebo'nun paket kaynaklarini bulmasi icin
    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([os.path.dirname(pkg_share),
                               os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world],
            'on_exit_shutdown': 'true',
        }.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rk6', '-z', '0.0'])

    # Istege bagli: /ee_pose yayincisi (rk6_control paketi). Varsayilan kapali, cunku
    # rk6_control'un demo.launch.py ve fk_analysis.launch.py dosyalari bu node'u zaten baslatir.
    ee_pose = Node(
        package='rk6_control',
        executable='ee_pose_publisher',
        name='ee_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('ee_pose')))

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        # Lyrical'da gz_ros2_control YAML'i kontrolcu node'larina aktarmiyor; --param-file sart
        arguments=['arm_controller', 'gripper_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers_file])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rk6_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'rk6_world.sdf'),
            description='Gazebo dunya (SDF) dosyasi'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz acilsin mi'),
        DeclareLaunchArgument('ee_pose', default_value='false',
                              description='rk6_control/ee_pose_publisher (/ee_pose) da baslatilsin mi'),
        gz_resource,
        gazebo,
        robot_state_publisher,
        bridge,
        spawn,
        # Model spawn edildikten sonra once joint_state_broadcaster, ardindan arm_controller
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])),
        ee_pose,
        rviz,
    ])
