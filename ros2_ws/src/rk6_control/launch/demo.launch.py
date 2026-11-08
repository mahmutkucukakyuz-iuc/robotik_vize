"""demo.launch.py: ee_pose_publisher + visualizer + motion_sequence.

Gazebo ve kontrolculer once ayri baslatilmali:
  ros2 launch rk6_description gazebo.launch.py
  ros2 launch rk6_control demo.launch.py
Argumanlar: use_sim_time (true), motion_delay (5.0 s), show_camera (true)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    show_camera = ParameterValue(LaunchConfiguration('show_camera'), value_type=bool)

    ee_pose = Node(
        package='rk6_control', executable='ee_pose_publisher', name='ee_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'base_frame': 'base_link',
                     'ee_frame': 'tool0', 'rate_hz': 20.0}])

    visualizer = Node(
        package='rk6_control', executable='visualizer', name='visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'show_camera': show_camera}])

    motion = Node(
        package='rk6_control', executable='motion_sequence', name='motion_sequence',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('show_camera', default_value='true'),
        DeclareLaunchArgument('motion_delay', default_value='5.0',
                              description='motion_sequence baslamadan once bekleme [s]'),
        ee_pose,
        visualizer,
        TimerAction(period=LaunchConfiguration('motion_delay'), actions=[motion]),
    ])
