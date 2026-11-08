"""fk_analysis.launch.py: ee_pose_publisher + fk_error_analysis (T1..T10).

Argumanlar: use_sim_time (true), gt_topic (/gz_link_poses), gt_msg_type (tf),
results_dir (~/rk6_results)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gt_topic', default_value='/gz_link_poses'),
        DeclareLaunchArgument('gt_msg_type', default_value='tf'),
        DeclareLaunchArgument('results_dir', default_value='~/rk6_results'),
        Node(package='rk6_control', executable='ee_pose_publisher',
             name='ee_pose_publisher', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='rk6_control', executable='fk_error_analysis',
             name='fk_error_analysis', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'gt_topic': LaunchConfiguration('gt_topic'),
                          'gt_msg_type': LaunchConfiguration('gt_msg_type'),
                          'results_dir': LaunchConfiguration('results_dir')}]),
    ])
