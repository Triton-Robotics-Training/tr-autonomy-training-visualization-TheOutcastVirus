from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# TODO this is should instead your_solution
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sim_node'),
                    'launch',
                    'sim_node_launch.py'
                ])
            ])
        ),

        Node(
            package='sim_node',
            executable='keyboard_controls',
            name='keyboard_controls',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='huskybot_cv',
            executable='huskybot_cv',
            name='huskybot_cv',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='your_solution',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='your_solution',
            executable='calc_error',
            name='calc_error',
            parameters=[{'use_sim_time': True}]
        ),
    ])