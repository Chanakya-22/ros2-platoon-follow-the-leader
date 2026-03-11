import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('br2_robodog')

    # Include the full Gazebo launch (sim + bridges + RViz)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
        ),
    )

    # Walk node — delay start to let the robot spawn and settle first
    walk_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='br2_robodog',
                executable='walk.py',
                name='walk_node',
                output='screen',
                parameters=[{
                    'gait_frequency': 1.5,
                    'hip_amplitude': 0.35,
                    'knee_amplitude': 0.5,
                    'knee_offset': -0.3,
                    'update_rate': 50.0,
                }],
            ),
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        walk_node,
    ])
