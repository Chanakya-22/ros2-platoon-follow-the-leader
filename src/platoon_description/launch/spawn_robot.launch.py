from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    model_path = os.path.expanduser(
        '~/platoon_ws/src/platoon_description/models/platoon_bot/model.sdf'
    )

    return LaunchDescription([

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'platoon_bot',
                '-file', model_path
            ],
            output='screen'
        )
    ])
