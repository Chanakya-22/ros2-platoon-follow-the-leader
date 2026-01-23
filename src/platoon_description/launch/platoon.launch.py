import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_platoon_description = get_package_share_directory('platoon_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the SDF model
    sdf_path = os.path.join(pkg_platoon_description, 'models', 'platoon_bot', 'model.sdf')
    
    # 1. Start Gazebo Simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Spawn Leader Robot at (0, 0)
    spawn_leader = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'leader',
            '-file', sdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 3. Spawn Follower Robot at (-1, 0)
    spawn_follower = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'follower',
            '-file', sdf_path,
            '-x', '-1.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 4. Bridge Topics (ROS <-> Gazebo)
    # This connects the simulation topics to standard ROS topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Leader Bridges
            '/model/leader/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/leader/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Follower Bridges
            '/model/follower/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/follower/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen',
        remappings=[
            ('/model/leader/cmd_vel', '/leader/cmd_vel'),
            ('/model/leader/odometry', '/leader/odom'),
            ('/model/follower/cmd_vel', '/follower/cmd_vel'),
            ('/model/follower/odometry', '/follower/odom'),
        ]
    )

    # 5. Start the Follower Controller
    follower_node = Node(
        package='platoon_control',
        executable='follower_controller',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_leader,
        spawn_follower,
        bridge,
        follower_node
    ])