import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

# All joint command topics to bridge from ROS 2 -> Gazebo for a single robodog
_JOINT_CMD_TOPICS = [
    'front_left_hip_joint',
    'front_left_knee_joint',
    'front_right_hip_joint',
    'front_right_knee_joint',
    'rear_left_hip_joint',
    'rear_left_knee_joint',
    'rear_right_hip_joint',
    'rear_right_knee_joint',
    'head_joint',
]

def generate_launch_description():
    pkg_robodog = get_package_share_directory('br2_robodog')
    pkg_platoon_control = get_package_share_directory('platoon_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Process xacro to URDF
    xacro_file = os.path.join(pkg_robodog, 'urdf', 'robodog.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # World file
    world_file = os.path.join(pkg_robodog, 'worlds', 'empty.sdf')

    # 1. Start Gazebo Simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Spawn Leader Robodog
    spawn_leader = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'leader',
            '-string', robot_description_xml,
            '-x', '0.0', '-y', '0.0', '-z', '0.35'
        ],
        output='screen'
    )

    leader_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_leader',
        namespace='leader',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
            'frame_prefix': 'leader/'
        }],
    )

    # 3. Spawn Follower Robodog
    spawn_follower = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'follower',
            '-string', robot_description_xml,
            '-x', '-1.5', '-y', '0.0', '-z', '0.35'
        ],
        output='screen'
    )

    follower_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_follower',
        namespace='follower',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
            'frame_prefix': 'follower/'
        }],
    )

    # 4. Bridge Topics (ROS <-> Gazebo)
    bridge_args = []

    # Leader bridges
    for joint_name in _JOINT_CMD_TOPICS:
        bridge_args.append(
            f'/model/leader/joint/{joint_name}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        )
    bridge_args.append('/model/leader/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance')

    # Follower bridges
    for joint_name in _JOINT_CMD_TOPICS:
        bridge_args.append(
            f'/model/follower/joint/{joint_name}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        )
    bridge_args.append('/model/follower/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
        remappings=[
            ('/model/leader/odometry_with_covariance', '/leader/odom'),
            ('/model/follower/odometry_with_covariance', '/follower/odom'),
        ]
    )

    # 5. Robodog Gait Controllers (translates cmd_vel to walking gait)
    # Using TimerAction to ensure Gazebo loads before nodes command joints
    leader_walk_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='br2_robodog',
                executable='walk_cmd_vel.py',
                name='walk_cmd_vel_node_leader',
                namespace='leader',
                output='screen',
                parameters=[{
                    'gz_model_name': 'leader',
                    'base_gait_frequency': 1.5,
                    'update_rate': 50.0,
                }],
            )
        ]
    )

    follower_walk_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='br2_robodog',
                executable='walk_cmd_vel.py',
                name='walk_cmd_vel_node_follower',
                namespace='follower',
                output='screen',
                parameters=[{
                    'gz_model_name': 'follower',
                    'base_gait_frequency': 1.5,
                    'update_rate': 50.0,
                }],
            )
        ]
    )

    # 6. Start the Platoon Follower Controller (with tuned parameters)
    platoon_follower_node = TimerAction(
        period=8.0,  # Increased to allow Gazebo to settle
        actions=[
            Node(
                package='platoon_control',
                executable='follower_controller',
                output='screen',
                remappings=[
                    ('leader_odom', '/leader/odom'),
                    ('follower_odom', '/follower/odom'),
                    ('cmd_vel', '/follower/cmd_vel')
                ],
                parameters=[{
                    'lookahead_dist': 1.0,
                    'k_v': 0.8,
                    'k_w': 1.2,
                }]
            )
        ]
    )

    return LaunchDescription([
        gz_sim,
        spawn_leader,
        leader_state_publisher,
        spawn_follower,
        follower_state_publisher,
        bridge,
        leader_walk_node,
        follower_walk_node,
        platoon_follower_node
    ])