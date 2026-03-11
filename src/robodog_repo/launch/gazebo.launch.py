import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


# All joint command topics to bridge from ROS 2 → Gazebo
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
    pkg_dir = get_package_share_directory('br2_robodog')

    # Process xacro to URDF
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robodog.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # World file
    world_file = os.path.join(pkg_dir, 'worlds', 'empty.sdf')

    # Robot State Publisher — uses sim time so TFs sync with Gazebo clock
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn the robot in Gazebo
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robodog',
            '-topic', '/robot_description',
            '-z', '0.35',
        ],
        output='screen',
    )

    # Bridge: Gazebo ←→ ROS
    #  - /clock              Gazebo → ROS  (simulation time)
    #  - /joint_states        Gazebo → ROS  (joint angles for robot_state_publisher)
    #  - /camera/image_raw    Gazebo → ROS  (camera feed)
    #  - /camera/camera_info   Gazebo → ROS  (camera intrinsics for RViz)
    #  - /model/robodog/joint/*/cmd_pos  ROS → Gazebo  (joint commands)
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]
    # Add a ROS→Gazebo bridge for each joint command topic
    for joint_name in _JOINT_CMD_TOPICS:
        bridge_args.append(
            f'/model/robodog/joint/{joint_name}/cmd_pos'
            f'@std_msgs/msg/Float64'
            f']gz.msgs.Double'
        )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
    )

    # RViz2 — uses sim time to stay in sync with Gazebo
    rviz_config = os.path.join(pkg_dir, 'rviz', 'robodog.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        gz_spawn,
        gz_bridge,
        rviz2,
    ])
