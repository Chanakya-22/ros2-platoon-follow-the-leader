"""
=============================================================================
gazebo.launch.py  —  Single Robodog Gazebo Simulation Launch File
=============================================================================
PURPOSE:
    Launches a SINGLE robodog in Gazebo for standalone testing (without the
    platoon/follower setup).  Use this file when you want to test the robot
    model, gait, or camera in isolation before running the full two-robot demo.

    Launch with:
        ros2 launch br2_robodog gazebo.launch.py

WHAT THIS FILE STARTS:
    1. robot_state_publisher — broadcasts URDF-based TF frames to the ROS network
    2. Gazebo Sim            — the physics simulator running an empty world
    3. gz_spawn              — drops the robodog into Gazebo at position (0, 0, 0.35)
    4. gz_bridge             — two-way message bridge between Gazebo and ROS 2
    5. RViz2                 — optional visualisation tool (uses robodog.rviz config)

TOPIC MAP (after bridge is running):
    Gazebo → ROS:
        /clock               — simulation time (needed for use_sim_time nodes)
        /joint_states        — current joint angles (consumed by robot_state_publisher)
        /camera/image_raw    — 640×480 RGB camera mounted on the head
        /camera/camera_info  — camera intrinsics (focal length, principal point, etc.)

    ROS → Gazebo:
        /model/robodog/joint/<name>/cmd_pos  — position commands for each joint
            (published by walk_cmd_vel.py at 50 Hz during operation)

DIFFERENCE FROM platoon_robodogs.launch.py:
    This file only spawns ONE robot (no leader/follower namespaces).
    It also launches RViz for convenient visualisation.
    platoon_robodogs.launch.py is used for the full follow-the-leader demo.
=============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory  # Resolves ROS package install paths
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription                  # Pulls in another launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node   # Represents a single ROS 2 node to start
import xacro                          # Xacro preprocessor: expands macros in the URDF


# ── Joint bridge topic list ───────────────────────────────────────────────────
# 8 leg joints + 1 head joint, matching the names in robodog.urdf.xacro.
# Used to auto-generate one ROS→Gazebo bridge entry per joint below.
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
    """
    Required function called by the ROS 2 launch system.

    Builds and returns a LaunchDescription listing every node / include
    that should start when this file is launched.
    """

    # ── Resolve installed package path ────────────────────────────────────
    # get_package_share_directory('br2_robodog') returns something like:
    #   /home/<user>/robodog_ws/install/br2_robodog/share/br2_robodog/
    # All asset paths (URDF, world, RViz config) are relative to this.
    pkg_dir = get_package_share_directory('br2_robodog')

    # ── URDF from Xacro ───────────────────────────────────────────────────
    # robodog.urdf.xacro uses xacro macros to DRY up the 4 identical leg
    # definitions.  process_file() evaluates all ${...} expressions and
    # <xacro:macro> calls, producing a plain URDF XML string in memory.
    xacro_file        = os.path.join(pkg_dir, 'urdf', 'robodog.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── World SDF file ────────────────────────────────────────────────────
    # empty.sdf provides a flat ground plane with gravity but no obstacles.
    world_file = os.path.join(pkg_dir, 'worlds', 'empty.sdf')

    # =========================================================================
    # NODE 1: robot_state_publisher
    # =========================================================================
    # Reads the URDF and publishes TF (transform) frames for every link/joint.
    # Other tools (RViz, navigation stack) rely on these transforms to know
    # the spatial position of each part of the robot relative to 'base_link'.
    #
    # use_sim_time=True: this node uses Gazebo's /clock topic as its time source
    # instead of wall-clock time, keeping TF timestamps aligned with simulation.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,  # Full URDF XML string
            'use_sim_time': True,                    # Sync to Gazebo clock
        }],
    )

    # =========================================================================
    # NODE 2 (INCLUDE): Gazebo Sim
    # =========================================================================
    # IncludeLaunchDescription pulls in ros_gz_sim's launch file, which starts
    # the gz-sim process (Gazebo Harmonic / Ignition Gazebo).
    #
    # gz_args:
    #   '-r'          → automatically RUN the simulation on startup (don't pause)
    #   world_file    → the SDF world to load (empty.sdf = flat ground plane)
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

    # =========================================================================
    # NODE 3: Spawn the robot in Gazebo
    # =========================================================================
    # ros_gz_sim's 'create' executable reads a robot description from a ROS
    # topic and inserts the robot model into the running Gazebo world.
    #
    # '-topic', '/robot_description' — robot_state_publisher publishes the URDF
    #   to this topic, which 'create' reads to get the model definition.
    # '-z', '0.35' — spawn 35 cm above the ground so legs don't intersect the
    #   floor during the first physics timestep (avoids explosive joint forces).
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'robodog',            # Gazebo model name (appears in Gazebo UI)
            '-topic', '/robot_description', # Topic to read URDF from
            '-z',     '0.35',               # Initial height above ground (m)
        ],
        output='screen',
    )

    # =========================================================================
    # NODE 4: ROS ↔ Gazebo Topic Bridge
    # =========================================================================
    # Gazebo Harmonic uses its own Ignition Transport messaging system.
    # ros_gz_bridge's parameter_bridge translates between Ignition and ROS 2
    # messages in both directions.
    #
    # Bridge string syntax:
    #   /topic@ros_type[gz_type   →  Gazebo publishes, ROS subscribes
    #   /topic@ros_type]gz_type   →  ROS publishes, Gazebo subscribes
    bridge_args = [
        # Gazebo → ROS: simulation clock (mandatory for use_sim_time nodes)
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

        # Gazebo → ROS: joint states (JointStatePublisher plugin outputs this)
        # robot_state_publisher reads /joint_states to update TF link angles.
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

        # Gazebo → ROS: camera image from the head-mounted camera sensor
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

        # Gazebo → ROS: camera intrinsics (focal length, distortion, etc.)
        # Required by RViz's Camera display and any vision pipelines.
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]

    # ROS → Gazebo: one joint command bridge per joint
    # Each entry lets walk_cmd_vel.py send a Float64 position target that the
    # Gazebo JointPositionController plugin uses to servo the joint via PID.
    for joint_name in _JOINT_CMD_TOPICS:
        bridge_args.append(
            f'/model/robodog/joint/{joint_name}/cmd_pos'  # Topic name in Gazebo
            f'@std_msgs/msg/Float64'                       # ROS message type
            f']gz.msgs.Double'                            # ']' = ROS→Gazebo direction
        )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
    )

    # =========================================================================
    # NODE 5: RViz2 Visualisation (optional — safe to kill without breaking sim)
    # =========================================================================
    # Loads a pre-saved RViz configuration that shows the robot model, TF
    # frames, and optionally the camera feed.
    # use_sim_time=True keeps the display timestamps in sync with Gazebo.
    rviz_config = os.path.join(pkg_dir, 'rviz', 'robodog.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],          # Load the pre-saved config file
        parameters=[{'use_sim_time': True}],    # Sync to Gazebo clock
    )

    # =========================================================================
    # Return all launch actions.
    # ROS 2 launch starts them roughly in order, but exact timing is not
    # guaranteed — that's why the platoon launch file uses TimerAction delays.
    # =========================================================================
    return LaunchDescription([
        robot_state_publisher,  # 1. Start TF publisher (needs URDF)
        gz_sim,                 # 2. Start Gazebo physics engine
        gz_spawn,               # 3. Spawn robot into Gazebo
        gz_bridge,              # 4. Connect Gazebo ↔ ROS topics
        rviz2,                  # 5. Start RViz visualisation
    ])
