# ============================================================
#  platoon_robodogs.launch.py
#
#  WHAT DOES THIS FILE DO?
#  This is the MASTER SWITCH for the entire project.
#  Run this ONE file and it automatically starts EVERYTHING:
#    - Opens the Gazebo simulation world
#    - Drops TWO robodogs into the world
#    - Sets up communication between ROS and Gazebo
#    - Starts the walking controller for both dogs
#    - Starts the follow-the-leader brain (follower_controller.py)
#
#  Command to run it:
#    ros2 launch platoon_description platoon_robodogs.launch.py
#
#  STARTUP ORDER (important — things need time to load):
#    t = 0s → Gazebo starts, both dogs spawn, bridge starts
#    t = 3s → Walking controllers start (Gazebo needs 3s to fully load joints)
#    t = 8s → Follower brain starts (needs robots to be stable first)
# ============================================================


# --- IMPORTS ---

import os
# Python's built-in "operating system" library.
# We use it only for os.path.join() — which builds file paths correctly
# on any OS (avoids hardcoding slashes like '/').

from ament_index_python.packages import get_package_share_directory
# ROS 2 helper function.
# When you install a ROS package, its files go into a special folder called "share/".
# get_package_share_directory('package_name') tells you exactly WHERE that folder is.
# Example: get_package_share_directory('br2_robodog')
#          might return: /home/user/ws/install/br2_robodog/share/br2_robodog/

from launch import LaunchDescription
# LaunchDescription is the container that holds everything this launch file starts.
# Think of it as a list: "start these things when ros2 launch is called."

from launch.actions import IncludeLaunchDescription, TimerAction
# IncludeLaunchDescription = "run another launch file from inside this one"
#   (like calling a function from another file)
# TimerAction = "wait X seconds before starting something"
#   (lets us delay certain nodes so Gazebo has time to load first)

from launch.launch_description_sources import PythonLaunchDescriptionSource
# Tells ROS 2 that the launch file we're including is written in Python.
# (Launch files can also be XML or YAML — this clarifies the format.)

from launch_ros.actions import Node
# Node = represents ONE ROS 2 program (node) that we want to start.
# We'll create many of these — one per program in our system.

import xacro
# Xacro is a preprocessor for URDF robot description files.
# Our robot file (robodog.urdf.xacro) uses macros and math expressions
# like ${body_length/2}. xacro expands all of that into plain XML/URDF.


# ── JOINT NAMES LIST ─────────────────────────────────────────────────────────

# This list holds the names of all joints we need to control in Gazebo.
# Each joint is a motor — the eight leg joints + the head motor.
# We'll loop over this list later to set up communication channels for each joint.
_JOINT_CMD_TOPICS = [
    'front_left_hip_joint',    # Motor that swings the front-left leg forward/backward
    'front_left_knee_joint',   # Motor that bends the front-left knee (lifts the foot)
    'front_right_hip_joint',   # Front-right leg swing
    'front_right_knee_joint',  # Front-right knee bend
    'rear_left_hip_joint',     # Rear-left leg swing
    'rear_left_knee_joint',    # Rear-left knee bend
    'rear_right_hip_joint',    # Rear-right leg swing
    'rear_right_knee_joint',   # Rear-right knee bend
    'head_joint',              # Motor that pans the head left/right
]


# ── MAIN FUNCTION ─────────────────────────────────────────────────────────────

def generate_launch_description():
    # ROS 2 launch looks for a function EXACTLY named generate_launch_description().
    # This is the entry point — everything we add here gets started when launched.


    # ── FIND PACKAGE FOLDERS ──────────────────────────────────────────────────

    pkg_robodog = get_package_share_directory('br2_robodog')
    # Find where the robodog package is installed.
    # We need this to locate the URDF file and world file inside it.

    pkg_platoon_control = get_package_share_directory('platoon_control')
    # Find the platoon_control package folder.
    # (Not heavily used here, but good to have for future use.)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Find the ros_gz_sim package.
    # This package contains the launch file that starts Gazebo itself.


    # ── LOAD THE ROBOT DESCRIPTIONS (TWO SEPARATE URDFs) ──────────────────────
    #
    # WHY TWO SEPARATE URDFs?
    # The URDF contains JointPositionController plugins whose <topic> tag must
    # match the Gazebo model name so messages reach the right robot.
    # Leader  → controllers listen on /model/leader/joint/*/cmd_pos
    # Follower → controllers listen on /model/follower/joint/*/cmd_pos
    #
    # We pass 'gz_model_name' as a xacro argument so each URDF gets the correct
    # model name baked in.  xacro.process_file() + mappings does the substitution.

    xacro_file = os.path.join(pkg_robodog, 'urdf', 'robodog.urdf.xacro')
    # Path to the shared xacro template (both robots use the same template file).

    leader_description_xml = xacro.process_file(
        xacro_file, mappings={'gz_model_name': 'leader'}
    ).toxml()
    # Generate the LEADER URDF: gz_model_name='leader'
    # → JointPositionController topics become /model/leader/joint/*/cmd_pos
    # → walk_cmd_vel (leader namespace) publishes to /model/leader/joint/*/cmd_pos ✓

    follower_description_xml = xacro.process_file(
        xacro_file, mappings={'gz_model_name': 'follower'}
    ).toxml()
    # Generate the FOLLOWER URDF: gz_model_name='follower'
    # → JointPositionController topics become /model/follower/joint/*/cmd_pos
    # → walk_cmd_vel (follower namespace) publishes to /model/follower/joint/*/cmd_pos ✓


    # ── WORLD FILE ────────────────────────────────────────────────────────────

    world_file = os.path.join(pkg_robodog, 'worlds', 'empty.sdf')
    # Path to the Gazebo world file.
    # 'empty.sdf' is just a flat ground plane with gravity — no obstacles.
    # SDF = Simulation Description Format (Gazebo's world format).


    # =========================================================================
    # 1. START GAZEBO
    # =========================================================================

    gz_sim = IncludeLaunchDescription(
        # IncludeLaunchDescription = "run this other launch file"
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            # Path to the official Gazebo launch file provided by the ros_gz_sim package.
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        # Pass arguments INTO the Gazebo launch file.
        # '-r' means "run immediately" — start the simulation clock right away
        #      (instead of pausing and waiting for you to click Play).
        # world_file = the path to our empty.sdf world.
    )


    # =========================================================================
    # 2. SPAWN THE LEADER ROBODOG
    # =========================================================================

    spawn_leader = Node(
        package='ros_gz_sim',          # The ROS package that contains the spawner tool
        executable='create',           # The actual program to run (ros_gz_sim's 'create' command)
        arguments=[
            '-name',   'leader',       # Name the Gazebo model "leader"
                                       #   → its topics become /model/leader/...
            '-string', leader_description_xml,  # Leader URDF: /model/leader/joint/... topics
            '-x', '0.0',               # Spawn at X = 0 metres (no left/right offset)
            '-y', '0.0',               # Spawn at Y = 0 metres (no forward/backward offset)
            '-z', '0.35'               # Spawn at Z = 0.35 metres (35cm above ground)
                                       #   → needed so the legs don't clip through the floor
        ],
        output='screen'                # Show this node's output in the terminal
    )

    leader_state_publisher = Node(
        # robot_state_publisher reads the URDF and publishes TF transforms.
        # TF = a system that tracks WHERE every part of the robot is in 3D space.
        # RViz and other tools use TF to visualise the robot correctly.
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_leader',   # Unique name (we'll have two RSPs running)
        namespace='leader',                    # All topics this node uses go under /leader/
        output='screen',
        parameters=[{
            'robot_description': leader_description_xml,  # Leader URDF
            'use_sim_time': True,     # Use Gazebo's clock instead of real wall-clock time
            'frame_prefix': 'leader/'
            # Prefix all TF frame names with "leader/".
            # Without this, both robots would publish a frame called "base_link"
            # and they'd overwrite each other. With the prefix:
            #   Leader  → "leader/base_link", "leader/front_left_upper_leg", etc.
            #   Follower→ "follower/base_link", "follower/front_left_upper_leg", etc.
        }],
    )


    # =========================================================================
    # 3. SPAWN THE FOLLOWER ROBODOG
    # =========================================================================
    # Same as the leader, but placed 1.5 metres BEHIND (negative X direction).
    # This gives them a realistic starting gap so the follower has room to react.

    spawn_follower = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',   'follower',     # Gazebo model name = "follower"
            '-string', follower_description_xml,  # Follower URDF: /model/follower/joint/... topics
            '-x', '-1.5',             # 1.5 metres behind the leader (negative X)
            '-y',  '0.0',
            '-z',  '0.35'
        ],
        output='screen'
    )

    follower_state_publisher = Node(
        # Same as the leader's robot_state_publisher but for the follower.
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_follower',
        namespace='follower',          # All topics under /follower/
        output='screen',
        parameters=[{
            'robot_description': follower_description_xml,  # Follower URDF
            'use_sim_time': True,
            'frame_prefix': 'follower/'  # TF frames: "follower/base_link", etc.
        }],
    )


    # =========================================================================
    # 4. SET UP THE ROS ↔ GAZEBO BRIDGE
    # =========================================================================
    # Gazebo and ROS 2 are two separate systems — they DON'T talk to each other
    # directly. The "bridge" is a translator that converts messages between them.
    #
    # Bridge string format:
    #   /topic_name @ ros_type [ gz_type    →  '[' means Gazebo → ROS  (subscribe)
    #   /topic_name @ ros_type ] gz_type    →  ']' means ROS → Gazebo  (publish)
    #
    # We build a LIST of all the bridges we need, then pass it to one bridge node.

    bridge_args = []
    # Start with an empty list. We'll append each bridge string one by one.


    # ── LEADER JOINT COMMAND BRIDGES (ROS → Gazebo) ───────────────────────────
    # For each joint motor, we need a channel so walk_cmd_vel.py can send
    # position commands FROM ROS INTO Gazebo.
    for joint_name in _JOINT_CMD_TOPICS:
        bridge_args.append(
            f'/model/leader/joint/{joint_name}/cmd_pos'  # The Gazebo-side topic name
            f'@std_msgs/msg/Float64'                      # ROS message type (a single number)
            f']gz.msgs.Double'   # ']' = ROS publishes this, Gazebo subscribes
        )
    # After this loop, we've added 9 bridge entries (one per joint) for the leader.

    bridge_args.append(
        '/model/leader/odometry_with_covariance'
        '@nav_msgs/msg/Odometry'
        '[gz.msgs.OdometryWithCovariance'
        # '[' = Gazebo publishes the leader's position, ROS subscribes.
        # This is how follower_controller.py knows where the leader is.
    )

    # ── FOLLOWER JOINT COMMAND BRIDGES (ROS → Gazebo) ─────────────────────────
    # Same as above but for the follower model.
    for joint_name in _JOINT_CMD_TOPICS:
        bridge_args.append(
            f'/model/follower/joint/{joint_name}/cmd_pos'
            f'@std_msgs/msg/Float64'
            f']gz.msgs.Double'
        )

    bridge_args.append(
        '/model/follower/odometry_with_covariance'
        '@nav_msgs/msg/Odometry'
        '[gz.msgs.OdometryWithCovariance'
        # Gazebo publishes the follower's position → ROS receives it.
    )

    bridge = Node(
        package='ros_gz_bridge',      # The bridge package
        executable='parameter_bridge',  # The specific bridge program
        arguments=bridge_args,         # Pass all the bridge strings we built above
        output='screen',
        remappings=[
            # Rename the long, ugly Gazebo topic names to short, clean ROS names.
            # Before remapping: /model/leader/odometry_with_covariance
            # After remapping:  /leader/odom    ← what follower_controller.py listens to
            ('/model/leader/odometry_with_covariance',   '/leader/odom'),
            ('/model/follower/odometry_with_covariance', '/follower/odom'),
        ]
    )


    # =========================================================================
    # 5. START WALKING CONTROLLERS — delayed by 3 seconds
    # =========================================================================
    # walk_cmd_vel.py converts /leader/cmd_vel or /follower/cmd_vel into
    # actual joint position commands that make the robot walk.
    #
    # WHY the 3-second delay?
    # Gazebo takes a couple of seconds to fully load the robot and its joint controllers.
    # If walk_cmd_vel starts too early, its joint commands are just dropped/ignored.
    # Waiting 3 seconds ensures Gazebo is ready to receive joint commands.

    leader_walk_node = TimerAction(
        period=3.0,   # Wait 3 seconds after launch before starting this
        actions=[
            Node(
                package='br2_robodog',       # Package where walk_cmd_vel.py lives
                executable='walk_cmd_vel.py',  # The script to run
                name='walk_cmd_vel_node_leader',
                namespace='leader',
                # namespace='leader' means it auto-subscribes to /leader/cmd_vel
                # (the namespace is prepended to the relative topic name 'cmd_vel')
                output='screen',
                parameters=[{
                    'gz_model_name': 'leader',
                    'base_gait_frequency': 2.0,   # Faster cadence to compensate for smaller strides
                    'base_hip_amplitude': 0.25,    # Reduced: smaller strides = less body sway
                    'knee_amplitude': 0.4,          # Reduced: less foot lift = more ground contact
                    'update_rate': 50.0,
                }],
            )
        ]
    )

    follower_walk_node = TimerAction(
        period=3.0,   # Also delayed 3 seconds
        actions=[
            Node(
                package='br2_robodog',
                executable='walk_cmd_vel.py',
                name='walk_cmd_vel_node_follower',
                namespace='follower',
                # Subscribes to /follower/cmd_vel automatically
                output='screen',
                parameters=[{
                    'gz_model_name': 'follower',
                    'base_gait_frequency': 2.0,
                    'base_hip_amplitude': 0.25,
                    'knee_amplitude': 0.4,
                    'update_rate': 50.0,
                }],
            )
        ]
    )


    # =========================================================================
    # 6. START THE PLATOON FOLLOWER BRAIN — delayed by 8 seconds
    # =========================================================================
    # This starts follower_controller.py — the Pure Pursuit brain that reads
    # odometry from both robots and outputs /follower/cmd_vel.
    #
    # WHY 8 seconds (not 3)?
    # The follower brain needs STABLE odometry data to work correctly.
    # Right after spawning, Gazebo's physics engine is still settling the robot
    # (the robot might bounce/wobble). 8 seconds gives plenty of time for
    # everything to settle so odometry readings are accurate before the controller starts.

    platoon_follower_node = TimerAction(
        period=8.0,   # Wait 8 full seconds after launch
        actions=[
            Node(
                package='platoon_control',       # The package containing our controller
                executable='follower_controller', # Runs follower_controller.py
                output='screen',
                remappings=[
                    # The controller uses generic topic names internally.
                    # These remappings connect them to the actual simulation topics.
                    ('leader_odom',   '/leader/odom'),    # Where to read leader position from
                    ('follower_odom', '/follower/odom'),  # Where to read follower position from
                    ('cmd_vel',       '/follower/cmd_vel') # Where to send speed commands to
                ],
                parameters=[{
                    # These override the default values declared in follower_controller.py.
                    'lookahead_dist': 1.5,  # Larger lookahead = smoother, less jerky path following
                    'k_v': 0.6,             # Linear speed gain (slightly reduced for stability)
                    'k_w': 0.5,             # Angular gain REDUCED: gentler turns = no joint hard-stop hits
                    # k_w × max_error (π rad) = 1.57, clamped to ±0.5 → very smooth correction
                }]
            )
        ]
    )


    # =========================================================================
    # RETURN — hand the full list of everything to start back to ROS 2 launch
    # =========================================================================
    # ROS 2 launch reads this list and starts each item.
    # TimerAction items fire after their delay periods automatically.

    return LaunchDescription([
        gz_sim,                    # 1. Start Gazebo simulator  (t = 0s)
        spawn_leader,              # 2. Drop leader dog into world (t = 0s)
        leader_state_publisher,    #    Start leader TF broadcaster  (t = 0s)
        spawn_follower,            # 3. Drop follower dog into world (t = 0s)
        follower_state_publisher,  #    Start follower TF broadcaster (t = 0s)
        bridge,                    # 4. Start ROS ↔ Gazebo bridge    (t = 0s)
        leader_walk_node,          # 5. Start leader walking controller (t = 3s)
        follower_walk_node,        #    Start follower walking controller (t = 3s)
        platoon_follower_node,     # 6. Start follow-the-leader brain (t = 8s)
    ])