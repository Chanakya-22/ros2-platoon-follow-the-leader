# ROS 2 Platoon Follow the Leader (Robodog Edition)

This project implements a leader-follower platoon system using two simulated Robodogs in ROS 2 and Gazebo Harmonic. The leader Robodog can be controlled manually via keyboard teleoperation, while the follower autonomously tracks and maintains a safe distance behind the leader.

## Features
- **Robodog Integration**: Replaced standard car models with fully articulated quadruped Robodogs.
- **Custom Gait Controller**: Uses an adaptive cmd_vel based controller (`walk_cmd_vel.py`) to map ROS 2 Twist commands into dynamic trot-gait joint commands in Gazebo.
- **Odometry Ground Truth**: Utilizes the `gz-sim-odometry-publisher-system` to feed accurate positioning data into the ROS 2 navigation stack.
- **Pure Pursuit Tracking**: The follower mathematically chases the leader's path using a pure pursuit algorithm.
- **Safe Distance Halting**: The follower robot will come to a smooth stop when it reaches 1.0m behind the leader.

## Dependencies
- ROS 2 (Humble or newer)
- Gazebo Harmonic
- `ros_gz` (ROS 2 / Gazebo bridge)
- `teleop_twist_keyboard`

## Building the Workspace
```bash
cd ~/Desktop/LeaderFollowingProject/ros2-platoon-follow-the-leader
colcon build
source install/setup.bash
```

## Running the Project

### 1. Launch the Simulation and Platoon Controller
In your first terminal, source the workspace and launch the core simulation. This will spawn both the leader and follower robodogs, start the gait controllers, and start the follower's autonomous tracking node.
```bash
source install/setup.bash
ros2 launch platoon_description platoon_robodogs.launch.py
```

### 2. Control the Leader (Teleop)
In a **new terminal**, run the standard ROS 2 teleop keyboard node, mapping the `cmd_vel` topic to `/leader/cmd_vel`:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/leader/cmd_vel
```

**Controls:**
- **I**: Move Forward
- **J**: Turn Left
- **L**: Turn Right
- **,** (comma): Move Backward
- **K**: STOP

As you drive the leader around the Gazebo world, the follower Robodog will adapt its gait frequency and steering to track behind you!

## Tuning Parameters
If you want to modify the tracking behavior, you can edit the launch file, or pass arguments to the `follower_controller`.
Parameters include:
- `lookahead_dist`: Distance along the leader's path the follower aims for (Default: 1.0m)
- `stop_dist`: Safe stopping distance between the robots (Default: 1.0m)
- `k_v`: Proportional gain for linear velocity (Default: 0.8)
- `k_w`: Proportional gain for angular velocity (Default: 1.2)
