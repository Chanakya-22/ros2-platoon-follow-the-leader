# br2_robodog

A simple quadruped robot (RoboDog) package for ROS 2 Jazzy, featuring a URDF model with a camera sensor, RViz visualization, and Gazebo simulation.

## Package Structure

```
br2_robodog/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   └── robodog.urdf.xacro        # Robot model (Xacro)
├── scripts/
│   └── walk.py                    # Trot gait walking controller
├── launch/
│   ├── rviz.launch.py             # Standalone RViz visualization
│   ├── gazebo.launch.py           # Gazebo simulation + RViz
│   └── walk.launch.py             # Gazebo + walking gait
├── rviz/
│   └── robodog.rviz               # Pre-configured RViz layout
└── worlds/
    └── empty.sdf                  # Minimal Gazebo world
```

## Robot Model

The RoboDog is a simple quadruped built from box and cylinder primitives.

### Link Tree

```
base_link (body — orange box)
├── head_link (head — dark box, revolute yaw joint)
│   └── camera_link (camera — black box, fixed joint)
│       └── camera_optical_frame (fixed, Z-forward convention)
├── front_left_upper_leg (cylinder, revolute hip joint)
│   └── front_left_lower_leg (cylinder, revolute knee joint)
├── front_right_upper_leg
│   └── front_right_lower_leg
├── rear_left_upper_leg
│   └── rear_left_lower_leg
└── rear_right_upper_leg
    └── rear_right_lower_leg
```

**Total:** 12 links, 11 joints (1 head yaw + 1 camera fixed + 1 optical frame fixed + 8 leg revolute)

### Dimensions

| Part | Shape | Size |
|------|-------|------|
| Body | Box | 0.4 × 0.2 × 0.1 m |
| Head | Box | 0.1 × 0.12 × 0.08 m |
| Camera | Box | 0.02 × 0.04 × 0.02 m |
| Upper Leg | Cylinder | r=0.02, l=0.15 m |
| Lower Leg | Cylinder | r=0.015, l=0.15 m |

### Joints

| Joint | Type | Axis | Limits |
|-------|------|------|--------|
| `head_joint` | revolute | Z (yaw) | ±0.5 rad |
| `*_hip_joint` (×4) | revolute | Y (pitch) | ±0.8 rad |
| `*_knee_joint` (×4) | revolute | Y (pitch) | -1.2 to 0.0 rad |
| `camera_joint` | fixed | — | — |
| `camera_optical_joint` | fixed | — | — |

### Sensors

- **Camera** on the head link (640×480, 30 Hz, 80° FOV), published to `/camera/image_raw`

### Gazebo Plugins

| Plugin | Purpose |
|--------|---------|
| `gz::sim::systems::JointStatePublisher` | Publishes joint states from simulation |

## Dependencies

- `robot_state_publisher` — publishes TF from joint states + URDF
- `joint_state_publisher_gui` — interactive joint sliders (RViz-only mode)
- `xacro` — processes `.xacro` into URDF
- `rviz2` — 3D visualization
- `ros_gz_sim` — launches Gazebo and spawns models
- `ros_gz_bridge` — bridges topics between Gazebo and ROS 2

## Build

```bash
cd ~/Documents/Job/Amrita/ROS2/Book-Workspace
colcon build --packages-select br2_robodog
source install/setup.bash
```

## Launch

### Standalone RViz (no simulation)

Visualize the URDF with interactive joint sliders — useful for inspecting the model.

```bash
ros2 launch br2_robodog rviz.launch.py
```

- Uses `joint_state_publisher_gui` for manual joint control
- No physics, no simulation

### Gazebo Simulation + RViz

Run the full simulation with Gazebo driving the robot state.

```bash
ros2 launch br2_robodog gazebo.launch.py
```

This launch file:
1. Starts **Gazebo** with an empty world (ground plane + sun)
2. Spawns the RoboDog at z=0.35 m
3. Starts **robot_state_publisher** with `use_sim_time: true`
4. Bridges the following topics from Gazebo ↔ ROS 2:

   | Gazebo Topic | ROS 2 Topic | Direction | Message Type |
   |-------------|-------------|-----------|--------------|
   | `/clock` | `/clock` | Gz → ROS | `rosgraph_msgs/msg/Clock` |
   | `/joint_states` | `/joint_states` | Gz → ROS | `sensor_msgs/msg/JointState` |
   | `/camera/image_raw` | `/camera/image_raw` | Gz → ROS | `sensor_msgs/msg/Image` |
   | `/model/robodog/joint/*/cmd_pos` | (same) | ROS → Gz | `std_msgs/msg/Float64` |

5. Opens **RViz2** with the pre-configured layout (`use_sim_time: true`)

### Walking Gait (Gazebo + Walk Node)

Launch the robot and make it walk with a trot gait:

```bash
ros2 launch br2_robodog walk.launch.py
```

This includes the Gazebo launch and starts the `walk_node` after a 5-second delay (letting the robot spawn and settle). The trot gait moves diagonal leg pairs in sync.

#### Gait Parameters

You can tune these via launch arguments or `ros2 param set`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gait_frequency` | 1.5 Hz | Step cycles per second |
| `hip_amplitude` | 0.35 rad | Hip swing range |
| `knee_amplitude` | 0.5 rad | Knee bend depth |
| `knee_offset` | -0.3 rad | Resting knee bend |
| `update_rate` | 50.0 Hz | Control loop rate |

### Viewing the Camera Feed

While the Gazebo launch is running, you can view the camera in a separate terminal:

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Or enable the **Camera** display in RViz (it's included but disabled by default).

## ROS 2 Topics

| Topic | Type | Source |
|-------|------|--------|
| `/robot_description` | `std_msgs/msg/String` | `robot_state_publisher` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo bridge or `joint_state_publisher_gui` |
| `/tf` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo bridge |
