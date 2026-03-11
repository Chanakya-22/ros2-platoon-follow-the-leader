#!/bin/bash

# Source ros 2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting improved teleop for Leader Robot..."
echo "Use WASD or Arrow Keys!"

python3 src/robodog_repo/scripts/teleop_wasd.py
